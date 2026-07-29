//! ============================================================================
//! USART1 DMA Driver
//! ============================================================================
//!
//! Target MCU
//! ----------
//! STM32C031K4/K6
//!
//! Features
//! --------
//! - PAC only (stm32c0 v0.16)
//! - DMA driven transmit and receive
//! - No USART interrupts
//! - No DMA interrupts
//! - Hardware RS485 Driver Enable (DEM)
//! - Hardware Modbus RTU frame detection using Receiver Timeout (RTOR)
//! - Designed for deterministic RTIC polling (typically every 1 ms)
//!
//! DMA Resources
//! -------------
//! DMA Channel 0 : USART1_RX
//! DMA Channel 1 : USART1_TX
//!
//! DMAMUX Requests
//! ---------------
//! USART1 RX : Request 50
//! USART1 TX : Request 51
//!
//! Driver Model
//! ------------
//!
//! RX Path
//!
//!     RS485
//!        │
//!        ▼
//!     USART1
//!        │
//!        ▼
//!     DMA CH0
//!        │
//!        ▼
//!     rx_buf[]
//!        │
//!        ▼
//!     receive_data()
//!
//! TX Path
//!
//!     send_data()
//!        │
//!        ▼
//!     tx_buf[]
//!        │
//!        ▼
//!     DMA CH1
//!        │
//!        ▼
//!     USART1
//!        │
//!        ▼
//!      RS485
//!
//! Operation
//! ---------
//! The application must periodically call `poll()`.
//!
//! `poll()`
//!
//! - recovers from RX overrun
//! - detects Modbus RTU frame completion
//! - finalises DMA transmissions
//! - restarts the receiver after transmission
//!
//! The driver never enables USART interrupts or DMA interrupts.

use stm32c0::stm32c031 as pac;

use crate::bsp::SYSCLK_HZ;

//=============================================================================
// Configuration
//=============================================================================

/// USART baud rate.
const BAUDRATE: u32 = 9_600;

/// Receive buffer size.
const RX_BUF_SIZE: usize = 256;

/// Transmit buffer size.
const TX_BUF_SIZE: usize = 256;

/// Receiver timeout expressed in bit times.
///
/// Modbus RTU defines a frame boundary after approximately
/// 3.5 character times of bus silence.
///
/// RTOR performs this detection completely in hardware.
const RX_TIMEOUT_BITS: u16 = 40;

/// DMAMUX request numbers (RM0490 Table 49).
const USART1_RX_DMA_REQ: u8 = 50;
const USART1_TX_DMA_REQ: u8 = 51;

/// DMA interrupt flag clear masks.
const DMA_CH0_CLEAR_FLAGS: u32 = 0x000F;
const DMA_CH1_CLEAR_FLAGS: u32 = 0x00F0;

/// DMA-driven USART1 transport.
///
/// This driver owns:
///
/// - USART1 peripheral
/// - DMA Channel 0 (RX)
/// - DMA Channel 1 (TX)
///
/// The application interacts through:
///
/// - `poll()`
/// - `receive_data()`
/// - `send_data()`
///
/// No interrupts are used.
pub struct UartDma {
    /// USART peripheral.
    usart: pac::USART1,

    // ---------------------------------------------------------------------
    // Receive state
    // ---------------------------------------------------------------------
    /// DMA receive buffer.
    rx_buf: [u8; RX_BUF_SIZE],

    /// Number of received bytes.
    rx_len: usize,

    /// Complete frame available.
    rx_ready: bool,

    /// RX DMA channel currently enabled.
    rx_dma_active: bool,

    // ---------------------------------------------------------------------
    // Transmit state
    // ---------------------------------------------------------------------
    /// DMA transmit buffer.
    tx_buf: [u8; TX_BUF_SIZE],

    /// Number of bytes to transmit.
    tx_len: usize,

    /// Transmission currently active.
    tx_busy: bool,
}

impl UartDma {
    /// Creates and initializes the USART1 DMA driver.
    ///
    /// Initialization sequence:
    ///
    /// 1. Enable peripheral clocks.
    /// 2. Configure USART.
    /// 3. Configure Modbus RTOR.
    /// 4. Enable hardware RS485 Driver Enable.
    /// 5. Configure DMA.
    /// 6. Start continuous DMA reception.
    pub fn new(usart: pac::USART1, dma: &pac::DMA, dmamux: &pac::DMAMUX, rcc: &pac::RCC) -> Self {
        //------------------------------------------------------------------
        // Enable peripheral clocks
        //------------------------------------------------------------------

        rcc.apbenr2().modify(|_, w| w.usart1en().set_bit());
        rcc.ahbenr().modify(|_, w| w.dma1en().set_bit());

        //------------------------------------------------------------------
        // Disable USART while configuring
        //------------------------------------------------------------------

        usart.cr1().modify(|_, w| w.ue().clear_bit());

        // FIFO is unnecessary for DMA operation.
        usart.cr1().modify(|_, w| w.fifoen().clear_bit());

        //------------------------------------------------------------------
        // Baud rate
        //------------------------------------------------------------------

        usart
            .brr()
            .write(|w| unsafe { w.bits(SYSCLK_HZ / BAUDRATE) });

        //------------------------------------------------------------------
        // Receiver timeout (Modbus RTU frame detection)
        //------------------------------------------------------------------

        usart
            .rtor()
            .write(|w| unsafe { w.rto().bits(RX_TIMEOUT_BITS.into()) });

        usart.cr2().modify(|_, w| w.rtoen().set_bit());

        //------------------------------------------------------------------
        // Hardware RS485 Driver Enable
        //------------------------------------------------------------------

        usart.cr3().modify(|_, w| {
            w.dem().set_bit();
            w.dep().set_bit()
        });

        //------------------------------------------------------------------
        // Clear stale Transmission Complete flag
        //------------------------------------------------------------------

        usart.icr().write(|w| w.tccf().bit(true));

        //------------------------------------------------------------------
        // Enable DMA requests
        //------------------------------------------------------------------

        usart.cr3().modify(|_, w| {
            w.dmar().set_bit();
            w.dmat().set_bit()
        });

        //------------------------------------------------------------------
        // Enable transmitter and receiver
        //------------------------------------------------------------------

        usart.cr1().modify(|_, w| {
            w.re().set_bit();
            w.te().set_bit()
        });

        //------------------------------------------------------------------
        // Enable USART
        //------------------------------------------------------------------

        usart.cr1().modify(|_, w| w.ue().set_bit());

        // Wait until hardware acknowledges.
        while !usart.isr().read().teack().bit_is_set() {}
        while !usart.isr().read().reack().bit_is_set() {}

        //------------------------------------------------------------------
        // Construct driver
        //------------------------------------------------------------------

        let mut uart = Self {
            usart,

            rx_buf: [0; RX_BUF_SIZE],
            rx_len: 0,
            rx_ready: false,
            rx_dma_active: false,

            tx_buf: [0; TX_BUF_SIZE],
            tx_len: 0,
            tx_busy: false,
        };

        //------------------------------------------------------------------
        // Configure DMA hardware
        //------------------------------------------------------------------

        uart.configure_dma(dma, dmamux);

        //------------------------------------------------------------------
        // Begin continuous reception
        //------------------------------------------------------------------

        uart.start_rx_dma();

        uart
    }
    //=========================================================================
    // Public API
    //=========================================================================

    /// Advances the UART DMA state machine.
    ///
    /// This function must be called periodically by the application
    /// (typically every 1 ms from an RTIC task).
    ///
    /// The state machine performs three independent operations:
    ///
    /// 1. Recover from USART overrun errors.
    /// 2. Detect Modbus RTU frame completion using RTOR.
    /// 3. Detect completion of DMA transmissions.
    ///
    /// State flow
    /// ----------
    ///
    /// ```text
    ///          poll()
    ///             │
    ///             ▼
    ///        ORE detected?
    ///         │        │
    ///       Yes       No
    ///         │        │
    ///         ▼        ▼
    ///   restart_rx()  RTOF?
    ///                    │
    ///          ┌─────────┴─────────┐
    ///          │                   │
    ///         No                  Yes
    ///          │                   │
    ///          ▼                   ▼
    ///        TX done?       RX frame ready
    ///          │
    ///     ┌────┴────┐
    ///     │         │
    ///    No        Yes
    ///     │         │
    ///     ▼         ▼
    ///    Exit   Restart RX DMA
    /// ```
    pub fn poll(&mut self) {
        let isr = self.usart.isr().read();

        //------------------------------------------------------------------
        // 1. Recover from USART overrun.
        //------------------------------------------------------------------

        if isr.ore().bit_is_set() {
            self.usart.icr().write(|w| w.orecf().bit(true));

            self.restart_rx();

            return;
        }

        //------------------------------------------------------------------
        // 2. Receiver timeout.
        //
        // RTOR indicates that the Modbus silent interval has elapsed.
        // The current DMA transfer therefore contains one complete frame.
        //------------------------------------------------------------------

        if isr.rtof().bit_is_set() {
            self.stop_rx_dma();

            self.usart.icr().write(|w| w.rtocf().bit(true));

            self.rx_len = self.rx_length();

            if self.rx_len > 0 {
                self.rx_ready = true;
            } else {
                // Ignore empty frames.
                self.start_rx_dma();
            }
        }

        //------------------------------------------------------------------
        // 3. Transmission complete.
        //------------------------------------------------------------------

        if self.tx_busy && isr.tc().bit_is_set() {
            self.usart.icr().write(|w| w.tccf().bit(true));

            let dma = unsafe { &*pac::DMA::ptr() };

            dma.ch(1).cr().modify(|_, w| w.en().clear_bit());

            self.tx_busy = false;

            // Return to receive mode.
            self.start_rx_dma();
        }
    }

    /// Copies the completed receive frame into `dst`.
    ///
    /// Parameters
    /// ----------
    ///
    /// * `dst` - Application supplied receive buffer.
    ///
    /// Returns
    /// -------
    ///
    /// * `Some(length)` if a complete frame was available.
    /// * `None` if reception is still in progress.
    ///
    /// Notes
    /// -----
    ///
    /// The driver owns the internal DMA buffer.
    ///
    /// Copying the frame allows RX DMA to be restarted immediately,
    /// avoiding lifetime issues and keeping the driver independent
    /// from application memory.
    pub fn receive_data(&mut self, dst: &mut [u8]) -> Option<usize> {
        if !self.rx_ready {
            return None;
        }

        let len = self.rx_len.min(dst.len());

        dst[..len].copy_from_slice(&self.rx_buf[..len]);

        // Consume the current frame.
        self.rx_ready = false;
        self.rx_len = 0;

        // Resume reception immediately.
        self.start_rx_dma();

        Some(len)
    }

    /// Starts transmission of a frame using DMA.
    ///
    /// Parameters
    /// ----------
    ///
    /// * `data` - Frame to transmit.
    ///
    /// Returns
    /// -------
    ///
    /// * `Ok(())` if DMA transmission started.
    /// * `Err(())` if:
    ///     * another transmission is active,
    ///     * the frame is empty,
    ///     * the frame exceeds the TX buffer size.
    ///
    /// Reception is suspended during transmission and automatically
    /// resumed when the USART Transmission Complete flag is detected
    /// by `poll()`.
    pub fn send_data(&mut self, data: &[u8]) -> Result<(), ()> {
        if self.tx_busy {
            return Err(());
        }

        if data.is_empty() {
            return Err(());
        }

        // Prevent silent frame truncation.
        if data.len() > TX_BUF_SIZE {
            return Err(());
        }

        self.tx_buf[..data.len()].copy_from_slice(data);

        self.tx_len = data.len();

        // Any previous receive frame has now been consumed.
        self.rx_ready = false;
        self.rx_len = 0;

        self.stop_rx_dma();

        self.tx_busy = true;

        self.start_tx_dma();

        Ok(())
    }

    /// Discards the current receive state and restarts DMA reception.
    ///
    /// Normally used after detecting a UART overrun or protocol error.
    pub fn restart_rx(&mut self) {
        self.stop_rx_dma();

        self.rx_ready = false;
        self.rx_len = 0;

        self.start_rx_dma();
    }

    /// Returns `true` while a DMA transmission is active.
    #[inline(always)]
    pub fn tx_busy(&self) -> bool {
        self.tx_busy
    }

    //=========================================================================
    // DMA Configuration
    //=========================================================================
    //
    // DMA Channel Allocation
    //
    //      Channel 0  -> USART1 RX
    //      Channel 1  -> USART1 TX
    //
    // DMAMUX Allocation
    //
    //      CH0 -> Request 50 (USART1_RX)
    //      CH1 -> Request 51 (USART1_TX)
    //
    // The DMA channels are configured once during driver
    // initialization. Runtime operation only reloads MAR and NDTR.
    //=========================================================================

    /// Returns the address of the USART receive data register.
    #[inline(always)]
    fn rdr_addr(&self) -> u32 {
        self.usart.rdr().as_ptr() as u32
    }

    /// Returns the address of the USART transmit data register.
    #[inline(always)]
    fn tdr_addr(&self) -> u32 {
        self.usart.tdr().as_ptr() as u32
    }

    /// Configures both DMA channels used by the driver.
    ///
    /// This function is called only once during initialization.
    ///
    /// Channel allocation
    ///
    ///     CH0 -> USART1 RX
    ///     CH1 -> USART1 TX
    ///
    /// After configuration the channels remain disabled until
    /// `start_rx_dma()` or `start_tx_dma()` is called.
    fn configure_dma(&mut self, dma: &pac::DMA, dmamux: &pac::DMAMUX) {
        //------------------------------------------------------------------
        // Ensure both DMA channels are disabled.
        //------------------------------------------------------------------

        dma.ch(0).cr().modify(|_, w| w.en().clear_bit());
        dma.ch(1).cr().modify(|_, w| w.en().clear_bit());

        //------------------------------------------------------------------
        // DMAMUX configuration
        //
        // RM0490 Table 49
        //
        // CH0 -> USART1_RX
        // CH1 -> USART1_TX
        //------------------------------------------------------------------

        dmamux
            .ccr(0)
            .write(|w| unsafe { w.dmareq_id().bits(USART1_RX_DMA_REQ) });

        dmamux
            .ccr(1)
            .write(|w| unsafe { w.dmareq_id().bits(USART1_TX_DMA_REQ) });

        //------------------------------------------------------------------
        // DMA Channel 0
        //
        // USART1 RX
        //
        // Peripheral -> Memory
        //------------------------------------------------------------------

        // Peripheral address is fixed.
        dma.ch(0)
            .par()
            .write(|w| unsafe { w.bits(self.rdr_addr()) });

        // Destination buffer.
        dma.ch(0)
            .mar()
            .write(|w| unsafe { w.bits(self.rx_buf.as_mut_ptr() as u32) });

        // Maximum transfer length.
        dma.ch(0)
            .ndtr()
            .write(|w| unsafe { w.bits(RX_BUF_SIZE as u32) });

        dma.ch(0).cr().write(|w| {
            // Peripheral -> Memory
            w.dir().clear_bit();

            // Peripheral address remains fixed.
            w.pinc().clear_bit();

            // Increment memory pointer.
            w.minc().set_bit();

            // Normal mode.
            w.circ().clear_bit();

            // Channel remains disabled.
            w.en().clear_bit();

            w
        });

        //------------------------------------------------------------------
        // DMA Channel 1
        //
        // USART1 TX
        //
        // Memory -> Peripheral
        //------------------------------------------------------------------

        dma.ch(1)
            .par()
            .write(|w| unsafe { w.bits(self.tdr_addr()) });

        dma.ch(1)
            .mar()
            .write(|w| unsafe { w.bits(self.tx_buf.as_ptr() as u32) });

        // Loaded immediately before every transmission.
        dma.ch(1).ndtr().write(|w| unsafe { w.bits(0) });

        dma.ch(1).cr().write(|w| {
            // Memory -> Peripheral
            w.dir().set_bit();

            // Peripheral address remains fixed.
            w.pinc().clear_bit();

            // Increment memory pointer.
            w.minc().set_bit();

            // Normal mode.
            w.circ().clear_bit();

            // Channel remains disabled.
            w.en().clear_bit();

            w
        });
    }

    //=========================================================================
    // RX DMA Control
    //=========================================================================
    //
    // Reception Sequence
    //
    //      start_rx_dma()
    //             │
    //             ▼
    //      DMA receives bytes
    //             │
    //             ▼
    //      RTOR detects end of frame
    //             │
    //             ▼
    //        poll()
    //             │
    //             ▼
    //      stop_rx_dma()
    //             │
    //             ▼
    //      receive_data()
    //             │
    //             ▼
    //      start_rx_dma()
    //
    // Only one receive operation is active at any time.
    //=========================================================================

    /// Starts DMA reception.
    ///
    /// This function:
    ///
    /// - clears any previous USART overrun condition,
    /// - clears DMA status flags,
    /// - reloads the receive buffer address,
    /// - reloads the transfer counter,
    /// - enables DMA Channel 0.
    ///
    /// After this function returns, USART1 continuously transfers every
    /// received byte directly into `rx_buf`.
    fn start_rx_dma(&mut self) {
        let dma = unsafe { &*pac::DMA::ptr() };

        //------------------------------------------------------------------
        // Clear any previous USART overrun.
        //------------------------------------------------------------------

        self.usart.icr().write(|w| w.orecf().bit(true));

        //------------------------------------------------------------------
        // Disable DMA channel before reconfiguration.
        //------------------------------------------------------------------

        dma.ch(0).cr().modify(|_, w| w.en().clear_bit());

        //------------------------------------------------------------------
        // Clear all DMA Channel 0 status flags.
        //------------------------------------------------------------------

        dma.ifcr().write(|w| unsafe { w.bits(DMA_CH0_CLEAR_FLAGS) });

        //------------------------------------------------------------------
        // Reload peripheral address.
        //------------------------------------------------------------------

        dma.ch(0)
            .par()
            .write(|w| unsafe { w.bits(self.rdr_addr()) });

        //------------------------------------------------------------------
        // Reload destination buffer.
        //------------------------------------------------------------------

        dma.ch(0)
            .mar()
            .write(|w| unsafe { w.bits(self.rx_buf.as_mut_ptr() as u32) });

        //------------------------------------------------------------------
        // Reload transfer length.
        //------------------------------------------------------------------

        dma.ch(0)
            .ndtr()
            .write(|w| unsafe { w.bits(RX_BUF_SIZE as u32) });

        //------------------------------------------------------------------
        // Enable DMA channel.
        //------------------------------------------------------------------

        dma.ch(0).cr().write(|w| {
            // Peripheral -> Memory
            w.dir().clear_bit();

            // Fixed peripheral address
            w.pinc().clear_bit();

            // Increment memory pointer
            w.minc().set_bit();

            // Normal mode
            w.circ().clear_bit();

            // Start reception
            w.en().set_bit();

            w
        });

        //------------------------------------------------------------------
        // Update driver state.
        //------------------------------------------------------------------

        self.rx_len = 0;
        self.rx_dma_active = true;
    }

    /// Stops DMA reception.
    ///
    /// This function is called after a complete Modbus RTU frame has been
    /// detected by the Receiver Timeout hardware.
    ///
    /// The received bytes remain stored inside `rx_buf`.
    fn stop_rx_dma(&mut self) {
        if !self.rx_dma_active {
            return;
        }

        let dma = unsafe { &*pac::DMA::ptr() };

        dma.ch(0).cr().modify(|_, w| w.en().clear_bit());

        self.rx_dma_active = false;
    }

    /// Returns the number of bytes received by DMA.
    ///
    /// DMA decrements NDTR after every transferred byte.
    ///
    /// Example
    ///
    /// ```text
    /// Buffer size : 256
    /// NDTR        : 249
    ///
    /// Received = 256 - 249 = 7 bytes
    /// ```
    ///
    /// This function should only be called after DMA reception has stopped.
    fn rx_length(&self) -> usize {
        let dma = unsafe { &*pac::DMA::ptr() };

        let remaining = dma.ch(0).ndtr().read().bits() as usize;

        RX_BUF_SIZE.saturating_sub(remaining)
    }

    //=========================================================================
    // TX DMA Control
    //=========================================================================
    //
    // Transmission Sequence
    //
    //          send_data()
    //               │
    //               ▼
    //        Copy frame to tx_buf
    //               │
    //               ▼
    //         start_tx_dma()
    //               │
    //               ▼
    //      DMA transfers bytes to USART
    //               │
    //               ▼
    //      USART shifts final stop bit
    //               │
    //               ▼
    //          TC flag becomes set
    //               │
    //               ▼
    //             poll()
    //               │
    //               ▼
    //      Disable DMA Channel 1
    //               │
    //               ▼
    //      Restart RX DMA
    //
    // Note
    // ----
    // DMA Transfer Complete (TCIF) only indicates that the final byte has
    // been written into the USART transmit register.
    //
    // The driver instead waits for the USART TC flag, which indicates that
    // the final stop bit has left the RS485 bus.
    //
    // This guarantees that the hardware Driver Enable (DEM) remains asserted
    // until transmission is physically complete.
    //=========================================================================

    /// Starts a DMA transmission.
    ///
    /// The transmit buffer (`tx_buf`) must already contain the complete frame.
    ///
    /// This function:
    ///
    /// - clears the USART Transmission Complete flag,
    /// - clears DMA Channel 1 status flags,
    /// - reloads the source buffer,
    /// - reloads the transfer count,
    /// - enables DMA Channel 1.
    ///
    /// Completion is detected later by `poll()` using the USART TC flag.
    fn start_tx_dma(&mut self) {
        let dma = unsafe { &*pac::DMA::ptr() };

        //------------------------------------------------------------------
        // Clear stale USART Transmission Complete flag.
        //------------------------------------------------------------------

        self.usart.icr().write(|w| w.tccf().bit(true));

        //------------------------------------------------------------------
        // Disable DMA channel before reconfiguration.
        //------------------------------------------------------------------

        dma.ch(1).cr().modify(|_, w| w.en().clear_bit());

        //------------------------------------------------------------------
        // Clear DMA Channel 1 status flags.
        //------------------------------------------------------------------

        dma.ifcr().write(|w| unsafe { w.bits(DMA_CH1_CLEAR_FLAGS) });

        //------------------------------------------------------------------
        // Peripheral address.
        //------------------------------------------------------------------

        dma.ch(1)
            .par()
            .write(|w| unsafe { w.bits(self.tdr_addr()) });

        //------------------------------------------------------------------
        // Source buffer.
        //------------------------------------------------------------------

        dma.ch(1)
            .mar()
            .write(|w| unsafe { w.bits(self.tx_buf.as_ptr() as u32) });

        //------------------------------------------------------------------
        // Number of bytes to transmit.
        //------------------------------------------------------------------

        dma.ch(1)
            .ndtr()
            .write(|w| unsafe { w.bits(self.tx_len as u32) });

        //------------------------------------------------------------------
        // Enable DMA Channel 1.
        //------------------------------------------------------------------

        dma.ch(1).cr().write(|w| {
            // Memory -> Peripheral
            w.dir().set_bit();

            // Fixed peripheral address
            w.pinc().clear_bit();

            // Increment memory pointer
            w.minc().set_bit();

            // Normal mode
            w.circ().clear_bit();

            // Begin transmission
            w.en().set_bit();

            w
        });
    }
}
