//! # UART DMA Driver Module
//!
//! ## Overview
//!
//! This module implements a DMA based UART driver for STM32C031 USART1
//! configured for Modbus RTU communication over RS485.
//!
//! The driver is designed for deterministic embedded operation:
//!
//! - No USART interrupts
//! - No DMA interrupts
//! - Periodic polling from RTIC task
//! - Hardware controlled RS485 DE signal
//! - DMA based receive and transmit
//! - USART receiver timeout (RTOR) used for Modbus frame detection
//!
//! ## Hardware Configuration
//!
//! USART1:
//!
//! ```text
//! USART1_RX  <---- RS485 transceiver RX
//! USART1_TX  ----> RS485 transceiver TX
//!
//! DE control:
//! USART hardware DEM controls RS485 direction pin
//! ```
//!
//! RS485 direction switching:
//!
//! ```text
//! RX mode:
//!
//!     USART DEM = LOW
//!     RX DMA enabled
//!
//!
//! TX mode:
//!
//!     USART DEM = HIGH
//!     TX DMA enabled
//!
//!
//! TX complete:
//!
//!     USART TC flag
//!          |
//!          v
//!     RX DMA restarted
//! ```
//!
//! ## DMA Allocation
//!
//! DMA channels:
//!
//! ```text
//! DMA Channel 0
//!
//! USART1_RDR
//!      |
//!      v
//! rx_buf[256]
//!
//!
//!
//! DMA Channel 1
//!
//! tx_buf[256]
//!      |
//!      v
//! USART1_TDR
//! ```
//!
//! ## Receive Operation
//!
//! RX DMA is continuously armed while the device is listening.
//!
//! Sequence:
//!
//! ```text
//! start_rx_dma()
//!
//!       |
//!       v
//!
//! USART receives bytes
//!
//!       |
//!       v
//!
//! DMA stores bytes into rx_buf[]
//!
//!       |
//!       v
//!
//! USART RTOR timeout expires
//!
//!       |
//!       v
//!
//! poll() detects RTOF
//!
//!       |
//!       v
//!
//! RX DMA stopped
//!
//!       |
//!       v
//!
//! rx_length() calculates received bytes
//!
//!       |
//!       v
//!
//! frame_available() = true
//! ```
//!
//! The RX buffer always starts from index zero for every Modbus frame.
//!
//! After Modbus processing:
//!
//! ```text
//! clear_rx()
//! ```
//!
//! restarts RX DMA operation.
//!
//! ## Transmit Operation
//!
//! The application fills the TX buffer obtained from:
//!
//! ```rust
//! uart.tx_buffer_mut()
//! ```
//!
//! Then starts transmission:
//!
//! ```rust
//! uart.start_tx(length);
//! ```
//!
//! Transmission sequence:
//!
//! ```text
//! start_tx()
//!
//!       |
//!       v
//!
//! RX DMA disabled
//!
//!       |
//!       v
//!
//! TX DMA configured
//!
//!       |
//!       v
//!
//! USART sends bytes
//!
//!       |
//!       v
//!
//! DMA complete
//!
//!       |
//!       v
//!
//! USART TC flag
//!
//!       |
//!       v
//!
//! RX DMA restarted
//! ```
//!
//! ## RTIC Integration
//!
//! The driver does not create interrupts.
//!
//! It should be polled periodically from an RTIC task.
//!
//! Example:
//!
//! ```rust
//! #[task(
//!     priority = 2,
//!     shared = [uart]
//! )]
//! fn uart_task(ctx: uart_task::Context) {
//!
//!     ctx.shared.uart.lock(|uart| {
//!
//!         uart.poll();
//!
//!         if uart.frame_available() {
//!
//!             let request = uart.rx_data();
//!
//!             let response_len =
//!                 modbus::process(
//!                     request,
//!                     uart.tx_buffer_mut()
//!                 );
//!
//!             uart.clear_rx();
//!
//!             uart.start_tx(response_len);
//!         }
//!     });
//! }
//! ```
//!
//! ## Modbus Layer Separation
//!
//! The UART driver does not contain Modbus protocol logic.
//!
//! Responsibilities:
//!
//! UART DMA driver:
//!
//! - Byte transport
//! - DMA management
//! - RS485 direction control
//! - Frame timing detection
//!
//!
//! Modbus layer:
//!
//! - Address validation
//! - Function decoding
//! - Register access
//! - CRC generation/checking
//! - Response generation
//!
//! The Modbus implementation can therefore remain stateless.
//!
//! ## Application Data Flow
//!
//! ```text
//!
//! Encoder
//!    |
//!    v
//! RTIC shared variables
//!    |
//!    v
//! Modbus request
//!    |
//!    v
//! UART RX DMA
//!    |
//!    v
//! Modbus processor
//!    |
//!    v
//! UART TX DMA
//!    |
//!    v
//! RS485 bus
//!
//! ```
//!
//! ## Design Notes
//!
//! - Buffer sizes are fixed to avoid dynamic allocation.
//! - No heap usage.
//! - No blocking delays during communication.
//! - Suitable for bare-metal `no_std` Rust.
//! - Suitable for RTIC based STM32C031 applications.
//!
//! ## Future Improvements
//!
//! Possible improvements:
//!
//! - Replace RX DMA stop/restart with circular DMA.
//! - Add DMA transfer complete flag checking.
//! - Add CRC acceleration if required.
//! - Add configurable baud rate.
//! - Add configurable Modbus timeout calculation based on baud rate.
//!

use stm32c0::stm32c031 as pac;

use crate::bsp::SYSCLK_HZ;

const BAUDRATE: u32 = 9600;

const RX_BUF_SIZE: usize = 256;
const TX_BUF_SIZE: usize = 256;

const RX_TIMEOUT: u16 = 40;

pub struct UartDma {
    usart: pac::USART1,

    rx_buf: [u8; RX_BUF_SIZE],
    rx_len: usize,
    rx_ready: bool,
    rx_dma_active: bool,

    tx_buf: [u8; TX_BUF_SIZE],
    tx_len: usize,
    tx_busy: bool,
}

impl UartDma {
    pub fn new(usart: pac::USART1, dma: &pac::DMA, dmamux: &pac::DMAMUX, rcc: &pac::RCC) -> Self {
        //
        // Enable USART clock
        //
        rcc.apbenr2().modify(|_, w| w.usart1en().set_bit());

        //
        // Enable DMA clock
        //
        rcc.ahbenr().modify(|_, w| w.dma1en().set_bit());

        //
        // Disable USART
        //
        usart.cr1().modify(|_, w| w.ue().clear_bit());

        //
        // Disable FIFO
        //
        usart.cr1().modify(|_, w| w.fifoen().clear_bit());

        //
        // Baud rate
        //
        usart
            .brr()
            .write(|w| unsafe { w.bits(SYSCLK_HZ / BAUDRATE) });

        //
        // Receiver timeout
        //
        usart
            .rtor()
            .write(|w| unsafe { w.rto().bits(RX_TIMEOUT as u32) });

        usart.cr2().modify(|_, w| w.rtoen().set_bit());

        //
        // RS485 DE control
        //
        usart
            .cr3()
            .modify(|_, w| w.dem().set_bit().dep().clear_bit());

        //
        // DMA enable
        //
        usart
            .cr3()
            .modify(|_, w| w.dmar().set_bit().dmat().set_bit());

        //
        // Enable RX/TX
        //
        usart.cr1().modify(|_, w| w.re().set_bit().te().set_bit());

        //
        // Enable USART
        //
        usart.cr1().modify(|_, w| w.ue().set_bit());

        while !usart.isr().read().teack().bit_is_set() {}

        while !usart.isr().read().reack().bit_is_set() {}

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

        uart.configure_dma(dma, dmamux);

        uart.start_rx_dma();

        uart
    }

    //
    // Call periodically from RTIC task.
    //
    pub fn poll(&mut self) {
        let isr = self.usart.isr().read();

        //
        // Modbus frame complete
        //
        if isr.rtof().bit_is_set() {
            self.usart.icr().write(|w| w.rtocf().bit(true));

            self.stop_rx_dma();

            self.rx_len = self.rx_length();

            if self.rx_len != 0 {
                self.rx_ready = true;
            }
        }

        //
        // TX finished
        //
        if self.tx_busy && isr.tc().bit_is_set() {
            self.usart.icr().write(|w| w.tccf().bit(true));

            let dma = unsafe { &*pac::DMA::ptr() };

            dma.ch(1).cr().modify(|_, w| w.en().clear_bit());

            self.tx_busy = false;

            self.start_rx_dma();
        }
    }

    pub fn frame_available(&self) -> bool {
        self.rx_ready
    }

    pub fn rx_data(&self) -> &[u8] {
        &self.rx_buf[..self.rx_len]
    }

    pub fn clear_rx(&mut self) {
        self.rx_ready = false;

        self.rx_len = 0;

        self.start_rx_dma();
    }

    pub fn tx_buffer_mut(&mut self) -> &mut [u8] {
        &mut self.tx_buf
    }

    pub fn start_tx(&mut self, len: usize) {
        let len = len.min(TX_BUF_SIZE);

        if len == 0 {
            return;
        }

        self.tx_len = len;
        self.tx_busy = true;

        self.stop_rx_dma();

        self.usart.icr().write(|w| w.tccf().bit(true));

        self.start_tx_dma();
    }

    pub fn tx_busy(&self) -> bool {
        self.tx_busy
    }

    //
    // DMA configuration
    //
    fn configure_dma(&mut self, dma: &pac::DMA, dmamux: &pac::DMAMUX) {
        //
        // DMA channel assignment:
        //
        // CH0 : USART1_RX
        // CH1 : USART1_TX
        //

        //
        // Disable DMA channels before configuration
        //
        dma.ch(0).cr().modify(|_, w| w.en().clear_bit());

        dma.ch(1).cr().modify(|_, w| w.en().clear_bit());

        //
        // Configure DMAMUX
        //
        // USART1_RX request = 40
        // USART1_TX request = 41
        //
        dmamux.ccr(0).write(|w| unsafe { w.dmareq_id().bits(40) });

        dmamux.ccr(1).write(|w| unsafe { w.dmareq_id().bits(41) });

        //
        // ============================
        // RX DMA CONFIGURATION
        // ============================
        //
        // USART1_RDR
        //      |
        //      v
        //   rx_buf[]
        //

        dma.ch(0)
            .par()
            .write(|w| unsafe { w.bits(pac::USART1::ptr() as u32 + 0x24) });

        dma.ch(0)
            .mar()
            .write(|w| unsafe { w.bits(self.rx_buf.as_mut_ptr() as u32) });

        dma.ch(0)
            .ndtr()
            .write(|w| unsafe { w.bits(RX_BUF_SIZE as u32) });

        //
        // Peripheral -> Memory
        //
        dma.ch(0).cr().write(|w| {
            w.dir().clear_bit();

            w.minc().set_bit();

            w.pinc().clear_bit();

            w.circ().clear_bit();

            w.en().clear_bit();

            w
        });

        //
        // ============================
        // TX DMA CONFIGURATION
        // ============================
        //
        // tx_buf[]
        //      |
        //      v
        // USART1_TDR
        //

        dma.ch(1)
            .par()
            .write(|w| unsafe { w.bits(pac::USART1::ptr() as u32 + 0x28) });

        dma.ch(1)
            .mar()
            .write(|w| unsafe { w.bits(self.tx_buf.as_ptr() as u32) });

        dma.ch(1).ndtr().write(|w| unsafe { w.bits(0) });

        //
        // Memory -> Peripheral
        //
        dma.ch(1).cr().write(|w| {
            w.dir().set_bit();

            w.minc().set_bit();

            w.pinc().clear_bit();

            w.circ().clear_bit();

            w.en().clear_bit();

            w
        });
    }

    fn start_rx_dma(&mut self) {
        let dma = unsafe { &*pac::DMA::ptr() };

        dma.ch(0).cr().modify(|_, w| w.en().clear_bit());

        dma.ifcr().write(|w| unsafe { w.bits(0x0F) });

        dma.ch(0)
            .par()
            .write(|w| unsafe { w.bits(pac::USART1::ptr() as u32 + 0x24) });

        dma.ch(0)
            .mar()
            .write(|w| unsafe { w.bits(self.rx_buf.as_mut_ptr() as u32) });

        dma.ch(0)
            .ndtr()
            .write(|w| unsafe { w.bits(RX_BUF_SIZE as u32) });

        dma.ch(0).cr().write(|w| {
            w.dir().clear_bit();

            w.minc().set_bit();

            w.pinc().clear_bit();

            w.circ().clear_bit();

            w.en().set_bit();

            w
        });

        self.rx_len = 0;

        self.rx_dma_active = true;
    }

    fn stop_rx_dma(&mut self) {
        if self.rx_dma_active {
            let dma = unsafe { &*pac::DMA::ptr() };

            //
            // Disable channel
            //
            dma.ch(0).cr().modify(|_, w| w.en().clear_bit());

            self.rx_dma_active = false;
        }
    }

    fn rx_length(&self) -> usize {
        let dma = unsafe { &*pac::DMA::ptr() };

        let remaining = dma.ch(0).ndtr().read().bits() as usize;

        RX_BUF_SIZE - remaining
    }
    fn start_tx_dma(&mut self) {
        let dma = unsafe { &*pac::DMA::ptr() };

        //
        // Disable TX DMA
        //
        dma.ch(1).cr().modify(|_, w| w.en().clear_bit());

        //
        // Clear channel flags
        //
        dma.ifcr().write(|w| unsafe { w.bits(0x0F << 4) });

        //
        // USART1 TDR
        //
        dma.ch(1)
            .par()
            .write(|w| unsafe { w.bits(pac::USART1::ptr() as u32 + 0x28) });

        //
        // TX buffer
        //
        dma.ch(1)
            .mar()
            .write(|w| unsafe { w.bits(self.tx_buf.as_ptr() as u32) });

        //
        // Length
        //
        dma.ch(1)
            .ndtr()
            .write(|w| unsafe { w.bits(self.tx_len as u32) });

        //
        // Memory -> USART
        //
        dma.ch(1).cr().write(|w| {
            w.dir().set_bit();

            w.minc().set_bit();

            w.pinc().clear_bit();

            w.circ().clear_bit();

            w.en().set_bit();

            w
        });
    }
}
