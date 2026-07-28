use stm32c0::stm32c031 as pac;

const TX_BUF_SIZE: usize = 256;

/// UART driver for Modbus RTU over RS485
///
/// Target:
/// - MCU: STM32C031
/// - CPU Clock: **48 MHz (HSI)**
/// - Baud: **9600, 8N1**
///
/// Design goals:
/// - Deterministic ISR execution
/// - Zero allocation
/// - Non-blocking TX (interrupt driven)
/// - Hardware-assisted frame detection (RTO)
/// - RS485 DE handled by hardware (DEM)
///
/// ---------------------------------------------------------------------------
/// Timing Reference @ 48 MHz / 9600 baud:
///
/// Bit time      = 1 / 9600  ≈ 104.166 µs
/// Char time     = 10 bits   ≈ 1.041 ms (8N1)
/// Modbus gap    = 3.5 chars ≈ 3.645 ms
///
/// RTO is configured in *bit times* (NOT CPU cycles)
/// → independent of CPU frequency once baud is configured.
///
/// Transmission is interrupt-driven:
///
/// 1. TX FIFO interrupt feeds bytes into TDR.
/// 2. After the final byte is written, TX FIFO interrupt is disabled.
/// 3. TC interrupt is then enabled to detect when the shift register
///    becomes empty (true end of UART transmission).
/// 4. TC interrupt is disabled immediately after completion.
///
/// `Event::TxDone` is generated after the final stop bit has been
/// transmitted. RS485 DE release then follows the programmed DEDT delay.
///
/// ---------------------------------------------------------------------------
pub struct Uart {
    pub usart: pac::USART1,

    // ---------------- TX STATE ----------------
    /// Transmit buffer (ISR drained)
    tx_buf: [u8; TX_BUF_SIZE],

    /// Total bytes to send
    tx_len: usize,

    /// Current index (next byte to send)
    tx_idx: usize,

    /// TX active flag
    tx_busy: bool,
}

// ---------------------------------------------------------------------------
// ISR EVENT MODEL
// ---------------------------------------------------------------------------

/// Events emitted from UART ISR.
///
/// Single-event model avoids borrow conflicts in RTIC.
pub enum Event {
    /// Byte received.
    Rx(u8),

    /// End of Modbus frame detected (Receiver Timeout).
    FrameEnd,

    /// Entire UART transmission finished.
    ///
    /// This event is generated from the USART TC interrupt,
    /// meaning:
    ///
    /// - last byte has left the shift register
    /// - final stop bit has been transmitted
    /// - RS485 DE release delay begins after this point
    ///
    /// This is the correct point for the UART peripheral to
    /// consider transmission complete.
    TxDone,
}

impl Uart {
    /// Initialize USART1 for Modbus RTU.
    ///
    /// # Arguments
    ///
    /// * `usart`    - USART1 peripheral
    /// * `rcc`      - RCC peripheral
    /// * `slave_id` - Modbus slave address
    pub fn new(usart: pac::USART1, rcc: &pac::RCC, slave_id: u8) -> Self {
        // -------------------------------------------------------------------
        // Enable peripheral clock
        // -------------------------------------------------------------------
        rcc.apbenr2().modify(|_, w| w.usart1en().set_bit());

        // Disable USART before configuration.
        usart.cr1().modify(|_, w| w.ue().clear_bit());

        // -------------------------------------------------------------------
        // Enable USART FIFO mode.
        //
        // TXFNF and RXFNE flags used by the interrupt path
        // operate on FIFO status.
        // -------------------------------------------------------------------
        usart.cr1().modify(|_, w| w.fifoen().set_bit());

        // -------------------------------------------------------------------
        // Baud rate
        //
        // BRR = Fclk / Baud
        //
        // 48 MHz / 9600 = 5000
        //
        // Exact division
        // Baud error = 0%
        // -------------------------------------------------------------------
        usart.brr().write(|w| unsafe { w.bits(48_000_000 / 9_600) });

        // -------------------------------------------------------------------
        // Receiver Timeout (RTO)
        //
        // Used for Modbus RTU frame detection.
        //
        // Value is specified in bit periods.
        //
        // 35 bits ≈ 3.5 character times
        // -------------------------------------------------------------------
        usart.rtor().write(|w| unsafe { w.rto().bits(35) });

        usart.cr2().modify(|_, w| w.rtoen().set_bit());

        // -------------------------------------------------------------------
        // RS485 Driver Enable
        //
        // DEM = automatic DE control
        // DEP = active LOW
        //
        // DEAT = 3 bit times
        // DEDT = 3 bit times
        // -------------------------------------------------------------------
        usart
            .cr3()
            .modify(|_, w| w.dem().set_bit().dep().clear_bit());

        usart
            .cr1()
            .modify(|_, w| unsafe { w.deat().bits(3).dedt().bits(3) });

        // -------------------------------------------------------------------
        // Address register
        //
        // Stored for future multiprocessor support.
        //
        // MME remains disabled.
        // -------------------------------------------------------------------
        usart.cr2().modify(|_, w| unsafe { w.add().bits(slave_id) });

        usart.cr1().modify(|_, w| w.mme().clear_bit());

        // -------------------------------------------------------------------
        // Enable transmitter and receiver.
        //
        // Interrupts enabled:
        //
        // RXNEIE
        //      Receive byte interrupt
        //
        // RTOIE
        //      Receiver timeout interrupt
        //
        // NOTE:
        //
        // TCIE is intentionally NOT enabled here.
        //
        // The Transmission Complete interrupt is enabled only
        // after the final byte has been loaded into the transmitter.
        //
        // This prevents continuous TC interrupts while USART
        // is idle.
        // -------------------------------------------------------------------
        usart.cr1().modify(|_, w| {
            w.re()
                .set_bit()
                .te()
                .set_bit()
                .rxneie()
                .set_bit()
                .rtoie()
                .set_bit()
        });

        // -------------------------------------------------------------------
        // Enable USART
        // -------------------------------------------------------------------
        usart.cr1().modify(|_, w| w.ue().set_bit());

        assert!(usart.cr1().read().ue().bit_is_set());

        while !usart.isr().read().teack().bit_is_set() {}
        while !usart.isr().read().reack().bit_is_set() {}

        Self {
            usart,
            tx_buf: [0; TX_BUF_SIZE],
            tx_len: 0,
            tx_idx: 0,
            tx_busy: false,
        }
    }

    // -----------------------------------------------------------------------
    // Start TX (NON-BLOCKING)
    // -----------------------------------------------------------------------

    /// Start transmission of a Modbus frame.
    ///
    /// - Non-blocking.
    /// - Copies the frame into the internal TX buffer.
    /// - ISR performs the remainder of transmission.
    ///
    /// If another transmission is already active,
    /// the new frame is silently dropped.
    pub fn start_tx(&mut self, data: &[u8]) {
        if data.is_empty() {
            return;
        }

        if self.tx_busy {
            return;
        }

        let len = data.len().min(TX_BUF_SIZE);

        self.tx_buf[..len].copy_from_slice(&data[..len]);

        self.tx_len = len;
        self.tx_idx = 0;
        self.tx_busy = true;

        // Kick-start transmission immediately if FIFO has space.

        if self.usart.isr().read().txfnf().bit_is_set() {
            let b = self.tx_buf[self.tx_idx];

            self.tx_idx += 1;

            self.usart.tdr().write(|w| unsafe { w.bits(b as u32) });
        }

        // Enable TX FIFO Not Full interrupt.
        //
        // TXFNF indicates that space is available for another byte.
        //
        // TCIE remains disabled until the final byte has
        // been loaded into the transmitter.
        self.usart.cr1().modify(|_, w| w.txeie().set_bit());
    }

    // -----------------------------------------------------------------------
    // Runtime Modbus address update
    // -----------------------------------------------------------------------

    pub fn set_slave_id(&self, id: u8) {
        self.usart.cr2().modify(|_, w| unsafe { w.add().bits(id) });
    }

    // -----------------------------------------------------------------------
    // ISR Handler
    // -----------------------------------------------------------------------

    /// UART ISR handler
    ///
    /// Emits events:
    ///
    /// - Rx(byte)
    /// - FrameEnd (Receiver Timeout)
    /// - TxDone (Transmission Complete)
    ///
    /// Design goals:
    ///
    /// - Constant execution time
    /// - Minimal register accesses
    /// - No allocation
    /// - No application-specific logic
    pub fn isr<F>(&mut self, mut f: F)
    where
        F: FnMut(Event),
    {
        let isr = self.usart.isr().read();

        // ---------------------------------------------------------------
        // RX
        // ---------------------------------------------------------------
        if isr.rxfne().bit_is_set() {
            let b = self.usart.rdr().read().bits() as u8;

            f(Event::Rx(b));
        }

        // ---------------------------------------------------------------
        // Receiver Timeout (Modbus frame end)
        // ---------------------------------------------------------------
        if isr.rtof().bit_is_set() {
            self.usart.icr().write(|w| w.rtocf().bit(true));

            f(Event::FrameEnd);
        }

        // ---------------------------------------------------------------
        // TX FIFO Not Full
        //
        // Feed the transmitter until the final byte has been loaded into
        // TDR.
        //
        // Once all bytes have been queued:
        //
        // - TX FIFO interrupt is disabled
        // - TC interrupt is enabled
        //
        // The following TC interrupt indicates that the final
        // stop bit has left the shift register.
        // ---------------------------------------------------------------
        if isr.txfnf().bit_is_set() && self.tx_busy {
            if self.tx_idx < self.tx_len {
                let b = self.tx_buf[self.tx_idx];

                self.tx_idx += 1;

                self.usart.tdr().write(|w| unsafe { w.bits(b as u32) });
            } else {
                // All bytes have been loaded.
                //
                // Stop TX FIFO interrupts and wait for the true
                // end of transmission using TC.

                self.usart
                    .cr1()
                    .modify(|_, w| w.txeie().clear_bit().tcie().set_bit());
            }
        }

        // ---------------------------------------------------------------
        // Transmission Complete
        //
        // TC is asserted only after:
        //
        // - TDR is empty
        // - Shift register is empty
        // - Final stop bit has been transmitted
        //
        // RS485 DE remains asserted for the programmed DEDT
        // interval before automatically returning to receive mode.
        // ---------------------------------------------------------------
        if self.tx_busy && isr.tc().bit_is_set() {
            self.usart.icr().write(|w| w.tccf().bit(true));

            // Disable TC interrupt until the next transmission.
            self.usart.cr1().modify(|_, w| w.tcie().clear_bit());

            self.tx_busy = false;

            f(Event::TxDone);
        }

        // ---------------------------------------------------------------
        // Error handling
        // ---------------------------------------------------------------
        if isr.ore().bit_is_set() {
            self.usart.icr().write(|w| w.orecf().bit(true));
        }

        if isr.fe().bit_is_set() {
            self.usart.icr().write(|w| w.fecf().bit(true));
        }

        if isr.ne().bit_is_set() {
            self.usart.icr().write(|w| w.necf().bit(true));
        }
    }

    /// ------------------------------------------------------------------------
    /// UART transmission test
    /// ------------------------------------------------------------------------
    ///
    /// Sends:
    ///
    /// Hello\r\n
    ///
    /// Uses direct polling transmission for basic UART verification.
    pub fn send_test(&mut self) {
        for &b in b"Hello\r\n" {
            while self.usart.isr().read().txfnf().bit_is_clear() {}

            self.usart.tdr().write(|w| unsafe { w.bits(b as u32) });
        }

        while self.usart.isr().read().tc().bit_is_clear() {}
    }
}
