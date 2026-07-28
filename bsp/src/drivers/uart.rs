use stm32c0::stm32c031 as pac;

use crate::bsp::SYSCLK_HZ;

const BAUDRATE: u32 = 115_200;
const RX_TIMEOUT: u16 = 300;
const DE_ASSERT_TIME: u8 = 3;
const DE_DEASSERT_TIME: u8 = 3;

const TX_BUF_SIZE: usize = 256;
const RX_BUF_SIZE: usize = 256;

pub struct Uart {
    usart: pac::USART1,

    tx_buf: [u8; TX_BUF_SIZE],
    tx_len: usize,
    tx_idx: usize,
    tx_busy: bool,

    rx_buf: [u8; RX_BUF_SIZE],
    rx_len: usize,
}

impl Uart {
    pub fn new(usart: pac::USART1, rcc: &pac::RCC) -> Self {
        rcc.apbenr2().modify(|_, w| w.usart1en().set_bit());

        usart.cr1().modify(|_, w| w.ue().clear_bit());

        // Disable FIFO.
        usart.cr1().modify(|_, w| w.fifoen().clear_bit());

        // Configure baud rate.
        usart
            .brr()
            .write(|w| unsafe { w.bits(SYSCLK_HZ / BAUDRATE) });

        // Configure receiver timeout.
        usart
            .rtor()
            .write(|w| unsafe { w.rto().bits(RX_TIMEOUT.into()) });

        usart.cr2().modify(|_, w| w.rtoen().set_bit());

        // Enable RS485 driver control.
        usart.cr3().modify(|_, w| w.dem().set_bit().dep().set_bit());

        // Configure DE timing.
        usart
            .cr1()
            .modify(|_, w| unsafe { w.deat().bits(DE_ASSERT_TIME).dedt().bits(DE_DEASSERT_TIME) });

        // Enable transmitter and receiver.
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

        usart.cr1().modify(|_, w| w.ue().set_bit());

        while !usart.isr().read().teack().bit_is_set() {}
        while !usart.isr().read().reack().bit_is_set() {}

        Self {
            usart,

            tx_buf: [0; TX_BUF_SIZE],
            tx_len: 0,
            tx_idx: 0,
            tx_busy: false,

            rx_buf: [0; RX_BUF_SIZE],
            rx_len: 0,
        }
    }

    #[inline(always)]
    fn write_tdr(&self, b: u8) {
        self.usart.tdr().write(|w| unsafe { w.bits(b as u32) });
    }

    #[inline(always)]
    fn drain_rx_fifo(&mut self) {
        while self.usart.isr().read().rxfne().bit_is_set() {
            let b = self.usart.rdr().read().bits() as u8;

            if self.rx_len < RX_BUF_SIZE {
                self.rx_buf[self.rx_len] = b;
                self.rx_len += 1;
            }
        }
    }

    #[inline(always)]
    fn fill_tx_fifo(&mut self) {
        while self.usart.isr().read().txfnf().bit_is_set()
            && self.tx_busy
            && self.tx_idx < self.tx_len
        {
            let b = self.tx_buf[self.tx_idx];
            self.tx_idx += 1;
            self.write_tdr(b);
        }
    }

    pub fn isr(&mut self) {
        // Receive pending bytes.
        self.drain_rx_fifo();

        let isr = self.usart.isr().read();

        // Handle end of frame.
        if isr.rtof().bit_is_set() {
            self.drain_rx_fifo();

            self.usart.icr().write(|w| w.rtocf().bit(true));

            if !self.tx_busy && self.rx_len > 0 {
                self.tx_len = self.rx_len.min(TX_BUF_SIZE);
                self.tx_idx = 0;
                self.tx_busy = true;

                self.tx_buf[..self.tx_len].copy_from_slice(&self.rx_buf[..self.tx_len]);

                self.fill_tx_fifo();

                self.usart.cr1().modify(|_, w| w.txeie().set_bit());

                // Packet accepted.
                self.rx_len = 0;
            }
        }

        // Continue transmission.
        self.fill_tx_fifo();

        // Switch to transmission complete interrupt.
        if self.tx_busy
            && self.tx_idx >= self.tx_len
            && self.usart.cr1().read().tcie().bit_is_clear()
        {
            self.usart
                .cr1()
                .modify(|_, w| w.txeie().clear_bit().tcie().set_bit());
        }

        // Transmission complete.
        if self.tx_busy && self.usart.isr().read().tc().bit_is_set() {
            self.usart.icr().write(|w| w.tccf().bit(true));

            self.usart.cr1().modify(|_, w| w.tcie().clear_bit());

            self.tx_busy = false;
        }

        // Clear overrun error.
        if isr.ore().bit_is_set() {
            self.usart.icr().write(|w| w.orecf().bit(true));
        }

        // Clear framing error.
        if isr.fe().bit_is_set() {
            self.usart.icr().write(|w| w.fecf().bit(true));
        }

        // Clear noise error.
        if isr.ne().bit_is_set() {
            self.usart.icr().write(|w| w.necf().bit(true));
        }
    }
}
