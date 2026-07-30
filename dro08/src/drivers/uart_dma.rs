use crate::bsp::SYSCLK_HZ;
use stm32c0::stm32c031 as pac;

const BAUDRATE: u32 = 9_600;
const RX_BUF_SIZE: usize = 256;
const TX_BUF_SIZE: usize = 256;
const RX_TIMEOUT_BITS: u16 = 40;

const USART1_RX_DMA_REQ: u8 = 50;
const USART1_TX_DMA_REQ: u8 = 51;

const DMA_CH0_CLEAR_FLAGS: u32 = 0x000F;
const DMA_CH1_CLEAR_FLAGS: u32 = 0x00F0;

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
        rcc.apbenr2().modify(|_, w| w.usart1en().set_bit());
        rcc.ahbenr().modify(|_, w| w.dma1en().set_bit());

        usart
            .cr1()
            .modify(|_, w| w.ue().clear_bit().fifoen().clear_bit());

        usart
            .brr()
            .write(|w| unsafe { w.bits(SYSCLK_HZ / BAUDRATE) });
        usart
            .rtor()
            .write(|w| unsafe { w.rto().bits(RX_TIMEOUT_BITS.into()) });
        usart.cr2().modify(|_, w| w.rtoen().set_bit());

        usart.cr3().modify(|_, w| {
            w.dem()
                .set_bit()
                .dep()
                .set_bit()
                .dmar()
                .set_bit()
                .dmat()
                .set_bit()
        });
        usart.icr().write(|w| w.tccf().bit(true));

        usart
            .cr1()
            .modify(|_, w| w.re().set_bit().te().set_bit().ue().set_bit());

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

    pub fn poll(&mut self) {
        let isr = self.usart.isr().read();

        // 1. Recover from Overrun
        if isr.ore().bit_is_set() {
            self.usart.icr().write(|w| w.orecf().bit(true));
            self.restart_rx();
            return;
        }

        // 2. Modbus Receiver Timeout Frame Boundary
        if isr.rtof().bit_is_set() {
            self.stop_rx_dma();
            self.usart.icr().write(|w| w.rtocf().bit(true));

            self.rx_len = self.rx_length();
            if self.rx_len > 0 {
                self.rx_ready = true;
            } else {
                self.start_rx_dma();
            }
        }

        // 3. Transmission Complete
        if self.tx_busy && isr.tc().bit_is_set() {
            self.usart.icr().write(|w| w.tccf().bit(true));

            let dma = unsafe { &*pac::DMA::ptr() };
            dma.ch(1).cr().modify(|_, w| w.en().clear_bit());

            self.tx_busy = false;
            self.start_rx_dma();
        }
    }

    pub fn receive_data(&mut self, dst: &mut [u8]) -> Option<usize> {
        if !self.rx_ready {
            return None;
        }

        let len = self.rx_len.min(dst.len());
        dst[..len].copy_from_slice(&self.rx_buf[..len]);

        self.rx_ready = false;
        self.rx_len = 0;

        self.start_rx_dma();
        Some(len)
    }

    pub fn send_data(&mut self, data: &[u8]) -> Result<(), ()> {
        if self.tx_busy || data.is_empty() || data.len() > TX_BUF_SIZE {
            return Err(());
        }

        self.tx_buf[..data.len()].copy_from_slice(data);
        self.tx_len = data.len();

        self.rx_ready = false;
        self.rx_len = 0;

        self.stop_rx_dma();
        self.tx_busy = true;

        self.start_tx_dma();
        Ok(())
    }

    pub fn restart_rx(&mut self) {
        self.stop_rx_dma();
        self.rx_ready = false;
        self.rx_len = 0;
        self.start_rx_dma();
    }

    #[inline(always)]
    pub fn tx_busy(&self) -> bool {
        self.tx_busy
    }

    #[inline(always)]
    fn rdr_addr(&self) -> u32 {
        self.usart.rdr().as_ptr() as u32
    }

    #[inline(always)]
    fn tdr_addr(&self) -> u32 {
        self.usart.tdr().as_ptr() as u32
    }

    fn configure_dma(&mut self, dma: &pac::DMA, dmamux: &pac::DMAMUX) {
        dma.ch(0).cr().modify(|_, w| w.en().clear_bit());
        dma.ch(1).cr().modify(|_, w| w.en().clear_bit());

        dmamux
            .ccr(0)
            .write(|w| unsafe { w.dmareq_id().bits(USART1_RX_DMA_REQ) });
        dmamux
            .ccr(1)
            .write(|w| unsafe { w.dmareq_id().bits(USART1_TX_DMA_REQ) });

        // RX DMA Configuration (CH0)
        dma.ch(0)
            .par()
            .write(|w| unsafe { w.bits(self.rdr_addr()) });
        dma.ch(0)
            .mar()
            .write(|w| unsafe { w.bits(self.rx_buf.as_mut_ptr() as u32) });
        dma.ch(0).cr().write(|w| {
            w.dir()
                .clear_bit()
                .pinc()
                .clear_bit()
                .minc()
                .set_bit()
                .circ()
                .clear_bit()
                .en()
                .clear_bit()
        });

        // TX DMA Configuration (CH1)
        dma.ch(1)
            .par()
            .write(|w| unsafe { w.bits(self.tdr_addr()) });
        dma.ch(1)
            .mar()
            .write(|w| unsafe { w.bits(self.tx_buf.as_ptr() as u32) });
        dma.ch(1).cr().write(|w| {
            w.dir()
                .set_bit()
                .pinc()
                .clear_bit()
                .minc()
                .set_bit()
                .circ()
                .clear_bit()
                .en()
                .clear_bit()
        });
    }

    fn start_rx_dma(&mut self) {
        let dma = unsafe { &*pac::DMA::ptr() };

        self.usart.icr().write(|w| w.orecf().bit(true));

        dma.ch(0).cr().modify(|_, w| w.en().clear_bit());
        dma.ifcr().write(|w| unsafe { w.bits(DMA_CH0_CLEAR_FLAGS) });

        dma.ch(0)
            .mar()
            .write(|w| unsafe { w.bits(self.rx_buf.as_mut_ptr() as u32) });
        dma.ch(0)
            .ndtr()
            .write(|w| unsafe { w.bits(RX_BUF_SIZE as u32) });

        dma.ch(0).cr().modify(|_, w| w.en().set_bit());

        self.rx_len = 0;
        self.rx_dma_active = true;
    }

    fn stop_rx_dma(&mut self) {
        if !self.rx_dma_active {
            return;
        }

        let dma = unsafe { &*pac::DMA::ptr() };
        dma.ch(0).cr().modify(|_, w| w.en().clear_bit());
        self.rx_dma_active = false;
    }

    fn rx_length(&self) -> usize {
        let dma = unsafe { &*pac::DMA::ptr() };
        let remaining = dma.ch(0).ndtr().read().bits() as usize;
        RX_BUF_SIZE.saturating_sub(remaining)
    }

    fn start_tx_dma(&mut self) {
        let dma = unsafe { &*pac::DMA::ptr() };

        self.usart.icr().write(|w| w.tccf().bit(true));

        dma.ch(1).cr().modify(|_, w| w.en().clear_bit());
        dma.ifcr().write(|w| unsafe { w.bits(DMA_CH1_CLEAR_FLAGS) });

        dma.ch(1)
            .mar()
            .write(|w| unsafe { w.bits(self.tx_buf.as_ptr() as u32) });
        dma.ch(1)
            .ndtr()
            .write(|w| unsafe { w.bits(self.tx_len as u32) });

        dma.ch(1).cr().modify(|_, w| w.en().set_bit());
    }
}
