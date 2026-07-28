use stm32c0::stm32c031 as pac;

use crate::bsp::SYSCLK_HZ;

const BAUDRATE: u32 = 9600;

const RX_BUF_SIZE: usize = 256;
const TX_BUF_SIZE: usize = 256;

// Receiver timeout in bit times (40 bit times = ~3.5 to 4 characters for Modbus RTU)
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
        // Enable USART1 and DMA clocks
        rcc.apbenr2().modify(|_, w| w.usart1en().set_bit());
        rcc.ahbenr().modify(|_, w| w.dma1en().set_bit());

        // Disable USART during configuration
        usart.cr1().modify(|_, w| w.ue().clear_bit());
        usart.cr1().modify(|_, w| w.fifoen().clear_bit());

        // Baud rate
        usart
            .brr()
            .write(|w| unsafe { w.bits(SYSCLK_HZ / BAUDRATE) });

        // Receiver timeout (RTOR) setup for Modbus frame boundary detection
        usart
            .rtor()
            .write(|w| unsafe { w.rto().bits(RX_TIMEOUT as u32) });
        usart.cr2().modify(|_, w| w.rtoen().set_bit());

        // RS485 Hardware DE setup (Hardware DEM active high)
        usart
            .cr3()
            .modify(|_, w| w.dem().set_bit().dep().set_bit());

        // Enable USART DMA RX/TX requests
        usart
            .cr3()
            .modify(|_, w| w.dmar().set_bit().dmat().set_bit());

        // Enable RX and TX
        usart.cr1().modify(|_, w| w.re().set_bit().te().set_bit());

        // Enable USART peripheral
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

    /// Primary state machine processing — call periodically inside an RTIC task.
    pub fn poll(&mut self) {
        let isr = self.usart.isr().read();

        // 1. Recover from hardware Overrun Errors if line noise occurs
        if isr.ore().bit_is_set() {
            self.usart.icr().write(|w| w.orecf().bit(true));
            self.restart_rx();
            return;
        }

        // 2. Modbus frame complete (RTOF flag set by RTOR hardware)
        if isr.rtof().bit_is_set() {
            self.stop_rx_dma();
            self.usart.icr().write(|w| w.rtocf().bit(true));

            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
            self.rx_len = self.rx_length();

            if self.rx_len > 0 {
                self.rx_ready = true;
            } else {
                self.start_rx_dma();
            }
        }

        // 3. Transmission finished (TC flag set after final stop bit)
        if self.tx_busy && isr.tc().bit_is_set() {
            self.usart.icr().write(|w| w.tccf().bit(true));

            let dma = unsafe { &*pac::DMA::ptr() };
            dma.ch(1).cr().modify(|_, w| w.en().clear_bit());

            self.tx_busy = false;

            // Automatically revert to listening mode on RS485 bus
            self.start_rx_dma();
        }
    }

    /// Returns `Some(&[u8])` with the received frame if a complete payload is available.
    pub fn receive_data(&self) -> Option<&[u8]> {
        if self.rx_ready {
            Some(&self.rx_buf[..self.rx_len])
        } else {
            None
        }
    }

    /// Transmits a payload slice via DMA and toggles RS485 direction pin (DEM) high.
    /// Re-arms RX DMA automatically once transmission finishes in `poll()`.
    pub fn send_data(&mut self, data: &[u8]) -> Result<(), ()> {
        if self.tx_busy || data.is_empty() {
            return Err(());
        }

        let send_len = data.len().min(TX_BUF_SIZE);
        self.tx_buf[..send_len].copy_from_slice(&data[..send_len]);
        self.tx_len = send_len;

        // Clear RX state after validation
        self.rx_ready = false;
        self.rx_len = 0;

        self.stop_rx_dma();

        // Clear Transmission Complete flag before starting DMA
        self.usart.icr().write(|w| w.tccf().bit(true));

        self.tx_busy = true;
        self.start_tx_dma();

        Ok(())
    }

    /// Flushes current RX packet and re-arms DMA for the next incoming request.
    pub fn restart_rx(&mut self) {
        self.rx_ready = false;
        self.rx_len = 0;
        self.start_rx_dma();
    }

    /// Returns whether a DMA transmission is currently in progress.
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

    // --- Private DMA Control Functions ---

    fn configure_dma(&mut self, dma: &pac::DMA, dmamux: &pac::DMAMUX) {
        dma.ch(0).cr().modify(|_, w| w.en().clear_bit());
        dma.ch(1).cr().modify(|_, w| w.en().clear_bit());

        // CH0 -> USART1_RX (Req 40), CH1 -> USART1_TX (Req 41)
        dmamux.ccr(0).write(|w| unsafe { w.dmareq_id().bits(40) });
        dmamux.ccr(1).write(|w| unsafe { w.dmareq_id().bits(41) });

        // RX DMA Setup (CH0)
        dma.ch(0)
            .par()
            .write(|w| unsafe { w.bits(self.rdr_addr()) });
        dma.ch(0)
            .mar()
            .write(|w| unsafe { w.bits(self.rx_buf.as_mut_ptr() as u32) });
        dma.ch(0)
            .ndtr()
            .write(|w| unsafe { w.bits(RX_BUF_SIZE as u32) });
        dma.ch(0).cr().write(|w| {
            w.dir().clear_bit(); // Peripheral -> Memory
            w.minc().set_bit(); // Increment memory pointer
            w.pinc().clear_bit();
            w.circ().clear_bit();
            w.en().clear_bit();
            w
        });

        // TX DMA Setup (CH1)
        dma.ch(1)
            .par()
            .write(|w| unsafe { w.bits(self.tdr_addr()) });
        dma.ch(1)
            .mar()
            .write(|w| unsafe { w.bits(self.tx_buf.as_ptr() as u32) });
        dma.ch(1).ndtr().write(|w| unsafe { w.bits(0) });
        dma.ch(1).cr().write(|w| {
            w.dir().set_bit(); // Memory -> Peripheral
            w.minc().set_bit(); // Increment memory pointer
            w.pinc().clear_bit();
            w.circ().clear_bit();
            w.en().clear_bit();
            w
        });
    }

    fn start_rx_dma(&mut self) {
        let dma = unsafe { &*pac::DMA::ptr() };

        // Clear USART overrun errors to keep receiver active
        self.usart.icr().write(|w| w.orecf().bit(true));

        dma.ch(0).cr().modify(|_, w| w.en().clear_bit());
        dma.ifcr().write(|w| unsafe { w.bits(0x0F) }); // Clear CH0 flags

        dma.ch(0)
            .par()
            .write(|w| unsafe { w.bits(self.rdr_addr()) });

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
            dma.ch(0).cr().modify(|_, w| w.en().clear_bit());
            self.rx_dma_active = false;
        }
    }

    fn rx_length(&self) -> usize {
        let dma = unsafe { &*pac::DMA::ptr() };
        let remaining = dma.ch(0).ndtr().read().bits() as usize;
        RX_BUF_SIZE.saturating_sub(remaining)
    }

    fn start_tx_dma(&mut self) {
        let dma = unsafe { &*pac::DMA::ptr() };

        dma.ch(1).cr().modify(|_, w| w.en().clear_bit());
        dma.ifcr().write(|w| unsafe { w.bits(0x0F << 4) }); // Clear CH1 flags

        dma.ch(1)
            .par()
            .write(|w| unsafe { w.bits(self.tdr_addr()) });
        dma.ch(1)
            .mar()
            .write(|w| unsafe { w.bits(self.tx_buf.as_ptr() as u32) });
        dma.ch(1)
            .ndtr()
            .write(|w| unsafe { w.bits(self.tx_len as u32) });

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
