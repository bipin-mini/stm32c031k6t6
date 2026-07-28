//! ST24C02 EEPROM Driver (Blocking / Polling Mode)
//! Target: STM32C031K6T6

use stm32c0::stm32c031 as pac;

const EEPROM_ADDR: u8 = 0x50; // Standard 7-bit base address for 24C02
const PAGE_SIZE: usize = 8;
const TIMEOUT: u32 = 100_000;

// 100 kHz Standard Mode timing @ 48 MHz I2C kernel clock
const I2C_TIMING_100KHZ: u32 = 0x2030_3E5D;

pub struct Eeprom {
    i2c: pac::I2C1,
}

impl Eeprom {
    /// Initialize I2C1 peripheral for EEPROM operations
    pub fn new(i2c: pac::I2C1, rcc: &pac::RCC) -> Self {
        // 1. Force I2C1 kernel clock source to SYSCLK/PCLK (48 MHz)
        rcc.ccipr().modify(|_, w| unsafe { w.i2c1sel().bits(0) });

        // 2. Enable I2C1 APB clock
        rcc.apbenr1().modify(|_, w| w.i2c1en().set_bit());

        // 3. Delay for RCC clock domain synchronization
        let _ = rcc.apbenr1().read();

        // 4. Disable peripheral to write configuration registers
        i2c.cr1().modify(|_, w| w.pe().clear_bit());

        // 5. Load timing configuration for 100 kHz I2C bus speed
        i2c.timingr()
            .write(|w| unsafe { w.bits(I2C_TIMING_100KHZ) });

        // 6. Enable peripheral
        i2c.cr1().modify(|_, w| w.pe().set_bit());

        Self { i2c }
    }

    /// Read a single byte from the given memory address
    pub fn read_byte(&mut self, mem_addr: u8) -> u8 {
        let mut buf = [0u8; 1];
        self.read(mem_addr, &mut buf);
        buf[0]
    }

    /// Read multiple bytes sequentially in one I2C transaction
    pub fn read(&mut self, mem_addr: u8, buf: &mut [u8]) {
        if buf.is_empty() {
            return;
        }

        self.wait_idle();

        // Phase 1: Set internal memory address pointer (Write mode, AUTOEND = 0)
        self.i2c.cr2().write(|w| unsafe {
            w.sadd()
                .bits((EEPROM_ADDR as u16) << 1) // Correct 7-bit address placement in SADD
                .nbytes()
                .bits(1)
                .rd_wrn()
                .clear_bit()
                .autoend()
                .clear_bit()
                .start()
                .set_bit()
        });

        if !self.wait_txis() {
            return;
        }
        self.i2c
            .txdr()
            .write(|w| unsafe { w.bits(mem_addr as u32) });

        // Wait for Transmit Complete before issuing Repeated START
        if !self.wait_tc() {
            return;
        }

        // Phase 2: Read N bytes back (Read mode, AUTOEND = 1)
        self.i2c.cr2().write(|w| unsafe {
            w.sadd()
                .bits((EEPROM_ADDR as u16) << 1)
                .nbytes()
                .bits(buf.len() as u8)
                .rd_wrn()
                .set_bit()
                .autoend()
                .set_bit()
                .start()
                .set_bit()
        });

        for byte in buf.iter_mut() {
            if !self.wait_rxne() {
                return;
            }
            *byte = self.i2c.rxdr().read().bits() as u8;
        }

        self.wait_stop();
        self.clear_stop();
    }

    /// Write a single byte to the given memory address
    pub fn write_byte(&mut self, mem_addr: u8, data: u8) {
        self.write(mem_addr, &[data]);
    }

    /// Write an arbitrary slice of data, handling page-boundary wraps automatically
    pub fn write(&mut self, mut mem_addr: u8, data: &[u8]) {
        let mut offset = 0;

        while offset < data.len() {
            let page_offset = (mem_addr as usize) % PAGE_SIZE;
            let bytes_left_in_page = PAGE_SIZE - page_offset;
            let chunk_size = bytes_left_in_page.min(data.len() - offset);

            self.write_page(mem_addr, &data[offset..offset + chunk_size]);

            mem_addr = mem_addr.wrapping_add(chunk_size as u8);
            offset += chunk_size;
        }
    }

    /// Internal helper: Write a single page (up to 8 bytes within one boundary)
    fn write_page(&mut self, mem_addr: u8, slice: &[u8]) {
        if slice.is_empty() || slice.len() > PAGE_SIZE {
            return;
        }

        self.wait_idle();

        self.i2c.cr2().write(|w| unsafe {
            w.sadd()
                .bits((EEPROM_ADDR as u16) << 1)
                .nbytes()
                .bits((slice.len() + 1) as u8)
                .rd_wrn()
                .clear_bit()
                .autoend()
                .set_bit()
                .start()
                .set_bit()
        });

        // Write address pointer
        if !self.wait_txis() {
            return;
        }
        self.i2c
            .txdr()
            .write(|w| unsafe { w.bits(mem_addr as u32) });

        // Write slice bytes
        for &byte in slice {
            if !self.wait_txis() {
                return;
            }
            self.i2c.txdr().write(|w| unsafe { w.bits(byte as u32) });
        }

        self.wait_stop();
        self.clear_stop();

        // Internal EEPROM write delay (~5 ms @ 48 MHz)
        self.wait_write_cycle();
    }

    // -------------------------------------------------------------------------
    // Non-Blocking / Timeout Flag Checks
    // -------------------------------------------------------------------------

    fn wait_idle(&self) {
        let mut timeout = TIMEOUT;
        while self.i2c.isr().read().busy().bit_is_set() {
            if timeout == 0 {
                return;
            }
            timeout -= 1;
        }
    }

    fn wait_txis(&self) -> bool {
        let mut timeout = TIMEOUT;
        loop {
            let isr = self.i2c.isr().read();
            if isr.txis().bit_is_set() {
                return true;
            }
            if isr.nackf().bit_is_set() {
                self.i2c.icr().write(|w| w.nackcf().bit(true));
                return false;
            }
            if timeout == 0 {
                return false;
            }
            timeout -= 1;
        }
    }

    fn wait_rxne(&self) -> bool {
        let mut timeout = TIMEOUT;
        loop {
            let isr = self.i2c.isr().read();
            if isr.rxne().bit_is_set() {
                return true;
            }
            if isr.nackf().bit_is_set() {
                self.i2c.icr().write(|w| w.nackcf().bit(true));
                return false;
            }
            if timeout == 0 {
                return false;
            }
            timeout -= 1;
        }
    }

    fn wait_tc(&self) -> bool {
        let mut timeout = TIMEOUT;
        loop {
            let isr = self.i2c.isr().read();
            if isr.tc().bit_is_set() {
                return true;
            }
            if isr.nackf().bit_is_set() {
                self.i2c.icr().write(|w| w.nackcf().bit(true));
                return false;
            }
            if timeout == 0 {
                return false;
            }
            timeout -= 1;
        }
    }

    fn wait_stop(&self) {
        let mut timeout = TIMEOUT;
        while self.i2c.isr().read().stopf().bit_is_clear() {
            if timeout == 0 {
                return;
            }
            timeout -= 1;
        }
    }

    fn clear_stop(&self) {
        self.i2c.icr().write(|w| w.stopcf().bit(true));
    }

    fn wait_write_cycle(&self) {
        cortex_m::asm::delay(240_000);
    }
}
