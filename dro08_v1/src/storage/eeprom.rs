//! ST24C02 EEPROM Driver (Free Functions Engine)
//! Target: STM32C031K6T6
//! Memory Strategy: 1 Parameter per 8-Byte Page

use stm32c0::stm32c031 as pac;

pub const EEPROM_ADDR: u8 = 0x50; // Standard 7-bit base address for 24C02
const PAGE_SIZE: usize = 8;
const TIMEOUT: u32 = 100_000;

// 100 kHz Standard Mode timing @ 48 MHz I2C kernel clock
pub const I2C_TIMING_100KHZ: u32 = 0x2030_3E5D;

// =========================================================================
// Memory Map (1 Page per Parameter)
// =========================================================================
pub const PAGE_SCALE_FACTOR: u8 = 0x00; // 5 Bytes (Page 0)
pub const PAGE_PRESET_COUNT: u8 = 0x08; // 4 Bytes (Page 1)
pub const PAGE_LIMIT1: u8       = 0x10; // 4 Bytes (Page 2)
pub const PAGE_LIMIT2: u8       = 0x18; // 4 Bytes (Page 3)
pub const PAGE_SCALED_VALUE: u8 = 0x20; // 4 Bytes (Page 4 - Power-Fail Reserved)
pub const PAGE_RELAYTIME: u8    = 0x28; // 1 Byte  (Page 5)
pub const PAGE_DECIMAL_POS: u8  = 0x30; // 1 Byte  (Page 6)
pub const PAGE_SLAVE_ADDR: u8   = 0x38; // 1 Byte  (Page 7)

/// Represents the 5-byte Scale Factor parameter
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScaleFactor {
    pub value: u32,  // 4-byte raw value
    pub dec_pos: u8, // 1-byte decimal point position
}

// =========================================================================
// Initialization Helper
// =========================================================================

/// Initialize I2C1 peripheral registers directly
pub fn init_i2c1(i2c: &pac::I2C1, rcc: &pac::RCC) {
    // 1. Force I2C1 kernel clock source to SYSCLK/PCLK (48 MHz)
    rcc.ccipr().modify(|_, w| unsafe { w.i2c1sel().bits(0) });

    // 2. Enable I2C1 APB clock
    rcc.apbenr1().modify(|_, w| w.i2c1en().set_bit());

    // 3. Delay for RCC clock domain synchronization
    let _ = rcc.apbenr1().read();

    // 4. Disable peripheral to write configuration registers
    i2c.cr1().modify(|_, w| w.pe().clear_bit());

    // 5. Load timing configuration for 100 kHz I2C bus speed
    i2c.timingr().write(|w| unsafe { w.bits(I2C_TIMING_100KHZ) });

    // 6. Enable peripheral
    i2c.cr1().modify(|_, w| w.pe().set_bit());
}

// =========================================================================
// High-Level Parameter API (Free Functions)
// =========================================================================

/// Write 5-byte Scale Factor (4-byte u32 value + 1-byte decimal position)
pub fn write_scale_factor(i2c: &pac::I2C1, sf: ScaleFactor) {
    let bytes = sf.value.to_le_bytes();
    let payload = [bytes[0], bytes[1], bytes[2], bytes[3], sf.dec_pos];
    write_page(i2c, PAGE_SCALE_FACTOR, &payload);
}

/// Read 5-byte Scale Factor
pub fn read_scale_factor(i2c: &pac::I2C1) -> ScaleFactor {
    let mut buf = [0u8; 5];
    read(i2c, PAGE_SCALE_FACTOR, &mut buf);
    ScaleFactor {
        value: u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]),
        dec_pos: buf[4],
    }
}

/// Write a 4-byte u32 parameter (preset_count, limit1, limit2)
pub fn write_u32(i2c: &pac::I2C1, page_addr: u8, value: u32) {
    write_page(i2c, page_addr, &value.to_le_bytes());
}

/// Read a 4-byte u32 parameter
pub fn read_u32(i2c: &pac::I2C1, page_addr: u8) -> u32 {
    let mut buf = [0u8; 4];
    read(i2c, page_addr, &mut buf);
    u32::from_le_bytes(buf)
}

/// Write a 1-byte u8 parameter (relaytime, decimal_pos, slave_addr)
pub fn write_u8(i2c: &pac::I2C1, page_addr: u8, value: u8) {
    write_page(i2c, page_addr, &[value]);
}

/// Read a 1-byte u8 parameter
pub fn read_u8(i2c: &pac::I2C1, page_addr: u8) -> u8 {
    let mut buf = [0u8; 1];
    read(i2c, page_addr, &mut buf);
    buf[0]
}

/// Emergency write for scaled_value during Power-Fail Interrupt
pub fn power_fail_save_scaled_value(i2c: &pac::I2C1, scaled_val: u32) {
    write_u32(i2c, PAGE_SCALED_VALUE, scaled_val);
}

// =========================================================================
// Low-Level Read / Write Engine
// =========================================================================

pub fn read(i2c: &pac::I2C1, mem_addr: u8, buf: &mut [u8]) {
    if buf.is_empty() {
        return;
    }

    wait_idle(i2c);

    // Phase 1: Set internal memory address pointer
    i2c.cr2().write(|w| unsafe {
        w.sadd()
            .bits((EEPROM_ADDR as u16) << 1)
            .nbytes()
            .bits(1)
            .rd_wrn()
            .clear_bit()
            .autoend()
            .clear_bit()
            .start()
            .set_bit()
    });

    if !wait_txis(i2c) {
        return;
    }
    i2c.txdr().write(|w| unsafe { w.bits(mem_addr as u32) });

    if !wait_tc(i2c) {
        return;
    }

    // Phase 2: Read N bytes back
    i2c.cr2().write(|w| unsafe {
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
        if !wait_rxne(i2c) {
            return;
        }
        *byte = i2c.rxdr().read().bits() as u8;
    }

    wait_stop(i2c);
    clear_stop(i2c);
}

pub fn write_page(i2c: &pac::I2C1, mem_addr: u8, slice: &[u8]) {
    if slice.is_empty() || slice.len() > PAGE_SIZE {
        return;
    }

    wait_idle(i2c);

    i2c.cr2().write(|w| unsafe {
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

    if !wait_txis(i2c) {
        return;
    }
    i2c.txdr().write(|w| unsafe { w.bits(mem_addr as u32) });

    for &byte in slice {
        if !wait_txis(i2c) {
            return;
        }
        i2c.txdr().write(|w| unsafe { w.bits(byte as u32) });
    }

    wait_stop(i2c);
    clear_stop(i2c);

    wait_write_cycle();
}

// =========================================================================
// Flag Polling Helpers
// =========================================================================

fn wait_idle(i2c: &pac::I2C1) {
    let mut timeout = TIMEOUT;
    while i2c.isr().read().busy().bit_is_set() {
        if timeout == 0 {
            return;
        }
        timeout -= 1;
    }
}

fn wait_txis(i2c: &pac::I2C1) -> bool {
    let mut timeout = TIMEOUT;
    loop {
        let isr = i2c.isr().read();
        if isr.txis().bit_is_set() {
            return true;
        }
        if isr.nackf().bit_is_set() {
            i2c.icr().write(|w| w.nackcf().bit(true));
            return false;
        }
        if timeout == 0 {
            return false;
        }
        timeout -= 1;
    }
}

fn wait_rxne(i2c: &pac::I2C1) -> bool {
    let mut timeout = TIMEOUT;
    loop {
        let isr = i2c.isr().read();
        if isr.rxne().bit_is_set() {
            return true;
        }
        if isr.nackf().bit_is_set() {
            i2c.icr().write(|w| w.nackcf().bit(true));
            return false;
        }
        if timeout == 0 {
            return false;
        }
        timeout -= 1;
    }
}

fn wait_tc(i2c: &pac::I2C1) -> bool {
    let mut timeout = TIMEOUT;
    loop {
        let isr = i2c.isr().read();
        if isr.tc().bit_is_set() {
            return true;
        }
        if isr.nackf().bit_is_set() {
            i2c.icr().write(|w| w.nackcf().bit(true));
            return false;
        }
        if timeout == 0 {
            return false;
        }
        timeout -= 1;
    }
}

fn wait_stop(i2c: &pac::I2C1) {
    let mut timeout = TIMEOUT;
    while i2c.isr().read().stopf().bit_is_clear() {
        if timeout == 0 {
            return;
        }
        timeout -= 1;
    }
}

fn clear_stop(i2c: &pac::I2C1) {
    i2c.icr().write(|w| w.stopcf().bit(true));
}

fn wait_write_cycle() {
    cortex_m::asm::delay(240_000);
}