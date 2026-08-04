#![allow(dead_code)]

// Verified TM1638 RAM mapping:
//
// Even addresses:
// data[0]  -> RAM0  Unused
// data[2]  -> RAM1  Sign (segment G)
// data[4]  -> RAM2  Leftmost digit
// data[6]  -> RAM3
// data[8]  -> RAM4
// data[10] -> RAM5
// data[12] -> RAM6
// data[14] -> RAM7  Rightmost digit
//
// Odd addresses:
// data[3]  -> LED1
// data[5]  -> LED2
// data[7]  -> LED3
// data[9]  -> LED4
// data[11] -> LED5
// data[13] -> LED6
// data[15] -> LED7

// Button bit mapping:
// Key 1: 0x0010_0000
// Key 2: 0x0001_0000
// Key 3: 0x0000_1000
// Key 4: 0x0000_0100
// Key 5: 0x0000_0010
// Key 6: 0x0000_0001

use stm32c0::stm32c031 as pac;

// GPIOA pin masks.
const STB: u32 = 1 << 4;
const CLK: u32 = 1 << 5;
const DIO: u32 = 1 << 7;

// TM1638 commands.
const CMD_DATA_AUTO_INC: u8 = 0x40;
const CMD_DATA_READ: u8 = 0x42;
const CMD_ADDR: u8 = 0xC0;
const CMD_DISPLAY_OFF: u8 = 0x80;
const CMD_DISPLAY_ON: u8 = 0x88;

// GPIO register block.
type GpioaRb = pac::gpioa::RegisterBlock;

// TM1638 driver.
pub struct Tm1638;

// Return GPIOA registers.
fn gpio() -> &'static GpioaRb {
    unsafe { &*pac::GPIOA::ptr() }
}

// Drive STB high.
fn stb_high() {
    gpio().bsrr().write(|w| w.bs4().set_bit());
}

// Drive STB low.
fn stb_low() {
    gpio().bsrr().write(|w| w.br4().set_bit());
}

// Drive CLK high.
fn clk_high() {
    gpio().bsrr().write(|w| w.bs5().set_bit());
}

// Drive CLK low.
fn clk_low() {
    gpio().bsrr().write(|w| w.br5().set_bit());
}

// Drive DIO high.
fn dio_high() {
    gpio().moder().modify(|_, w| w.mode7().input());
}

// Drive DIO low.
fn dio_low() {
    gpio().bsrr().write(|w| w.br7().set_bit());
    gpio().moder().modify(|_, w| w.mode7().output());
}

// Read DIO input.
fn dio_read() -> bool {
    (gpio().idr().read().bits() & DIO) != 0
}

// Short timing delay.
fn delay() {
    cortex_m::asm::nop();
    cortex_m::asm::nop();
}

// Write one byte LSB first.
fn write_byte(mut data: u8) {
    for _ in 0..8 {
        clk_low();

        if (data & 1) != 0 {
            dio_high(); // release
        } else {
            dio_low(); // drive low
        }

        delay();

        clk_high();
        delay();

        data >>= 1;
    }

    dio_high(); // leave bus released
    clk_low();
}

// Read one byte LSB first.
fn read_byte() -> u8 {
    dio_high(); // Release bus once

    let mut data = 0u8;

    for i in 0..8 {
        clk_low();
        delay();

        clk_high();
        delay();

        if dio_read() {
            data |= 1 << i;
        }

        clk_low();
        dio_high();
    }

    data
}

// 7-segment font (abcdefg)
pub const FONT: [u8; 16] = [
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F, // 9
    0x77, // A
    0x7C, // b
    0x39, // C
    0x5E, // d
    0x79, // E
    0x71, // F
];

pub const KEY6: u32 = 0x0010_0000; // Left most key ACK/Mode
pub const KEY5: u32 = 0x0001_0000;
pub const KEY4: u32 = 0x0000_1000;
pub const KEY3: u32 = 0x0000_0100;
pub const KEY2: u32 = 0x0000_0010;
pub const KEY1: u32 = 0x0000_0001;

impl Tm1638 {
    // Create and initialize the driver.
    pub fn new() -> Self {
        // Put the bus into its idle state.
        stb_high();
        clk_high();
        dio_high();

        let mut tm = Self;
        tm.set_display(true, 7);
        tm
    }

    // Enable or disable the display.
    pub fn set_display(&mut self, on: bool, brightness: u8) {
        let cmd = if on {
            CMD_DISPLAY_ON | (brightness & 0x07)
        } else {
            CMD_DISPLAY_OFF
        };

        stb_low();
        write_byte(cmd);
        stb_high();
    }

    // Write the complete 16-byte display RAM.
    pub fn write_display(&mut self, data: &[u8; 16]) {
        // Select auto-increment mode.
        stb_low();
        write_byte(CMD_DATA_AUTO_INC);
        stb_high();

        // Set display RAM address.
        stb_low();
        write_byte(CMD_ADDR);

        // Transfer all display bytes.
        for &b in data {
            write_byte(b);
        }

        stb_high();
    }

    // Clear the display.
    pub fn clear(&mut self) {
        self.write_display(&[0; 16]);
    }
    // Read the four key scan bytes.
    pub fn read_keys(&mut self, buf: &mut [u8; 4]) {
        stb_low();
        write_byte(CMD_DATA_READ);

        for b in buf.iter_mut() {
            *b = read_byte();
        }

        stb_high();

        // Restore idle bus state.
        dio_high();
    }
}
