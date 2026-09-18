use crate::Tm1638Pins;

pub const KEY8: u32 = 0x1000_0000;
pub const KEY7: u32 = 0x0100_0000;
pub const KEY6: u32 = 0x0010_0000;
pub const KEY5: u32 = 0x0001_0000;
pub const KEY4: u32 = 0x0000_1000;
pub const KEY3: u32 = 0x0000_0100;
pub const KEY2: u32 = 0x0000_0010;
pub const KEY1: u32 = 0x0000_0001;

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

const CMD_DATA_AUTO_INC: u8 = 0x40;
const CMD_DATA_READ: u8 = 0x42;
const CMD_ADDR: u8 = 0xC0;
const CMD_DISPLAY_OFF: u8 = 0x80;
const CMD_DISPLAY_ON: u8 = 0x88;

pub struct Tm1638<P> {
    pins: P,
}

impl<P: Tm1638Pins> Tm1638<P> {
    pub fn new(pins: P) -> Self {
        let mut tm = Self { pins };
        tm.pins.stb_high();
        tm.pins.clk_high();
        tm.pins.dio_high();
        tm.set_display(true, 7);
        tm
    }

    pub fn set_display(&mut self, on: bool, brightness: u8) {
        let cmd = if on {
            CMD_DISPLAY_ON | (brightness & 0x07)
        } else {
            CMD_DISPLAY_OFF
        };

        self.pins.stb_low();
        self.write_byte(cmd);
        self.pins.stb_high();
    }

    pub fn write_display(&mut self, data: &[u8; 16]) {
        self.pins.stb_low();
        self.write_byte(CMD_DATA_AUTO_INC);
        self.pins.stb_high();

        self.pins.stb_low();
        self.write_byte(CMD_ADDR);

        for &b in data {
            self.write_byte(b);
        }

        self.pins.stb_high();
    }

    pub fn clear(&mut self) {
        self.write_display(&[0; 16]);
    }

    pub fn read_keys(&mut self, buf: &mut [u8; 4]) {
        self.pins.stb_low();
        self.write_byte(CMD_DATA_READ);

        for b in buf.iter_mut() {
            *b = self.read_byte();
        }

        self.pins.stb_high();
        self.pins.dio_high();
    }

    fn write_byte(&mut self, mut data: u8) {
        for _ in 0..8 {
            self.pins.clk_low();

            if (data & 1) != 0 {
                self.pins.dio_high();
            } else {
                self.pins.dio_low();
            }

            cortex_m::asm::nop();
            cortex_m::asm::nop();

            self.pins.clk_high();
            cortex_m::asm::nop();
            cortex_m::asm::nop();

            data >>= 1;
        }

        self.pins.dio_high();
        self.pins.clk_low();
    }

    fn read_byte(&mut self) -> u8 {
        self.pins.dio_high();
        let mut data = 0u8;

        for i in 0..8 {
            self.pins.clk_low();
            cortex_m::asm::nop();
            cortex_m::asm::nop();

            self.pins.clk_high();
            cortex_m::asm::nop();
            cortex_m::asm::nop();

            if self.pins.dio_read() {
                data |= 1 << i;
            }

            self.pins.clk_low();
            self.pins.dio_high();
        }

        data
    }
}

// Keep FONT and KEY constants here as before...
