#![no_std]

mod blink;
mod keyboard;
mod tm1638;

pub use blink::Blinker;
pub use keyboard::{Key, KeyEvent, Keyboard};
pub use tm1638::{FONT, KEY1, KEY2, KEY3, KEY4, KEY5, KEY6, KEY7, KEY8, Tm1638};

/// Trait defining the pin operations required by the TM1638 driver.
pub trait Tm1638Pins {
    fn stb_high(&mut self);
    fn stb_low(&mut self);
    fn clk_high(&mut self);
    fn clk_low(&mut self);
    fn dio_high(&mut self);
    fn dio_low(&mut self);
    fn dio_read(&mut self) -> bool;
}

// Inside tm1638_system src/lib.rs (or wherever DisplayManager is defined)

pub struct ConsoleManager<P> {
    tm1638: Tm1638<P>,
    keyboard: Keyboard,
    blinker: Blinker,
    ram: [u8; 16],
    last_rendered_ram: [u8; 16],
    blink_mask: Option<u16>,
    latest_event: Option<KeyEvent>,
}

impl<P: Tm1638Pins> ConsoleManager<P> {
    pub fn new(pins: P) -> Self {
        Self {
            tm1638: Tm1638::new(pins),
            keyboard: Keyboard::new(),
            blinker: Blinker::new(),
            ram: [0; 16],
            last_rendered_ram: [0; 16],
            blink_mask: None,
            latest_event: None,
        }
    }

    pub fn update(&mut self) {
        // 1. Poll keys
        let mut key_buf = [0u8; 4];
        self.tm1638.read_keys(&mut key_buf);
        let raw_keys = u32::from_le_bytes(key_buf);
        self.latest_event = self.keyboard.update(raw_keys);

        // 2. Render display with blinking
        let mut render_ram = self.ram;
        self.blinker.update(&mut render_ram, self.blink_mask);

        if render_ram != self.last_rendered_ram {
            self.tm1638.write_display(&render_ram);
            self.last_rendered_ram = render_ram;
        }
    }

    pub fn set_ram(&mut self, ram: [u8; 16]) {
        self.ram = ram;
    }

    pub fn set_blink_mask(&mut self, mask: Option<u16>) {
        self.blink_mask = mask;
    }

    pub fn poll_event(&mut self) -> Option<KeyEvent> {
        self.latest_event.take()
    }
}