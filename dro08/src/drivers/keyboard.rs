//! keyboard.rs
//!
//! Minimal keyboard driver.
//! Call update() every 10 ms.


use crate::drivers::tm1638::{KEY1, KEY2, KEY3, KEY4, KEY5, KEY6};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Key {
    Key1,
    Key2,
    Key3,
    Key4,
    Key5,
    Key6,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum KeyEvent {
    Short(Key),
    Long(Key),
}

pub struct Keyboard;

impl Keyboard {
    pub const fn new() -> Self {
        Self
    }

pub fn update(&mut self, raw_keys: u32) -> Option<KeyEvent> {
    match raw_keys {
        KEY1 => Some(KeyEvent::Short(Key::Key1)),
        KEY2 => Some(KeyEvent::Short(Key::Key2)),
        KEY3 => Some(KeyEvent::Short(Key::Key3)),
        KEY4 => Some(KeyEvent::Short(Key::Key4)),
        KEY5 => Some(KeyEvent::Short(Key::Key5)),
        KEY6 => Some(KeyEvent::Short(Key::Key6)),
        _ => None,
    }
}
}
