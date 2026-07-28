//! keyboard.rs
//!
//! Periodic keyboard algorithm.
//!
//! Call Keyboard::update() every 20-50 ms with the raw TM1638 key bitmap.
//!
//! The driver performs:
//! - key decoding
//! - edge detection
//! - long-press detection
//!
//! Events are latched internally and consumed once by calling:
//!
//! - press_key()
//! - release_key()
//! - long_press()

//! # Example
//!
//! ```ignore
//! use crate::drivers::keyboard::{Keyboard, Key};
//!
//! let mut keyboard = Keyboard::new();
//!
//! // Called every 20 ms from the TM1638 task.
//! loop {
//!     // Raw 32-bit key bitmap read from the TM1638.
//!     let raw_keys: u32 = tm1638.read_keys();
//!
//!     // Update keyboard state.
//!     keyboard.update(raw_keys);
//!
//!     // Normal key press.
//!     if let Some(key) = keyboard.press_key() {
//!         match key {
//!             Key::Key1 => {
//!                 // Increment parameter
//!             }
//!             Key::Key2 => {
//!                 // Decrement parameter
//!             }
//!             Key::Key3 => {
//!                 // Next menu
//!             }
//!             _ => {}
//!         }
//!     }
//!
//!     // Long press (3 seconds).
//!     if let Some(key) = keyboard.long_press() {
//!         match key {
//!             Key::Key1 => {
//!                 // Enter edit mode
//!             }
//!             Key::Key6 => {
//!                 // Factory reset
//!             }
//!             _ => {}
//!         }
//!     }
//!
//!     // Key released.
//!     if let Some(key) = keyboard.release_key() {
//!         match key {
//!             Key::Key1 => {
//!                 // Finish operation
//!             }
//!             _ => {}
//!         }
//!     }
//! }
//! ```
//!

#![allow(dead_code)]

/// Number of update() calls before a long press is generated.
///
/// Example:
/// 20 ms period × 150 = 3 seconds
pub const LONG_PRESS_TICKS: u16 = 150;

/// Logical keyboard keys.
#[derive(Copy, Clone, Eq, PartialEq)]
pub enum Key {
    Key1,
    Key2,
    Key3,
    Key4,
    Key5,
    Key6,
}

/// Raw TM1638 key bit definitions.
pub const KEY1: u32 = 0x0000_0001;
pub const KEY2: u32 = 0x0000_0010;
pub const KEY3: u32 = 0x0000_0100;
pub const KEY4: u32 = 0x0000_1000;
pub const KEY5: u32 = 0x0001_0000;
pub const KEY6: u32 = 0x0010_0000;

/// Keyboard algorithm.
pub struct Keyboard {
    current: Option<Key>,

    hold_ticks: u16,

    long_sent: bool,

    press_event: Option<Key>,
    release_event: Option<Key>,
    long_event: Option<Key>,
}

impl Keyboard {
    /// Create keyboard object.
    pub const fn new() -> Self {
        Self {
            current: None,

            hold_ticks: 0,

            long_sent: false,

            press_event: None,
            release_event: None,
            long_event: None,
        }
    }

    /// Call periodically (20-50 ms).
    pub fn update(&mut self, raw: u32) {
        let key = Self::decode(raw);

        //
        // Key unchanged.
        //
        if key == self.current {
            if let Some(k) = key {
                let _ = k;

                self.hold_ticks = self.hold_ticks.saturating_add(1);

                if !self.long_sent && self.hold_ticks >= LONG_PRESS_TICKS {
                    self.long_sent = true;
                    self.long_event = Some(k);
                }
            }

            return;
        }

        //
        // Previous key released.
        //
        if let Some(old) = self.current {
            self.release_event = Some(old);
        }

        //
        // New key pressed.
        //
        if let Some(new) = key {
            self.press_event = Some(new);
        }

        self.current = key;
        self.hold_ticks = 0;
        self.long_sent = false;
    }

    /// Returns a pending key press event.
    pub fn press_key(&mut self) -> Option<Key> {
        self.press_event.take()
    }

    /// Returns a pending key release event.
    pub fn release_key(&mut self) -> Option<Key> {
        self.release_event.take()
    }

    /// Returns a pending long-press event.
    pub fn long_press(&mut self) -> Option<Key> {
        self.long_event.take()
    }

    /// Decode raw TM1638 bitmap.
    ///
    /// Returns only one key.
    /// Multiple simultaneous keys are ignored.
    fn decode(raw: u32) -> Option<Key> {
        match raw {
            KEY1 => Some(Key::Key1),
            KEY2 => Some(Key::Key2),
            KEY3 => Some(Key::Key3),
            KEY4 => Some(Key::Key4),
            KEY5 => Some(Key::Key5),
            KEY6 => Some(Key::Key6),
            _ => None,
        }
    }
}
