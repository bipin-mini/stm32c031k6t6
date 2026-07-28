//! blink.rs
//!
//! Seven-segment blinking helper.
//!
//! Call `update()` periodically from the TM1638 task.
//!
//! The blink module never owns the display buffer.
//! It only temporarily blanks selected segment bytes before the
//! display buffer is written to the TM1638.
//!
//! # Example
//!
//! ```ignore
//! use crate::drivers::blink::Blink;
//!
//! // TM1638 task executes every 20 ms.
//! // Blink period = 25 ticks = 500 ms.
//! let mut blink = Blink::new(25);
//!
//! // Blink the third digit.
//! blink.enable(true);
//! blink.set_mask(1 << 2);
//!
//! loop {
//!     // Called every 20 ms.
//!     blink.update();
//!
//!     // Obtain the display buffer from the state machine.
//!     let mut display = display_buffer;
//!
//!     // Apply blinking.
//!     blink.apply(&mut display);
//!
//!     // Write to TM1638.
//!     tm1638.write_display(&display);
//! }
//! ```
//!
//! # Blink Mask
//!
//! ```text
//! Bit0 -> Leftmost digit
//! Bit1
//! Bit2
//! Bit3
//! Bit4
//! Bit5 -> Rightmost digit
//! ```
//!
//! Examples:
//!
//! ```ignore
//! // Blink first digit.
//! blink.set_mask(1 << 0);
//!
//! // Blink fourth digit.
//! blink.set_mask(1 << 3);
//!
//! // Blink digits 2,3,4.
//! blink.set_mask((1 << 1) | (1 << 2) | (1 << 3));
//!
//! // Blink all digits.
//! blink.set_mask(0x3F);
//!
//! // Disable blinking.
//! blink.enable(false);
//! ```
//!
//! # TM1638 Display RAM Mapping
//!
//! Only segment bytes are modified.
//!
//! ```text
//! Digit0 -> data[4]
//! Digit1 -> data[6]
//! Digit2 -> data[8]
//! Digit3 -> data[10]
//! Digit4 -> data[12]
//! Digit5 -> data[14]
//! ```
//!
//! LEDs and the sign byte are never modified.

#![allow(dead_code)]

/// Bit mask selecting digits to blink.
///
/// Bit0 = leftmost digit.
/// Bit5 = rightmost digit.
pub type BlinkMask = u8;

/// Index of the leftmost TM1638 digit in the display RAM.
const FIRST_SEG_RAM: usize = 4;

/// Number of display digits.
const NUM_DIGITS: usize = 6;

pub struct Blink {
    enabled: bool,
    visible: bool,

    ticks: u16,
    period: u16,

    mask: BlinkMask,
}

impl Blink {
    /// Create a new blink generator.
    ///
    /// `period` is expressed in update ticks.
    ///
    /// Example:
    ///
    /// * update every 20 ms
    /// * period = 25
    /// * blink period = 500 ms
    pub const fn new(period: u16) -> Self {
        Self {
            enabled: false,
            visible: true,
            ticks: 0,
            period,
            mask: 0,
        }
    }

    /// Enable or disable blinking.
    pub fn enable(&mut self, enable: bool) {
        self.enabled = enable;

        if !enable {
            self.visible = true;
            self.ticks = 0;
            self.mask = 0;
        }
    }

    /// Select which digits blink.
    ///
    /// Bit0 = leftmost digit.
    /// Bit5 = rightmost digit.
    pub fn set_mask(&mut self, mask: BlinkMask) {
        self.mask = mask;
    }

    /// Periodically update the blink state.
    ///
    /// Call once every TM1638 task execution.
    #[inline]
    pub fn update(&mut self) {
        if !self.enabled {
            return;
        }

        self.ticks += 1;

        if self.ticks >= self.period {
            self.ticks = 0;
            self.visible = !self.visible;
        }
    }

    /// Apply blinking to the TM1638 display RAM.
    ///
    /// Only segment bytes are modified.
    /// LEDs and sign remain unchanged.
    pub fn apply(&self, display: &mut [u8; 16]) {
        if !self.enabled || self.visible {
            return;
        }

        for digit in 0..NUM_DIGITS {
            if (self.mask & (1 << digit)) != 0 {
                display[FIRST_SEG_RAM + digit * 2] = 0;
            }
        }
    }

    /// Returns whether the blinked digits are currently visible.
    pub fn visible(&self) -> bool {
        self.visible
    }

    /// Disable blinking and clear the blink mask.
    pub fn clear(&mut self) {
        self.enable(false);
    }
}
