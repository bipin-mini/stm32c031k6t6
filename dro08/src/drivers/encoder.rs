//! # Quadrature Encoder Driver
//!
//! Deterministic software quadrature decoder for STM32C031.
//!
//! ## Overview
//!
//! This module implements a high-speed incremental quadrature encoder
//! decoder using a Gray-code lookup table.
//!
//! The decoder is intended to be called whenever either encoder input
//! changes, typically from the EXTI interrupt servicing encoder channels
//! A and B.
//!
//! The implementation features:
//!
//! - No conditional branches
//! - Constant execution path
//! - No dynamic allocation
//! - PAC-only implementation (no HAL)
//! - Suitable for RTIC interrupt context
//!
//! ## Pin Assignment
//!
//! The decoder assumes the encoder inputs are connected as follows:
//!
//! ```text
//! PA0 -> Encoder Channel A (Bit 0)
//! PA1 -> Encoder Channel B (Bit 1)
//! ```
//!
//! The optional encoder index (Z) input is not processed by this module
//! and should be handled separately by the application if required.
//!
//! ## Operating Principle
//!
//! The current quadrature state is formed from the two encoder inputs:
//!
//! ```text
//! PA1 PA0
//! --------
//!  0   0   -> 0
//!  0   1   -> 1
//!  1   0   -> 2
//!  1   1   -> 3
//! ```
//!
//! The previous and current states form a 4-bit lookup index:
//!
//! ```text
//! index = (previous << 2) | current
//! ```
//!
//! The lookup table returns:
//!
//! * +1 : Forward transition
//! * -1 : Reverse transition
//! *  0 : No movement or invalid transition
//!
//! > **Note:** The sign convention (+1/-1) depends on the assignment of
//! > encoder channels A and B. Swapping the A and B inputs reverses the
//! > reported direction.
//!
//! Valid quadrature transitions are:
//!
//! ```text
//! Forward:
//! 00 -> 10 -> 11 -> 01 -> 00
//!
//! Reverse:
//! 00 -> 01 -> 11 -> 10 -> 00
//! ```
//!
//! Invalid transitions caused by contact bounce, electrical noise, or
//! missed edges are automatically rejected by the lookup table.
//!
//! During construction, the decoder samples the current quadrature state
//! so that counting begins from the actual shaft position without
//! generating a false transition after startup.
//!
//! ## X4 Decoding
//!
//! This decoder performs true X4 quadrature decoding when the application
//! calls [`Encoder::update()`] on every rising and falling edge of both
//! encoder channels.
//!
//! On STM32C031 this is typically achieved by configuring EXTI on both
//! encoder inputs for both rising and falling edge detection.
//!
//! ## Timing
//!
//! Each call to [`Encoder::update()`] performs:
//!
//! * One GPIO input register read
//! * One bit mask
//! * One lookup-table access
//! * One state update
//!
//! The implementation contains:
//!
//! * No conditional branches
//! * No loops
//! * No multiplication or division
//! * No heap allocation
//!
//! The instruction path is constant for every invocation, making the
//! decoder well suited for interrupt-driven applications.
//!
//! Typical execution time on STM32C031 running at 48 MHz is well below
//! 1 µs.
//!
//! ## Usage
//!
//! During initialization:
//!
//! ```ignore
//! let mut encoder = Encoder::new(gpioa);
//! ```
//!
//! Whenever either encoder input changes (typically from the EXTI
//! interrupt):
//!
//! ```ignore
//! let delta = encoder.update(gpioa);
//!
//! if delta != 0 {
//!     position += delta as i32;
//! }
//! ```
//!
//! ## Thread Safety
//!
//! The driver contains mutable state (`prev`) and therefore must be owned
//! by exactly one execution context.
//!
//! In an RTIC application this is typically:
//!
//! * A local resource of the EXTI task
//!
//! so no locking or synchronization is required.
//!
//! ## Design Goals
//!
//! * Deterministic execution behaviour
//! * Minimal interrupt latency
//! * Suitable for high-speed incremental encoders
//! * Minimal RAM usage
//! * PAC-only implementation
//! * Production-quality firmware

use stm32c0::stm32c031::gpioa;

/// Encoder input bit mask (PA0 = Channel A, PA1 = Channel B).
const ENC_MASK: u32 = 0x03;

/// Quadrature transition lookup table.
///
/// Lookup index:
///
/// ```text
/// (previous_state << 2) | current_state
/// ```
///
/// Returned value:
///
/// | Value | Meaning |
/// |------:|---------|
/// |  +1   | Forward transition |
/// |  -1   | Reverse transition |
/// |   0   | Invalid transition or no movement |
///
/// Valid transitions:
///
/// ```text
/// Forward:
/// 00 -> 10 -> 11 -> 01 -> 00
///
/// Reverse:
/// 00 -> 01 -> 11 -> 10 -> 00
/// ```
const LUT: [i8; 16] = [0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0];

/// Software quadrature decoder.
///
/// The decoder stores only the previous quadrature state and uses a
/// Gray-code lookup table to determine the direction of movement.
///
/// The implementation is deterministic, contains no conditional
/// branches, and is intended for execution directly from an interrupt
/// service routine.
pub struct Encoder {
    /// Previous quadrature state.
    prev: u8,
}

impl Encoder {
    /// Creates a new quadrature decoder.
    ///
    /// The constructor samples the current quadrature state and stores it
    /// so that decoding begins without generating a false count after
    /// startup.
    ///
    /// # Arguments
    ///
    /// * `gpioa` - GPIOA peripheral register block.
    pub fn new(gpioa: &gpioa::RegisterBlock) -> Self {
        let prev = (gpioa.idr().read().bits() & ENC_MASK) as u8;
        Self { prev }
    }

    /// Decodes one quadrature transition.
    ///
    /// Reads the current encoder inputs, compares them with the previous
    /// quadrature state and returns the movement direction.
    ///
    /// # Returns
    ///
    /// * `+1` : Forward transition
    /// * `-1` : Reverse transition
    /// * `0`  : Invalid transition or no movement
    ///
    /// The sign convention depends on the assignment of encoder channels
    /// A and B.
    ///
    /// See the module-level documentation for details of the decoding
    /// algorithm and valid state transitions.
    ///
    /// This function has a constant execution path and should be called
    /// whenever either encoder input changes, typically from the EXTI
    /// interrupt.
    #[inline(always)]
    pub fn update(&mut self, gpioa: &gpioa::RegisterBlock) -> i8 {
        let curr = (gpioa.idr().read().bits() & ENC_MASK) as u8;

        let index = ((self.prev << 2) | curr) as usize;

        let delta = LUT[index];

        self.prev = curr;

        delta
    }
}
