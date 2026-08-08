use stm32c0::stm32c031::TIM1;

/// Driver for reading a quadrature encoder using TIM1 on STM32C0.
pub struct QuadratureEncoder {
    tim: TIM1,
    accumulated_count: i32,
    last_raw_count: u16,
}

impl QuadratureEncoder {
    /// Initializes PA0 and PA1 as TIM1_CH1 and TIM1_CH2 in Alternate Function mode,
    /// and configures TIM1 into Encoder Mode 3 (X4 resolution on both channels).
    pub fn new(tim: TIM1) -> Self {
        // 3. Configure TIM1 Peripheral for Quadrature Decoding
        tim.cr1().modify(|_, w| w.cen().clear_bit());

        // Full 16-bit auto-reload range
        tim.arr().write(|w| unsafe { w.bits(0xFFFF) });

        // Map CH1 -> TI1, CH2 -> TI2, and configure input digital filtering
        tim.ccmr1_input().modify(|_, w| unsafe {
            w.cc1s().bits(0b01);
            w.cc2s().bits(0b01);
            w.ic1f().bits(0b0011); // Filter clock sampling
            w.ic2f().bits(0b0011)
        });

        // Configure CCER for non-inverted signal polarities
        tim.ccer().modify(|_, w| {
            w.cc1p().clear_bit();
            w.cc1np().clear_bit();
            w.cc2p().clear_bit();
            w.cc2np().clear_bit()
        });

        // Set Slave Mode to Encoder Mode 3
        tim.smcr().modify(|_, w| unsafe { w.sms().bits(0b0011) });

        // Reset counter to zero initially
        tim.cnt().write(|w| unsafe { w.cnt().bits(0) });

        // Enable Counter (CEN)
        tim.cr1().modify(|_, w| w.cen().set_bit());

        Self {
            tim,
            accumulated_count: 0,
            last_raw_count: 0,
        }
    }

    /// Polls the hardware timer, updates the internal 32-bit tracking,
    /// and returns the current absolute position as an `i32`.
    ///
    /// *Note: This requires `&mut self` to update the internal tracking variables.*
    pub fn update_and_read_i32(&mut self) -> i32 {
        let current_raw = self.read_u16();

        // Calculate wrapping delta using standard wrapping subtraction
        // E.g., if current_raw is 2 and last_raw was 65535: 2 - 65535 = 3 (as u16)
        let delta_u16 = current_raw.wrapping_sub(self.last_raw_count);

        // Casting u16 directly to i16 interprets values >= 32768 as negative steps.
        // E.g., 3 as i16 is +3. If wrapping backward: 65534 - 0 = 65534 -> -2 as i16.
        let delta_i16 = delta_u16 as i16;

        // Add the signed difference to our i32 tracking
        self.accumulated_count += delta_i16 as i32;

        // Store current position for the next delta evaluation
        self.last_raw_count = current_raw;

        self.accumulated_count
    }

    /// Reads the current raw 16-bit unsigned encoder count directly from hardware.
    #[inline]
    pub fn read_u16(&self) -> u16 {
        self.tim.cnt().read().cnt().bits()
    }

    /// Resets both the hardware counter and the 32-bit tracking state back to `0`,
    /// temporarily freezing the timer to eliminate race conditions under high speed.
    #[inline]
    pub fn reset(&mut self) {
        // 1. Disable the hardware timer counter to freeze edge accumulation
        self.tim.cr1().modify(|_, w| w.cen().clear_bit());

        // 2. Force the hardware counter register to 0
        self.tim.cnt().write(|w| unsafe { w.cnt().bits(0) });

        // 3. Align all internal software tracking states to a clean 0
        self.last_raw_count = 0;
        self.accumulated_count = 0;

        // 4. Re-enable the hardware counter to resume tracking
        self.tim.cr1().modify(|_, w| w.cen().set_bit());
    }

    /// Presets the hardware encoder counter to a specific u16 value,
    /// and matches the 32-bit accumulator to it.
    /// Presets the encoder tracking by temporarily freezing the hardware timer
    /// to guarantee race-free alignment under high-speed movement.
    #[inline]
    pub fn preset(&mut self, value: i32) {
        // 1. Disable the hardware timer counter to freeze edge accumulation
        self.tim.cr1().modify(|_, w| w.cen().clear_bit());

        // 2. Safely force the hardware counter register to 0 while frozen
        self.tim.cnt().write(|w| unsafe { w.cnt().bits(0) });

        // 3. Align the software tracking structures to the new baseline perfectly
        self.last_raw_count = 0;
        self.accumulated_count = value;

        // 4. Re-enable the hardware counter to resume active quadrature tracking
        self.tim.cr1().modify(|_, w| w.cen().set_bit());
    }

    /// Reads the direction of counter movement.
    /// Returns `true` if counting down, `false` if counting up.
    #[inline]
    pub fn is_counting_down(&self) -> bool {
        self.tim.cr1().read().dir().bit_is_set()
    }
}
