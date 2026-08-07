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

    /// Reads the encoder counter directly as a signed 16-bit integer (`i16`).
    #[inline]
    pub fn read_i16(&self) -> i16 {
        self.read_u16() as i16
    }

    /// Resets both the hardware counter and the 32-bit tracking state back to `0`.
    #[inline]
    pub fn reset(&mut self) {
        // Force the hardware back to 0
        self.tim.cnt().write(|w| unsafe { w.cnt().bits(0) });

        // Align all internal state so the next delta calculation starts from a clean 0
        self.last_raw_count = 0;
        self.accumulated_count = 0;
    }

    /// Presets the hardware encoder counter to a specific u16 value,
    /// and matches the 32-bit accumulator to it.
    #[inline]
    pub fn preset(&mut self, value: u16) {
        self.tim.cnt().write(|w| unsafe { w.cnt().bits(value) });

        // Align state so the next update starts tracking from this baseline
        self.last_raw_count = value;
        self.accumulated_count = value as i32;
    }

    /// Presets the absolute tracking state to a specific `i32` value,
    /// automatically aligning the underlying hardware timer to match.
    #[inline]
    pub fn preset_i32(&mut self, target_i32: i32) {
        // Map the i32 down to the raw u16 equivalent for the hardware register
        let raw_u16 = (target_i32 & 0xFFFF) as u16;

        self.tim.cnt().write(|w| unsafe { w.cnt().bits(raw_u16) });

        // Both trackers are updated together to match the new timeline seamlessly
        self.last_raw_count = raw_u16;
        self.accumulated_count = target_i32;
    }
    /// Reads the direction of counter movement.
    /// Returns `true` if counting down, `false` if counting up.
    #[inline]
    pub fn is_counting_down(&self) -> bool {
        self.tim.cr1().read().dir().bit_is_set()
    }
}
