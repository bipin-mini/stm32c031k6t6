// Fixed: 25 ticks * 10ms task rate = 250ms toggle interval
const DEFAULT_BLINK_INTERVAL_TICKS: u16 = 25;

#[derive(Debug, Clone, Copy, Default)]
pub struct Blinker {
    ticks: u16,
    off_phase: bool,
}

impl Blinker {
    pub const fn new() -> Self {
        Self {
            ticks: 0,
            off_phase: false,
        }
    }

    /// Advances blinker timing by 1 tick (10 ms) and modifies `ram` in place according to `blink_mask`.
    pub fn update(&mut self, ram: &mut [u8; 16], blink_mask: Option<u16>) {
        // Increment and handle tick rollover
        self.ticks += 1;
        if self.ticks >= DEFAULT_BLINK_INTERVAL_TICKS {
            self.ticks = 0;
            self.off_phase = !self.off_phase;
        }

        // Early return if we are in the ON phase or if there's no work to do
        if !self.off_phase {
            return;
        }

        if let Some(mask) = blink_mask {
            // Check if any bits are actually set to bypass completely if 0
            if mask != 0 {
                // Loop optimized for minimal code generation:
                // We use a shifting mask instead of moving a dynamic index variable `1 << i`
                let mut current_bit = 1u16;
                for byte in ram.iter_mut() {
                    if (mask & current_bit) != 0 {
                        *byte = 0; // Clear digit/LED data during off-phase
                    }
                    current_bit <<= 1;
                }
            }
        }
    }
}
