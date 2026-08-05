const DEFAULT_BLINK_INTERVAL_TICKS: u16 = 100; // 250 ms toggle interval

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
        self.ticks += 1;
        if self.ticks >= DEFAULT_BLINK_INTERVAL_TICKS {
            self.ticks = 0;
            self.off_phase = !self.off_phase;
        }

        if self.off_phase {
            if let Some(mask) = blink_mask {
                if mask != 0 {
                    for i in 0..16 {
                        if (mask & (1 << i)) != 0 {
                            ram[i] = 0;
                        }
                    }
                }
            }
        }
    }
}
