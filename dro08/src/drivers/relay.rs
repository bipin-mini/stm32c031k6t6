use stm32c0::stm32c031 as pac;

pub struct RelayController {
    rl1_active: bool,
    rl2_active: bool,
    ticks_remaining: u32,
    prev_triggered: bool,
}

impl RelayController {
    pub fn new() -> Self {
        Self {
            rl1_active: false,
            rl2_active: false,
            ticks_remaining: 0,
            prev_triggered: false,
        }
    }

    /// Evaluates limits and updates internal relay states.
    /// MUST be called every 1 ms inside `relay_task`.
    pub fn update(
        &mut self,
        scaled_value: i32,
        limit_1: i32,
        limit_2: i32,
        relay_time_sec: u8,
        reset_pressed: bool,
    ) {
        // 1. Explicit Reset Key handling
        if reset_pressed {
            self.reset();
            return;
        }

        let l1_crossed = scaled_value >= limit_1;
        let l2_crossed = scaled_value >= limit_2;
        let threshold_hit = l1_crossed || l2_crossed;

        // 2. Mode Evaluation
        if relay_time_sec == 0 {
            // --- LATCHING MODE ---
            if l1_crossed {
                self.rl1_active = true;
            }
            if l2_crossed {
                self.rl2_active = true;
            }
        } else {
            // --- MONOSTABLE (TIMED) MODE ---
            // Trigger pulse on initial rising edge crossing
            if threshold_hit && !self.prev_triggered {
                if l1_crossed {
                    self.rl1_active = true;
                }
                if l2_crossed {
                    self.rl2_active = true;
                }
                // 1 ms tick rate: 1 second = 1000 ticks
                self.ticks_remaining = u32::from(relay_time_sec) * 1000;
            }

            // Countdown timer
            if self.ticks_remaining > 0 {
                self.ticks_remaining -= 1;
                if self.ticks_remaining == 0 {
                    self.rl1_active = false;
                    self.rl2_active = false;
                }
            }
        }

        self.prev_triggered = threshold_hit;
    }

    /// Drives physical pins on GPIOB (Active LOW: PB0 = RL1, PB1 = RL2)
    pub fn write_hardware(&self, gpiob: &pac::gpiob::RegisterBlock) {
        gpiob.bsrr().write(|w| {
            // RL1 (PB0): Active LOW -> Reset bit (br0) = ON, Set bit (bs0) = OFF
            if self.rl1_active {
                w.br0().set_bit();
            } else {
                w.bs0().set_bit();
            }

            // RL2 (PB1): Active LOW -> Reset bit (br1) = ON, Set bit (bs1) = OFF
            if self.rl2_active {
                w.br1().set_bit();
            } else {
                w.bs1().set_bit();
            }

            w
        });
    }

    pub fn reset(&mut self) {
        self.rl1_active = false;
        self.rl2_active = false;
        self.ticks_remaining = 0;
        self.prev_triggered = true; // Prevent immediate re-trigger if count is still high
    }

    pub fn is_rl1_active(&self) -> bool {
        self.rl1_active
    }

    pub fn is_rl2_active(&self) -> bool {
        self.rl2_active
    }
}
