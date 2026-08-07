use stm32c0::stm32c031 as pac;

pub struct RelayController {
    rl1_active: bool,
    rl2_active: bool,
    ticks_remaining: u32,
    prev_both_triggered: bool,
    monostable_done: bool, // Latching flag to prevent turning back ON while still above limits
}

impl RelayController {
    pub fn new() -> Self {
        Self {
            rl1_active: false,
            rl2_active: false,
            ticks_remaining: 0,
            prev_both_triggered: false,
            monostable_done: false,
        }
    }

    pub fn update(
        &mut self,
        scaled_value: i32,
        limit_1: i32,
        limit_2: i32,
        relay_time_sec: u8,
        reset_pressed: bool,
    ) {
        if reset_pressed {
            self.reset();
            return;
        }

        let l1_crossed = if limit_1 >= 0 {
            scaled_value > limit_1
        } else {
            scaled_value < limit_1
        };

        let l2_crossed = if limit_2 >= 0 {
            scaled_value > limit_2
        } else {
            scaled_value < limit_2
        };
        let both_crossed = l1_crossed && l2_crossed;

        // Reset the monostable latch once the system drops back below the trigger threshold
        if !both_crossed {
            self.monostable_done = false;
        }

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

            // Start timer on fresh rising edge (when not already completed in this cycle)
            if both_crossed && !self.prev_both_triggered && !self.monostable_done {
                self.ticks_remaining = u32::from(relay_time_sec) * 1000;
            }

            if self.ticks_remaining > 0 {
                self.rl1_active = true;
                self.rl2_active = true;

                self.ticks_remaining -= 1;

                if self.ticks_remaining == 0 {
                    self.rl1_active = false;
                    self.rl2_active = false;
                    self.monostable_done = true; // Block re-activation until both_crossed becomes false
                }
            } else if self.monostable_done {
                // Timer finished for this cycle -> Keep OFF
                self.rl1_active = false;
                self.rl2_active = false;
            } else {
                // Active as individual limits are reached before both are met
                self.rl1_active = l1_crossed;
                self.rl2_active = l2_crossed;
            }
        }

        self.prev_both_triggered = both_crossed;
    }

    pub fn write_hardware(&self, gpiob: &pac::gpiob::RegisterBlock) {
        gpiob.bsrr().write(|w| {
            if self.rl1_active {
                w.br0().set_bit();
            } else {
                w.bs0().set_bit();
            }

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
        self.prev_both_triggered = false;
        self.monostable_done = false;
    }

    pub fn is_rl1_active(&self) -> bool {
        self.rl1_active
    }

    pub fn is_rl2_active(&self) -> bool {
        self.rl2_active
    }
}
