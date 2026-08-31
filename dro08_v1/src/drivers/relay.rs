use stm32c0::stm32c031 as pac;

const TICK_FACTOR: u32 = 500; // For 2 ms execution cadence

#[derive(Default)]
pub struct RelayController {
    rl1_active: bool,
    rl2_active: bool,
    ticks_remaining: u32,
    prev_both_triggered: bool,
    monostable_done: bool,
}

impl RelayController {
    /// Creates a runtime state tracker assuming GPIOB pins 0 and 1 are already initialized
    pub fn new() -> Self {
        Self {
            rl1_active: false,
            rl2_active: false,
            ticks_remaining: 0,
            prev_both_triggered: false,
            monostable_done: false,
        }
    }

    #[inline(always)]
    fn gpiob(&self) -> &pac::gpiob::RegisterBlock {
        unsafe { &*pac::GPIOB::ptr() }
    }

    // --- Public Manual Control Interface Methods (Active-Low Logic) ---

    pub fn relay1_on(&self) {
        // Clear Pin 0 (BR0) to pull it Low
        self.gpiob().bsrr().write(|w| w.br0().set_bit());
    }

    pub fn relay1_off(&self) {
        // Set Pin 0 (BS0) to pull it High
        self.gpiob().bsrr().write(|w| w.bs0().set_bit());
    }

    pub fn relay2_on(&self) {
        // Clear Pin 1 (BR1) to pull it Low
        self.gpiob().bsrr().write(|w| w.br1().set_bit());
    }

    pub fn relay2_off(&self) {
        // Set Pin 1 (BS1) to pull it High
        self.gpiob().bsrr().write(|w| w.bs1().set_bit());
    }

    // --- State Machine Updates ---

    /// Explicit helper to clear states and physically turn off relays
    pub fn reset(&mut self) {
        self.rl1_active = false;
        self.rl2_active = false;
        self.ticks_remaining = 0;
        self.prev_both_triggered = false;
        self.monostable_done = false;

        self.relay1_off();
        self.relay2_off();
    }

    /// Evaluates limits, updates internal flags, and flushes states straight to pins
    pub fn update(&mut self, scaled_value: i32, limit_1: i32, limit_2: i32, relay_time_sec: u8) {
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
            if both_crossed && !self.prev_both_triggered && !self.monostable_done {
                self.ticks_remaining = u32::from(relay_time_sec) * TICK_FACTOR;
            }

            if self.ticks_remaining > 0 {
                self.rl1_active = true;
                self.rl2_active = true;
                self.ticks_remaining -= 1;

                if self.ticks_remaining == 0 {
                    self.rl1_active = false;
                    self.rl2_active = false;
                    self.monostable_done = true;
                }
            } else if self.monostable_done {
                self.rl1_active = false;
                self.rl2_active = false;
            } else {
                self.rl1_active = l1_crossed;
                self.rl2_active = l2_crossed;
            }
        }

        self.prev_both_triggered = both_crossed;

        // Drive hardware pins directly using BSRR register mappings via helpers
        if self.rl1_active {
            self.relay1_on();
        } else {
            self.relay1_off();
        }
        if self.rl2_active {
            self.relay2_on();
        } else {
            self.relay2_off();
        }
    }

    pub fn is_rl1_active(&self) -> bool {
        self.rl1_active
    }
    pub fn is_rl2_active(&self) -> bool {
        self.rl2_active
    }
}
