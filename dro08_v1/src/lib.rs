#![no_std]

pub mod drivers;
pub mod protocol;

// --- Re-exports for convenient top-level access ---
pub use drivers::blink::Blinker;
pub use drivers::bsp;
pub use drivers::encoder::QuadratureEncoder;
pub use drivers::keyboard::{self, Key, KeyEvent, Keyboard};
pub use drivers::relay::RelayController;
pub use drivers::tm1638::{self, FONT, Tm1638};
pub use drivers::uart_dma::UartDma;
pub use protocol::modbus::{self, DEFAULT_ADDRESS, HoldingRegisters, Modbus};

const POW10: [i64; 6] = [1, 10, 100, 1_000, 10_000, 100_000];

#[derive(Clone, Copy)]
pub struct ScaleRatio {
    pub val: u32,
    pub dp: u8,
}

impl ScaleRatio {
    pub const fn new(val: u32, dp: u8) -> Self {
        Self {
            val: if val == 0 { 1 } else { val },
            dp: dp % 6,
        }
    }

    /// Apply ratio using 64-bit integer arithmetic
    #[inline(always)]
    pub fn apply(&self, raw_count: i32) -> i32 {
        let raw = raw_count as i64;
        let num = self.val as i64;
        let den = POW10[self.dp as usize];

        let scaled = (raw * num) / den;
        scaled as i32
    }

    /// Convert scaled value back to raw encoder count
    #[inline(always)]
    pub fn unapply(&self, scaled_val: i32) -> i32 {
        let scaled = scaled_val as i64;
        let num = POW10[self.dp as usize];
        let den = self.val as i64;

        let half_den = den / 2;
        let raw = if scaled >= 0 {
            (scaled * num + half_den) / den
        } else {
            (scaled * num - half_den) / den
        };

        raw as i32
    }
}

pub fn display_i32(n: i32, ram_data: &mut [u8; 16], decimal_pos: u8) {
    let negative = n < 0;
    let mut value = n.unsigned_abs();

    for i in 0..6 {
        let digit = (value % 10) as usize;
        value /= 10;
        ram_data[(7 - i) * 2] = FONT[digit];
    }

    if decimal_pos > 0 && decimal_pos < 6 {
        ram_data[(7 - decimal_pos as usize) * 2] |= 0x80;
    }

    if negative {
        ram_data[2] = 0x40;
    }
}

/// Suppresses leading zeros and shifts negative sign adjacent to the first active digit/decimal.
/// Must be applied *after* `display_i32` populates `ram_data`.
pub fn suppress_leading_zeros(ram_data: &mut [u8; 16]) {
    // Check if the number was negative (display_i32 puts 0x40 in position index 2 / leftmost digit)
    let is_negative = ram_data[2] == 0x40;
    if is_negative {
        ram_data[2] = 0; // Clear the fixed negative sign
    }

    // 1. Scan from left to right (digits 5 down to 0) to find the highest active digit index.
    // RAM indices for digits: Digit 5 -> 4, Digit 4 -> 6, Digit 3 -> 8, Digit 2 -> 10, Digit 1 -> 12, Digit 0 -> 14
    let mut max_active_digit = 0;
    for digit_idx in (0..6).rev() {
        let ram_idx = (7 - digit_idx) * 2;
        let segment_data = ram_data[ram_idx];

        // An active digit has segment content beyond just '0' (FONT[0]),
        // OR it contains a decimal point dot (0x80 bit set).
        let digit_segments = segment_data & !0x80; // strip decimal point bit
        let has_dp = (segment_data & 0x80) != 0;

        // FONT[0] is 0x3F. If digit is not '0', or it has a decimal point, or it's digit 0 (units place)
        if (digit_segments != 0x3F && digit_segments != 0) || has_dp || digit_idx == 0 {
            max_active_digit = digit_idx;
            break;
        }
    }

    // 2. Clear out unused leading zero digits to the left of `max_active_digit`
    for digit_idx in (max_active_digit + 1)..6 {
        let ram_idx = (7 - digit_idx) * 2;
        ram_data[ram_idx] = 0;
    }

    // 3. Position the negative sign adjacent to max_active_digit
    if is_negative {
        let sign_digit_idx = max_active_digit + 1;
        if sign_digit_idx < 7 {
            let sign_ram_idx = (7 - sign_digit_idx) * 2;
            ram_data[sign_ram_idx] = 0x40; // '-' segment
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct EditContext {
    pub active_digit: u8,
    pub active_dp: u8,
    pub current_value: i32,
}

impl EditContext {
    #[inline]
    pub fn move_cursor(&mut self) {
        self.active_digit = (self.active_digit + 1) % 7;
    }

    #[inline]
    pub fn move_decimal(&mut self) {
        self.active_dp = (self.active_dp + 1) % 6;
    }

    #[inline]
    pub fn increment_digit(&mut self) {
        if self.active_digit >= 6 {
            return;
        }

        let place_weight = POW10[self.active_digit as usize] as i32;
        let isolated_digit = (self.current_value.abs() / place_weight) % 10;
        let sign = if self.current_value >= 0 { 1 } else { -1 };

        if isolated_digit == 9 {
            self.current_value -= sign * 9 * place_weight;
        } else {
            self.current_value += sign * place_weight;
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct DisplayContext {
    pub value: i32,
    pub decimal_pos: u8,
    pub param_select: u8,
    pub rl1_active: bool,
    pub rl2_active: bool,
}

impl DisplayContext {
    pub fn render(&self) -> [u8; 16] {
        let mut ram_buf = [0u8; 16];
        display_i32(self.value, &mut ram_buf, self.decimal_pos);

        if (1..=5).contains(&self.param_select) {
            let led_idx = (2 * (self.param_select + 2) + 1) as usize;
            if led_idx < ram_buf.len() {
                ram_buf[led_idx] = 1;
            }
        }

        if self.rl1_active {
            ram_buf[3] = 1;
        }
        if self.rl2_active {
            ram_buf[5] = 1;
        }
        ram_buf
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum UiMode {
    Normal,
    Edit(EditContext),
}

pub struct FsmInput {
    pub key_event: Option<KeyEvent>,
    pub current_mode: UiMode,
    pub param_select: u8,
    pub decimal_dp: u8,
    pub preset_count: i32,
    pub scale_factor: ScaleRatio,
    pub scaled_value: i32,
    pub limit_1: i32,
    pub limit_2: i32,
    pub relay_time: u8,
    pub rl1_active: bool,
    pub rl2_active: bool,
}

pub struct FsmOutput {
    pub next_mode: UiMode,
    pub next_param_select: u8,
    pub next_decimal_dp: u8,
    pub next_blink_mask: Option<u16>,
    pub action: DisplayAction,
    pub ram_buf: [u8; 16],
}

#[derive(Debug, PartialEq, Eq)]
pub enum DisplayAction {
    None,
    ApplyPreset,
    ResetEncoder,
    SaveParam(u8, i32),
    SaveScale(i32, u8),
}

// ============================================================================
// CORE STATE MACHINE
// ============================================================================
pub fn step_system_fsm(input: &FsmInput) -> FsmOutput {
    // ------------------------------------------------------------------------
    // 1. Initial FSM State Copy
    // ------------------------------------------------------------------------
    let mut next_mode = input.current_mode;
    let mut next_param_select = input.param_select;
    let mut next_decimal_dp = input.decimal_dp;
    let mut next_blink_mask = None;
    let mut action = DisplayAction::None;

    // ------------------------------------------------------------------------
    // 2. Evaluate Transitions & Apply Mode Logic
    // ------------------------------------------------------------------------
    match input.current_mode {
        UiMode::Normal => {
            handle_normal_mode(
                input,
                &mut next_mode,
                &mut next_param_select,
                &mut next_decimal_dp,
                &mut action,
            );
        }
        UiMode::Edit(edit_ctx) => {
            handle_edit_mode(
                input,
                edit_ctx,
                &mut next_mode,
                &mut next_param_select,
                &mut next_decimal_dp,
                &mut next_blink_mask,
                &mut action,
            );
        }
    }

    // ------------------------------------------------------------------------
    // 3. Resolve Values & Construct Display Snapshot
    // ------------------------------------------------------------------------
    let (display_val, display_dp) = resolve_display_parameters(
        &next_mode,
        next_param_select,
        next_decimal_dp,
        input,
        &action,
    );

    let display = DisplayContext {
        value: display_val,
        decimal_pos: display_dp,
        param_select: next_param_select,
        rl1_active: input.rl1_active,
        rl2_active: input.rl2_active,
    };

    // ------------------------------------------------------------------------
    // 4. Render Hardware RAM Buffer & Apply Display Overrides
    // ------------------------------------------------------------------------
    let mut ram_buf = display.render();

    if let UiMode::Normal = next_mode {
        // Remove leading zeroes and adjust negative sign for standard view
        suppress_leading_zeros(&mut ram_buf);
    } else if display.param_select == 4_u8 {
        // Clear specific segment pairs when editing Parameter 4
        ram_buf[4] = 0;
        ram_buf[6] = 0;
        ram_buf[8] = 0;
        ram_buf[10] = 0;
    }

    // ------------------------------------------------------------------------
    // 5. Package Output State
    // ------------------------------------------------------------------------
    FsmOutput {
        next_mode,
        next_param_select,
        next_decimal_dp,
        next_blink_mask,
        action,
        ram_buf,
    }
}

// --- PRIVATE STATE MACHINE MUTATION ACTIONS ---

fn handle_normal_mode(
    input: &FsmInput,
    next_mode: &mut UiMode,
    next_param_select: &mut u8,
    next_decimal_dp: &mut u8,
    action: &mut DisplayAction,
) {
    match input.key_event {
        Some(KeyEvent::Short(Key::Key1)) => *next_param_select = (*next_param_select + 1) % 6,
        Some(KeyEvent::Short(Key::Key2)) => *next_param_select = 0,
        Some(KeyEvent::Short(Key::Key4)) if *next_param_select == 0 => {
            *action = DisplayAction::ApplyPreset
        }
        Some(KeyEvent::Short(Key::Key5)) if *next_param_select == 0 => {
            *next_decimal_dp = (*next_decimal_dp + 1) % 6
        }
        Some(KeyEvent::Short(Key::Key6)) if *next_param_select == 0 => {
            *action = DisplayAction::ResetEncoder
        }
        Some(KeyEvent::Long(Key::Key3)) if (1..=5).contains(next_param_select) => {
            let initial_val = match *next_param_select {
                1 => input.preset_count,
                2 => input.limit_1,
                3 => input.limit_2,
                4 => input.relay_time as i32,
                5 => input.scale_factor.val as i32,
                _ => 0,
            };
            *next_decimal_dp = input.decimal_dp;
            *next_mode = UiMode::Edit(EditContext {
                active_digit: 0,
                active_dp: input.scale_factor.dp,
                current_value: initial_val,
            });
        }
        _ => {}
    }
}

fn handle_edit_mode(
    input: &FsmInput,
    mut edit_ctx: EditContext,
    next_mode: &mut UiMode,
    next_param_select: &mut u8,
    next_decimal_dp: &mut u8,
    next_blink_mask: &mut Option<u16>,
    action: &mut DisplayAction,
) {
    let blink_bit = 14 - (2 * edit_ctx.active_digit);
    *next_blink_mask = Some(1u16 << blink_bit);
    *next_decimal_dp = edit_ctx.active_dp;

    let led_idx = (2 * (*next_param_select + 2) + 1) as usize;
    // Blink parameter select led to edit -ve sign
    if let Some(mask) = next_blink_mask {
        if *next_param_select < 4_u8 && edit_ctx.active_digit == 6_u8 {
            *mask |= 1 << led_idx;
        } else {
            *mask &= !(1 << led_idx);
        }
    }

    match input.key_event {
        Some(KeyEvent::Short(Key::Key4)) => {
            edit_ctx.move_cursor();
            // Negative sign not applicable to relay_time and scale factor
            match next_param_select {
                // Relay timer max value 99
                4 => {
                    if edit_ctx.active_digit == 2 {
                        edit_ctx.active_digit = 0;
                    }
                }
                // No -ve sign in scale factorS
                5 => {
                    if edit_ctx.active_digit == 6 {
                        edit_ctx.active_digit = 0;
                    }
                }
                _ => {}
            }
            *next_mode = UiMode::Edit(edit_ctx);
        }
        Some(KeyEvent::Short(Key::Key5)) => {
            if *next_param_select == 5 {
                edit_ctx.move_decimal();
                *next_decimal_dp = edit_ctx.active_dp;
            }
            *next_mode = UiMode::Edit(edit_ctx);
        }
        Some(KeyEvent::Short(Key::Key6)) => {
            if edit_ctx.active_digit < 6 {
                edit_ctx.increment_digit();
            } else {
                edit_ctx.current_value *= -1;
            }
            *next_mode = UiMode::Edit(edit_ctx);
        }
        Some(KeyEvent::Short(Key::Key2)) => {
            *next_decimal_dp = input.decimal_dp;
            *next_mode = UiMode::Normal;
        }
        Some(KeyEvent::Long(Key::Key1)) => {
            if *next_param_select == 5 {
                if edit_ctx.current_value == 0 {
                    edit_ctx.current_value = 1;
                    edit_ctx.active_dp = 0;
                }
                *action = DisplayAction::SaveScale(edit_ctx.current_value, edit_ctx.active_dp);
            } else {
                *action = DisplayAction::SaveParam(*next_param_select, edit_ctx.current_value);
            }
            *next_decimal_dp = input.decimal_dp;
            *next_mode = UiMode::Normal;
        }
        _ => {}
    }
}

fn resolve_display_parameters(
    next_mode: &UiMode,
    next_param_select: u8,
    next_decimal_dp: u8,
    input: &FsmInput,
    action: &DisplayAction,
) -> (i32, u8) {
    match *next_mode {
        UiMode::Edit(edit_ctx) => (
            edit_ctx.current_value,
            if next_param_select == 5 {
                edit_ctx.active_dp
            } else {
                0
            },
        ),
        UiMode::Normal => match next_param_select {
            1 => (input.preset_count, 0),
            2 => (input.limit_1, 0),
            3 => (input.limit_2, 0),
            4 => (input.relay_time as i32, 0),
            5 => (input.scale_factor.val as i32, input.scale_factor.dp),
            _ => (
                match action {
                    DisplayAction::ApplyPreset => input.preset_count,
                    DisplayAction::ResetEncoder => 0,
                    _ => input.scaled_value,
                },
                next_decimal_dp,
            ),
        },
    }
}

/// Processes pending UART Modbus traffic and handles node address updates.
pub fn process_uart(
    uart: &mut UartDma,
    modbus: &mut Modbus,
    current_scaled: i32,
    current_addr: u8,
) -> Option<u8> {
    uart.poll();

    if uart.tx_busy() {
        return None;
    }

    modbus.set_address(current_addr);

    let raw_bits = current_scaled as u32;

    let mut registers = HoldingRegisters {
        value_low: (raw_bits & 0xFFFF) as u16,
        value_high: ((raw_bits >> 16) & 0xFFFF) as u16,
        node_address: current_addr as u16,
        new_node_address: current_addr as u16,
    };

    if uart.process_modbus(modbus, &mut registers) {
        let new_addr = registers.new_node_address as u8;
        if new_addr != current_addr && new_addr > 0 && new_addr < 248 {
            modbus.set_address(new_addr);
            return Some(new_addr);
        }
    }

    None
}
