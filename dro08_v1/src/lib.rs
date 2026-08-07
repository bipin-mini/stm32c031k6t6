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

// ... (keep rest of ScaleRatio and display_i32 as is)

const POW10: [i64; 6] = [1, 10, 100, 1_000, 10_000, 100_000];

#[derive(Clone, Copy)]
pub struct ScaleRatio {
    pub val: u32,
    pub dp: u8,
}

impl ScaleRatio {
    pub const fn new(val: u32, dp: u8) -> Self {
        Self {
            val: if val == 0 {
                1
            } else if val > 999_999 {
                999_999
            } else {
                val
            },
            dp: if dp > 5 { 5 } else { dp },
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

/// Input snapshot needed to compute menu transitions and display RAM
pub struct DisplayState {
    pub key_event: Option<KeyEvent>,
    pub menu_select: u8,
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

/// Actions required by RTIC task after processing menu input
#[derive(Debug, PartialEq, Eq)]
pub enum DisplayAction {
    None,
    ApplyPreset { raw_target: i32, preset: i32 },
    ResetEncoder,
}

/// Helper to update display state, generate RAM buffer, and signal required actions
pub fn process_display_ui(
    state: &DisplayState,
    menu_select: &mut u8,
    decimal_dp: &mut u8,
) -> ([u8; 16], DisplayAction) {
    let mut action = DisplayAction::None;

    // 1. Process Key Presses & State Transitions
    match state.key_event {
        Some(KeyEvent::Short(Key::Key1)) => {
            *menu_select = (*menu_select + 1) % 6;
        }

        Some(KeyEvent::Short(Key::Key2)) => {
            *menu_select = 0;
        }

        Some(KeyEvent::Short(Key::Key4)) => {
            let raw_target = state.scale_factor.unapply(state.preset_count);
            action = DisplayAction::ApplyPreset {
                raw_target,
                preset: state.preset_count,
            };
        }

        Some(KeyEvent::Short(Key::Key5)) => {
            if *menu_select == 0 {
                *decimal_dp = (*decimal_dp + 1) % 6;
            }
        }

        Some(KeyEvent::Short(Key::Key6)) if *menu_select == 0 => {
            action = DisplayAction::ResetEncoder;
        }

        _ => {}
    }

    // 2. Select Value and Decimal Point to Display
    let (value, dp) = match *menu_select {
        1 => (state.preset_count, 0),
        2 => (state.limit_1, 0),
        3 => (state.limit_2, 0),
        4 => (state.relay_time as i32, 0),
        5 => (state.scale_factor.val as i32, state.scale_factor.dp),
        _ => (
            match action {
                DisplayAction::ApplyPreset { preset, .. } => preset,
                DisplayAction::ResetEncoder => 0,
                DisplayAction::None => state.scaled_value,
            },
            *decimal_dp,
        ),
    };

    // 3. Render TM1638 RAM Buffer
    let mut ram_buf = [0u8; 16];
    display_i32(value, &mut ram_buf, dp);

    // Menu Indicator LEDs
    if (1..=5).contains(menu_select) {
        let led_idx = (2 * (*menu_select + 2) + 1) as usize;
        if led_idx < ram_buf.len() {
            ram_buf[led_idx] = 1;
        }
    }

    // Relay Status LEDs
    if state.rl1_active {
        ram_buf[3] = 1;
    }
    if state.rl2_active {
        ram_buf[5] = 1;
    }

    (ram_buf, action)
}

// lib.rs

/// Processes pending UART Modbus traffic and handles node address updates.
///
/// Returns `Some(u8)` with the new slave address if a valid change was requested,
/// or `None` otherwise.
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
