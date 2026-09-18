#![no_std]

pub mod drivers;
//pub mod protocol;
//pub mod storage;

// --- Re-exports for convenient top-level access ---
//pub use drivers::blink::Blinker;
pub use drivers::bsp;
pub use drivers::display_pins::Stm32C0Tm1638Pins;
pub use tm1638_system::FONT;

/// Combined display renderer and leading zero suppressor.
/// Formats standard and menu display modes into the TM1638 RAM buffer in a single pass.
pub fn render_i32(n: i32, ram_data: &mut [u8; 16], decimal_pos: u8, suppress_zeros: bool) {
    let negative = n < 0;
    let mut value = n.unsigned_abs();

    let mut digits = [0u8; 6];
    for d in digits.iter_mut() {
        *d = (value % 10) as u8;
        value /= 10;
    }

    // Find highest active digit position
    let mut highest_active = 0;
    if suppress_zeros {
        for i in (1..6).rev() {
            if digits[i] != 0 || (decimal_pos > 0 && decimal_pos as usize == i) {
                highest_active = i;
                break;
            }
        }
    } else {
        highest_active = 5;
    }

    // Write segment patterns
    for i in 0..6 {
        let ram_idx = (7 - i) * 2;
        if i <= highest_active || !suppress_zeros {
            let mut seg = FONT[digits[i] as usize];
            if decimal_pos > 0 && (decimal_pos as usize) == i {
                seg |= 0x80;
            }
            ram_data[ram_idx] = seg;
        } else {
            ram_data[ram_idx] = 0;
        }
    }

    // Position negative sign
    if negative {
        if suppress_zeros {
            let sign_digit = highest_active + 1;
            if sign_digit <= 6 {
                ram_data[(7 - sign_digit) * 2] = 0x40;
            }
        } else {
            ram_data[2] = 0x40;
        }
    }
}
