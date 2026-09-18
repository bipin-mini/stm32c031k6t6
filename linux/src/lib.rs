#![no_std]

// 7-segment font (abcdefg)
pub const FONT: [u8; 16] = [
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F, // 9
    0x77, // A
    0x7C, // b
    0x39, // C
    0x5E, // d
    0x79, // E
    0x71, // F
];



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

