#![no_std]

pub mod drivers;
pub mod protocol;

use crate::drivers::tm1638::FONT;

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
