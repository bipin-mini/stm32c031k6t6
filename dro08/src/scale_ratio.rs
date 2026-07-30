const POW10: [i64; 6] = [1, 10, 100, 1_000, 10_000, 100_000];

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct ScaleRatio {
    pub val: u32, // 1 to 999_999 (6 digits)
    pub dp: u8,   // 0 to 5 decimal places
}

impl ScaleRatio {
    pub const fn new(val: u32, dp: u8) -> Self {
        Self {
            val: if val == 0 { 1 } else if val > 999_999 { 999_999 } else { val },
            dp: if dp > 5 { 5 } else { dp },
        }
    }

    /// Apply ratio using 64-bit integer arithmetic (zero floating-point math)
    #[inline(always)]
    pub fn apply(&self, raw_count: i32) -> i32 {
        let raw = raw_count as i64;
        let num = self.val as i64;
        let den = POW10[self.dp as usize];

        let scaled = (raw * num) / den;
        scaled as i32
    }
}