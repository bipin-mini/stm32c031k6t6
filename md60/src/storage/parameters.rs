//! Application EEPROM Parameters
//!
//! The EEPROM driver remains generic.
//! This module contains application-specific parameters,
//! EEPROM addresses, defaults and load/save functions.

use crate::drivers::eeprom::Eeprom;
use crate::protocol::modbus::DEFAULT_ADDRESS;

pub const POW10: [i64; 6] = [1, 10, 100, 1_000, 10_000, 100_000];

// -------------------------------------------------------------------------
// ScaleRatio
// -------------------------------------------------------------------------

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

    #[inline(always)]
    pub fn apply(&self, raw_count: i32) -> i32 {
        let raw = raw_count as i64;
        let num = self.val as i64;
        let den = POW10[self.dp as usize] as i64;
        ((raw * num) / den) as i32
    }

    #[inline(always)]
    pub fn unapply(&self, scaled_val: i32) -> i32 {
        let scaled = scaled_val as i64;
        let num = POW10[self.dp as usize] as i64;
        let den = self.val as i64;
        let half_den = den / 2;

        let raw = if scaled >= 0 {
            (scaled * num + half_den) / den
        } else {
            (scaled * num - half_den) / den
        };
        (raw) as i32
    }
}

// -------------------------------------------------------------------------
// Application Parameters
// -------------------------------------------------------------------------

#[derive(Clone, Copy)]
pub struct Parameters {
    // To be written on edit
    pub preset_count: i32,
    pub limit_1: i32,
    pub limit_2: i32,
    pub relay_time: u8,
    pub slave_addr: u8,
    pub decimal_dp: u8,
    pub scale_factor: ScaleRatio,

    // To be written on power fail
    pub scaled_value: i32,
}

#[derive(Clone, Copy)]
pub enum EepromRequest {
    ScaleFactor { val: u32, dp: u8 },
    PresetCount(i32),
    Limit1(i32),
    Limit2(i32),
    RelayTime(u8),
    DecimalDp(u8),
    SlaveAddr(u8),
}

// -------------------------------------------------------------------------
// Default values
// -------------------------------------------------------------------------

pub const DEFAULT_PRESET_COUNT: i32 = -5000;
pub const DEFAULT_LIMIT_1: i32 = 100;
pub const DEFAULT_LIMIT_2: i32 = 200;

pub const DEFAULT_RELAY_TIME: u8 = 0;
pub const DEFAULT_SLAVE_ADDR: u8 = DEFAULT_ADDRESS;
pub const DEFAULT_DECIMAL_DP: u8 = 0;

pub const DEFAULT_SCALE_FACTOR: ScaleRatio = ScaleRatio::new(25, 2);

// Runtime value
pub const DEFAULT_SCALED_VALUE: i32 = 0;

// -------------------------------------------------------------------------
// EEPROM addresses
//
// Each parameter starts on an 8-byte EEPROM page.
// -------------------------------------------------------------------------

pub const ADDR_PRESET_COUNT: u8 = 8;
pub const ADDR_LIMIT_1: u8 = 16;
pub const ADDR_LIMIT_2: u8 = 24;

pub const ADDR_RELAY_TIME: u8 = 32;
pub const ADDR_SLAVE_ADDR: u8 = 40;
pub const ADDR_DECIMAL_DP: u8 = 48;

pub const ADDR_SCALE_FACTOR: u8 = 56;
pub const ADDR_SCALED_VALUE: u8 = 64;

// -------------------------------------------------------------------------
// EEPROM initialization marker
//
// Page 0 is reserved for EEPROM initialization information.
// -------------------------------------------------------------------------

const MAGIC_ADDR: u8 = 0;

const MAGIC: u32 = 0x5343_414C;

// -------------------------------------------------------------------------
// Primitive read/write helpers
// -------------------------------------------------------------------------

pub fn write_i32(eeprom: &mut Eeprom, addr: u8, value: i32) {
    eeprom.write(addr, &value.to_le_bytes());
}

pub fn read_i32(eeprom: &mut Eeprom, addr: u8) -> i32 {
    let mut buf = [0u8; 4];

    eeprom.read(addr, &mut buf);

    i32::from_le_bytes(buf)
}

pub fn write_u8(eeprom: &mut Eeprom, addr: u8, value: u8) {
    eeprom.write_byte(addr, value);
}

pub fn read_u8(eeprom: &mut Eeprom, addr: u8) -> u8 {
    eeprom.read_byte(addr)
}

// -------------------------------------------------------------------------
// ScaleRatio read/write
// -------------------------------------------------------------------------

pub fn write_scale_ratio(eeprom: &mut Eeprom, addr: u8, ratio: &ScaleRatio) {
    let mut buf = [0u8; 5];

    buf[0..4].copy_from_slice(&ratio.val.to_le_bytes());
    buf[4] = ratio.dp;

    eeprom.write(addr, &buf);
}

fn read_scale_ratio(eeprom: &mut Eeprom, addr: u8) -> ScaleRatio {
    let mut buf = [0u8; 5];

    eeprom.read(addr, &mut buf);

    ScaleRatio {
        val: u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]),
        dp: buf[4],
    }
}

// -------------------------------------------------------------------------
// Magic number
// -------------------------------------------------------------------------

fn write_magic(eeprom: &mut Eeprom) {
    eeprom.write(MAGIC_ADDR, &MAGIC.to_le_bytes());
}

fn read_magic(eeprom: &mut Eeprom) -> u32 {
    let mut buf = [0u8; 4];

    eeprom.read(MAGIC_ADDR, &mut buf);

    u32::from_le_bytes(buf)
}

// -------------------------------------------------------------------------
// Default Parameters
// -------------------------------------------------------------------------

pub const fn default_parameters() -> Parameters {
    Parameters {
        preset_count: DEFAULT_PRESET_COUNT,
        limit_1: DEFAULT_LIMIT_1,
        limit_2: DEFAULT_LIMIT_2,

        relay_time: DEFAULT_RELAY_TIME,
        slave_addr: DEFAULT_SLAVE_ADDR,
        decimal_dp: DEFAULT_DECIMAL_DP,

        scale_factor: DEFAULT_SCALE_FACTOR,

        scaled_value: DEFAULT_SCALED_VALUE,
    }
}

// -------------------------------------------------------------------------
// Read Parameters
// -------------------------------------------------------------------------

pub fn read_parameters(eeprom: &mut Eeprom) -> Parameters {
    Parameters {
        preset_count: read_i32(eeprom, ADDR_PRESET_COUNT),

        limit_1: read_i32(eeprom, ADDR_LIMIT_1),

        limit_2: read_i32(eeprom, ADDR_LIMIT_2),

        relay_time: read_u8(eeprom, ADDR_RELAY_TIME),

        slave_addr: read_u8(eeprom, ADDR_SLAVE_ADDR),

        decimal_dp: read_u8(eeprom, ADDR_DECIMAL_DP),

        scale_factor: read_scale_ratio(eeprom, ADDR_SCALE_FACTOR),

        scaled_value: read_i32(eeprom, ADDR_SCALED_VALUE),
    }
}

// -------------------------------------------------------------------------
// Write Parameters
//
// MAGIC is deliberately written LAST.
// -------------------------------------------------------------------------

pub fn write_parameters(eeprom: &mut Eeprom, params: &Parameters) {
    write_i32(eeprom, ADDR_PRESET_COUNT, params.preset_count);

    write_i32(eeprom, ADDR_LIMIT_1, params.limit_1);

    write_i32(eeprom, ADDR_LIMIT_2, params.limit_2);

    write_i32(eeprom, ADDR_SCALED_VALUE, params.limit_2);

    write_u8(eeprom, ADDR_RELAY_TIME, params.relay_time);

    write_u8(eeprom, ADDR_SLAVE_ADDR, params.slave_addr);

    write_u8(eeprom, ADDR_DECIMAL_DP, params.decimal_dp);

    write_scale_ratio(eeprom, ADDR_SCALE_FACTOR, &params.scale_factor);

    // IMPORTANT:
    // Write MAGIC only after all parameters have been written.
    write_magic(eeprom);
}

// -------------------------------------------------------------------------
// Load Parameters
//
// If EEPROM is blank/uninitialized:
//
//     1. Create default parameters
//     2. Write defaults to EEPROM
//     3. Write MAGIC last
//     4. Return defaults
//
// Otherwise:
//
//     Read parameters from EEPROM
// -------------------------------------------------------------------------

pub fn load_parameters(eeprom: &mut Eeprom) -> Parameters {
    if read_magic(eeprom) != MAGIC {
        // EEPROM is blank or invalid.
        let defaults = default_parameters();

        // Write default values.
        write_parameters(eeprom, &defaults);

        // Return the same defaults immediately.
        defaults
    } else {
        // EEPROM already initialized.
        read_parameters(eeprom)
    }
}

/*
```

### Startup

Your application code can now simply do:

```rust
let mut eeprom = Eeprom::new(i2c1, &rcc);

let mut params = load_parameters(&mut eeprom);
```

The first boot behaves like:

```text
EEPROM
   │
   ▼
read MAGIC
   │
   ├── MAGIC valid ──────► read parameters
   │
   └── MAGIC invalid
           │
           ▼
      create defaults
           │
           ▼
      write parameters
           │
           ▼
       write MAGIC
           │
           ▼
       use defaults
```

This is a good arrangement for your 24C02 because **page 0 is reserved for metadata and every actual parameter begins on its own 8-byte page**.

One additional improvement I would make later is adding a **parameter version number alongside the magic**. That will let you change the `Parameters` structure in a future firmware version without accidentally interpreting an old EEPROM layout as the new one.
*/
