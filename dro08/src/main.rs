#![no_std]
#![no_main]

use panic_halt as _;
use stm32c0::stm32c031 as pac;

mod bsp;
mod utils;
mod drivers {
    pub mod blink;
    pub mod encoder;
    pub mod keyboard;
    pub mod relay;
    pub mod tm1638;
    pub mod uart_dma;
}

mod protocol {
    pub mod modbus;
}

use bsp::SYSCLK_HZ;
use drivers::relay::RelayController;
use rtic::app;
use systick_monotonic::*;
use utils::{ScaleRatio, display_i32};

#[app(device = pac, peripherals = true, dispatchers = [RTC, SPI, ADC])]
mod app {
    use super::*;
    use crate::drivers::{encoder::Encoder, tm1638::Tm1638, uart_dma::UartDma};
    use crate::protocol::modbus::{DEFAULT_ADDRESS, HoldingRegisters, Modbus};

    #[monotonic(binds = SysTick, default = true)]
    type SysMono = Systick<1000>;

    #[shared]
    struct Shared {
        encoder_count: i32,
        preset_count: i32,
        scale_factor: ScaleRatio,
        scaled_value: i32,
        limit_1: i32,
        limit_2: i32,
        relay_time: u8,
        slave_addr: u8,
        tm1638_ram: Option<[u8; 16]>,
        tm1638_keys: Option<u32>,

        rl1_active: bool,
        rl2_active: bool,
        reset_requested: bool,
    }

    #[local]
    struct Local {
        uart: UartDma,
        tm1638: Tm1638,
        encoder: Encoder,
        modbus: Modbus,
        relay: RelayController,
        decimal_dp: u8,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let dp = ctx.device;

        bsp::init_clocks(&dp.RCC);
        bsp::init_pins(&dp.GPIOA, &dp.GPIOB, &dp.EXTI);

        let tm1638 = Tm1638::new();
        let uart = UartDma::new(dp.USART1, &dp.DMA, &dp.DMAMUX, &dp.RCC);
        let modbus = Modbus::new(DEFAULT_ADDRESS);
        let relay = RelayController::new();

        let encoder_count = 0;
        let preset_count = -5000;
        let limit_1 = 100;
        let limit_2 = 200;
        let relay_time = 10;
        let slave_addr = DEFAULT_ADDRESS;
        let decimal_dp = 0;
        let scale_factor = ScaleRatio::new(25, 2);
        let scaled_value = scale_factor.apply(encoder_count);

        let mono = Systick::new(ctx.core.SYST, SYSCLK_HZ);

        cortex_m::asm::delay(9_600_000);

        let encoder = Encoder::new(&dp.GPIOA);
        bsp::init_interrupts(&dp.EXTI);

        // Spawn tasks
        tm1638_task::spawn().ok();
        uart_task::spawn().ok();
        relay_task::spawn().ok();
        fsm_task::spawn().ok();

        (
            Shared {
                encoder_count,
                preset_count,
                scale_factor,
                scaled_value,
                limit_1,
                limit_2,
                relay_time,
                slave_addr,

                tm1638_ram: None,
                tm1638_keys: None,

                rl1_active: false,
                rl2_active: false,
                reset_requested: false,
            },
            Local {
                uart,
                tm1638,
                encoder,
                modbus,
                relay,
                decimal_dp,
            },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(
        binds = EXTI0_1,
        priority = 3,
        shared = [encoder_count],
        local = [encoder],
    )]
    fn exti0_1(mut ctx: exti0_1::Context) {
        let exti = unsafe { &*pac::EXTI::ptr() };
        let gpioa = unsafe { &*pac::GPIOA::ptr() };

        exti.rpr1().write(|w| w.rpif0().set_bit().rpif1().set_bit());
        exti.fpr1().write(|w| w.fpif0().set_bit().fpif1().set_bit());

        let delta = ctx.local.encoder.update(gpioa);
        if delta != 0 {
            // High-priority single-variable lock takes < 3 CPU cycles
            ctx.shared.encoder_count.lock(|count| {
                *count += i32::from(delta);
            });
        }
    }
    #[task(
        binds = EXTI4_15,
        priority = 4,
        shared = [encoder_count],
    )]
    fn exti4_15(mut ctx: exti4_15::Context) {
        let exti = unsafe { &*pac::EXTI::ptr() };
        exti.fpr1().write(|w| w.fpif6().set_bit());

        ctx.shared.encoder_count.lock(|count| {
            *count = -123_456;
        });
    }

#[task(
        priority = 2,
        local = [uart, modbus],
        shared = [slave_addr, scaled_value] 
    )]
    fn uart_task(mut ctx: uart_task::Context) {
        let uart = ctx.local.uart;
        let modbus = ctx.local.modbus;

        let current_scaled = ctx.shared.scaled_value.lock(|sv| *sv);
        let current_addr = ctx.shared.slave_addr.lock(|a| *a);

        uart.poll();

        if !uart.tx_busy() {
            modbus.set_address(current_addr);

            let raw_bits = current_scaled as u32;

            let mut registers = HoldingRegisters {
                value_low: (raw_bits & 0xFFFF) as u16,
                value_high: ((raw_bits >> 16) & 0xFFFF) as u16,
                node_address: current_addr as u16,
                new_node_address: current_addr as u16,
            };

            if uart.process_modbus(modbus, &mut registers) {
                // scaled_value is strictly READ-ONLY over Modbus.
                // We only check and update slave node address changes:
                let new_addr = registers.new_node_address as u8;
                if new_addr != current_addr && new_addr > 0 && new_addr < 248 {
                    ctx.shared.slave_addr.lock(|a| *a = new_addr);
                    modbus.set_address(new_addr);
                }
            }
        }

        uart_task::spawn_after(1.millis()).ok();
    }

    #[task(
        priority = 3,
        local = [relay],
        shared = [
            encoder_count, scale_factor, scaled_value, 
            limit_1, limit_2, relay_time, 
            reset_requested, rl1_active, rl2_active
        ]
    )]
    fn relay_task(mut ctx: relay_task::Context) {
        let relay = ctx.local.relay;

        // 1. Fetch raw count and scale factor
        let raw_count = ctx.shared.encoder_count.lock(|c| *c);
        let scale = ctx.shared.scale_factor.lock(|f| *f);

        // 2. Compute scaled value and publish to Shared memory
        let scale_val = scale.apply(raw_count);
        ctx.shared.scaled_value.lock(|sv| *sv = scale_val);

        // 3. Fetch limits and relay timing
        let (l1, l2, t) = (
            &mut ctx.shared.limit_1,
            &mut ctx.shared.limit_2,
            &mut ctx.shared.relay_time,
        )
            .lock(|l1, l2, t| (*l1, *l2, *t));

        let reset_pressed = ctx.shared.reset_requested.lock(|r| {
            let val = *r;
            *r = false;
            val
        });

        // 4. Update physical relay controller
        relay.update(scale_val, l1, l2, t, reset_pressed);

        let gpiob = unsafe { &*pac::GPIOB::ptr() };
        relay.write_hardware(gpiob);

        let rl1_state = relay.is_rl1_active();
        let rl2_state = relay.is_rl2_active();
        ctx.shared.rl1_active.lock(|r1| *r1 = rl1_state);
        ctx.shared.rl2_active.lock(|r2| *r2 = rl2_state);

        relay_task::spawn_after(1.millis()).ok();
    }

    #[task(
        local = [
            tm1638,
            active_key: u32 = 0,
        ],
        shared = [tm1638_ram, tm1638_keys, reset_requested]
    )]
    fn tm1638_task(mut ctx: tm1638_task::Context) {
        let tm = ctx.local.tm1638;

        let mut key_buf = [0u8; 4];
        tm.read_keys(&mut key_buf);

        let raw_keys = (key_buf[3] as u32) << 24
            | (key_buf[2] as u32) << 16
            | (key_buf[1] as u32) << 8
            | (key_buf[0] as u32);

        let active_key = ctx.local.active_key;

        if raw_keys != 0 && *active_key == 0 {
            use crate::drivers::tm1638::{KEY2, KEY6};
            // Immediate check for reset key press to speed up hardware reaction[cite: 1]
            if raw_keys == KEY2 || raw_keys == KEY6 {
                ctx.shared.reset_requested.lock(|r| *r = true);
            }

            ctx.shared.tm1638_keys.lock(|k| *k = Some(raw_keys));
            *active_key = raw_keys;
        } else if raw_keys == 0 {
            *active_key = 0;
        }

        let ram_data = ctx.shared.tm1638_ram.lock(|ram| *ram);
        if let Some(data) = ram_data {
            tm.write_display(&data);
        }

        tm1638_task::spawn_after(10.millis()).ok();
    }

#[task(
        priority = 1,
        local = [
            menu_select: u8 = 0,
            decimal_dp,
        ],
        shared = [
            encoder_count, scale_factor, scaled_value,
            preset_count, relay_time, limit_1, limit_2,
            tm1638_keys, tm1638_ram, rl1_active, rl2_active,
            reset_requested
        ]
    )]
    fn fsm_task(mut ctx: fsm_task::Context) {
        use crate::drivers::tm1638::{KEY1, KEY2, KEY4, KEY5, KEY6};

        let pressed = ctx.shared.tm1638_keys.lock(|k| k.take()).unwrap_or(0);
        let menu_select = ctx.local.menu_select;
        let decimal_dp = ctx.local.decimal_dp;

        let (preset, scale, scale_val, l1, l2, t) = (
            &mut ctx.shared.preset_count,
            &mut ctx.shared.scale_factor,
            &mut ctx.shared.scaled_value,
            &mut ctx.shared.limit_1,
            &mut ctx.shared.limit_2,
            &mut ctx.shared.relay_time,
        )
            .lock(|p, f, s, l1, l2, t| (*p, *f, *s, *l1, *l2, *t));

        match pressed {
            KEY1 => *menu_select = (*menu_select + 1) % 6,
            KEY2 => *menu_select = 0,
            KEY4 => {
                let raw_target = scale.unapply(preset);
                ctx.shared.encoder_count.lock(|c| *c = raw_target);
                ctx.shared.scaled_value.lock(|sv| *sv = preset);
            }
            KEY5 => {
                if *menu_select == 0 {
                    *decimal_dp = (*decimal_dp + 1) % 6;
                }
            }
            KEY6 => {
                if *menu_select == 0 {
                    ctx.shared.encoder_count.lock(|c| *c = 0);
                    ctx.shared.scaled_value.lock(|sv| *sv = 0);
                }
            }
            _ => {}
        }

        // Select value and decimal point position
        let (value, dp) = match *menu_select {
            1 => (preset, 0),
            2 => (l1, 0),
            3 => (l2, 0),
            4 => (t as i32, 0),
            5 => (scale.val as i32, scale.dp),
            _ => (
                if pressed == KEY4 {
                    preset
                } else if pressed == KEY6 {
                    0
                } else {
                    scale_val
                },
                *decimal_dp,
            ),
        };

        let mut ram_buf = [0u8; 16];
        display_i32(value, &mut ram_buf, dp);

        if *menu_select > 0 && *menu_select <= 5 {
            let led_idx = (2 * (*menu_select + 2) + 1) as usize;
            if led_idx < ram_buf.len() {
                ram_buf[led_idx] |= 1;
            }
        }

        let (rl1_on, rl2_on) =
            (&mut ctx.shared.rl1_active, &mut ctx.shared.rl2_active).lock(|r1, r2| (*r1, *r2));

        if rl1_on {
            ram_buf[1] |= 1;
        }
        if rl2_on {
            ram_buf[3] |= 1;
        }

        ctx.shared.tm1638_ram.lock(|ram| *ram = Some(ram_buf));

        // Fixed 300ms loop period
        fsm_task::spawn_after(300.millis()).ok();
    }
}
