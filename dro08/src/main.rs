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
    pub mod tm1638;
    pub mod uart_dma;
}

mod protocol {
    pub mod modbus;
}

use bsp::SYSCLK_HZ;
use rtic::app;
use systick_monotonic::*;
use utils::{ScaleRatio, display_i32};

#[app(device = pac, peripherals = true, dispatchers = [RTC, SPI, ADC])]
mod app {
    use super::*;
    use crate::drivers::keyboard::KEY2;
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
    }

    #[local]
    struct Local {
        uart: UartDma,
        tm1638: Tm1638,
        encoder: Encoder,
        modbus: Modbus,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let dp = ctx.device;

        bsp::init_clocks(&dp.RCC);
        bsp::init_pins(&dp.GPIOA, &dp.GPIOB, &dp.EXTI);

        let tm1638 = Tm1638::new();
        let uart = UartDma::new(dp.USART1, &dp.DMA, &dp.DMAMUX, &dp.RCC);
        let modbus = Modbus::new(DEFAULT_ADDRESS);
        let encoder_count = 0;
        let preset_count = -5000;
        let limit_1 = 100;
        let limit_2 = 200;
        let relay_time = 10;
        let scale_factor = ScaleRatio::new(25, 2);
        let scaled_value = scale_factor.apply(encoder_count);

        let mono = Systick::new(ctx.core.SYST, SYSCLK_HZ);

        cortex_m::asm::delay(9_600_000);

        let encoder = Encoder::new(&dp.GPIOA);
        bsp::init_interrupts(&dp.EXTI);

        tm1638_task::spawn().ok();
        uart_task::spawn().ok();
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
                slave_addr: 127,
                tm1638_ram: None,
                tm1638_keys: None,
            },
            Local {
                uart,
                tm1638,
                encoder,
                modbus,
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

    // Updates scaled value and publishes it to Modbus registers.
    // Also handles Modbus register writes to update encoder count and slave address.
    #[task(
        priority = 2,
        local = [uart, modbus],
        shared = [encoder_count, slave_addr, scaled_value, scale_factor]
    )]
    fn uart_task(mut ctx: uart_task::Context) {
        let uart = ctx.local.uart;
        let modbus = ctx.local.modbus;

        // Snapshot state atomically
        let (current_count, current_addr, scale) = (
            &mut ctx.shared.encoder_count,
            &mut ctx.shared.slave_addr,
            &mut ctx.shared.scale_factor,
        )
            .lock(|c, a, s| (*c, *a, *s));

        // Update scaled value for system-wide access
        let current_scaled = scale.apply(current_count);
        ctx.shared.scaled_value.lock(|sv| *sv = current_scaled);

        uart.poll();

        if !uart.tx_busy() {
            modbus.set_address(current_addr);

            // Modbus registers carry the SCALED value
            let raw_bits = current_scaled as u32;

            let mut registers = HoldingRegisters {
                value_low: (raw_bits & 0xFFFF) as u16,
                value_high: ((raw_bits >> 16) & 0xFFFF) as u16,
                node_address: current_addr as u16,
                new_node_address: current_addr as u16,
            };

            if uart.process_modbus(modbus, &mut registers) {
                let incoming_scaled =
                    (((registers.value_high as u32) << 16) | (registers.value_low as u32)) as i32;

                // Check against current_scaled to prevent feedback scaling loop
                if incoming_scaled != current_scaled {
                    let raw_count = scale.unapply(incoming_scaled);
                    (&mut ctx.shared.encoder_count, &mut ctx.shared.scaled_value).lock(|c, sv| {
                        *c = raw_count;
                        *sv = incoming_scaled;
                    });
                }

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
        local = [
            tm1638,
            active_key: u32 = 0, // Task-local key state
        ],
        shared = [tm1638_ram, tm1638_keys]
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

        // Send event ONLY on initial press down transition
        if raw_keys != 0 && *active_key == 0 {
            ctx.shared.tm1638_keys.lock(|k| *k = Some(raw_keys));
            *active_key = raw_keys;
        } else if raw_keys == 0 {
            // Reset when user releases the button
            *active_key = 0;
        }

        // Write display RAM to physical chip
        let ram_data = ctx.shared.tm1638_ram.lock(|ram| *ram);
        if let Some(data) = ram_data {
            tm.write_display(&data);
        }

        tm1638_task::spawn_after(100.millis()).ok();
    }

    #[task(
        priority = 1,
        local = [menu_select: u8 = 0],
        shared = [
            encoder_count, scale_factor, scaled_value,
            preset_count, relay_time, limit_1, limit_2,
            tm1638_keys, tm1638_ram,
        ]
    )]
    fn fsm_task(mut ctx: fsm_task::Context) {
        use crate::drivers::tm1638::{KEY1, KEY4, KEY6};

        // 1. Process key press events
        let pressed = ctx.shared.tm1638_keys.lock(|k| k.take()).unwrap_or(0);
        let menu_select = ctx.local.menu_select;

        // 2. Read state atomically
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
            KEY2 => *menu_select = 0, // Decrement with wrap-around
            KEY4 => {
                // Preset is already scaled -> descale to find raw encoder value
                let raw_target = scale.unapply(preset);
                (&mut ctx.shared.encoder_count, &mut ctx.shared.scaled_value).lock(|c, sv| {
                    *c = raw_target;
                    *sv = preset;
                });
            }
            KEY6 => {
                if *menu_select == 0 {
                    (&mut ctx.shared.encoder_count, &mut ctx.shared.scaled_value).lock(|c, sv| {
                        *c = 0;
                        *sv = 0;
                    });
                }
            }
            _ => {}
        }

        // 3. Match menu page
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
                0,
            ),
        };

        let mut ram_buf = [0u8; 16];
        display_i32(value, &mut ram_buf, dp);

        // Turn on status LED for menu items 1..=5 safely without overwriting digits
        if *menu_select > 0 && *menu_select <= 5 {
            let led_idx = (2 * (*menu_select + 2) + 1) as usize;
            if led_idx < ram_buf.len() {
                ram_buf[led_idx] |= 1;
            }
        }

        ctx.shared.tm1638_ram.lock(|ram| *ram = Some(ram_buf));

        fsm_task::spawn_after(100.millis()).ok();
    }
}
