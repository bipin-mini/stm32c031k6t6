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
    use crate::drivers::{encoder::Encoder, tm1638::Tm1638, uart_dma::UartDma};
    use crate::protocol::modbus::{DEFAULT_ADDRESS, HoldingRegisters, Modbus};

    #[monotonic(binds = SysTick, default = true)]
    type SysMono = Systick<1000>;

    #[shared]
    struct Shared {
        encoder_count: i32,
        scale_factor: ScaleRatio,
        limit_1: i32,
        limit_2: i32,
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
        let scale_factor = ScaleRatio::new(5, 1);

        let mono = Systick::new(ctx.core.SYST, SYSCLK_HZ);

        cortex_m::asm::delay(9_600_000);

        let encoder = Encoder::new(&dp.GPIOA);
        bsp::init_interrupts(&dp.EXTI);

        tm1638_task::spawn().ok();
        uart_task::spawn().ok();
        fsm_task::spawn().ok();

        (
            Shared {
                encoder_count: 0,
                scale_factor,
                limit_1: 100,
                limit_2: 200,
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
        local = [tm1638],
        shared = [tm1638_ram, tm1638_keys]
    )]
    fn tm1638_task(mut ctx: tm1638_task::Context) {
        let tm = ctx.local.tm1638;

        // 1. Read keys from TM1638 hardware
        let mut key_buf = [0u8; 4];
        tm.read_keys(&mut key_buf);

        // Convert 4 bytes into 32-bit key mask
        let current_keys = (key_buf[3] as u32) << 24
            | (key_buf[2] as u32) << 16
            | (key_buf[1] as u32) << 8
            | (key_buf[0] as u32);

        // Update shared key state for FSM task to process
        let keys_option = (current_keys != 0).then_some(current_keys);
        ctx.shared.tm1638_keys.lock(|k| *k = keys_option);
        // 2. Write display RAM buffer to physical TM1638 if available
        let ram_data = ctx.shared.tm1638_ram.lock(|ram| *ram);
        if let Some(data) = ram_data {
            tm.write_display(&data);
        }

        // Reschedule task to run every 100 ms
        tm1638_task::spawn_after(100.millis()).ok();
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

    #[task(
        priority = 2,
        local = [uart, modbus],
        shared = [encoder_count, slave_addr]
    )]
    fn uart_task(mut ctx: uart_task::Context) {
        let uart = ctx.local.uart;
        let modbus = ctx.local.modbus;

        uart.poll();

        if !uart.tx_busy() {
            let current_count = ctx.shared.encoder_count.lock(|c| *c);
            let current_addr = ctx.shared.slave_addr.lock(|a| *a);

            modbus.set_address(current_addr);

            let raw_bits = current_count as u32;

            let mut registers = HoldingRegisters {
                value_low: (raw_bits & 0xFFFF) as u16,
                value_high: ((raw_bits >> 16) & 0xFFFF) as u16,
                node_address: current_addr as u16,
                new_node_address: current_addr as u16,
            };

            let responded = uart.process_modbus(modbus, &mut registers);

            if responded {
                let new_raw = ((registers.value_high as u32) << 16) | (registers.value_low as u32);
                let new_count = new_raw as i32;

                if new_count != current_count {
                    ctx.shared.encoder_count.lock(|c| *c = new_count);
                }

                let new_addr = registers.node_address as u8;
                if new_addr != current_addr {
                    ctx.shared.slave_addr.lock(|a| *a = new_addr);
                    modbus.set_address(new_addr);
                }
            }
        }

        uart_task::spawn_after(1.millis()).ok();
    }

    #[task(
        priority = 1,
        local = [
            last_keys: u32 = 0,
            menu_select: u8 = 0,
        ],
        shared = [
            encoder_count,
            scale_factor,
            limit_1,
            limit_2,
            tm1638_keys,
            tm1638_ram,
        ]
    )]
    fn fsm_task(mut ctx: fsm_task::Context) {
        use crate::drivers::tm1638::{KEY1, KEY3, KEY4};
        use rtic::Mutex;

        // 1. Read and clear (take) the shared key Option in a single atomic lock
        let raw_keys = ctx.shared.tm1638_keys.lock(|k| k.take()).unwrap_or(0);

        // 2. Rising-edge detection for single button presses
        let last_keys = ctx.local.last_keys;
        let pressed = raw_keys & !*last_keys;
        *last_keys = raw_keys;

        // 3. Read shared DRO parameters safely
        let (count, scale, l1, l2) = (
            &mut ctx.shared.encoder_count,
            &mut ctx.shared.scale_factor,
            &mut ctx.shared.limit_1,
            &mut ctx.shared.limit_2,
        )
            .lock(|c, s, l1, l2| (*c, *s, *l1, *l2));

        // 4. Handle Button Actions
        if pressed == KEY1 {
            // KEY1: Reset live encoder count to zero
            ctx.shared.encoder_count.lock(|c| *c = 0);
        }

        if pressed == KEY4 {
            // KEY4: Cycle parameter views (0 = Limit 1, 1 = Limit 2, 2 = Scale Factor)
            *ctx.local.menu_select = (*ctx.local.menu_select + 1) % 3;
        }

        // 5. Select value to render based on key active state
        let preset_val: i32 = 5000; // Sample preset value
        let display_val: i32 = if (raw_keys & KEY3) != 0 {
            // KEY3 held: Display Preset Value
            preset_val
        } else if (raw_keys & KEY4) != 0 {
            // KEY4 held/pressed: Display selected parameter
            match *ctx.local.menu_select {
                0 => l1,
                1 => l2,
                _ => scale.val as i32,
            }
        } else {
            // Default: Display live scaled position
            scale.apply(count)
        };

        // 6. Format and publish to shared display RAM
        let mut ram_buf = [0u8; 16];
        display_i32(display_val, &mut ram_buf);

        ctx.shared.tm1638_ram.lock(|ram| *ram = Some(ram_buf));

        // Reschedule task every 100 ms
        fsm_task::spawn_after(100.millis()).ok();
    }
}
