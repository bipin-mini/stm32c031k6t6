#![no_std]
#![no_main]

use panic_halt as _;
use stm32c0::stm32c031 as pac;

use dro08::KeyEvent;
use dro08::{DEFAULT_ADDRESS, Encoder, Modbus, RelayController, ScaleRatio, Tm1638, UartDma, bsp};

use rtic::app;
use systick_monotonic::*;

const DELAY_2MS: u32 = bsp::SYSCLK_HZ / 500; // 2 MHz delay for TM1638 timing

#[app(device = pac, peripherals = true, dispatchers = [RTC, SPI, ADC])]
mod app {
    use super::*;

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
        key_event: Option<KeyEvent>,

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

        dro08::bsp::init_clocks(&dp.RCC);
        dro08::bsp::init_pins(&dp.GPIOA, &dp.GPIOB, &dp.EXTI);

        let tm1638 = Tm1638::new();
        let uart = UartDma::new(dp.USART1, &dp.DMA, &dp.DMAMUX, &dp.RCC);
        let modbus = Modbus::new(DEFAULT_ADDRESS);
        let relay = RelayController::new();

        let encoder_count = 0;
        let preset_count = -5000;
        let limit_1 = 100;
        let limit_2 = 200;
        let relay_time = 0;
        let slave_addr = DEFAULT_ADDRESS;
        let decimal_dp = 0;
        let scale_factor = ScaleRatio::new(25, 2);
        let scaled_value = scale_factor.apply(encoder_count);

        let mono = Systick::new(ctx.core.SYST, bsp::SYSCLK_HZ);

        cortex_m::asm::delay(DELAY_2MS); // Wait for TM1638 to power up

        let encoder = Encoder::new(&dp.GPIOA);
        bsp::init_interrupts(&dp.EXTI);

        // Spawn tasks
        relay_task::spawn().ok();
        uart_task::spawn().ok();
        tm1638_task::spawn().ok();
        display_refresh_task::spawn().ok();

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
                key_event: None,

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
        priority = 2,
        local = [relay],
        shared = [
            encoder_count, scale_factor, scaled_value,limit_1,
            limit_2, relay_time,reset_requested, rl1_active, rl2_active
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
        priority = 2,
        local = [uart, modbus],
        shared = [slave_addr, scaled_value]
    )]
    fn uart_task(mut ctx: uart_task::Context) {
        let current_scaled = ctx.shared.scaled_value.lock(|sv| *sv);
        let current_addr = ctx.shared.slave_addr.lock(|a| *a);

        if let Some(new_addr) = dro08::process_uart(
            ctx.local.uart,
            ctx.local.modbus,
            current_scaled,
            current_addr,
        ) {
            ctx.shared.slave_addr.lock(|a| *a = new_addr);
        }

        uart_task::spawn_after(1.millis()).ok();
    }

    #[task(
        priority = 1,
    local = [
        tm1638,
        keyboard: dro08::Keyboard = dro08::Keyboard::new(),
    ],
    shared = [tm1638_ram, key_event, reset_requested]
)]
    fn tm1638_task(mut ctx: tm1638_task::Context) {
        let tm = ctx.local.tm1638;

        let mut key_buf = [0u8; 4];
        tm.read_keys(&mut key_buf);

        let raw_keys = u32::from_le_bytes(key_buf);

        if let Some(event) = ctx.local.keyboard.update(raw_keys) {
            use dro08::{Key, KeyEvent};

            ctx.shared.key_event.lock(|e| *e = Some(event));

            if matches!(
                event,
                KeyEvent::Short(Key::Key2) | KeyEvent::Short(Key::Key6)
            ) {
                ctx.shared.reset_requested.lock(|r| *r = true);
            }
        }

        let ram_data = ctx.shared.tm1638_ram.lock(|ram| ram.take());

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
        key_event, tm1638_ram, rl1_active, rl2_active,
        reset_requested
    ]
)]
    fn display_refresh_task(mut ctx: display_refresh_task::Context) {
        let key_event = ctx.shared.key_event.lock(|k| k.take());

        // 1. Snapshot shared state
        let (preset_count, scale_factor, scaled_value, limit_1, limit_2, relay_time) = (
            &mut ctx.shared.preset_count,
            &mut ctx.shared.scale_factor,
            &mut ctx.shared.scaled_value,
            &mut ctx.shared.limit_1,
            &mut ctx.shared.limit_2,
            &mut ctx.shared.relay_time,
        )
            .lock(|p, f, s, l1, l2, t| (*p, *f, *s, *l1, *l2, *t));

        let (rl1_active, rl2_active) =
            (&mut ctx.shared.rl1_active, &mut ctx.shared.rl2_active).lock(|r1, r2| (*r1, *r2));

        let state = dro08::DisplayState {
            key_event,
            menu_select: *ctx.local.menu_select,
            decimal_dp: *ctx.local.decimal_dp,
            preset_count,
            scale_factor,
            scaled_value,
            limit_1,
            limit_2,
            relay_time,
            rl1_active,
            rl2_active,
        };

        // 2. Compute display RAM and state actions in lib
        let (ram_buf, action) =
            dro08::process_display_ui(&state, ctx.local.menu_select, ctx.local.decimal_dp);

        // 3. Execute actions on RTIC shared state
        match action {
            dro08::DisplayAction::ApplyPreset { raw_target, preset } => {
                ctx.shared.encoder_count.lock(|c| *c = raw_target);
                ctx.shared.scaled_value.lock(|sv| *sv = preset);
            }
            dro08::DisplayAction::ResetEncoder => {
                ctx.shared.encoder_count.lock(|c| *c = 0);
                ctx.shared.scaled_value.lock(|sv| *sv = 0);
                ctx.shared.reset_requested.lock(|r| *r = true);
            }
            dro08::DisplayAction::None => {}
        }

        // 4. Update TM1638 RAM buffer
        ctx.shared.tm1638_ram.lock(|ram| *ram = Some(ram_buf));

        display_refresh_task::spawn_after(300.millis()).ok();
    }
}
