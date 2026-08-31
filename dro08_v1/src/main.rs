#![no_std]
#![no_main]

use panic_halt as _;
use stm32c0::stm32c031 as pac;

use dro08::KeyEvent;
use dro08::UiMode;
use dro08::parameters::load_parameters;
use dro08::storage::eeprom::Eeprom;
use dro08::{Modbus, QuadratureEncoder, RelayController, ScaleRatio, Tm1638, UartDma, bsp};

use rtic::app;
use systick_monotonic::*;

//const DELAY_2MS: u32 = bsp::SYSCLK_HZ / 500; // 2 MHz delay for TM1638 timing

#[app(device = pac, peripherals = true, dispatchers = [RTC, SPI, ADC])]
mod app {

    use super::*;

    #[monotonic(binds = SysTick, default = true, priority = 1)]
    type SysMono = Systick<1000>;

    #[shared]
    struct Shared {
        // Parameters
        preset_count: i32,
        limit_1: i32,
        limit_2: i32,
        relay_time: u8,
        slave_addr: u8,
        scale_factor: ScaleRatio,

        // State
        scaled_value: i32,

        tm1638_ram: Option<[u8; 16]>,
        key_event: Option<KeyEvent>,
        blink_mask: Option<u16>,
        tm1638: Tm1638,
        relay: RelayController,

        rl1_active: bool,
        rl2_active: bool,

        reset_requested: bool,
        preset_requested: bool,
    }

    #[local]
    struct Local {
        uart: UartDma,
        encoder: QuadratureEncoder,
        modbus: Modbus,
        decimal_dp: u8,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let dp = ctx.device;

        bsp::init_clocks(&dp.RCC);
        bsp::init_pins(&dp.GPIOA, &dp.GPIOB, &dp.EXTI);

        let mut eeprom = Eeprom::new(dp.I2C1, &dp.RCC);
        let params = load_parameters(&mut eeprom);

        let mut tm1638 = Tm1638::new();
        let uart = UartDma::new(dp.USART1, &dp.DMA, &dp.DMAMUX, &dp.RCC);
        let modbus = Modbus::new(params.slave_addr);
        let relay = RelayController::new();

        let preset_count = params.preset_count;
        let limit_1 = params.limit_1;
        let limit_2 = params.limit_2;
        let relay_time = params.relay_time;
        let slave_addr = params.slave_addr;
        let decimal_dp = params.decimal_dp;
        let scale_factor = params.scale_factor;
        let scaled_value = params.scaled_value;

        let mono = Systick::new(ctx.core.SYST, bsp::SYSCLK_HZ);

        // All segments & LEDS on
        let mut ram_data = [0xFFu8; 16];
        tm1638.write_display(&ram_data);
        cortex_m::asm::delay(bsp::SYSCLK_HZ);

        // Modbus Address
        ram_data = [0u8; 16];
        dro08::render_i32(slave_addr as i32, &mut ram_data, 0, true);
        tm1638.write_display(&ram_data);
        cortex_m::asm::delay(bsp::SYSCLK_HZ);

        // 5. Ensure the TM1638 internal state updates its display register
        cortex_m::asm::delay(bsp::SYSCLK_HZ);

        let mut encoder = QuadratureEncoder::new(dp.TIM1);

        // Restore encoder count from the loaded scaled value
        encoder.preset(scale_factor.unapply(scaled_value));

        bsp::init_interrupts(&dp.EXTI);

        // Spawn tasks
        encoder_task::spawn().ok();
        modbus_task::spawn().ok();
        console_task::spawn().ok();
        system_fsm_task::spawn().ok();

        (
            Shared {
                preset_count,
                scaled_value,
                limit_1,
                limit_2,
                relay_time,
                scale_factor,
                slave_addr,
                tm1638_ram: None,
                key_event: None,
                blink_mask: None,
                tm1638,
                relay,

                rl1_active: false,
                rl2_active: false,
                reset_requested: false,
                preset_requested: false,
            },
            Local {
                uart,
                encoder,
                modbus,
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
        priority = 2,
        shared = [tm1638, relay],
    )]
    fn power_fail_task(mut ctx: power_fail_task::Context) {
        let exti = unsafe { &*pac::EXTI::ptr() };
        exti.fpr1().write(|w| w.fpif6().set_bit());

        // 1. Disable SysTick timer and its interrupts immediately
        // so it stops waking the CPU from WFI sleep.

        unsafe {
            core::ptr::write_volatile(0xE000_E010 as *mut u32, 0);
        }

        // 2. Instantly kill high-current loads (Relays & TM1638 display driver)
        ctx.shared.relay.lock(|r| r.reset());
        ctx.shared.tm1638.lock(|t| t.set_display(false, 0));

        // 3. Disable Port A clock to save power
        unsafe {
            let rcc = &(*pac::RCC::ptr());
            rcc.iopenr().modify(|_, w| w.gpioaen().clear_bit());
        }

        // 4. Perform your EEPROM / Flash write operations here while
        // the tank capacitor voltage is still stable:
        // dro08::save_parameters_to_eeprom();

        // 5. Halt the CPU core execution completely
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(shared = [
        // Parameters
        preset_count,
        limit_1,
        limit_2,
        relay_time,
        scale_factor,
        slave_addr,
        // State
        scaled_value,
        relay,
        rl1_active,
        rl2_active,
        reset_requested,
        preset_requested
        ],

        local = [encoder],
    priority = 1)]
    fn encoder_task(mut ctx: encoder_task::Context) {
        // Use core::mem::replace inside RTIC resource locks safely
        let reset_req = ctx
            .shared
            .reset_requested
            .lock(|r| core::mem::replace(r, false));

        if reset_req {
            ctx.local.encoder.reset();
            ctx.shared.relay.lock(|r| r.reset());
        }

        let scale = ctx.shared.scale_factor.lock(|s| *s);

        let preset_req = ctx
            .shared
            .preset_requested
            .lock(|p| core::mem::replace(p, false));

        if preset_req {
            let preset_val = ctx.shared.preset_count.lock(|p| *p);
            ctx.local.encoder.preset(scale.unapply(preset_val));
            ctx.shared.relay.lock(|r| r.reset());
        }

        let count = ctx.local.encoder.count();
        let local_scaled = scale.apply(count);

        ctx.shared.scaled_value.lock(|s| {
            *s = local_scaled;
        });

        let l1 = ctx.shared.limit_1.lock(|l1| *l1);
        let l2 = ctx.shared.limit_2.lock(|l2| *l2);
        let t = ctx.shared.relay_time.lock(|t| *t);

        // STEP A: Safely MOVE the controller onto the local stack frame instantly.
        let mut local_relay = ctx.shared.relay.lock(|r| core::mem::take(r));
        // 🔓 Shared Lock is opened instantly! Global interrupts are completely alive.

        // STEP B: Perform heavy threshold checks completely UNLOCKED.
        local_relay.update(local_scaled, l1, l2, t);
        let rl1_state = local_relay.is_rl1_active();
        let rl2_state = local_relay.is_rl2_active();

        // STEP C: Perform hardware GPIO pin modifications completely UNLOCKED.
        if rl1_state {
            local_relay.relay1_on();
        } else {
            local_relay.relay1_off();
        }
        if rl2_state {
            local_relay.relay2_on();
        } else {
            local_relay.relay2_off();
        }

        // STEP D: Move updated controller state straight back into shared memory.
        ctx.shared.relay.lock(|r| *r = local_relay);

        // STEP E: Update active indicator states
        ctx.shared.rl1_active.lock(|r1| *r1 = rl1_state);
        ctx.shared.rl2_active.lock(|r2| *r2 = rl2_state);

        encoder_task::spawn_after(2.millis()).ok();
    }

    #[task(
        priority = 1,
        local = [uart, modbus],
        shared = [slave_addr, scaled_value]
    )]
    fn modbus_task(mut ctx: modbus_task::Context) {
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

        modbus_task::spawn_after(2.millis()).ok();
    }

    #[task(
        priority = 1,
        local = [
            keyboard: dro08::Keyboard = dro08::Keyboard::new(),
            blinker: dro08::Blinker = dro08::Blinker::new(),
            current_ram: [u8; 16] = [0u8; 16],
            last_rendered_ram: [u8; 16] = [0u8; 16],
        ],
        shared = [tm1638, tm1638_ram, blink_mask, key_event],
    )]
    fn console_task(mut ctx: console_task::Context) {
        let mut key_buf = [0u8; 4];
        ctx.shared.tm1638.lock(|tm| {
            tm.read_keys(&mut key_buf);
        });
        let raw_keys = u32::from_le_bytes(key_buf);

        if let Some(event) = ctx.local.keyboard.update(raw_keys) {
            ctx.shared.key_event.lock(|e| *e = Some(event));
        }

        if let Some(ram) = ctx.shared.tm1638_ram.lock(|r| r.take()) {
            *ctx.local.current_ram = ram;
        }

        let active_mask = ctx.shared.blink_mask.lock(|m| *m);

        let mut render_ram = *ctx.local.current_ram;
        ctx.local.blinker.update(&mut render_ram, active_mask);

        if render_ram != *ctx.local.last_rendered_ram {
            ctx.shared.tm1638.lock(|tm| {
                tm.write_display(&render_ram);
            });
            *ctx.local.last_rendered_ram = render_ram;
        }

        console_task::spawn_after(10.millis()).ok();
    }

    #[task(
        priority = 1,
        local = [
            decimal_dp,
            param_select: u8 = 0,
            ui_mode: dro08::UiMode = dro08::UiMode::Normal,
            update_ticks: u8 = 0,
        ],
        shared = [
            scale_factor, scaled_value,
            preset_count, relay_time, limit_1, limit_2,
            key_event, tm1638_ram, blink_mask, reset_requested, preset_requested,
            rl1_active, rl2_active
        ]
    )]
    fn system_fsm_task(mut ctx: system_fsm_task::Context) {
        let fsm_input = dro08::FsmInput {
            key_event: ctx.shared.key_event.lock(|k| k.take()),
            current_mode: *ctx.local.ui_mode,
            param_select: *ctx.local.param_select,
            decimal_dp: *ctx.local.decimal_dp,
            preset_count: ctx.shared.preset_count.lock(|p| *p),
            scale_factor: ctx.shared.scale_factor.lock(|f| *f),
            scaled_value: ctx.shared.scaled_value.lock(|s| *s),
            limit_1: ctx.shared.limit_1.lock(|l1| *l1),
            limit_2: ctx.shared.limit_2.lock(|l2| *l2),
            relay_time: ctx.shared.relay_time.lock(|t| *t),
            rl1_active: ctx.shared.rl1_active.lock(|r1| *r1),
            rl2_active: ctx.shared.rl2_active.lock(|r2| *r2),
        };

        let output = dro08::step_system_fsm(&fsm_input);

        if let UiMode::Normal = output.next_mode {
            if output.next_decimal_dp != *ctx.local.decimal_dp {
                *ctx.local.decimal_dp = output.next_decimal_dp;
            }
        }

        *ctx.local.ui_mode = output.next_mode;
        *ctx.local.param_select = output.next_param_select;

        ctx.shared.blink_mask.lock(|m| *m = output.next_blink_mask);

        if *ctx.local.param_select == 0 {
            *ctx.local.update_ticks += 1;
            if *ctx.local.update_ticks >= 6 {
                ctx.shared
                    .tm1638_ram
                    .lock(|ram| *ram = Some(output.ram_buf));
                *ctx.local.update_ticks = 0;
            }
        } else {
            ctx.shared
                .tm1638_ram
                .lock(|ram| *ram = Some(output.ram_buf));
            *ctx.local.update_ticks = 0;
        }

        match output.action {
            dro08::DisplayAction::ApplyPreset => {
                ctx.shared.preset_requested.lock(|p| *p = true);
            }
            dro08::DisplayAction::ResetEncoder => {
                ctx.shared.reset_requested.lock(|r| *r = true);
            }
            dro08::DisplayAction::SaveScale(new_val, new_dp) => {
                ctx.shared.scale_factor.lock(|s| {
                    s.val = new_val as u32;
                    s.dp = new_dp;
                });
            }
            dro08::DisplayAction::SaveParam(param, val) => match param {
                1 => ctx.shared.preset_count.lock(|p| *p = val),
                2 => ctx.shared.limit_1.lock(|l| *l = val),
                3 => ctx.shared.limit_2.lock(|l| *l = val),
                4 => ctx.shared.relay_time.lock(|t| *t = val as u8),
                _ => {}
            },
            dro08::DisplayAction::None => {}
        }

        system_fsm_task::spawn_after(50.millis()).ok();
    }
}
