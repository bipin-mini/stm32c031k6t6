#![no_std]
#![no_main]

use panic_halt as _;
use stm32c0::stm32c031 as pac;

use dro08::storage::parameters::{ADDR_SCALED_VALUE, EepromRequest, Parameters, load_parameters};
use dro08::{
    Eeprom, KeyEvent, Modbus, QuadratureEncoder, RelayController, Tm1638, UartDma, UiMode, bsp,
};

use rtic::app;
use systick_monotonic::*;

#[app(device = pac, peripherals = true, dispatchers = [RTC, SPI, ADC])]
mod app {
    use super::*;

    #[monotonic(binds = SysTick, default = true, priority = 1)]
    type SysMono = Systick<1000>;

    pub struct ControlState {
        pub rl1_active: bool,
        pub rl2_active: bool,
        pub reset_requested: bool,
        pub preset_requested: bool,
    }

    #[shared]
    struct Shared {
        eeprom: Eeprom,
        params: Parameters,
        scaled_value: i32,
        tm1638_ram: Option<[u8; 16]>,
        key_event: Option<KeyEvent>,
        blink_mask: Option<u16>,
        tm1638: Tm1638,
        relay: RelayController,
        control: ControlState,
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

        let decimal_dp = params.decimal_dp;
        let scaled_value = params.scaled_value;

        let mono = Systick::new(ctx.core.SYST, bsp::SYSCLK_HZ);

        // Turn on all segments and LEDs momentarily for testing
        let mut ram_data = [0xFFu8; 16];
        tm1638.write_display(&ram_data);
        cortex_m::asm::delay(bsp::SYSCLK_HZ);

        // Display the configured Modbus address
        ram_data = [0u8; 16];
        dro08::render_slave_address(params.slave_addr, &mut ram_data);
        tm1638.write_display(&ram_data);
        cortex_m::asm::delay(bsp::SYSCLK_HZ);

        let mut encoder = QuadratureEncoder::new(dp.TIM1);
        encoder.preset(params.scale_factor.unapply(scaled_value));

        bsp::init_interrupts(&dp.EXTI);

        let _ = encoder_task::spawn();
        let _ = modbus_task::spawn();
        let _ = console_task::spawn();
        let _ = system_fsm_task::spawn();

        (
            Shared {
                eeprom,
                params,
                scaled_value,
                tm1638_ram: None,
                key_event: None,
                blink_mask: None,
                tm1638,
                relay,
                control: ControlState {
                    rl1_active: false,
                    rl2_active: false,
                    reset_requested: false,
                    preset_requested: false,
                },
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
        shared = [tm1638, relay, scaled_value, eeprom],
    )]
    fn power_fail_task(mut ctx: power_fail_task::Context) {
        ctx.shared.relay.lock(|r| r.reset());
        ctx.shared.tm1638.lock(|t| t.set_display(false, 0));

        bsp::handle_power_fail_hardware();

        let value = ctx.shared.scaled_value.lock(|s| *s);

        ctx.shared.eeprom.lock(|eeprom| {
            dro08::storage::parameters::write_i32(eeprom, ADDR_SCALED_VALUE, value);
        });

        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(
        priority = 1,
        shared = [eeprom]
    )]
    fn eeprom_writer_task(mut ctx: eeprom_writer_task::Context, req: EepromRequest) {
        ctx.shared.eeprom.lock(|eeprom| match req {
            EepromRequest::ScaleFactor { val, dp } => {
                let sf = dro08::storage::parameters::ScaleRatio { val, dp };
                dro08::storage::parameters::write_scale_ratio(
                    eeprom,
                    dro08::storage::parameters::ADDR_SCALE_FACTOR,
                    &sf,
                );
            }
            EepromRequest::PresetCount(val) => {
                dro08::storage::parameters::write_i32(
                    eeprom,
                    dro08::storage::parameters::ADDR_PRESET_COUNT,
                    val,
                );
            }
            EepromRequest::Limit1(val) => {
                dro08::storage::parameters::write_i32(
                    eeprom,
                    dro08::storage::parameters::ADDR_LIMIT_1,
                    val,
                );
            }
            EepromRequest::Limit2(val) => {
                dro08::storage::parameters::write_i32(
                    eeprom,
                    dro08::storage::parameters::ADDR_LIMIT_2,
                    val,
                );
            }
            EepromRequest::RelayTime(val) => {
                dro08::storage::parameters::write_u8(
                    eeprom,
                    dro08::storage::parameters::ADDR_RELAY_TIME,
                    val,
                );
            }
            EepromRequest::DecimalDp(val) => {
                dro08::storage::parameters::write_u8(
                    eeprom,
                    dro08::storage::parameters::ADDR_DECIMAL_DP,
                    val,
                );
            }
            EepromRequest::SlaveAddr(val) => {
                dro08::storage::parameters::write_u8(
                    eeprom,
                    dro08::storage::parameters::ADDR_SLAVE_ADDR,
                    val,
                );
            }
        });
    }

    #[task(
        shared = [params, scaled_value, relay, control],
        local = [encoder],
        priority = 1
    )]
    fn encoder_task(mut ctx: encoder_task::Context) {
        let reset_req = ctx
            .shared
            .control
            .lock(|c| core::mem::replace(&mut c.reset_requested, false));
        if reset_req {
            ctx.local.encoder.reset();
            ctx.shared.relay.lock(|r| r.reset());
        }

        let (scale, preset_val, l1, l2, t) = ctx.shared.params.lock(|p| {
            (
                p.scale_factor,
                p.preset_count,
                p.limit_1,
                p.limit_2,
                p.relay_time,
            )
        });

        let preset_req = ctx
            .shared
            .control
            .lock(|c| core::mem::replace(&mut c.preset_requested, false));
        if preset_req {
            ctx.local.encoder.reset();
            ctx.local.encoder.preset(scale.unapply(preset_val));
            ctx.shared.relay.lock(|r| r.reset());
        }

        let count = ctx.local.encoder.count();
        let local_scaled = scale.apply(count) % 1_000_000;

        ctx.shared.scaled_value.lock(|s| *s = local_scaled);

        let mut local_relay = ctx.shared.relay.lock(|r| core::mem::take(r));
        local_relay.update(local_scaled, l1, l2, t);

        let rl1_state = local_relay.is_rl1_active();
        let rl2_state = local_relay.is_rl2_active();

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

        ctx.shared.relay.lock(|r| *r = local_relay);
        ctx.shared.control.lock(|c| {
            c.rl1_active = rl1_state;
            c.rl2_active = rl2_state;
        });

        let _ = encoder_task::spawn_after(2.millis());
    }

    #[task(
        priority = 1,
        local = [uart, modbus],
        shared = [params, scaled_value]
    )]
    fn modbus_task(mut ctx: modbus_task::Context) {
        let current_scaled = ctx.shared.scaled_value.lock(|sv| *sv);
        let current_addr = ctx.shared.params.lock(|p| p.slave_addr);

        if let Some(new_addr) = dro08::process_uart(
            ctx.local.uart,
            ctx.local.modbus,
            current_scaled,
            current_addr,
        ) {
            ctx.shared.params.lock(|p| p.slave_addr = new_addr);
            let _ = eeprom_writer_task::spawn(EepromRequest::SlaveAddr(new_addr));
        }

        let _ = modbus_task::spawn_after(2.millis());
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

        let _ = console_task::spawn_after(10.millis());
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
            params, scaled_value,
            key_event, tm1638_ram, blink_mask, control
        ]
    )]
    fn system_fsm_task(mut ctx: system_fsm_task::Context) {
        let p = ctx.shared.params.lock(|p| *p);
        let (rl1, rl2) = ctx.shared.control.lock(|c| (c.rl1_active, c.rl2_active));

        let fsm_input = dro08::FsmInput {
            key_event: ctx.shared.key_event.lock(|k| k.take()),
            current_mode: *ctx.local.ui_mode,
            param_select: *ctx.local.param_select,
            decimal_dp: *ctx.local.decimal_dp,
            preset_count: p.preset_count,
            scale_factor: p.scale_factor,
            scaled_value: ctx.shared.scaled_value.lock(|s| *s),
            limit_1: p.limit_1,
            limit_2: p.limit_2,
            relay_time: p.relay_time,
            rl1_active: rl1,
            rl2_active: rl2,
        };

        let output = dro08::step_system_fsm(&fsm_input);

        if let UiMode::Normal = output.next_mode {
            if output.next_decimal_dp != *ctx.local.decimal_dp {
                *ctx.local.decimal_dp = output.next_decimal_dp;
                let _ = eeprom_writer_task::spawn(EepromRequest::DecimalDp(output.next_decimal_dp));
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
                ctx.shared.control.lock(|c| c.preset_requested = true);
            }
            dro08::DisplayAction::ResetEncoder => {
                ctx.shared.control.lock(|c| c.reset_requested = true);
            }
            dro08::DisplayAction::SaveScale(new_val, new_dp) => {
                ctx.shared.params.lock(|p| {
                    p.scale_factor.val = new_val as u32;
                    p.scale_factor.dp = new_dp;
                });
                let _ = eeprom_writer_task::spawn(EepromRequest::ScaleFactor {
                    val: new_val as u32,
                    dp: new_dp,
                });
            }
            dro08::DisplayAction::SaveParam(param, val) => {
                let req = match param {
                    1 => {
                        ctx.shared.params.lock(|p| p.preset_count = val);
                        Some(EepromRequest::PresetCount(val))
                    }
                    2 => {
                        ctx.shared.params.lock(|p| p.limit_1 = val);
                        Some(EepromRequest::Limit1(val))
                    }
                    3 => {
                        ctx.shared.params.lock(|p| p.limit_2 = val);
                        Some(EepromRequest::Limit2(val))
                    }
                    4 => {
                        ctx.shared.params.lock(|p| p.relay_time = val as u8);
                        Some(EepromRequest::RelayTime(val as u8))
                    }
                    _ => None,
                };

                if let Some(r) = req {
                    let _ = eeprom_writer_task::spawn(r);
                }
            }
            dro08::DisplayAction::None => {}
        }

        let _ = system_fsm_task::spawn_after(50.millis());
    }
}
