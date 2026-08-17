#![no_std]
#![no_main]

use panic_halt as _;
use stm32c0::stm32c031 as pac;

use dro08::KeyEvent;
use dro08::UiMode;
use dro08::{
    DEFAULT_ADDRESS, Modbus, QuadratureEncoder, RelayController, ScaleRatio, Tm1638, UartDma, bsp,
};

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
        preset_count: i32,
        scale_factor: ScaleRatio,
        scaled_value: i32,
        limit_1: i32,
        limit_2: i32,
        relay_time: u8,
        slave_addr: u8,
        tm1638_ram: Option<[u8; 16]>,
        blink_mask: Option<u16>,

        key_event: Option<KeyEvent>,

        rl1_active: bool,
        rl2_active: bool,
        reset_requested: bool,
        preset_requested: bool,
    }

    #[local]
    struct Local {
        uart: UartDma,
        tm1638: Tm1638,
        encoder: QuadratureEncoder,
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

        let preset_count = -5000;
        let limit_1 = 100;
        let limit_2 = 200;
        let relay_time = 0;
        let slave_addr = DEFAULT_ADDRESS;
        let decimal_dp = 0;
        let scale_factor = ScaleRatio::new(25, 2);
        let scaled_value = 0;

        let mono = Systick::new(ctx.core.SYST, bsp::SYSCLK_HZ);

        cortex_m::asm::delay(DELAY_2MS); // Wait for TM1638 to power up

        let encoder = QuadratureEncoder::new(dp.TIM1);
        bsp::init_interrupts(&dp.EXTI);

        // Spawn tasks
        encoder_task::spawn().ok();
        modbus_task::spawn().ok();
        console_task::spawn().ok();
        system_fsm_task::spawn().ok();

        (
            Shared {
                preset_count,
                scale_factor,
                scaled_value,
                limit_1,
                limit_2,
                relay_time,
                slave_addr,

                tm1638_ram: None,
                blink_mask: None,
                key_event: None,

                rl1_active: false,
                rl2_active: false,

                reset_requested: false,
                preset_requested: false,
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
        shared = [tm1638_ram],
    )]
    fn power_fail_task(mut ctx: power_fail_task::Context) {
        let exti = unsafe { &*pac::EXTI::ptr() };
        exti.fpr1().write(|w| w.fpif6().set_bit());
        let mut data = [0u8; 16];
        dro08::display_i32(-123456, &mut data, 3);
        ctx.shared.tm1638_ram.lock(|r| *r = Some(data));
    }

    #[task(shared = [
        scale_factor,
        scaled_value,
        limit_1,
        limit_2,
        relay_time,
        rl1_active,
        rl2_active,
        reset_requested,
        preset_count,
        preset_requested
        ],
        local = [encoder, relay])]
    fn encoder_task(mut ctx: encoder_task::Context) {
        // 1. Check and clear the reset flag
        let mut reset_now = false;
        ctx.shared.reset_requested.lock(|r| {
            if *r {
                reset_now = true;
                *r = false;
            }
        });

        if reset_now {
            ctx.local.encoder.reset();
            ctx.local.relay.reset();
        }

        // 2. Take a quick snapshot of the scale factor first
        let scale = ctx.shared.scale_factor.lock(|s| *s);

        // 3. Check and clear the preset flag
        let mut preset_now = false;
        ctx.shared.preset_requested.lock(|p| {
            if *p {
                preset_now = true;
                *p = false;
            }
        });

        if preset_now {
            let preset_val = ctx.shared.preset_count.lock(|p| *p);
            ctx.local.encoder.preset(scale.unapply(preset_val));
            ctx.local.relay.reset();
        }

        // 4. Sample the encoder hardware (Fully outside of locks)
        let count = ctx.local.encoder.count();

        // 5. Perform FPU calculations (Fully outside of locks)
        let local_scaled = scale.apply(count);

        // 6. Write the final result to shared state
        ctx.shared.scaled_value.lock(|s| *s = local_scaled);

        // 7. Fetch control limits and relay timing
        let l1 = ctx.shared.limit_1.lock(|l1| *l1);
        let l2 = ctx.shared.limit_2.lock(|l2| *l2);
        let t = ctx.shared.relay_time.lock(|t| *t);

        // 8. Update physical relay controller state machine
        ctx.local.relay.update(local_scaled, l1, l2, t);
        let rl1_state = ctx.local.relay.is_rl1_active();
        let rl2_state = ctx.local.relay.is_rl2_active();

        // 9. Drive Physical Relay Pins (Active-Low Logic)
        if rl1_state {
            ctx.local.relay.relay1_on();
        } else {
            ctx.local.relay.relay1_off();
        }

        if rl2_state {
            ctx.local.relay.relay2_on();
        } else {
            ctx.local.relay.relay2_off();
        }

        // 10. Sync local tracking states to RTIC shared properties
        ctx.shared.rl1_active.lock(|r1| *r1 = rl1_state);
        ctx.shared.rl2_active.lock(|r2| *r2 = rl2_state);

        // 11. Re-schedule loop for a high frequency 2ms interval
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
            tm1638,
            keyboard: dro08::Keyboard = dro08::Keyboard::new(),
            blinker: dro08::Blinker = dro08::Blinker::new(),
            current_ram: [u8; 16] = [0u8; 16],
            last_rendered_ram: [u8; 16] = [0u8; 16], // Track previous output
        ],
        shared = [tm1638_ram, blink_mask, key_event],
    )]
    fn console_task(mut ctx: console_task::Context) {
        let tm = ctx.local.tm1638;

        // 1. Process Hardware Keys
        let mut key_buf = [0u8; 4];
        tm.read_keys(&mut key_buf);
        let raw_keys = u32::from_le_bytes(key_buf);

        if let Some(event) = ctx.local.keyboard.update(raw_keys) {
            ctx.shared.key_event.lock(|e| *e = Some(event));
        }

        // 2. Poll new display payload
        if let Some(ram) = ctx.shared.tm1638_ram.lock(|r| r.take()) {
            *ctx.local.current_ram = ram;
        }

        // 3. Fetch active blink mask
        let active_mask = ctx.shared.blink_mask.lock(|m| *m);

        // 4. Prepare frame copy and modify in-place via Blinker
        let mut render_ram = *ctx.local.current_ram;
        ctx.local.blinker.update(&mut render_ram, active_mask);

        // 5. Render output only on changes to save bus bandwidth
        if render_ram != *ctx.local.last_rendered_ram {
            tm.write_display(&render_ram);
            *ctx.local.last_rendered_ram = render_ram;
        }

        // Re-schedule loop
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
            key_event, tm1638_ram,blink_mask,reset_requested, preset_requested,
            rl1_active,rl2_active
        ]
    )]
    fn system_fsm_task(mut ctx: system_fsm_task::Context) {
        // 1. Snapshot all shared inputs using individual low-overhead primitive locks
        let key_event = ctx.shared.key_event.lock(|k| k.take());
        let preset_count = ctx.shared.preset_count.lock(|p| *p);
        let scale_factor = ctx.shared.scale_factor.lock(|f| *f);
        let scaled_value = ctx.shared.scaled_value.lock(|s| *s);
        let limit_1 = ctx.shared.limit_1.lock(|l1| *l1);
        let limit_2 = ctx.shared.limit_2.lock(|l2| *l2);
        let relay_time = ctx.shared.relay_time.lock(|t| *t);
        let rl1_active = ctx.shared.rl1_active.lock(|r1| *r1);
        let rl2_active = ctx.shared.rl2_active.lock(|r1| *r1);

        // 2. Package current hardware states for the library FSM
        let fsm_input = dro08::FsmInput {
            key_event,
            current_mode: *ctx.local.ui_mode,
            param_select: *ctx.local.param_select,
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

        // 3. Compute next states via pure library FSM logic
        let output = dro08::step_system_fsm(&fsm_input);

        // 4. Update local decimal position state (only for normal mode)
        if let UiMode::Normal = output.next_mode {
            if output.next_decimal_dp != *ctx.local.decimal_dp {
                *ctx.local.decimal_dp = output.next_decimal_dp;
            }
        }
        // 5. Persist updated values back to zero-cost local registers
        *ctx.local.ui_mode = output.next_mode;
        *ctx.local.param_select = output.next_param_select;

        // 6. Update the shared blink mask immediately (responsiveness at 50ms)
        ctx.shared.blink_mask.lock(|m| *m = output.next_blink_mask);

        // 7. Corrected conditional TM1638 RAM updates
        if *ctx.local.param_select == 0 {
            // Primary readout screen: Accumulate ticks up to 300ms (6 ticks * 50ms)
            *ctx.local.update_ticks += 1;
            if *ctx.local.update_ticks >= 6 {
                ctx.shared
                    .tm1638_ram
                    .lock(|ram| *ram = Some(output.ram_buf));
                *ctx.local.update_ticks = 0;
            }
        } else {
            // Menu parameter screens (1..=5): Bypass timer completely for instant display updates
            ctx.shared
                .tm1638_ram
                .lock(|ram| *ram = Some(output.ram_buf));
            *ctx.local.update_ticks = 0; // Clear so mode 0 starts clean upon return
        }

        // 8. Dispatch requested hardware actions
        match output.action {
            dro08::DisplayAction::ApplyPreset => {
                ctx.shared.preset_requested.lock(|p| *p = true);
            }
            dro08::DisplayAction::ResetEncoder => {
                ctx.shared.reset_requested.lock(|r| *r = true);
            }
            dro08::DisplayAction::SaveScale(new_val, new_dp) => {
                // Safely write the validated parameters back into the shared runtime ratio
                ctx.shared.scale_factor.lock(|s| {
                    s.val = new_val as u32;
                    s.dp = new_dp;
                });
                // todo!: Add EEPROM flash write logic here if needed
            }
            dro08::DisplayAction::SaveParam(param, val) => {
                match param {
                    1 => ctx.shared.preset_count.lock(|p| *p = val),
                    2 => ctx.shared.limit_1.lock(|l| *l = val),
                    3 => ctx.shared.limit_2.lock(|l| *l = val),
                    4 => ctx.shared.relay_time.lock(|t| *t = val as u8),
                    _ => {}
                }
                // todo!: Add EEPROM flash write logic here if needed
            }
            dro08::DisplayAction::None => {}
        }

        // Run loop at a high-speed 50ms cadence for snappy button filtering
        system_fsm_task::spawn_after(50.millis()).ok();
    }
}
