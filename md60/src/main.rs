#![no_std]
#![no_main]

use panic_halt as _;
use stm32c0::stm32c031 as pac;

use md60::Stm32C0Tm1638Pins;
use md60::bsp;
use tm1638_system::ConsoleManager;

use rtic::app;
use systick_monotonic::*;

#[app(device = pac, peripherals = true, dispatchers = [RTC, SPI, ADC])]
mod app {
    use super::*;

    #[monotonic(binds = SysTick, default = true, priority = 1)]
    type SysMono = Systick<1000>;

    #[shared]
    struct Shared {
        console_mgr: ConsoleManager<Stm32C0Tm1638Pins>,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let dp = ctx.device;

        bsp::init_clocks(&dp.RCC);
        //bsp::init_pins(&dp.GPIOB, &dp.EXTI);

        let display_pins = Stm32C0Tm1638Pins::new(&dp.GPIOA, &dp.RCC);
        let mut console_mgr = ConsoleManager::new(display_pins);

        let mono = Systick::new(ctx.core.SYST, bsp::SYSCLK_HZ);

        // Initial test render
        let ram_data = [0xFFu8; 16];
        console_mgr.set_ram(ram_data);
        console_mgr.update();
        cortex_m::asm::delay(bsp::SYSCLK_HZ);

        // Spawn the unified loop
        let _ = system_fsm_task::spawn();

        (Shared { console_mgr }, Local {}, init::Monotonics(mono))
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(
        priority = 1,
        shared = [console_mgr],
    )]
    fn system_fsm_task(mut ctx: system_fsm_task::Context) {
        // Update console manager, poll key events directly from the manager
        ctx.shared.console_mgr.lock(|mgr| {
            mgr.update();

            if let Some(event) = mgr.poll_event() {
                // Handle key event right here in your FSM/UI logic
                let _ = event;
            }
        });

        let _ = system_fsm_task::spawn_after(50.millis());
    }
}
