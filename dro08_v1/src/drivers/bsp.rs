use stm32c0::stm32c031 as pac;

pub const SYSCLK_HZ: u32 = 48_000_000;

mod config {
    /// Flash latency required for 48 MHz operation.
    pub const FLASH_LATENCY_48MHZ: u8 = 1;

    /// USART1 alternate function.
    pub const USART_AF: u8 = 1;

    /// I2C1 alternate function.
    pub const I2C_AF: u8 = 6;

    /// Power-fail EXTI mask (EXTI6).
    pub const POWER_FAIL_EXTI_MASK: u32 = 1 << 6;
}

/// Configure system clocks.
pub fn init_clocks(rcc: &pac::RCC) {
    let flash = unsafe { &*pac::FLASH::ptr() };

    // Set flash wait states before shifting clock frequencies
    flash
        .acr()
        .modify(|_, w| unsafe { w.latency().bits(config::FLASH_LATENCY_48MHZ) });

    // HSI divider configuration for high performance
    rcc.cr().modify(|_, w| unsafe { w.hsidiv().bits(0) });
    rcc.cfgr().modify(|_, w| unsafe { w.sw().bits(0) });

    while rcc.cfgr().read().sws().bits() != 0 {}

    // Enable I/O Port Buses
    rcc.iopenr().modify(|_, w| {
        w.gpioaen().set_bit();
        w.gpioben().set_bit()
    });

    // Enable system configuration and TIM1 peripheral clocks
    rcc.apbenr2().modify(|_, w| {
        w.syscfgen().set_bit();
        w.tim1en().set_bit()
    });

    cortex_m::asm::dsb();
}

/// Configure board GPIO and EXTI modes (Interrupts remain masked).
pub fn init_pins(gpioa: &pac::GPIOA, gpiob: &pac::GPIOB, exti: &pac::EXTI) {
    init_usart1_pins(gpioa);
    init_tm1638_pins(gpioa);
    init_i2c1_pins(gpiob);
    init_encoder_pins(gpioa);
    init_relay_pins(gpiob);
    init_power_fail_pin(gpioa, exti);

    cortex_m::asm::dsb();
    cortex_m::asm::isb();
}

/// Flush pending flags and enable application EXTI interrupts.
pub fn init_interrupts(exti: &pac::EXTI) {
    // 1. Clear falling-edge pending flag on line 6
    exti.fpr1().write(|w| w.fpif6().set_bit());

    cortex_m::asm::dsb();

    // 2. Unmask only the power-fail line to avoid drowning the core in encoder transitions
    exti.imr1()
        .modify(|r, w| unsafe { w.bits(r.bits() | config::POWER_FAIL_EXTI_MASK) });

    cortex_m::asm::isb();
}

fn init_usart1_pins(gpioa: &pac::GPIOA) {
    gpioa.moder().modify(|_, w| {
        w.mode9().alternate();
        w.mode10().alternate();
        w.mode12().alternate()
    });

    gpioa.afrh().modify(|_, w| unsafe {
        w.afr(1).bits(config::USART_AF);
        w.afr(2).bits(config::USART_AF);
        w.afr(4).bits(config::USART_AF)
    });

    gpioa.otyper().modify(|_, w| {
        w.ot9().clear_bit();
        w.ot10().clear_bit();
        w.ot12().clear_bit()
    });

    gpioa.ospeedr().modify(|_, w| {
        w.ospeed9().high_speed();
        w.ospeed10().high_speed();
        w.ospeed12().high_speed()
    });

    gpioa.pupdr().modify(|_, w| {
        w.pupd9().floating();
        w.pupd10().floating();
        w.pupd12().floating()
    });
}

fn init_tm1638_pins(gpioa: &pac::GPIOA) {
    gpioa.moder().modify(|_, w| {
        w.mode4().output();
        w.mode5().output()
    });

    gpioa.otyper().modify(|_, w| {
        w.ot4().clear_bit();
        w.ot5().clear_bit()
    });

    gpioa.ospeedr().modify(|_, w| {
        w.ospeed4().low_speed();
        w.ospeed5().low_speed()
    });

    gpioa.pupdr().modify(|_, w| {
        w.pupd4().floating();
        w.pupd5().floating()
    });

    gpioa.bsrr().write(|w| {
        w.br4().set_bit();
        w.br5().set_bit()
    });

    gpioa.moder().modify(|_, w| w.mode7().input());
    gpioa.otyper().modify(|_, w| w.ot7().set_bit());
    gpioa.pupdr().modify(|_, w| w.pupd7().pull_up());
}

fn init_i2c1_pins(gpiob: &pac::GPIOB) {
    gpiob.moder().modify(|_, w| {
        w.mode6().alternate();
        w.mode7().alternate()
    });

    gpiob.afrl().modify(|_, w| unsafe {
        w.afrel6().bits(config::I2C_AF);
        w.afrel7().bits(config::I2C_AF)
    });

    gpiob.otyper().modify(|_, w| {
        w.ot6().open_drain();
        w.ot7().open_drain()
    });

    gpiob.ospeedr().modify(|_, w| {
        w.ospeed6().very_high_speed();
        w.ospeed7().very_high_speed()
    });

    gpiob.pupdr().modify(|_, w| {
        w.pupd6().floating();
        w.pupd7().floating()
    });
}

fn init_encoder_pins(gpioa: &pac::GPIOA) {
    gpioa.moder().modify(|_, w| {
        w.mode0().alternate();
        w.mode1().alternate()
    });

    // AF5 = TIM1_CH1 / TIM1_CH2 (PAC v0.16.0 semantics)
    gpioa.afrl().modify(|_, w| {
        w.afrel0().af5();
        w.afrel1().af5()
    });

    gpioa.pupdr().modify(|_, w| {
        w.pupd0().pull_up();
        w.pupd1().pull_up()
    });
}

fn init_power_fail_pin(gpioa: &pac::GPIOA, exti: &pac::EXTI) {
    gpioa.moder().modify(|_, w| w.mode6().input());
    gpioa.pupdr().modify(|_, w| w.pupd6().floating());

    // Connect EXTI6 to Port A (Bits 0x00 maps to Port A)
    exti.exticr2().write(|w| unsafe { w.bits(0) });

    exti.rtsr1().modify(|_, w| w.rt6().clear_bit()); // Disable rising trigger
    exti.ftsr1().modify(|_, w| w.ft6().set_bit()); // Enable falling trigger
    exti.fpr1().write(|w| w.fpif6().set_bit()); // Clear pending
}

fn init_relay_pins(gpiob: &pac::GPIOB) {
    gpiob.moder().modify(|_, w| {
        w.mode0().output();
        w.mode1().output()
    });

    gpiob.otyper().modify(|_, w| {
        w.ot0().clear_bit();
        w.ot1().clear_bit()
    });

    gpiob.ospeedr().modify(|_, w| {
        w.ospeed0().low_speed();
        w.ospeed1().low_speed()
    });

    gpiob.pupdr().modify(|_, w| {
        w.pupd0().floating();
        w.pupd1().floating()
    });

    // Set Active LOW outputs to HIGH initially (Relays turned OFF)
    gpiob.bsrr().write(|w| {
        w.bs0().set_bit();
        w.bs1().set_bit()
    });
}

pub fn stop_systick() {
    unsafe {
        // Clear the SysTick Control and Status Register to stop the timer
        core::ptr::write_volatile(0xE000_E010 as *mut u32, 0);
    }
}

// In src/bsp.rs
pub fn handle_power_fail_hardware() {
    cortex_m::interrupt::disable();

    let exti = unsafe { &*pac::EXTI::ptr() };
    exti.fpr1().write(|w| w.fpif6().set_bit());

    stop_systick();

    unsafe {
        let rcc = &(*pac::RCC::ptr());
        rcc.iopenr().modify(|_, w| w.gpioaen().clear_bit());
    }
}
