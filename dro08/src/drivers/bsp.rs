use stm32c0::stm32c031 as pac;

pub const SYSCLK_HZ: u32 = 48_000_000;

mod config {
    /// Flash latency required for 48 MHz operation.
    pub const FLASH_LATENCY_48MHZ: u8 = 1;

    /// USART1 alternate function.
    pub const USART_AF: u8 = 1;

    /// I2C1 alternate function.
    pub const I2C_AF: u8 = 6;

    /// Encoder EXTI mask (EXTI0 + EXTI1).
    pub const ENCODER_EXTI_MASK: u32 = 0x0003;

    /// Power-fail EXTI mask (EXTI6).
    pub const POWER_FAIL_EXTI_MASK: u32 = 1 << 6;
}

/// Configure system clocks.
pub fn init_clocks(rcc: &pac::RCC) {
    let flash = unsafe { &*pac::FLASH::ptr() };

    flash
        .acr()
        .modify(|_, w| unsafe { w.latency().bits(config::FLASH_LATENCY_48MHZ) });

    rcc.cr().modify(|_, w| unsafe { w.hsidiv().bits(0) });

    rcc.cfgr().modify(|_, w| unsafe { w.sw().bits(0) });

    while rcc.cfgr().read().sws().bits() != 0 {}

    rcc.iopenr().modify(|_, w| {
        w.gpioaen().set_bit();
        w.gpioben().set_bit()
    });

    rcc.apbenr2().modify(|_, w| w.syscfgen().set_bit());

    cortex_m::asm::dsb();
}

/// Configure board GPIO and EXTI modes (Interrupts remain masked).
pub fn init_pins(gpioa: &pac::GPIOA, gpiob: &pac::GPIOB, exti: &pac::EXTI) {
    init_usart1_pins(gpioa);
    init_tm1638_pins(gpioa);
    init_i2c1_pins(gpiob);
    init_encoder_pins(gpioa, exti);
    init_relay_pins(gpiob);
    init_power_fail_pin(gpioa, exti);

    cortex_m::asm::dsb();
    cortex_m::asm::isb();
}

/// Flush pending flags and enable application EXTI interrupts.
pub fn init_interrupts(exti: &pac::EXTI) {
    // 1. Clear EXTI0/EXTI1 (rising + falling) and EXTI6 (falling only)
    exti.rpr1().write(|w| {
        w.rpif0().set_bit();
        w.rpif1().set_bit()
    });

    exti.fpr1().write(|w| {
        w.fpif0().set_bit();
        w.fpif1().set_bit();
        w.fpif6().set_bit()
    });

    cortex_m::asm::dsb();

    // 2. Unmask EXTI interrupt lines
    exti.imr1().modify(|r, w| unsafe {
        w.bits(r.bits() | config::ENCODER_EXTI_MASK | config::POWER_FAIL_EXTI_MASK)
    });

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
    // 1. Alternate function mode for PB6 and PB7
    gpiob.moder().modify(|_, w| {
        w.mode6().alternate();
        w.mode7().alternate()
    });

    // 2. Map PB6 and PB7 to I2C1 (AF6 on STM32C031)
    gpiob.afrl().modify(|_, w| {
        unsafe {
            w.afrel6().bits(config::I2C_AF);
            w.afrel7().bits(config::I2C_AF);
        }
        w
    });

    // 3. Open-drain configuration (Mandatory for I2C)
    gpiob.otyper().modify(|_, w| {
        w.ot6().open_drain();
        w.ot7().open_drain()
    });

    // 4. Output Speed
    gpiob.ospeedr().modify(|_, w| {
        w.ospeed6().very_high_speed();
        w.ospeed7().very_high_speed()
    });

    // 5. Internal Pull-up/Pull-down configuration
    gpiob.pupdr().modify(|_, w| {
        w.pupd6().floating(); // Floating assuming external pull-ups exist
        w.pupd7().floating()
    });
}

fn init_encoder_pins(gpioa: &pac::GPIOA, exti: &pac::EXTI) {
    gpioa.moder().modify(|_, w| {
        w.mode0().input();
        w.mode1().input()
    });

    gpioa.pupdr().modify(|_, w| {
        w.pupd0().pull_up();
        w.pupd1().pull_up()
    });

    exti.exticr1().write(|w| unsafe { w.bits(0) });

    exti.rtsr1().modify(|_, w| {
        w.rt0().set_bit();
        w.rt1().set_bit()
    });

    exti.ftsr1().modify(|_, w| {
        w.ft0().set_bit();
        w.ft1().set_bit()
    });

    exti.rpr1().write(|w| {
        w.rpif0().set_bit();
        w.rpif1().set_bit()
    });

    exti.fpr1().write(|w| {
        w.fpif0().set_bit();
        w.fpif1().set_bit()
    });

    // Note: Interrupt unmasking (exti.imr1()) is deferred to init_interrupts()
}

fn init_power_fail_pin(gpioa: &pac::GPIOA, exti: &pac::EXTI) {
    gpioa.moder().modify(|_, w| w.mode6().input());

    gpioa.pupdr().modify(|_, w| w.pupd6().floating());

    exti.exticr2().write(|w| unsafe { w.bits(0) });

    exti.rtsr1().modify(|_, w| w.rt6().clear_bit());

    exti.ftsr1().modify(|_, w| w.ft6().set_bit());

    exti.fpr1().write(|w| w.fpif6().set_bit());

    // Note: Interrupt unmasking (exti.imr1()) is deferred to init_interrupts()
}
fn init_relay_pins(gpiob: &pac::GPIOB) {
    //
    // PB0 = RL1
    // PB1 = RL2
    //
    // Active LOW:
    // HIGH = OFF
    // LOW  = ON
    //

    // Configure as outputs.
    gpiob.moder().modify(|_, w| {
        w.mode0().output();
        w.mode1().output()
    });

    // Push-pull outputs.
    gpiob.otyper().modify(|_, w| {
        w.ot0().clear_bit();
        w.ot1().clear_bit()
    });

    // Low speed is sufficient.
    gpiob.ospeedr().modify(|_, w| {
        w.ospeed0().low_speed();
        w.ospeed1().low_speed()
    });

    // No pull resistors.
    gpiob.pupdr().modify(|_, w| {
        w.pupd0().floating();
        w.pupd1().floating()
    });

    //
    // Both relays OFF.
    // (Active LOW)
    //
    gpiob.bsrr().write(|w| {
        w.bs0().set_bit();
        w.bs1().set_bit()
    });
}
