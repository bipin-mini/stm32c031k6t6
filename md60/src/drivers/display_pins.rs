// md60/src/drivers/display_pins.rs

use stm32c0::stm32c031 as pac;
use tm1638_system::Tm1638Pins;

const STB_PIN: u32 = 4; // PA4
const CLK_PIN: u32 = 5; // PA5
const DIO_PIN: u32 = 7; // PA7

pub struct Stm32C0Tm1638Pins;

impl Stm32C0Tm1638Pins {
    pub fn new(gpioa: &pac::GPIOA, rcc: &pac::RCC) -> Self {
        // Enable GPIOA clock
        rcc.iopenr().modify(|_, w| w.gpioaen().set_bit());

        // Configure STB (PA4) and CLK (PA5) as outputs
        gpioa
            .moder()
            .modify(|_, w| w.mode4().output().mode5().output());

        gpioa
            .otyper()
            .modify(|_, w| w.ot4().clear_bit().ot5().clear_bit());

        gpioa
            .ospeedr()
            .modify(|_, w| w.ospeed4().low_speed().ospeed5().low_speed());

        gpioa
            .pupdr()
            .modify(|_, w| w.pupd4().floating().pupd5().floating());

        gpioa.bsrr().write(|w| w.br4().set_bit().br5().set_bit());

        // Configure DIO (PA7) according to your working setup
        gpioa.moder().modify(|_, w| w.mode7().input());
        gpioa.otyper().modify(|_, w| w.ot7().set_bit());
        gpioa.pupdr().modify(|_, w| w.pupd7().pull_up());

        unsafe {
            gpioa.bsrr().write(|w| w.bits(1 << (DIO_PIN + 16))); // Ensure reset bit is set
        }

        Self {}
    }
}

impl Tm1638Pins for Stm32C0Tm1638Pins {
    fn stb_high(&mut self) {
        unsafe {
            (*pac::GPIOA::ptr()).bsrr().write(|w| w.bits(1 << STB_PIN));
        }
    }

    fn stb_low(&mut self) {
        unsafe {
            (*pac::GPIOA::ptr())
                .bsrr()
                .write(|w| w.bits(1 << (STB_PIN + 16)));
        }
    }

    fn clk_high(&mut self) {
        unsafe {
            (*pac::GPIOA::ptr()).bsrr().write(|w| w.bits(1 << CLK_PIN));
        }
    }

    fn clk_low(&mut self) {
        unsafe {
            (*pac::GPIOA::ptr())
                .bsrr()
                .write(|w| w.bits(1 << (CLK_PIN + 16)));
        }
    }

    fn dio_high(&mut self) {
        let gpio = unsafe { &*pac::GPIOA::ptr() };
        gpio.moder().modify(|_, w| w.mode7().input());
    }

    fn dio_low(&mut self) {
        let gpio = unsafe { &*pac::GPIOA::ptr() };
        unsafe {
            gpio.bsrr().write(|w| w.bits(1 << (DIO_PIN + 16)));
        }
        gpio.moder().modify(|_, w| w.mode7().output());
    }

    fn dio_read(&mut self) -> bool {
        (unsafe { (*pac::GPIOA::ptr()).idr().read().bits() & (1 << DIO_PIN) }) != 0
    }
}
