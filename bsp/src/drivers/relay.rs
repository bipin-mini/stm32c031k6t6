use stm32c0::stm32c031 as pac;

#[derive(Copy, Clone, Eq, PartialEq)]
pub enum Relay {
    RL1,
    RL2,
}

pub struct RelayDriver;

impl RelayDriver {
    pub const fn new() -> Self {
        Self
    }

    #[inline(always)]
    fn gpio() -> pac::GPIOB {
        unsafe { pac::Peripherals::steal().GPIOB }
    }

    pub fn on(&self, relay: Relay) {
        let gpio = Self::gpio();

        match relay {
            Relay::RL1 => gpio.bsrr().write(|w| w.br0().set_bit()),
            Relay::RL2 => gpio.bsrr().write(|w| w.br1().set_bit()),
        };
    }

    pub fn off(&self, relay: Relay) {
        let gpio = Self::gpio();

        match relay {
            Relay::RL1 => gpio.bsrr().write(|w| w.bs0().set_bit()),
            Relay::RL2 => gpio.bsrr().write(|w| w.bs1().set_bit()),
        };
    }

    pub fn set(&self, relay: Relay, on: bool) {
        if on {
            self.on(relay);
        } else {
            self.off(relay);
        }
    }

    pub fn is_on(&self, relay: Relay) -> bool {
        let gpio = Self::gpio();
        let odr = gpio.odr().read();

        match relay {
            Relay::RL1 => !odr.od0().bit_is_set(),
            Relay::RL2 => !odr.od1().bit_is_set(),
        }
    }

    pub fn toggle(&self, relay: Relay) {
        if self.is_on(relay) {
            self.off(relay);
        } else {
            self.on(relay);
        }
    }

    pub fn all_on(&self) {
        let gpio = Self::gpio();

        gpio.bsrr().write(|w| {
            w.br0().set_bit();
            w.br1().set_bit()
        });
    }

    pub fn all_off(&self) {
        let gpio = Self::gpio();

        gpio.bsrr().write(|w| {
            w.bs0().set_bit();
            w.bs1().set_bit()
        });
    }
}
