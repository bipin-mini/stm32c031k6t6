#![no_std]
#![no_main]

use panic_halt as _;
use stm32c0::stm32c031 as pac;

mod bsp;
mod drivers {
    pub mod blink;
    pub mod encoder;
    pub mod keyboard;
    pub mod relay;
    pub mod tm1638;
    pub mod uart_dma;
}
mod storage {
    pub mod eeprom;
}

mod protocol {
    pub mod modbus;
}

use bsp::SYSCLK_HZ;
use rtic::app;
use systick_monotonic::*;

#[app(device = pac, peripherals = true, dispatchers = [RTC, SPI, ADC])]
mod app {

    use super::*;
    use crate::drivers::relay::Relay::{RL1, RL2};
    use crate::drivers::relay::RelayDriver;
    use crate::drivers::{encoder::Encoder, tm1638::Tm1638, uart_dma::UartDma};
    use crate::protocol::modbus::{DEFAULT_ADDRESS, HoldingRegisters, Modbus};
    use crate::storage::eeprom::Eeprom;

    #[monotonic(binds = SysTick, default = true)]
    type SysMono = Systick<1000>;

    #[shared]
    struct Shared {
        encoder_count: i32,
        _scale_factor: u32,
        _limit_1: i32,
        _limit_2: i32,
        _slave_addr: u8,
    }

    #[local]
    struct Local {
        uart: UartDma,
        tm1638: Tm1638,
        encoder: Encoder,
        eeprom: Eeprom,
        relay: RelayDriver,
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
        let relay = RelayDriver::new();
        let eeprom = Eeprom::new(dp.I2C1, &dp.RCC);

        let mono = Systick::new(ctx.core.SYST, SYSCLK_HZ);

        cortex_m::asm::delay(9_600_000);

        let encoder = Encoder::new(&dp.GPIOA);
        bsp::init_interrupts(&dp.EXTI);

        tm1638_test::spawn().ok();
        uart_task::spawn().ok();

        (
            Shared {
                encoder_count: 0,
                _scale_factor: 1,
                _limit_1: 100,
                _limit_2: 200,
                _slave_addr: 127,
            },
            Local {
                uart,
                tm1638,
                encoder,
                eeprom,
                relay,
                modbus,
            },
            init::Monotonics(mono),
        )
    }

    #[task(
    priority = 2,
    local = [
        uart,
        modbus,
        rx_buf: [u8; 256] = [0; 256],
        tx_buf: [u8; 256] = [0; 256],
    ],
    shared = [encoder_count]
)]
    fn uart_task(mut ctx: uart_task::Context) {
        let uart = ctx.local.uart;
        let modbus = ctx.local.modbus;
        let rx_buf = ctx.local.rx_buf;
        let tx_buf = ctx.local.tx_buf;

        uart.poll();

        if !uart.tx_busy() {
            if let Some(rx_len) = uart.receive_data(rx_buf) {
                let current_count = ctx.shared.encoder_count.lock(|c| *c);

                let raw_bits = current_count as u32;
                let mut registers = HoldingRegisters {
                    value_low: (raw_bits & 0xFFFF) as u16,
                    value_high: ((raw_bits >> 16) & 0xFFFF) as u16,
                    node_address: modbus.address() as u16,
                    new_node_address: modbus.address() as u16,
                };

                let tx_len = modbus.process(&rx_buf[..rx_len], tx_buf, &mut registers);

                if tx_len != 0 {
                    // Sync modified encoder value back if written by master
                    let new_raw =
                        ((registers.value_high as u32) << 16) | (registers.value_low as u32);
                    let new_count = new_raw as i32;

                    if new_count != current_count {
                        ctx.shared.encoder_count.lock(|c| *c = new_count);
                    }

                    let _ = uart.send_data(&tx_buf[..tx_len]);
                }
            }
        }

        uart_task::spawn_after(1.millis()).ok();
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(
        local = [
            tm1638,
            keys: [u8; 4] = [0; 4],
            last_keys: u32 = 0,
            eeprom,
            relay,
        ],
        shared = [encoder_count]
    )]
    fn tm1638_test(mut ctx: tm1638_test::Context) {
        use crate::drivers::tm1638::{FONT, KEY1, KEY2, KEY3, KEY4, KEY6};

        const KNOWN_TEST_VALUE: i32 = 424_242;

        let tm = ctx.local.tm1638;
        let relays = ctx.local.relay;

        tm.read_keys(ctx.local.keys);

        let current_keys = (ctx.local.keys[3] as u32) << 24
            | (ctx.local.keys[2] as u32) << 16
            | (ctx.local.keys[1] as u32) << 8
            | (ctx.local.keys[0] as u32);

        let pressed_keys = current_keys & !*ctx.local.last_keys;
        *ctx.local.last_keys = current_keys;

        match pressed_keys {
            KEY1 => {
                let mut buf = [0u8; 4];
                ctx.local.eeprom.read(0, &mut buf);
                let loaded_count = i32::from_le_bytes(buf);
                ctx.shared.encoder_count.lock(|c| *c = loaded_count);
            }
            KEY2 => {
                let buf = KNOWN_TEST_VALUE.to_le_bytes();
                ctx.local.eeprom.write(0, &buf);
                ctx.shared.encoder_count.lock(|c| *c = KNOWN_TEST_VALUE);
            }
            KEY3 => relays.toggle(RL1),
            KEY4 => relays.toggle(RL2),
            KEY6 => ctx.shared.encoder_count.lock(|c| *c = 0),
            _ => {}
        }

        let count = ctx.shared.encoder_count.lock(|c| *c);
        let negative = count < 0;
        let mut value = count.unsigned_abs();

        let mut data = [0u8; 16];
        for i in 0..6 {
            let digit = (value % 10) as usize;
            value /= 10;
            data[(7 - i) * 2] = FONT[digit];
        }

        if negative {
            data[2] = 0x40;
        }

        tm.write_display(&data);

        tm1638_test::spawn_after(100.millis()).ok();
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
}
