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
    use crate::protocol::modbus::{Modbus,HoldingRegisters,DEFAULT_ADDRESS};
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

        // Configure system clock.
        bsp::init_clocks(&dp.RCC);

        // Configure pins (EXTI IMR remains masked).
        bsp::init_pins(&dp.GPIOA, &dp.GPIOB, &dp.EXTI);

        // Create TM1638 driver
        let tm1638 = Tm1638::new();

        // Create UART driver.
        let uart = UartDma::new(dp.USART1, &dp.DMA, &dp.DMAMUX, &dp.RCC);

        // Create Modbus
        let modbus = Modbus::new(DEFAULT_ADDRESS);

        //Create Relay driver
        let relay = RelayDriver::new();

        // Create EEPROM driver
        let eeprom = Eeprom::new(dp.I2C1, &dp.RCC);

        // Start monotonic timer.
        let mono = Systick::new(ctx.core.SYST, SYSCLK_HZ);

        // --- STABILIZATION WINDOW ---
        // Allow pull-up resistors and pin capacitance to fully charge to 3.3V (~2ms @ 48MHz)
        cortex_m::asm::delay(9600_000);

        // Create Encoder
        let encoder = Encoder::new(&dp.GPIOA);

        // Enable Interrupts (flushes stale flags and unmasks EXTI IMR)
        bsp::init_interrupts(&dp.EXTI);
        // ----------------------------

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
                // Read shared encoder count to update registers dynamically
                let current_count = ctx.shared.encoder_count.lock(|c| *c);

                let mut registers = HoldingRegisters {
                    value_low: (current_count & 0xFFFF) as u16,
                    value_high: ((current_count >> 16) & 0xFFFF) as u16,
                    node_address: modbus.address() as u16,
                    new_node_address: modbus.address() as u16,
                };

                let tx_len = modbus.process(&rx_buf[..rx_len], tx_buf, &mut registers);

                if tx_len != 0 {
                    let _ = uart.send_data(&tx_buf[..tx_len]);
                }
            }
        }

        uart_task::spawn_after(1.millis()).ok();
    }
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            //cortex_m::asm::wfi();
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

        const KNOWN_TEST_VALUE: i32 = 424242;

        let tm = ctx.local.tm1638;
        let relays = ctx.local.relay;

        // Read keyboard
        tm.read_keys(ctx.local.keys);

        let current_keys = (ctx.local.keys[3] as u32) << 24
            | (ctx.local.keys[2] as u32) << 16
            | (ctx.local.keys[1] as u32) << 8
            | (ctx.local.keys[0] as u32);

        // Detect rising edge
        let pressed_keys = current_keys & !*ctx.local.last_keys;
        *ctx.local.last_keys = current_keys;

        match pressed_keys {
            KEY1 => {
                let mut buf = [0u8; 4];
                // Sequential 4-byte read (1 transaction)
                ctx.local.eeprom.read(0, &mut buf);

                let loaded_count = i32::from_le_bytes(buf);

                ctx.shared.encoder_count.lock(|c| {
                    *c = loaded_count;
                });
            }
            KEY2 => {
                let buf = KNOWN_TEST_VALUE.to_le_bytes();
                // Page write (1 transaction, 1 internal write cycle delay)
                ctx.local.eeprom.write(0, &buf);

                ctx.shared.encoder_count.lock(|c| {
                    *c = KNOWN_TEST_VALUE;
                });
            }
            KEY3 => {
                relays.toggle(RL1);
            }
            KEY4 => {
                relays.toggle(RL2);
            }
            KEY6 => {
                ctx.shared.encoder_count.lock(|c| *c = 0);
            }
            _ => {}
        }

        // Display formatting
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

    // Encoder Hardware Task
    #[task(
        binds = EXTI0_1,
        priority = 3,
        shared = [encoder_count],
        local = [encoder],
    )]
    fn exti0_1(mut ctx: exti0_1::Context) {
        let exti = unsafe { &*pac::EXTI::ptr() };
        let gpioa = unsafe { &*pac::GPIOA::ptr() };

        // Clear pending EXTI flags.
        exti.rpr1().write(|w| {
            w.rpif0().set_bit();
            w.rpif1().set_bit()
        });

        exti.fpr1().write(|w| {
            w.fpif0().set_bit();
            w.fpif1().set_bit()
        });

        // Decode transition.
        let delta = ctx.local.encoder.update(gpioa);

        if delta != 0 {
            ctx.shared.encoder_count.lock(|count| {
                *count += i32::from(delta);
            });
        }
    }

    // Power fail Hardware Task
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
