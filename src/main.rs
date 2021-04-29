#![no_std]
#![no_main]

use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics

use embedded_hal::digital::v1_compat::OldOutputPin;
use hal::gpio::{AlternateFunction, Input, InterruptMode, Output, PullUp, PushPull, AF2};
use hd44780_driver::{Cursor, CursorBlink, Display, DisplayMode, HD44780};
use mfrc522::Mfrc522;
use numtoa::NumToA;
use rtic::cyccnt::U32Ext;
use tm4c123x::{SSI2, TIMER0};
use tm4c123x_hal::delay::DelayFromCountDownTimer;
use tm4c123x_hal::gpio::{
    gpioa::PA2,
    gpiob::{PB0, PB4, PB5, PB6, PB7},
    gpioc::{PC4, PC5, PC6, PC7},
    gpiod::PD6,
    gpiof::{PF0, PF1, PF2, PF3},
};
use tm4c123x_hal::spi::Spi;
use tm4c123x_hal::timer::Timer;
use tm4c123x_hal::{self as hal, prelude::*};

const PERIOD: u32 = 80_000_000;
const MASTER_CARD: [u8; 4] = [192, 33, 232, 239];

#[rtic::app(device = tm4c123x, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        delay: DelayFromCountDownTimer<Timer<TIMER0>>,
        red_led: PF1<Output<PushPull>>,
        blue_led: PF2<Output<PushPull>>,
        green_led: PF3<Output<PushPull>>,
        lcd: HD44780<
            hd44780_driver::bus::FourBitBus<
                PA2<Output<PushPull>>,
                PD6<Output<PushPull>>,
                PC7<Output<PushPull>>,
                PC6<Output<PushPull>>,
                PC5<Output<PushPull>>,
                PC4<Output<PushPull>>,
            >,
        >,
        button_two: PF0<Input<PullUp>>,
        buffer: [u8; 10],
        mfrc522_buffer: [u8; 10],
        irq: PB0<Input<PullUp>>,
        mfrc522: Mfrc522<
            Spi<
                SSI2,
                (
                    PB4<AlternateFunction<AF2, PushPull>>,
                    PB6<AlternateFunction<AF2, PushPull>>,
                    PB7<AlternateFunction<AF2, PushPull>>,
                ),
            >,
            OldOutputPin<PB5<Output<PushPull>>>,
        >,
        #[init(0)]
        lcd_counter: u32,
        #[init(false)]
        led_state: bool,
    }

    #[init(spawn = [lcd_increment])]
    fn init(cx: init::Context) -> init::LateResources {
        let mut core = cx.core;
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();
        let peripherals: hal::Peripherals = cx.device;

        let mut sysctl = peripherals.SYSCTL.constrain();
        sysctl.clock_setup.oscillator = hal::sysctl::Oscillator::Main(
            hal::sysctl::CrystalFrequency::_16mhz,
            hal::sysctl::SystemClock::UsePll(hal::sysctl::PllOutputFrequency::_80_00mhz),
        );
        let clocks = sysctl.clock_setup.freeze();

        let tim0 = hal::timer::Timer::timer0(
            peripherals.TIMER0,
            hal::time::Hertz(200),
            &sysctl.power_control,
            &clocks,
        );

        let mut delay = hal::delay::DelayFromCountDownTimer::new(tim0);

        let mut pins_f = peripherals.GPIO_PORTF.split(&sysctl.power_control);
        let mut red_led = pins_f.pf1.into_push_pull_output();
        let mut blue_led = pins_f.pf2.into_push_pull_output();
        let mut green_led = pins_f.pf3.into_push_pull_output();
        embedded_hal::digital::v2::OutputPin::set_low(&mut red_led).unwrap();
        embedded_hal::digital::v2::OutputPin::set_low(&mut blue_led).unwrap();
        embedded_hal::digital::v2::OutputPin::set_low(&mut green_led).unwrap();

        let pins_a = peripherals.GPIO_PORTA.split(&sysctl.power_control);
        let pins_c = peripherals.GPIO_PORTC.split(&sysctl.power_control);
        let pins_d = peripherals.GPIO_PORTD.split(&sysctl.power_control);

        let rs = pins_a.pa2.into_push_pull_output();
        let en = pins_d.pd6.into_push_pull_output();
        let b4 = pins_c.pc7.into_push_pull_output();
        let b5 = pins_c.pc6.into_push_pull_output();
        let b6 = pins_c.pc5.into_push_pull_output();
        let b7 = pins_c.pc4.into_push_pull_output();

        let mut lcd = HD44780::new_4bit(rs, en, b4, b5, b6, b7, &mut delay).unwrap();
        lcd.reset(&mut delay).unwrap();
        lcd.clear(&mut delay).unwrap();

        lcd.set_display_mode(
            DisplayMode {
                display: Display::On,
                cursor_visibility: Cursor::Invisible,
                cursor_blink: CursorBlink::Off,
            },
            &mut delay,
        )
        .unwrap();

        let mut button_two = pins_f.pf0.unlock(&mut pins_f.control).into_pull_up_input();
        button_two.set_interrupt_mode(InterruptMode::EdgeFalling);

        let mut pins_b = peripherals.GPIO_PORTB.split(&sysctl.power_control);
        let sck = pins_b.pb4.into_af_push_pull(&mut pins_b.control);
        let miso = pins_b.pb6.into_af_push_pull(&mut pins_b.control);
        let mosi = pins_b.pb7.into_af_push_pull(&mut pins_b.control);
        let nss = pins_b.pb5.into_push_pull_output();

        let mut irq = pins_b.pb0.into_pull_up_input();
        irq.set_interrupt_mode(InterruptMode::LevelHigh);

        let spi = tm4c123x_hal::spi::Spi::spi2(
            peripherals.SSI2,
            (sck, miso, mosi),
            mfrc522::MODE,
            hal::time::Hertz(1_000_000),
            &clocks,
            &sysctl.power_control,
        );
        let mfrc522 = Mfrc522::new(spi, OldOutputPin::from(nss)).unwrap();

        let buffer = [0u8; 10];
        let mfrc522_buffer = [0u8; 10];
        lcd.write_str("<<Scan Card", &mut delay).unwrap();

        cx.spawn.lcd_increment().ok();

        init::LateResources {
            delay: delay,
            red_led: red_led,
            blue_led: blue_led,
            green_led: green_led,
            lcd: lcd,
            button_two: button_two,
            buffer: buffer,
            mfrc522_buffer: mfrc522_buffer,
            irq: irq,
            mfrc522: mfrc522,
        }
    }

    #[task(schedule = [lcd_increment], resources = [delay, lcd, lcd_counter, buffer], priority = 2)]
    fn lcd_increment(cx: lcd_increment::Context) {
        let delay = cx.resources.delay;
        let lcd = cx.resources.lcd;
        let lcd_counter = cx.resources.lcd_counter;
        let buffer = cx.resources.buffer;

        lcd.set_cursor_pos(40, delay).unwrap();
        lcd.write_str(&lcd_counter.numtoa_str(10, buffer), delay)
            .unwrap();

        *lcd_counter += 1;
        cx.schedule
            .lcd_increment(cx.scheduled + PERIOD.cycles())
            .unwrap();
    }

    #[task(binds = GPIOF, resources = [button_two, led_state, blue_led], priority = 1)]
    fn button_two(cx: button_two::Context) {
        let button_two = cx.resources.button_two;
        let blue_led = cx.resources.blue_led;
        let led_state = cx.resources.led_state;
        if button_two.get_interrupt_status() {
            match *led_state {
                true => embedded_hal::digital::v2::OutputPin::set_low(blue_led).unwrap(),
                false => embedded_hal::digital::v2::OutputPin::set_high(blue_led).unwrap(),
            }
            *led_state = !*led_state;
        }

        button_two.clear_interrupt();
    }

    #[task(binds = GPIOB, resources = [irq, mfrc522, red_led, green_led, lcd, delay, mfrc522_buffer], priority = 1)]
    fn iss2_event(cx: iss2_event::Context) {
        let mfrc522 = cx.resources.mfrc522;
        let red_led = cx.resources.red_led;
        let green_led = cx.resources.green_led;
        let mut lcd = cx.resources.lcd;
        let mut delay = cx.resources.delay;
        let buffer = cx.resources.mfrc522_buffer;

        if cx.resources.irq.get_interrupt_status() {
            if let Ok(atqa) = mfrc522.reqa() {
                if let Ok(uid) = mfrc522.select(&atqa) {
                    let card_uid = uid.bytes();
                    lcd.lock(|lcd| {
                        delay.lock(|delay| {
                            lcd.set_cursor_pos(0, delay).unwrap();
                            lcd.write_str("ID: ", delay).unwrap();
                            for byte in card_uid {
                                lcd.write_str(byte.numtoa_str(16, buffer), delay).unwrap();
                            }
                            if card_uid == &MASTER_CARD {
                                lcd.write_str(" :)", delay).unwrap();
                                embedded_hal::digital::v2::OutputPin::set_high(green_led).unwrap();
                                delay.delay_ms(500u32);
                                embedded_hal::digital::v2::OutputPin::set_low(green_led).unwrap();
                            } else {
                                lcd.write_str(" :(", delay).unwrap();
                                embedded_hal::digital::v2::OutputPin::set_high(red_led).unwrap();
                                delay.delay_ms(500u32);
                                embedded_hal::digital::v2::OutputPin::set_low(red_led).unwrap();
                            }
                        })
                    });
                }
            }
        }

        cx.resources.irq.clear_interrupt();
    }

    extern "C" {
        fn ADC0SS0();
        fn ADC0SS1();
        fn ADC0SS2();
    }
};
