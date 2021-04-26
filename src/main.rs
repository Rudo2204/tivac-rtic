#![no_std]
#![no_main]

use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics

use hal::gpio::{Floating, Input, Output, PushPull};
use hd44780_driver::{Cursor, CursorBlink, Display, DisplayMode, HD44780};
use numtoa::NumToA;
use rtic::cyccnt::U32Ext;
use tm4c123x::TIMER0;
use tm4c123x_hal::delay::DelayFromCountDownTimer;
use tm4c123x_hal::gpio::{
    gpioa::PA2,
    gpioc::{PC4, PC5, PC6, PC7},
    gpiod::PD6,
    gpiof::{PF1, PF4},
};
use tm4c123x_hal::{self as hal, prelude::*};

const PERIOD: u32 = 50_000_000;

#[rtic::app(device = tm4c123x, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        delay: DelayFromCountDownTimer<hal::timer::Timer<TIMER0>>,
        led: PF1<Output<PushPull>>,
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
        button_one: PF4<Input<Floating>>,
        buffer: [u8; 10],
    }

    #[init(schedule = [blinker])]
    fn init(cx: init::Context) -> init::LateResources {
        let mut core = cx.core;
        core.DWT.enable_cycle_counter();
        let peripherals: hal::Peripherals = cx.device;

        let mut sysctl = peripherals.SYSCTL.constrain();
        sysctl.clock_setup.oscillator = hal::sysctl::Oscillator::Main(
            hal::sysctl::CrystalFrequency::_16mhz,
            hal::sysctl::SystemClock::UsePll(hal::sysctl::PllOutputFrequency::_66_67mhz),
        );
        let clocks = sysctl.clock_setup.freeze();

        let tim0 = hal::timer::Timer::timer0(
            peripherals.TIMER0,
            hal::time::Hertz(400),
            &sysctl.power_control,
            &clocks,
        );

        let mut delay = hal::delay::DelayFromCountDownTimer::new(tim0);
        //let mut delay = tm4c123x_hal::delay::Delay::new(core.SYST, &clocks);

        let pins_f = peripherals.GPIO_PORTF.split(&sysctl.power_control);
        let mut led = pins_f.pf1.into_push_pull_output();
        embedded_hal::digital::v2::OutputPin::set_low(&mut led).unwrap();

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
        lcd.write_str("HELLO RTIC", &mut delay).unwrap();

        lcd.set_display_mode(
            DisplayMode {
                display: Display::On,
                cursor_visibility: Cursor::Invisible,
                cursor_blink: CursorBlink::Off,
            },
            &mut delay,
        )
        .unwrap();

        let mut button_one = pins_f.pf4.into_floating_input();
        button_one.set_interrupt_mode(hal::gpio::InterruptMode::EdgeRising);
        button_one.clear_interrupt();

        let buffer = [0u8; 10];

        cx.schedule.blinker(cx.start + PERIOD.cycles()).unwrap();
        init::LateResources {
            delay: delay,
            led: led,
            lcd: lcd,
            button_one: button_one,
            buffer: buffer,
        }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {}
    }

    #[task(resources = [led], schedule = [blinker])]
    fn blinker(cx: blinker::Context) {
        static mut LED_STATE: bool = false;

        if *LED_STATE {
            embedded_hal::digital::v2::OutputPin::set_high(cx.resources.led).unwrap();
            *LED_STATE = false;
        } else {
            embedded_hal::digital::v2::OutputPin::set_low(cx.resources.led).unwrap();
            *LED_STATE = true;
        }
        cx.schedule.blinker(cx.scheduled + PERIOD.cycles()).unwrap();
    }

    #[task(binds = GPIOF, resources = [delay, lcd, button_one, buffer])]
    fn button_one_event(cx: button_one_event::Context) {
        static mut BUTTON_COUNTER: u8 = 0;

        let button_one = cx.resources.button_one;
        let delay = cx.resources.delay;
        let lcd = cx.resources.lcd;
        let buffer = cx.resources.buffer;

        lcd.set_cursor_pos(40, delay).unwrap();
        lcd.write_str(BUTTON_COUNTER.numtoa_str(10, buffer), delay)
            .unwrap();

        *BUTTON_COUNTER = BUTTON_COUNTER.wrapping_add(1);

        button_one.clear_interrupt();
    }

    extern "C" {
        fn ADC0SS0();
        fn ADC0SS1();
    }
};
