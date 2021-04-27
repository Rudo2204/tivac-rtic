#![no_std]
#![no_main]

use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics

use hal::gpio::{Input, Output, PullUp, PushPull};
use hd44780_driver::{Cursor, CursorBlink, Display, DisplayMode, HD44780};
use numtoa::NumToA;
use rtic::cyccnt::U32Ext;
use tm4c123x::TIMER0;
use tm4c123x_hal::delay::DelayFromCountDownTimer;
use tm4c123x_hal::gpio::{
    gpioa::PA2,
    gpioc::{PC4, PC5, PC6, PC7},
    gpiod::PD6,
    gpiof::{PF0, PF1},
};
use tm4c123x_hal::timer::Timer;
use tm4c123x_hal::{self as hal, prelude::*};

const PERIOD: u32 = 80_000_000;

#[rtic::app(device = tm4c123x, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        delay: DelayFromCountDownTimer<Timer<TIMER0>>,
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
        button_two: PF0<Input<PullUp>>,
        buffer: [u8; 10],
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
        //let mut delay = tm4c123x_hal::delay::Delay::new(core.SYST, &clocks);

        let mut pins_f = peripherals.GPIO_PORTF.split(&sysctl.power_control);
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
        lcd.write_str("Hello RTIC", &mut delay).unwrap();

        lcd.set_display_mode(
            DisplayMode {
                display: Display::On,
                cursor_visibility: Cursor::Invisible,
                cursor_blink: CursorBlink::Off,
            },
            &mut delay,
        )
        .unwrap();

        //let mut button_two = pins_f.pf4.into_pull_up_input();
        let mut button_two = pins_f.pf0.unlock(&mut pins_f.control).into_pull_up_input();
        button_two.set_interrupt_mode(hal::gpio::InterruptMode::EdgeFalling);

        let buffer = [0u8; 10];

        cx.spawn.lcd_increment().ok();

        init::LateResources {
            delay: delay,
            led: led,
            lcd: lcd,
            button_two: button_two,
            buffer: buffer,
        }
    }

    #[task(schedule = [lcd_increment], resources = [delay, lcd, lcd_counter, buffer])]
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

    #[task(binds = GPIOF, resources = [button_two, led_state, led])]
    fn button_two(cx: button_two::Context) {
        let button_two = cx.resources.button_two;
        let led = cx.resources.led;
        let led_state = cx.resources.led_state;
        if button_two.get_interrupt_status() {
            match *led_state {
                true => embedded_hal::digital::v2::OutputPin::set_low(led).unwrap(),
                false => embedded_hal::digital::v2::OutputPin::set_high(led).unwrap(),
            }
            *led_state = !*led_state;
        }

        button_two.clear_interrupt();
    }

    extern "C" {
        fn ADC0SS0();
        fn ADC0SS1();
    }
};
