#![no_std]
#![no_main]

use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics

use hal::gpio::{gpiof::PF1, Output, PushPull};
use rtic::cyccnt::U32Ext;
use tm4c123x_hal::{self as hal, prelude::*};

const PERIOD: u32 = 100_000_000;

#[rtic::app(device = tm4c123x, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        led: PF1<Output<PushPull>>,
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
        let _clocks = sysctl.clock_setup.freeze();

        let pins = peripherals.GPIO_PORTF.split(&sysctl.power_control);
        let mut led = pins.pf1.into_push_pull_output();
        embedded_hal::digital::v2::OutputPin::set_low(&mut led).unwrap();

        cx.schedule.blinker(cx.start + PERIOD.cycles()).unwrap();
        init::LateResources { led: led }
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

    extern "C" {
        fn GPIOF();
    }
};
