#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use rp2040_hal as hal;

use fugit::{
    // RateExtU32,
    MicrosDurationU32,
};

use hal::{
  clocks::{init_clocks_and_plls, Clock},
  pac,
  watchdog::Watchdog,
  timer::Alarm,
  Sio,
};

// interrupts
use embedded_hal::digital::v2::{ToggleableOutputPin, OutputPin};
use pac::interrupt;
use hal::gpio::Interrupt::EdgeHigh;
use core::cell::RefCell;
use critical_section::Mutex;

// testing
// use embedded_graphics::prelude::RgbColor;
// use embedded_graphics::pixelcolor::Rgb666;
// use embedded_graphics::draw_target::DrawTarget;

// mod ui;
// mod metrology;

// The linker will place this boot block at the start of our program image. We
// need this to help the ROM bootloader get our code up and running.
// Note: This boot block is not necessary when using a rp-hal based BSP
// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

// Period that each of the alarms will be set for - 1 second and 300ms respectively
const DEBOUNCE_INTERVAL: MicrosDurationU32 = MicrosDurationU32::millis(20);
const BTN_STATE_INTERVAL: MicrosDurationU32 = MicrosDurationU32::millis(250);

// This pin will be our output - it will drive an LED if you run this on a Pico
type LedPin = hal::gpio::Pin<hal::gpio::bank0::Gpio25, hal::gpio::PushPullOutput>;

// Create types for ButtonPins and GlobalAlarms
type ButtonPins = (
    hal::gpio::Pin<hal::gpio::bank0::Gpio10, hal::gpio::PullDownInput>, // Btn0
    hal::gpio::Pin<hal::gpio::bank0::Gpio11, hal::gpio::PullDownInput>, // Btn1
    hal::gpio::Pin<hal::gpio::bank0::Gpio12, hal::gpio::PullDownInput>, // Btn2
    hal::gpio::Pin<hal::gpio::bank0::Gpio13, hal::gpio::PullDownInput>, // Btn3
    LedPin, // LED (for testing purpose only)
);

type GlobalAlarms = (
    hal::timer::Alarm0 // Debouncing
    // hal::timer::Alarm1, // Long press detection
);

// Transfer button pins, LED and alarms to the interrupt handler.
static GLOBAL_PINS: Mutex<RefCell<Option<ButtonPins>>> = Mutex::new(RefCell::new(None));
static GLOBAL_ALARMS: Mutex<RefCell<Option<GlobalAlarms>>> = Mutex::new(RefCell::new(None));

/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
#[rp2040_hal::entry]
fn main() -> ! {
    // System config
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // // // SPI config
    // // These pins are implicitly used by the spi driver if they are in the correct mode
    // let _spi_sclk = pins.gpio18.into_mode::<hal::gpio::FunctionSpi>();
    // let _spi_mosi = pins.gpio19.into_mode::<hal::gpio::FunctionSpi>();
    // let _spi_miso = pins.gpio16.into_mode::<hal::gpio::FunctionSpi>();
    // let spi = hal::Spi::<hal::spi::Disabled, _, 8>::new(pac.SPI0);
    // let spi = spi.init(&mut pac.RESETS, clocks.peripheral_clock.freq(), 16.MHz(), &embedded_hal::spi::MODE_0);

    // ADC config
    let cs_adc = pins.gpio5.into_mode::<hal::gpio::FunctionSpi>();

    // DISPLAY config
    let cs_display = pins.gpio17.into_mode::<hal::gpio::FunctionSpi>();
    let dc: hal::gpio::DynPin = pins.gpio20.into();
    let rst: hal::gpio::DynPin = pins.gpio21.into();
    let bl: hal::gpio::DynPin = pins.gpio22.into();
    // // let mut display = ui::display::init_display(spi, cs_display, dc, rst, bl, &mut delay);
    // // display.clear(RgbColor::BLACK).unwrap();

    // BUTTONS config
    let led = pins.gpio25.into_push_pull_output();
    let buttons: ButtonPins = (
        pins.gpio10.into_mode(),
        pins.gpio11.into_mode(),
        pins.gpio12.into_mode(),
        pins.gpio13.into_mode(),
        led,
    );

    buttons.0.set_interrupt_enabled(EdgeHigh, true);
    buttons.1.set_interrupt_enabled(EdgeHigh, true);
    buttons.2.set_interrupt_enabled(EdgeHigh, true);
    buttons.3.set_interrupt_enabled(EdgeHigh, true);

    // create alarms and enable interrupts
    let mut timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut alarm_debounce = timer.alarm_0().unwrap();
    // let mut alarm_btnState = timer.alarm_1().unwrap();
    alarm_debounce.enable_interrupt();
    // alarm_btnState.enable_interrupt();

    // Give away our pins and alarm by moving them into the `GLOBAL_PINS` variable.
    // We won't need to access them in the main thread again
    critical_section::with(|cs| {
        GLOBAL_PINS.borrow(cs).replace(Some(buttons));
        GLOBAL_ALARMS.borrow(cs).replace(Some((alarm_debounce)));
    });

    // Unmask the IO_BANK0 and timer0 IRQ so that the NVIC interrupt controller
    // will jump to the interrupt function when the interrupt occurs.
    // We do this last so that the interrupt can't go off while
    // it is in the middle of being configured
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }

    // HEATING
    // let mut heater = pins.gpio15.into_push_pull_output();
    // heater.set_high().unwrap();
    // delay.delay_ms(5000);
    // heater.set_low().unwrap();

    let mut led_ext = pins.gpio0.into_push_pull_output();

   loop {
        // Wait for an interrupt to fire before doing any more work
        // cortex_m::asm::wfi();
        led_ext.set_high().unwrap();
        delay.delay_ms(5000);
        led_ext.set_low().unwrap();
        delay.delay_ms(5000);
   }
}

// Interrupt handler
#[interrupt]
fn IO_IRQ_BANK0() {
    // The `#[interrupt]` attribute covertly converts this to `&'static mut Option<ButtonPins>`
    static mut BUTTONS: Option<ButtonPins> = None;
    static mut ALARMS: Option<GlobalAlarms> = None;

    // This is one-time lazy initialisation. We steal the variables given to us
    // via `GLOBAL_PINS` and `GLOBAL_ALARMS`.
    if BUTTONS.is_none() {
        critical_section::with(|cs| {
            *BUTTONS = GLOBAL_PINS.borrow(cs).take();
        });
    }
    if ALARMS.is_none() {
        critical_section::with(|cs| {
            *ALARMS = GLOBAL_ALARMS.borrow(cs).take();
        });
    }

    // Need to check if our Option<ButtonPins> contains our pins
    if let Some(gpios) = BUTTONS {
        // borrow led and button by *destructuring* the tuple
        // these will be of type `&mut LedPin` and `&mut ButtonPin`, so we don't have
        // to move them back into the static after we use them
        let (btn0, btn1, btn2, btn3, led) = gpios;

        // // borrow alarms by *destructuring* the tuple
        // // these will be of type `&mut Alarm0` and `&mut Alarm1`
        let alarm_debounce;
        if let Some(alarms) = ALARMS {
            // let (alarm_debounce, alarm_btnState) = alarms;
            (alarm_debounce) = alarms;
        }

        // Check from which pushbutton the interrupt originated and if it is going from low-to-high.
        if btn0.interrupt_status(EdgeHigh) {
            // add debouncing and multiple button support

            // toggle can't fail, but the embedded-hal traits always allow for it
            // we can discard the return value by assigning it to an unnamed variable
            let _ = led.toggle();

            // Clear interrupt to prevent immediately jumping back here
            btn0.clear_interrupt(EdgeHigh);
            btn0.set_interrupt_enabled(EdgeHigh, false); // reenable it via alarm interrupt
            // Debouncing
            alarm_debounce.schedule(DEBOUNCE_INTERVAL);
        } else if btn1.interrupt_status(EdgeHigh) {
            // toggle can't fail, but the embedded-hal traits always allow for it
            // we can discard the return value by assigning it to an unnamed variable
            let _ = led.toggle();

            // Clear interrupt to prevent immediately jumping back here
            btn1.clear_interrupt(EdgeHigh);
            
        } else if btn2.interrupt_status(EdgeHigh) {
            // toggle can't fail, but the embedded-hal traits always allow for it
            // we can discard the return value by assigning it to an unnamed variable
            let _ = led.toggle();

            // Clear interrupt to prevent immediately jumping back here
            btn2.clear_interrupt(EdgeHigh);
            
        } else if btn3.interrupt_status(EdgeHigh) {
                        // toggle can't fail, but the embedded-hal traits always allow for it
            // we can discard the return value by assigning it to an unnamed variable
            let _ = led.toggle();

            // Clear interrupt to prevent immediately jumping back here
            btn3.clear_interrupt(EdgeHigh);
        }
    }
}

// #[interrupt]
// fn TIMER_IRQ_0() {
//     // The `#[interrupt]` attribute covertly converts this to `&'static mut Option<GlobalAlarms>`
//     static mut ALARMS: Option<GlobalAlarms> = None;

//     // This is one-time lazy initialisation. We steal the variables given to us
//     // via `GLOBAL_ALARMS`.
//     if ALARMS.is_none() {
//         critical_section::with(|cs| {
//             *ALARMS = GLOBAL_ALARMS.borrow(cs).take();
//         });
//     }

//     // Need to check if our Option<LedAndButtonAndAlarmPins> contains our pins
//     if let Some(alarms) = ALARMS {
//         // borrow led and button by *destructuring* the tuple
//         // these will be of type `&mut LedPin` and `&mut ButtonPin`, so we don't have
//         // to move them back into the static after we use them
//         let (alarm_debounce, alarm_btnState) = alarms;

//         // check both alarms
//         if alarm_debounce.finished() {
//             // Clear the alarm interrupt or this interrupt service routine will keep firing
//             alarm_debounce.clear_interrupt();
//         } else if alarm_btnState.finished() {
//             // Clear the alarm interrupt or this interrupt service routine will keep firing
//             alarm_btnState.clear_interrupt();
//         }
//         // Return ALARMS into our static variable
//         // critical_section::with(|cs| {
//         //     GLOBAL_ALARMS.borrow(cs).replace(Some((alarm_debounce, alarm_btnState)));
//         // });
//     }
// }
