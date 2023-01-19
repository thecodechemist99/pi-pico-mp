#![no_std]
#![no_main]

mod delay;
mod display;
mod buttons;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[rtic::app(device = rp2040_hal::pac, peripherals = true)]
mod app {
    use cortex_m::{prelude::{
        _embedded_hal_watchdog_Watchdog,
        _embedded_hal_watchdog_WatchdogEnable,
    }, interrupt::disable};
    use defmt_rtt as _;
    use panic_probe as _;
    use rp2040_hal as hal;

    use hal::{
        clocks::{init_clocks_and_plls, Clock},
        gpio::{bank0::*, Pin, Input, Output, PushPull, PullDown, Interrupt::EdgeHigh, dynpin::DynPin},
        pac::SPI0,
        sio::Sio,
        timer::{Timer, Alarm, Alarm0, Alarm1},
        watchdog::Watchdog,
    };
    use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};

    use fugit::{RateExtU32, MicrosDurationU32};

    // lcd traits
    use embedded_graphics::prelude::RgbColor;
    use embedded_graphics::draw_target::DrawTarget;
    use display_interface_spi::SPIInterface;
    use mipidsi::{Builder, Display};

    use crate::delay;

    const DISPLAY_UPDATE_TIME_US: MicrosDurationU32 = MicrosDurationU32::micros(1700);
    const EXTERNAL_XTAL_FREQ_HZ: u32 = 12_000_000;

    # [shared]
    struct Shared {
        timer: Timer,
        #[lock_free]
        watchdog: Watchdog,
        #[lock_free]
        display: Display<SPIInterface<rp2040_hal::Spi<rp2040_hal::spi::Enabled,SPI0,8>, Pin<Gpio6,Output<PushPull>>, Pin<Gpio5,Output<PushPull>>>, mipidsi::models::ST7789, Pin<Gpio7,Output<PushPull>>>,
        display_alarm: Alarm1,
        #[lock_free]
        buttons: (Pin<Gpio18,Input<PullDown>>, Pin<Gpio19,Input<PullDown>>, Pin<Gpio20,Input<PullDown>>, Pin<Gpio21,Input<PullDown>>),
        #[lock_free]
        test_led: Pin<Gpio25,Output<PushPull>>,
        debounce_alarm: Alarm0,
    }

    #[local]
    struct Local {}


    // /// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
    // /// as soon as all global variables and the spinlock are initialised.
    // #[rp2040_hal::entry]
    // fn main() -> ! {
    //     let i = 0;
    // }
    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut resets = c.device.RESETS;
        let mut watchdog = Watchdog::new(c.device.WATCHDOG);
        watchdog.pause_on_debug(false);

        let clocks = init_clocks_and_plls(
            EXTERNAL_XTAL_FREQ_HZ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let sio = Sio::new(c.device.SIO);
        let pins = rp2040_hal::gpio::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let mut timer = Timer::new(c.device.TIMER, &mut resets);
        let mut display_alarm = timer.alarm_1().unwrap();
        let _ = display_alarm.schedule(DISPLAY_UPDATE_TIME_US);
        display_alarm.enable_interrupt();
        let debounce_alarm = timer.alarm_0().unwrap();

        let btn0 = pins.gpio18.into_pull_down_input();
        let btn1 = pins.gpio19.into_pull_down_input();
        let btn2 = pins.gpio20.into_pull_down_input();
        let btn3 = pins.gpio21.into_pull_down_input();

        // enable interrupts
        btn0.set_interrupt_enabled(EdgeHigh, true);
        btn1.set_interrupt_enabled(EdgeHigh, true);
        btn2.set_interrupt_enabled(EdgeHigh, true);
        btn3.set_interrupt_enabled(EdgeHigh, true);

        let test_led = pins.gpio25.into_push_pull_output();
        let buttons = (btn0, btn1, btn2, btn3);

        // These are implicitly used by the spi driver if they are in the correct mode
        let _spi_sclk = pins.gpio2.into_mode::<rp2040_hal::gpio::FunctionSpi>();
        let _spi_mosi = pins.gpio4.into_mode::<rp2040_hal::gpio::FunctionSpi>();
        let _spi_miso = pins.gpio3.into_mode::<rp2040_hal::gpio::FunctionSpi>();
        let spi = rp2040_hal::Spi::<_, _, 8>::new(c.device.SPI0);

        let mut lcd_bl = pins.gpio8.into_push_pull_output();
        let dc = pins.gpio6.into_push_pull_output();
        let rst = pins.gpio7.into_push_pull_output();
        let lcd_cs = pins.gpio5.into_push_pull_output();

        // Exchange the uninitialised SPI driver for an initialised one
        let spi = spi.init(
            &mut resets,
            clocks.peripheral_clock.freq(),
            16.MHz(),
            &embedded_hal::spi::MODE_0,
        );

        // let delay: TimerDelay = TimerDelay::new(&timer);
        let mut delay = delay::Delay::new(&timer, EXTERNAL_XTAL_FREQ_HZ);

        let mut display = Builder::st7789(display_interface_spi::SPIInterface::new(spi, dc, lcd_cs))
            .with_display_size(320, 240)
            .with_invert_colors(true)
            .init(&mut delay, Some(rst)).unwrap();
        display.clear(RgbColor::BLACK).unwrap();

        // Wait until the background has been rendered otherwise
        // the screen will show random pixels for a brief moment

        delay.delay_ms(1000);
        lcd_bl.set_high().unwrap();

        // start watchdog after initialization
        // It needs to be fairly high though to account for screen drawing etc
        // watchdog.start(10_000.microseconds());
        watchdog.start(MicrosDurationU32::micros(1_000_000));

        (
            Shared {
                timer,
                watchdog,
                display,
                display_alarm,
                buttons,
                test_led,
                debounce_alarm,
            },
            Local {},
            init::Monotonics(),
        )
    }

    #[task(binds = IO_IRQ_BANK0, priority = 1, shared = [ buttons, test_led, debounce_alarm, watchdog])]
    fn btn_press_irq(c: btn_press_irq::Context) {
        c.shared.watchdog.feed();

        let buttons = c.shared.buttons;
        let (btn0, btn1, btn2, btn3) = buttons;

        let led = c.shared.test_led;

        if btn0.interrupt_status(EdgeHigh) {
            let _ = led.toggle();
            btn0.clear_interrupt(EdgeHigh);

            // Debouncing
            btn0.set_interrupt_enabled(EdgeHigh, false); // reenable it via debounce_alarm

        } else if btn1.interrupt_status(EdgeHigh) {
            let _ = led.toggle();
            btn1.clear_interrupt(EdgeHigh);

            // Debouncing
            btn1.set_interrupt_enabled(EdgeHigh, false); // reenable it via debounce_alarm

        } else if btn2.interrupt_status(EdgeHigh) {
            let _ = led.toggle();
            btn2.clear_interrupt(EdgeHigh);

            // Debouncing
            btn2.set_interrupt_enabled(EdgeHigh, false); // reenable it via debounce_alarm

        } else if btn3.interrupt_status(EdgeHigh) {
            let _ = led.toggle();
            btn3.clear_interrupt(EdgeHigh);

            // Debouncing
            btn3.set_interrupt_enabled(EdgeHigh, false); // reenable it via debounce_alarm
        }

        let mut alarm = c.shared.debounce_alarm;

        alarm.lock(|a| {
            a.enable_interrupt();
            let _ = a.schedule(crate::buttons::DEBOUNCE_INTERVAL);
        });
    }

    #[task(binds = TIMER_IRQ_0, priority = 1, shared = [ buttons, debounce_alarm, watchdog])]
    fn debounce_timer_irq(c: debounce_timer_irq::Context) {
        c.shared.watchdog.feed();

        let buttons = c.shared.buttons;
        let (btn0, btn1, btn2, btn3) = buttons;

        btn0.set_interrupt_enabled(EdgeHigh, true);
        btn1.set_interrupt_enabled(EdgeHigh, true);
        btn2.set_interrupt_enabled(EdgeHigh, true);
        btn3.set_interrupt_enabled(EdgeHigh, true);

        let mut alarm = c.shared.debounce_alarm;

        alarm.lock(|a| {
            a.clear_interrupt();
            a.disable_interrupt();
        });
    }

}
