#![no_std]
#![no_main]

mod delay;

//
// RTIC app configuration for raspberry pi pico
// Otherwise not needed interrupt controllers are assigned as dispatchers for software interrupts
//
#[rtic::app(device = rp_pico::pac, peripherals = true, dispatchers = [PIO0_IRQ_0, PIO0_IRQ_1, PIO1_IRQ_0])]
mod app {
    // Panic handler and debugging traits
    use panic_probe as _;
    use defmt_rtt as _;

    // Embedded hal traits
    use embedded_hal::{
        digital::v2::OutputPin,
        PwmPin,
    };

    // Pico traits
    use rp_pico::{
        hal::{
            self,
            Clock,
            clocks::init_clocks_and_plls,
            timer::{ Timer, Alarm, Alarm1, Alarm2 },
            watchdog::Watchdog,
            pac::SPI0,
            pac::I2C1,
            Sio,
            gpio::{ bank0::*, Pin, PullDownInput, PullUpInput, PushPullOutput },
        },
        XOSC_CRYSTAL_FREQ
    };

    // Time traits
    use fugit::{
        RateExtU32,
        MicrosDurationU32,
    };
    use crate::delay::Delay;

    // LCD traits
    use display_interface_spi::SPIInterface;
    use mipidsi::{ Builder, Display, Orientation }; 
    use embedded_graphics::{
        prelude::*,
        pixelcolor::Rgb565,
        draw_target::DrawTarget,
        mono_font::{
            ascii::{
                FONT_10X20,
            },
            MonoTextStyle
        },
        text::Text,
    };
    // use format_no_std::show;

    // ADC traits
    // use adc_mcp3008::{ Mcp3008, Channels8 };
    use nau7802::{
        Nau7802,
        Ldo,
        Gain,
        SamplesPerSecond,
    };

    // PID traits
    // use pid_ctrl::PidCtrl;

    // Global settings
    const DEBOUNCE_INTERVAL: MicrosDurationU32 = MicrosDurationU32::millis(200);
    const STARTUP_DELAY: MicrosDurationU32 = MicrosDurationU32::millis(2500);

    // Type declaration of globally shared variables (can be securely accessed by multiple interrupt handlers)
    #[shared]
    struct Shared {
        #[lock_free]
        buttons: (Pin<Gpio18,PullDownInput>, Pin<Gpio19,PullDownInput>, Pin<Gpio20,PullDownInput>, Pin<Gpio21,PullDownInput>),
        #[lock_free]
        heater_pwm: hal::pwm::Channel<hal::pwm::Pwm0, hal::pwm::FreeRunning, hal::pwm::A>,
        #[lock_free]
        fan_pwm: hal::pwm::Channel<hal::pwm::Pwm0, hal::pwm::FreeRunning, hal::pwm::B>,
        #[lock_free]
        adc_val: i32,
        #[lock_free]
        adc_data_ready: Pin<Gpio13,PullUpInput>,
        // delay: Delay,
        debounce_alarm: Alarm1,
    }

    // Type declaration of locally shared variables (can be assigned to only one interrupt handler)
    #[local]
    struct Local {
        startup_delay: Alarm2,
        // adc: Mcp3008<hal::spi::Spi<hal::spi::Enabled,SPI1,8>, Pin<Gpio13,PushPullOutput>>,
        // adc: Nau7802<hal::I2C<I2C1, (Pin<Gpio14, hal::gpio::FunctionI2C>, Pin<Gpio15, hal::gpio::FunctionI2C>)>>,
        display: Display<SPIInterface<hal::spi::Spi<hal::spi::Enabled,SPI0,8>, Pin<Gpio6,PushPullOutput>, Pin<Gpio5,PushPullOutput>>, mipidsi::models::ST7789, Pin<Gpio7,PushPullOutput>>,
    }

    // 
    // App initialisation
    // All local and shared variables are assigned here
    // 
    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        //
        // System initialisation
        //

        // System clock initialisation
        let mut resets = c.device.RESETS;
        let mut watchdog = Watchdog::new(c.device.WATCHDOG);

        let clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        // GPIO initialisation
        let sio = Sio::new(c.device.SIO);
        let pins = rp_pico::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        // Timer initialisation
        let mut timer = Timer::new(c.device.TIMER, &mut resets);

        let mut debounce_alarm = timer.alarm_1().unwrap();
        debounce_alarm.enable_interrupt();

        let mut startup_delay = timer.alarm_2().unwrap();
        startup_delay.schedule(STARTUP_DELAY).unwrap();
        startup_delay.enable_interrupt();

        let mut delay = Delay::new(&timer, XOSC_CRYSTAL_FREQ);

        //
        // SPI bus initialisation
        //

        // These are implicitly used by the spi driver if they are in the correct mode
        let _spi_sclk = pins.gpio2.into_mode::<hal::gpio::FunctionSpi>();
        let _spi_mosi = pins.gpio3.into_mode::<hal::gpio::FunctionSpi>();
        let _spi_miso = pins.gpio4.into_mode::<hal::gpio::FunctionSpi>();
        let spi = hal::Spi::<_, _, 8>::new(c.device.SPI0);

        // Exchange the uninitialised SPI driver for an initialised one
        let spi = spi.init(
            &mut resets,
            clocks.peripheral_clock.freq(),
            16.MHz(),
            &embedded_hal::spi::MODE_0,
        );

        //
        // I²C bus initialisation
        //
        let sda_pin = pins.gpio14.into_mode::<hal::gpio::FunctionI2C>();
        let scl_pin = pins.gpio15.into_mode::<hal::gpio::FunctionI2C>();

        let i2c = hal::I2C::i2c1(
            c.device.I2C1,
            sda_pin,
            scl_pin,
            100.kHz(),
            &mut resets,
            &clocks.system_clock,
        );

        //
        // PWM initialisation
        //
        let pwm_slices = hal::pwm::Slices::new(c.device.PWM, &mut resets);

        let mut pwm = pwm_slices.pwm0;
        pwm.set_ph_correct();
        pwm.set_div_int(255u8); // 133MHz/255=521.6kHz
        pwm.enable();

        // Output channel A on PWM0 to GPIO 16
        let mut heater_pwm = pwm.channel_a;
        heater_pwm.output_to(pins.gpio16);

        // Output channel B on PWM0 to GPIO 17
        let mut fan_pwm = pwm.channel_b;
        fan_pwm.output_to(pins.gpio17);

        //
        // LED initialisation
        //

        let mut led = pins.led.into_push_pull_output();
        led.set_high().unwrap();

        //
        // Buttons initialisation
        //
        let btn0 = pins.gpio18.into_pull_down_input();
        let btn1 = pins.gpio19.into_pull_down_input();
        let btn2 = pins.gpio20.into_pull_down_input();
        let btn3 = pins.gpio21.into_pull_down_input();
        let buttons = (btn0, btn1, btn2, btn3);

        //
        // Display initialisation
        //
        let mut lcd_cs = pins.gpio5.into_push_pull_output();
        let dc = pins.gpio6.into_push_pull_output();
        let rst = pins.gpio7.into_push_pull_output();
        let mut lcd_bl = pins.gpio8.into_push_pull_output();

        lcd_cs.set_high().unwrap();
        let mut display = Builder::st7789(SPIInterface::new(spi, dc, lcd_cs))
            .with_display_size(320, 240)
            .with_invert_colors(true)
            .with_orientation(Orientation::Landscape(true))
            .init(&mut delay, Some(rst)).unwrap();
        display.clear(Rgb565::WHITE).unwrap();

        // Wait until the background has been rendered otherwise the screen will show random pixels for a moment
        delay.delay_ms(1000);
        lcd_bl.set_high().unwrap();

        //
        // ADC initialisation
        //
        let adc_data_ready = pins.gpio13.into_pull_up_input();
        adc_data_ready.set_interrupt_enabled(hal::gpio::Interrupt::EdgeLow, true);
        // let adc = Mcp3008::new(spi1, adc_cs).unwrap();
        // let adc = Nau7802::new_with_settings(i2c, Ldo::L3v3, Gain::G1, SamplesPerSecond::SPS10, &mut delay).unwrap();

        update_display::spawn().unwrap();

        (
            Shared {
                buttons,
                heater_pwm,
                fan_pwm,
                adc_val: 0,
                adc_data_ready,
                // delay,
                debounce_alarm,
            },
            Local {
                startup_delay,
                // adc,
                display,
            },
            init::Monotonics(),
        )
    }

    // 
    // Idle function (optional)
    // Runs if no other task is active
    //
    #[idle]
    fn idle(_c: idle::Context) -> ! {
        loop {
            // Wait For Interrupt is used instead of a busy-wait loop to allow MCU to sleep between interrupts
            // https://developer.arm.com/documentation/ddi0406/c/Application-Level-Architecture/Instruction-Details/Alphabetical-list-of-instructions/WFI
            // rtic::export::wfi();

            
            // update_display::spawn().unwrap();
        }
    }

    //
    // Display Update
    // Takes current values of supplied variables and draws them on the connected display
    //
    #[task(priority = 2, local = [display], shared = [adc_val])]
    fn update_display(c: update_display::Context) {
        let update_display::LocalResources {
            display,
        } = c.local;
        let update_display::SharedResources {
            adc_val,
        } = c.shared;

        // Create character style
        let style = MonoTextStyle::new(&FONT_10X20, Rgb565::BLACK);

        // Render text to the screen
        let mut buf = [0u8; 64];
        let str = format_no_std::show(&mut buf, format_args!("Current ADC value: {}", adc_val)).unwrap();        
        Text::new(str, Point::new(20,30), style).draw(display).unwrap();

        // reschedule in 1 s
        // update_display::spawn_after(one_second).unwrap();
    }

    // //
    // // ADC reader
    // // Reads current values from ADC and saves them to variables
    // //
    // #[task(local = [adc], shared = [adc_val])]
    // fn read_adc(c: read_adc::Context) {
    //     let read_adc::LocalResources {
    //         adc,
    //     } = c.local;
    //     let read_adc::SharedResources {
    //         mut adc_val,
    //     } = c.shared;

    //     // read channel 0 and 1
    //     let r0 = adc.read_channel(adc_mcp3008::Channels8::CH0).unwrap();
    //     let r1 = adc.read_channel(adc_mcp3008::Channels8::CH1).unwrap();

    //     let mut reading = r0 - r1;

    //     adc_val = &mut reading;

    //     update_display::spawn().unwrap();

    //     // reschedule in 1 s
    //     // read_adc::spawn_after(one_second).unwrap();
    // }

    //
    // Debounce Timer
    // Re-enables button(s) via interrupt after set debounce duration (globally set above)
    // 
    #[task(binds = TIMER_IRQ_1, priority = 2, shared = [ buttons, debounce_alarm])]
    fn debounce_alarm_irq(c: debounce_alarm_irq::Context) {
        let debounce_alarm_irq::SharedResources {
            buttons,
            mut debounce_alarm
        } = c.shared;

        debounce_alarm.lock(|a| {
            a.clear_interrupt();
        });
        let (btn0, btn1, btn2, btn3) = buttons;
        btn0.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);
        btn1.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);
        btn2.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);
        btn3.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);
    }

    //
    // Startup delay
    // Settings to be applied after STARTUP_DELAY after the device is powered on and initialised
    // 
    #[task(binds = TIMER_IRQ_2, priority = 2, local = [startup_delay], shared = [ buttons, fan_pwm])]
    fn startup_delay_irq(c: startup_delay_irq::Context) {
        let startup_delay_irq::LocalResources {
            startup_delay
        } = c.local;
        let startup_delay_irq::SharedResources {
            buttons,
            fan_pwm,
        } = c.shared;

        // Clear and disable startup delay interrupt
        startup_delay.clear_interrupt();
        startup_delay.disable_interrupt();

        // Enable buttons
        let (btn0, btn1, btn2, btn3) = buttons;
        btn0.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);
        btn1.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);
        btn2.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);
        btn3.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);

        // Start fan
        fan_pwm.set_duty(8_000);
    }

    //
    // Interrupt handler for GPIO inputs
    // Handles all state changes on GPIO pins for which interrupts are defined and enabled
    //
    #[task(binds = IO_IRQ_BANK0, priority = 2, /* local = [adc], */ shared = [ adc_data_ready, adc_val, buttons, debounce_alarm, fan_pwm, heater_pwm])]
    fn io_irq_bank0(c: io_irq_bank0::Context) {
        // let io_irq_bank0::LocalResources {
        //     adc
        // } = c.local;
        let io_irq_bank0::SharedResources {
            adc_data_ready,
            mut adc_val,
            buttons,
            mut debounce_alarm,
            fan_pwm,
            heater_pwm,
        } = c.shared;

        // Debounce buttons
        let (btn0, btn1, btn2, btn3) = buttons;
        if btn0.interrupt_status(hal::gpio::Interrupt::EdgeHigh) || btn1.interrupt_status(hal::gpio::Interrupt::EdgeHigh) || btn2.interrupt_status(hal::gpio::Interrupt::EdgeHigh) || btn3.interrupt_status(hal::gpio::Interrupt::EdgeHigh) {
            (debounce_alarm).lock(|a| {
                let _ = a.schedule(DEBOUNCE_INTERVAL);
            });
        }

        // Adc data ready event
        // if adc_data_ready.interrupt_status(hal::gpio::Interrupt::EdgeLow) {
        //     // read from ADC
        //     let mut res = adc.read_unchecked().unwrap();
        //     adc_val = &mut res;

        //     // update display
        //     update_display::spawn().unwrap();
        // }
        // Button events
        if btn0.interrupt_status(hal::gpio::Interrupt::EdgeHigh) {
            btn0.clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
            btn0.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, false);

            // Increase fan speed
            let mut duty = fan_pwm.get_duty();
            if duty < 8_000 {
                duty = 8_000;
            } else if duty <= 64_535 {
                duty += 1_000;
            }
            fan_pwm.set_duty(duty);
        } else if btn1.interrupt_status(hal::gpio::Interrupt::EdgeHigh) {
            btn1.clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
            btn1.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, false);

            // Decrease fan speed
            let mut duty = fan_pwm.get_duty();
            if duty >= 9_000 {
                duty -= 1_000;
            } else {
                duty = 0;
            }
            fan_pwm.set_duty(duty);
        } else if btn2.interrupt_status(hal::gpio::Interrupt::EdgeHigh) {
            btn2.clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
            btn2.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, false);

            // Increase set temperature
            let mut duty = heater_pwm.get_duty();
            if duty <= 64_535 {
                duty += 1_000;
            }
            heater_pwm.set_duty(duty);
        } else if btn3.interrupt_status(hal::gpio::Interrupt::EdgeHigh) {
            btn3.clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
            btn3.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, false);

            // Decrease set temperature
            let mut duty = heater_pwm.get_duty();
            if duty >= 1_000 {
                duty -= 1000;
            } else {
                duty = 0;
            }
            heater_pwm.set_duty(duty);
        }
    }
}
