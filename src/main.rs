#![no_std]
#![no_main]

mod delay;
mod display;
mod rtd;

//
// RTIC app configuration for Raspberry Pi Pico
// Otherwise not needed interrupt controllers are assigned as dispatchers for software interrupts.
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
            pac::{ SPI0, I2C0 },
            Sio,
            gpio::{ bank0::*, Pin, PullDownInput, PushPullOutput },
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
                FONT_6X10,
                FONT_10X20,
            },
            MonoTextStyle
        },
        text::Text, primitives::Rectangle,
    };
    use crate::display::UI;

    // ADC traits
    use nau7802::{
        Nau7802,
        Ldo,
        Gain,
        SamplesPerSecond,
    };

    // PID traits
    // use pid_ctrl::PidCtrl;
    use pid::{
        Pid,
        ControlOutput,
    };

    // RTD traits
    use crate::rtd::{
        self,
        RTDType,
        ADCRes,
    };

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
        adc_data_ready: Pin<Gpio11,PullDownInput>,
        // delay: Delay,
        debounce_alarm: Alarm1,
    }

    // Type declaration of locally shared variables (can be assigned to only one interrupt handler)
    #[local]
    struct Local {
        startup_delay: Alarm2,
        display: Display<SPIInterface<hal::spi::Spi<hal::spi::Enabled,SPI0,8>, Pin<Gpio6,PushPullOutput>, Pin<Gpio5,PushPullOutput>>, mipidsi::models::ST7789, Pin<Gpio7,PushPullOutput>>,
        lcd_bl: Pin<Gpio8,PushPullOutput>,
        adc: Nau7802<hal::I2C<I2C0, (Pin<Gpio12, hal::gpio::FunctionI2C>, Pin<Gpio13, hal::gpio::FunctionI2C>)>>,
        pid: Pid<f32>,
        ui: UI,
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
        // Soft-reset does not release the hardware spinlocks
        // Release them now to avoid a deadlock after debug or watchdog reset
        unsafe {
            hal::sio::spinlock_reset();
        }

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

        // Wait until initialisation is comleted to run certain startup tasks
        let mut startup_delay = timer.alarm_2().unwrap();
        startup_delay.schedule(STARTUP_DELAY).unwrap();
        startup_delay.enable_interrupt();

        // Delay initialisation
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
        let sda_pin = pins.gpio12.into_mode::<hal::gpio::FunctionI2C>();
        let scl_pin = pins.gpio13.into_mode::<hal::gpio::FunctionI2C>();

        let i2c = hal::I2C::i2c0(
            c.device.I2C0,
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
        let lcd_bl = pins.gpio8.into_push_pull_output();

        lcd_cs.set_high().unwrap();
        let mut display = Builder::st7789(SPIInterface::new(spi, dc, lcd_cs))
            .with_display_size(320, 240)
            .with_invert_colors(true)
            .with_orientation(Orientation::Landscape(true))
            .init(&mut delay, Some(rst)).unwrap();
        display.clear(Rgb565::WHITE).unwrap();

        let ui = UI::new();

        //
        // ADC initialisation
        //
        let adc_data_ready = pins.gpio11.into_pull_down_input();
        adc_data_ready.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);

        let adc = Nau7802::new_with_settings(
            i2c,
            Ldo::L3v3,
            Gain::G16,
            SamplesPerSecond::SPS10,
            &mut delay
        )
        .ok()
        .unwrap();

        //
        // PID controller initialisation
        //
        // create new PID controller with setpoint and output limit
        let mut pid = Pid::new(50.0, 10_000.0);
        // use only proportional control
        pid.p(200.0, 10_000.0);
        pid.i(0.1, 100.0);
        pid.d(10.0, 10.0);

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
                display,
                lcd_bl,
                ui,
                adc,
                pid,
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
            rtic::export::wfi();
        }
    }

    //
    // Update Display
    // Takes current values of supplied variables and draws them on the connected display
    //
    #[task(priority = 2, local = [display, lcd_bl, ui], shared = [heater_pwm, fan_pwm])]
    fn update_display(c: update_display::Context, values: (f32, f32, f32, f32, f32)) {
        let update_display::LocalResources {
            display,
            lcd_bl,
            ui,
        } = c.local;
        let update_display::SharedResources {
            heater_pwm,
            fan_pwm,
        } = c.shared;

        // Enable backlight if it is not enabled (on startup to hide random pixels on power up)
        lcd_bl.set_high().unwrap();

        // Create character style
        let style = MonoTextStyle::new(&FONT_10X20, Rgb565::BLACK);

        // clear background
        display.clear(Rgb565::WHITE).unwrap();

        // Draw UI
        // ui

        // Render text to the screen
        let (t, p, i, d, o) = values;

        let mut buf = [0u8; 64];
        let str = format_no_std::show(&mut buf, format_args!("Current Temperature: {:.2}", t)).unwrap();
        Text::new(str, Point::new(20,30), style).draw(display).unwrap();

        let mut buf = [0u8; 64];
        let str = format_no_std::show(&mut buf, format_args!("Set Temperature: {:.1}", 50.0)).unwrap();
        Text::new(str, Point::new(20,60), style).draw(display).unwrap();

        let mut buf = [0u8; 64];
        let str = format_no_std::show(&mut buf, format_args!("P: {:.1}, I: {:.1}, D: {:.1}", p, i, d)).unwrap();
        Text::new(str, Point::new(20,90), style).draw(display).unwrap();

        let mut buf = [0u8; 64];
        let str = format_no_std::show(&mut buf, format_args!("PID Output: {:.1}", o)).unwrap();
        Text::new(str, Point::new(20,120), style).draw(display).unwrap();

        let mut buf = [0u8; 64];
        let heater_duty = heater_pwm.get_duty();
        let str = format_no_std::show(&mut buf, format_args!("Heater Duty: {}", heater_duty)).unwrap();
        Text::new(str, Point::new(20,150), style).draw(display).unwrap();

        let mut buf = [0u8; 64];
        let fan_duty = fan_pwm.get_duty();
        let str = format_no_std::show(&mut buf, format_args!("Fan Duty: {}", fan_duty)).unwrap();
        Text::new(str, Point::new(20,180), style).draw(display).unwrap();
    }

    //
    // Read ADC
    // Reads current values from ADC and saves them to variables
    //
    #[task(priority = 2, local = [adc, values: [i32; 10] = [0; 10], counter: usize = 0], shared = [adc_val])]
    fn read_adc(c: read_adc::Context) {
        let read_adc::LocalResources {
            adc,
            values,
            counter,
        } = c.local;
        let read_adc::SharedResources {
            adc_val,
        } = c.shared;

        match adc.read() {
            Ok(val) => {
                let r = rtd::conv_d_val_to_r(val, 5624, ADCRes::B24, 16).unwrap();
                let t = rtd::calc_t(r, RTDType::PT100).unwrap();
        //         values[*counter] = val;
        //         if *counter >= 9_usize {
        //             // Calculate average (src: https://codereview.stackexchange.com/questions/173338/calculate-mean-median-and-mode-in-rust)
        //             let mut sum: i32 = 0;
        //             for x in *values {
        //                 sum += x;
        //             }
        //             unsafe { *adc_val = roundf(sum as f32 / values.len() as f32).to_int_unchecked::<i32>(); }
        //             update_display::spawn(*adc_val).ok().unwrap();
        //         }
        //         *counter += 1;
                t_control::spawn(t).ok().unwrap();
            },
            Err(e) => defmt::panic!("Error reading from ADC."),
        };
    }

    //
    // Temperature controller
    // PID based temperature regulation based on current ADC readings
    //
    #[task(priority = 2, local = [pid], shared = [fan_pwm, heater_pwm])]
    fn t_control(c: t_control::Context, temp: f32) {
        let t_control::LocalResources {
            pid,
        } = c.local;
        let t_control::SharedResources {
            fan_pwm,
            heater_pwm,
        } = c.shared;

        // Get PID controller output
        let output = pid.next_control_output(temp);

        // Adjust heater
        let mut heater_duty = heater_pwm.get_duty() as i32;
        heater_duty += output.output as i32;
        if heater_duty >= 0 && heater_duty <= 64_535 {
            heater_pwm.set_duty(heater_duty as u16);
        }

        // Adjust fan
        let mut fan_duty = fan_pwm.get_duty() as i32;
        fan_duty -= output.output as i32;
        if fan_duty + 200 >= 0 && fan_duty <= 64_535 {
            if fan_duty <= 9_500 && output.output < 200_f32 { // somewhat bias the fan start value to turn the fan on before the PID output gets below 0
                fan_duty = 9_500;
            } else if fan_duty <= 9_500 {
                fan_duty = 0;
            }
            fan_pwm.set_duty(fan_duty as u16);
        }

        update_display::spawn((temp, output.p, output.i, output.d, output.output)).ok().unwrap();
    }

    //
    // Startup delay
    // Settings to be applied after STARTUP_DELAY after the device is powered on
    // 
    #[task(binds = TIMER_IRQ_2, priority = 2, local = [startup_delay], shared = [ buttons, fan_pwm, heater_pwm])]
    fn startup_delay_irq(c: startup_delay_irq::Context) {
        let startup_delay_irq::LocalResources {
            startup_delay,
        } = c.local;
        let startup_delay_irq::SharedResources {
            buttons,
            fan_pwm,
            heater_pwm,
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
        fan_pwm.set_duty(0);

        // Set heater off
        heater_pwm.set_duty(0);

        // Turn on and update display
        update_display::spawn((0_f32, 0_f32, 0_f32, 0_f32, 0_f32)).unwrap();
    }

    //
    // Debounce Timer
    // Re-enables button(s) via interrupt after set debounce duration
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
    // Interrupt handler for GPIO inputs
    // Handles all state changes on GPIO pins for which interrupts are defined and enabled
    //
    #[task(binds = IO_IRQ_BANK0, priority = 2, shared = [ adc_data_ready, buttons, debounce_alarm, fan_pwm, heater_pwm])]
    fn io_irq_bank0(c: io_irq_bank0::Context) {
        let io_irq_bank0::SharedResources {
            adc_data_ready,
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

        // ADC data ready event
        if adc_data_ready.interrupt_status(hal::gpio::Interrupt::EdgeHigh) {
            adc_data_ready.clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
            // read from ADC
            read_adc::spawn().unwrap();
        }
        // Button events
        // using no else if because of colision with regular adc data ready events
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
