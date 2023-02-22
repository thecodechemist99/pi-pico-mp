#![no_std]
#![no_main]

mod delay;
mod display;
// mod ui;

//
// RTIC app configuration for Raspberry Pi Pico
// Otherwise not needed interrupt controllers are assigned as dispatchers for software interrupts.
//
#[rtic::app(device = rp_pico::pac, peripherals = true, dispatchers = [PIO0_IRQ_0, PIO0_IRQ_1, PIO1_IRQ_0, PIO1_IRQ_1])]
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
            timer::{ Timer, Alarm, Alarm1, Alarm2, Alarm3 },
            watchdog::Watchdog,
            pac::{ SPI0, I2C0 },
            Sio,
            gpio::{ bank0::*, Pin, PullDownInput, PushPullOutput },
            pwm::*,
        },
        XOSC_CRYSTAL_FREQ,
    };

    // Time traits
    use fugit::{
        RateExtU32,
        MicrosDurationU32,
    };
    use crate::delay::Delay;

    // Display and UI traits
    use display_interface_spi::SPIInterface;
    use mipidsi::{
        Builder,
        Display,
        Orientation,
    }; 
    use embedded_graphics::{
        prelude::*,
        pixelcolor::Rgb565,
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
    use pid::Pid;

    // RTD traits
    use pt_rtd::{
        self as rtd,
        RTDType,
        ADCRes,
    };

    // Global settings
    const DEBOUNCE_INTERVAL: MicrosDurationU32 = MicrosDurationU32::millis(200);
    const STARTUP_DELAY: MicrosDurationU32 = MicrosDurationU32::millis(2_500);
    const UI_TIMER: MicrosDurationU32 = MicrosDurationU32::millis(1_000);

    enum State {
        Idle,
        Run,
        Cal,
        Error,
    }

    // Type declaration of globally shared variables (can be securely accessed by multiple interrupt handlers)
    #[shared]
    struct Shared {
        #[lock_free]
        ui: UI,
        ui_timer: Alarm3,
        #[lock_free]
        buttons: (Pin<Gpio18,PullDownInput>, Pin<Gpio19,PullDownInput>, Pin<Gpio20,PullDownInput>, Pin<Gpio21,PullDownInput>),
        #[lock_free]
        pwm: (Channel<Pwm0, FreeRunning, A>, Channel<Pwm0, FreeRunning, B>),
        #[lock_free]
        adc_data_ready: Pin<Gpio11,PullDownInput>,
        debounce_alarm: Alarm1,
    }

    // Type declaration of locally shared variables (can be assigned to only one interrupt handler)
    #[local]
    struct Local {
        startup_delay: Alarm2,
        display: Display<SPIInterface<hal::spi::Spi<hal::spi::Enabled,SPI0,8>, Pin<Gpio6,PushPullOutput>, Pin<Gpio5,PushPullOutput>>, mipidsi::models::ST7789, Pin<Gpio7,PushPullOutput>>,
        lcd_bl: Pin<Gpio8,PushPullOutput>,
        adc: Nau7802<hal::I2C<I2C0, (Pin<Gpio12, hal::gpio::FunctionI2C>, Pin<Gpio13, hal::gpio::FunctionI2C>)>>,
        pid: (Pid<f32>, Pid<f32>),
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

        let mut ui_timer = timer.alarm_3().unwrap();
        ui_timer.enable_interrupt();

        // Wait until initialisation is comleted to run certain startup tasks
        let mut startup_delay = timer.alarm_2().unwrap();
        startup_delay.schedule(STARTUP_DELAY).unwrap();
        startup_delay.enable_interrupt();

        // Delay initialisation
        let mut delay = Delay::new();

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

        defmt::info!("SPI bus initialised.");

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
        defmt::info!("I²C bus initialised.");

        //
        // PWM initialisation
        //
        let pwm_slices = hal::pwm::Slices::new(c.device.PWM, &mut resets);
        let mut pwm = pwm_slices.pwm0;
        pwm.set_ph_correct();
        pwm.set_div_int(255u8); // 133MHz/255=521.6kHz
        pwm.enable();

        // Output channel A on PWM0 to GPIO 16
        let mut fan_pwm = pwm.channel_a;
        fan_pwm.output_to(pins.gpio16);
        fan_pwm.set_duty(8_000);

        // Output channel B on PWM0 to GPIO 17
        let mut heater_pwm = pwm.channel_b;
        heater_pwm.output_to(pins.gpio17);
        heater_pwm.set_duty(0);

        defmt::info!("PWM initialised.");

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
        let mut display = match Builder::st7789(SPIInterface::new(spi, dc, lcd_cs))
            .with_display_size(320, 240)
            .with_invert_colors(true)
            .with_orientation(Orientation::Landscape(true))
            .init(&mut delay, Some(rst))
        {
            Ok(disp) => disp,
            Err(e) => defmt::panic!("Error initialising display: {:?}", defmt::Debug2Format(&e)),
        };

        display.clear(Rgb565::WHITE).unwrap();
        
        let mut ui = UI::new();
        defmt::info!("Display initialised.");

        //
        // ADC initialisation
        //
        let adc_data_ready = pins.gpio11.into_pull_down_input();
        let adc = match Nau7802::new_with_settings(
            i2c,
            Ldo::L3v3,
            Gain::G16,
            SamplesPerSecond::SPS10,
            &mut delay,
        ) {
            Ok(adc) => adc,
            Err(e) => defmt::panic!("Error initilising ADC: {:?}", defmt::Debug2Format(&e)), // TODO: Retry after 100 ms before panicking (how to reaccess i2c?)
        };
        defmt::info!("ADC initialised.");

        //
        // PID controller initialisation
        //
        let setpoint = 50.0;
        
        // Show setpoint in UI
        static mut buf: [u8; 8] = [0u8; 8];
        unsafe { // FIXME: Find a solution without unsafe code
            ui.update_setpoint(setpoint, &mut buf);
        }
        
        // Create PID controller with setpoint and output limit
        let mut fan_pid = Pid::new(setpoint, 10_000.0);
        let mut heater_pid = Pid::new(setpoint, 10_000.0);

        // Set fan PID constants
        fan_pid.p(500.0, 10_000.0);
        fan_pid.i(0.5, 200.0);
        fan_pid.d(10.0, 10.0);

        // Set heater PID constants
        heater_pid.p(200.0, 10_000.0);
        heater_pid.i(0.1, 100.0);
        heater_pid.d(10.0, 10.0);

        defmt::info!("PID controller initialised with setpoint {:?}.", setpoint);

        (
            Shared {
                ui,
                ui_timer,
                buttons,
                pwm: (fan_pwm, heater_pwm),
                adc_data_ready,
                debounce_alarm,
            },
            Local {
                startup_delay,
                display,
                lcd_bl,
                adc,
                pid: (fan_pid, heater_pid),
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
    #[task(priority = 2, local = [display, lcd_bl], shared = [ui])]
    fn update_display(c: update_display::Context) {
        let update_display::LocalResources {
            display,
            lcd_bl,
            // ui,
        } = c.local;
        let update_display::SharedResources {
            ui,
        } = c.shared;

        // Draw UI
        crate::display::draw(ui, display);

        // Enable backlight if it is not enabled (on startup to hide random pixels on power up)
        lcd_bl.set_high().unwrap();
    }

    //
    // Read ADC
    // Reads current values from ADC and saves them to variables
    //
    #[task(priority = 2, local = [adc], shared = [ui])]
    fn read_adc(c: read_adc::Context) {
        let read_adc::LocalResources {
            adc,
        } = c.local;
        let read_adc::SharedResources {
            ui,
        } = c.shared;

        match adc.read() {
            Ok(val) => {
                // Convert ADC reading to resistance value
                match rtd::conv_d_val_to_r(val as u32, 5623, ADCRes::B24, 16) {
                    Ok(r) => {
                        // Get temperature from resistance value
                        defmt::debug!("ADC value: {:?}, resistance value: {:?}", val as u32, r);
                        match rtd::calc_t(r, RTDType::PT100) {
                            Ok(t) => {
                                // Hand temperature value to PID controller
                                t_control::spawn(t).ok().unwrap();

                                // Update temperature in UI
                                static mut buf: [u8; 8] = [0u8; 8];
                                unsafe { // FIXME: Find a solution without unsafe code
                                    ui.update_temp(t, &mut buf);
                                }
                            },
                            Err(e) => defmt::error!("Error calculating RTD temperature: {:?}", defmt::Debug2Format(&e)),
                        }
                    },
                    Err(e) => defmt::error!("Error calculating RTD resistance: {:?}", defmt::Debug2Format(&e)),
                }
            },
            Err(e) => defmt::panic!("Error reading from ADC: {:?}", defmt::Debug2Format(&e)),
        };
    }

    //
    // Temperature controller
    // PID temperature regulation based on current ADC readings
    //
    #[task(priority = 3, local = [pid], shared = [pwm])]
    fn t_control(c: t_control::Context, t: f32) {
        let (fan_pid, heater_pid) = c.local.pid;
        let (fan_pwm, heater_pwm) = c.shared.pwm;

        // Adjust heater
        let output = heater_pid.next_control_output(t);
        let heater_duty = heater_pwm.get_duty() as i32 + output.output as i32;
        match heater_duty {
            0..=64_535 => heater_pwm.set_duty(heater_duty as u16),
            _ => (),
        }

        // Adjust fan
        let output = fan_pid.next_control_output(t);
        let fan_duty = fan_pwm.get_duty() as i32 - output.output as i32;
        match fan_duty {
            0..=9_500 => fan_pwm.set_duty(0),
            9_500..=64_535 => fan_pwm.set_duty(fan_duty as u16),
            _ => (),
        }
    }

    //
    // Startup delay
    // Settings to be applied after STARTUP_DELAY when the device is powered on
    // 
    #[task(binds = TIMER_IRQ_2, priority = 3, local = [startup_delay], shared = [ui_timer, adc_data_ready, buttons])]
    fn startup_delay_irq(c: startup_delay_irq::Context) {
        let startup_delay_irq::LocalResources {
            startup_delay,
        } = c.local;
        let startup_delay_irq::SharedResources {
            mut ui_timer,
            adc_data_ready,
            buttons,
        } = c.shared;

        // Clear and disable startup delay interrupt
        startup_delay.clear_interrupt();
        startup_delay.disable_interrupt();

        // Start ui timer
        ui_timer.lock(|a| {
            let _ = a.schedule(UI_TIMER);
        });

        // Enable buttons
        let (btn0, btn1, btn2, btn3) = buttons;
        btn0.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);
        btn1.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);
        btn2.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);
        btn3.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);

        // Enable ADC data ready interrupt
        adc_data_ready.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);

        // Turn on and update display
        update_display::spawn().unwrap();

        defmt::info!("Startup finished successfully.");
    }

    //
    // UI timer
    // Counts seconds of runtime after measurement is started and initiates display updates
    // 
    #[task(binds = TIMER_IRQ_3, priority = 2, local = [ms: u32 = 0], shared = [ui, ui_timer])]
    fn ui_timer_irq(c: ui_timer_irq::Context) {
        let ui_timer_irq::LocalResources {
            ms,
        } = c.local;
        let ui_timer_irq::SharedResources {
            ui,
            mut ui_timer,
        } = c.shared;

        // Clear and reschedule ui timer interrupt
        ui_timer.lock(|a| {
            a.clear_interrupt();
            let _ = a.schedule(UI_TIMER);
        });

        // Increment runtime
        *ms += 1_000;
        static mut buf: [u8; 8] = [0u8; 8];
        unsafe { // FIXME: Find a solution without unsafe code
            ui.update_time(*ms, &mut buf);
        }

        // Update display every 5 seconds
        if *ms % 5000 == 0 {
            update_display::spawn().unwrap();
        }

    }

    //
    // Debounce Timer
    // Re-enables button(s) via interrupt after set debounce interval
    // 
    #[task(binds = TIMER_IRQ_1, priority = 3, shared = [ buttons, debounce_alarm])]
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
    #[task(binds = IO_IRQ_BANK0, priority = 3, shared = [ adc_data_ready, buttons, debounce_alarm])]
    fn io_irq_bank0(c: io_irq_bank0::Context) {
        let io_irq_bank0::SharedResources {
            adc_data_ready,
            buttons,
            mut debounce_alarm,
        } = c.shared;
        
        // Debounce buttons
        let (btn0, btn1, btn2, btn3) = buttons;
        if btn0.interrupt_status(hal::gpio::Interrupt::EdgeHigh) || btn1.interrupt_status(hal::gpio::Interrupt::EdgeHigh) || btn2.interrupt_status(hal::gpio::Interrupt::EdgeHigh) || btn3.interrupt_status(hal::gpio::Interrupt::EdgeHigh) {
            debounce_alarm.lock(|a| {
                let _ = a.schedule(DEBOUNCE_INTERVAL);
            });
        }

        // ADC data ready event
        if adc_data_ready.interrupt_status(hal::gpio::Interrupt::EdgeHigh) {
            adc_data_ready.clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
            read_adc::spawn().unwrap();
        }

        // Button press events
        if btn0.interrupt_status(hal::gpio::Interrupt::EdgeHigh) {
            btn0.clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
            btn0.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, false);

        } else if btn1.interrupt_status(hal::gpio::Interrupt::EdgeHigh) {
            btn1.clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
            btn1.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, false);

        } else if btn2.interrupt_status(hal::gpio::Interrupt::EdgeHigh) {
            btn2.clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
            btn2.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, false);

        } else if btn3.interrupt_status(hal::gpio::Interrupt::EdgeHigh) {
            btn3.clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
            btn3.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, false);

        }
    }
}
