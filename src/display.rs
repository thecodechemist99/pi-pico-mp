use embedded_graphics::{
    prelude::*,
    pixelcolor::Rgb565,
    draw_target::DrawTarget,
    mono_font::{
        iso_8859_1::{
            FONT_4X6,
            FONT_6X13,
            FONT_6X13_BOLD,
            FONT_8X13,
            FONT_8X13_BOLD,
            FONT_10X20,
        },
        MonoTextStyle
    },
    text::Text, primitives::Rectangle,
};
use profont::{
    PROFONT_24_POINT,
};
use mipidsi::{
    Display,
    models::ST7789,
};
use display_interface_spi::SPIInterface;

use rp_pico::{
    hal::{
        gpio::{
            bank0::*,
            Pin,
            PushPullOutput,
        },
        Spi,
        spi::Enabled,
    },
    pac::SPI0
};

#[derive(Clone)]
#[derive(Copy)]
enum TextStyle {
    H1,
    H2,
    P,
    PLarge,
    PSmall
}

/// Style annotated text for embedded graphics display
#[allow(dead_code)]
#[derive(Clone)]
#[derive(Copy)]
struct StyledText {
    fmt_text: &'static str,
    style: TextStyle,
}

impl StyledText {
    fn new(text: &'static str, style: TextStyle) -> Self {
        Self { fmt_text: text, style }
    }
}

type HeadingValuePair = [StyledText; 2];

/// Point for graph
#[derive(Clone)]
#[derive(Copy)]
#[allow(dead_code)]
struct GraphPoint {
    x: i32,
    y: i32,
}

/// Graph
type Graph = [GraphPoint; 100];

/// UI Configuration for embedded graphics display
#[allow(dead_code)]
#[derive(Clone)]
#[derive(Copy)]
pub struct UI {
    setpoint: HeadingValuePair,
    temp: HeadingValuePair,
    measurements: [HeadingValuePair; 3],
    gradient: Graph,
    time: HeadingValuePair,
}

impl UI {
    pub fn new() -> Self {
        Self {
            setpoint: [
                StyledText::new(
                    "Setpoint",
                    TextStyle::H1,
                ),
                StyledText::new(
                    "",
                    TextStyle::PLarge,
                ),
            ],
            temp: [
                StyledText::new(
                    "Temperature",
                    TextStyle::H1,
                ),
                StyledText::new(
                    "",
                    TextStyle::PLarge,
                ),
            ],
            measurements: [
                [
                    StyledText::new(
                        "Sample 1: ",
                        TextStyle::H2,
                    ),
                    StyledText::new(
                        "",
                        TextStyle::P,
                    ),
                ],
                [
                    StyledText::new(
                        "Sample 2: ",
                        TextStyle::H2,
                    ),
                    StyledText::new(
                        "",
                        TextStyle::P,
                    ),
                ],
                [
                    StyledText::new(
                        "Sample 3: ",
                        TextStyle::H2,
                    ),
                    StyledText::new(
                        "",
                        TextStyle::P,
                    ),
                ],
            ],
            gradient: [
                GraphPoint {x: 0, y: 0}; 100
            ],
            time: [
                StyledText::new(
                    "Time:",
                    TextStyle::H2,
                ),
                StyledText::new(
                    "0:00 m",
                    TextStyle::P,
                ),
            ],
        }
    }

    pub fn update_setpoint(&mut self, setpoint: f32, buf: &'static mut [u8; 8]) {
        self.setpoint[1].fmt_text = format_no_std::show(buf, format_args!("{:.1} °C", setpoint)).unwrap();
    }

    pub fn update_temp(&mut self, temp: f32, buf: &'static mut [u8; 8]) {
        self.temp[1].fmt_text = format_no_std::show(buf, format_args!("{:.1} °C", temp)).unwrap();
    }

    pub fn update_measurements(&mut self, sample: usize, temp: f32, buf: &'static mut [u8; 8]) {
        let sample = &mut self.measurements[sample - 1];
        sample[1].fmt_text = format_no_std::show(buf, format_args!("{:.1} °C", temp)).unwrap();
    }

    pub fn update_time(&mut self, ms: u32, buf: &'static mut [u8; 8]) {
        let m = ms / 60_000;
        let s = ms / 1_000 % 60;
        self.time[1].fmt_text = format_no_std::show(buf, format_args!("{}:{} m", m, s)).unwrap();
    }
}

/// Draw UI to embedded graphics display
pub fn draw(ui: &mut UI, display: &mut Display<SPIInterface<Spi<Enabled, SPI0, 8>, Pin<Gpio6, PushPullOutput>, Pin<Gpio5, PushPullOutput>>, ST7789, Pin<Gpio7, PushPullOutput>>) {
        // TODO: Only clear sections and redraw once a value has changed
        // clear background
        display.clear(Rgb565::WHITE).unwrap();

        // Draw setpoint
        let setpoint_box = display.bounding_box();
        for (i, text) in ui.setpoint.iter().enumerate() {
            let style = match text.style {
                TextStyle::H1 => Ok(MonoTextStyle::new(&FONT_10X20, Rgb565::BLACK)),
                TextStyle::PLarge => Ok(MonoTextStyle::new(&PROFONT_24_POINT, Rgb565::BLACK)),
                _ => Err(Error::UnexpectedTextStyle),
            }.unwrap();
            Text::new(text.fmt_text, Point::new(20 + i as i32 * 10, 20 + i as i32 * 40), style).draw(display).unwrap();
        };

        // Draw temperature
        let temp_box = display.bounding_box();
        for (i, text) in ui.temp.iter().enumerate() {
            let style = match text.style {
                TextStyle::H1 => Ok(MonoTextStyle::new(&FONT_10X20, Rgb565::BLACK)),
                TextStyle::PLarge => Ok(MonoTextStyle::new(&PROFONT_24_POINT, Rgb565::BLACK)),
                _ => Err(Error::UnexpectedTextStyle),
            }.unwrap();
            Text::new(text.fmt_text, Point::new(180 + i as i32 * 10, 20 + i as i32 * 40), style).draw(display).unwrap();
        };

        // Draw measurements
        let measurements_box = display.bounding_box();
        for (i, measurement) in ui.measurements.iter().enumerate() {
            for (j, text) in measurement.iter().enumerate() {
                let style = match text.style {
                    TextStyle::H2 => Ok(MonoTextStyle::new(&FONT_8X13_BOLD, Rgb565::BLACK)),
                    TextStyle::P => Ok(MonoTextStyle::new(&FONT_8X13, Rgb565::BLACK)),
                    _ => Err(Error::UnexpectedTextStyle),
                }.unwrap();
                Text::new(text.fmt_text, Point::new(180 + j as i32 * 60, 100 + i as i32 * 25), style).draw(display).unwrap();
            };
        };

        // Draw time
        let time_box = display.bounding_box();
        for (i, text) in ui.time.iter().enumerate() {
            let style = match text.style {
                TextStyle::H2 => Ok(MonoTextStyle::new(&FONT_8X13_BOLD, Rgb565::BLACK)),
                TextStyle::P => Ok(MonoTextStyle::new(&FONT_8X13, Rgb565::BLACK)),
                _ => Err(Error::UnexpectedTextStyle),
            }.unwrap();
            Text::new(text.fmt_text, Point::new(20 + i as i32 * 60, 150), style).draw(display).unwrap();
        };

        // Draw button descriptions
}

#[derive(Debug)]
pub enum Error {
    UnexpectedTextStyle,
    DisplayError,
}
