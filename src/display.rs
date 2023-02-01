/// Text styles for UI elements
enum TextStyle {
    H1,
    H2,
    P,
    PSmall,
}

// #[non_exhaustive]
// struct TextStyle;

// impl TextStyle {
//     pub const H1: MonoTextStyle<'static, Rgb565> = MonoTextStyle::new(&FONT_10X20, Rgb565::BLACK);
//     pub const H2: MonoTextStyle<'static, Rgb565> = MonoTextStyle::new(&FONT_6X13_BOLD, Rgb565::BLACK);
//     pub const P: MonoTextStyle<'static, Rgb565> = MonoTextStyle::new(&FONT_6X13, Rgb565::BLACK);
//     pub const PSmall: MonoTextStyle<'static, Rgb565> = MonoTextStyle::new(&FONT_4X6, Rgb565::BLACK);
// }

/// Style annotated text for embedded graphics display
#[allow(dead_code)]
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
struct Point {
    x: i32,
    y: i32,
}

/// Graph
type Graph = [Point; 100];

/// UI Configuration for embedded graphics display
#[allow(dead_code)]
pub struct UI {
    temp_set: HeadingValuePair,
    temp: HeadingValuePair,
    measurements: [HeadingValuePair; 3],
    gradient: Graph,
    time: HeadingValuePair,
}

impl UI {
    pub fn new() -> Self {
        Self {
            temp_set: [
                StyledText::new(
                    "Set Temperature [°C]",
                    TextStyle::H1,
                ),
                StyledText::new(
                    "20",
                    TextStyle::P,
                ),
            ],
            temp: [
                StyledText::new(
                    "Current Temperature [°C]",
                    TextStyle::H1,
                ),
                StyledText::new(
                    "",
                    TextStyle::P,
                ),
            ],
            measurements: [
                [
                    StyledText::new(
                        "Sample 1",
                        TextStyle::H2,
                    ),
                    StyledText::new(
                        "",
                        TextStyle::PSmall,
                    ),
                ],
                [
                    StyledText::new(
                        "Sample 2",
                        TextStyle::H2,
                    ),
                    StyledText::new(
                        "",
                        TextStyle::PSmall,
                    ),
                ],
                [
                    StyledText::new(
                        "Sample 3",
                        TextStyle::H2,
                    ),
                    StyledText::new(
                        "",
                        TextStyle::PSmall,
                    ),
                ],
            ],
            gradient: [
                Point {x: 0, y: 0}; 100
            ],
            time: [
                StyledText::new(
                    "Elapsed Time",
                    TextStyle::H1,
                ),
                StyledText::new(
                    "0:00",
                    TextStyle::P,
                ),
            ],
        }
    }
}
