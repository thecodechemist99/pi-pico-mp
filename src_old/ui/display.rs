// device related imports
use embedded_hal::digital::v2::OutputPin;
use cortex_m::delay::Delay;

// Spi
use crate::hal::{
    Spi,
    spi::*,
};
use crate::pac::SPI0;
use crate::hal::gpio::{DynPin, Pin, Function};

// display driver related imports
use display_interface_spi::{SPIInterfaceNoCS, SPIInterface};
use mipidsi::models::*;
use mipidsi::{
    Builder,
    Display,
};

// add error handling!
pub fn init_display (spi: Spi<Enabled, SPI0, 8>, cs: DynPin, mut dc: DynPin, mut rst: DynPin, mut bl: DynPin, delay: &mut Delay) -> Display<SPIInterfaceNoCS<Spi<Enabled, SPI0, 8>, DynPin>, ST7789, DynPin> {
    // set dc, rst to PushPullOutput
    // see https://docs.rs/rp2040-hal/latest/rp2040_hal/gpio/dynpin/index.html for reference
    dc.into_push_pull_output();
    rst.into_push_pull_output();
    bl.into_push_pull_output();
    bl.set_high().unwrap(); // Error handling!

    // create display interface from spi interface and dc pin
    // code derived from https://github.com/almindor/mipidsi
    let display_interface = SPIInterfaceNoCS::new(spi, dc);
    // let display_interface = SPIInterface::new(spi, dc, cs);

    // return display instance

    Builder::st7789(display_interface)
        .with_display_size(320, 240)
        .with_invert_colors(true)
        .init(delay, Some(rst))
        .unwrap()
}