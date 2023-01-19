// Hardware interface with ADC
// (c) 2023 Florian Beck

use cortex_m::prelude::_embedded_hal_adc_OneShot;
use rp2040_hal::{
  Adc,
  gpio::{
    Pin,
    bank0::*,
    Input,
    Floating
  },
};

mod pt_100;

pub fn get_temp (adc: &mut Adc, adc_pin: &mut Pin<Gpio26, Input<Floating>>) -> f32 {
  let adc_counts: u16 = adc.read(adc_pin).unwrap();
  pt_100::get_temp(adc_counts)
}
