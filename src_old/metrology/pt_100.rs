// Sensor specific methods according to DIN EN 60751:2009-05
// (c) 2023 Florian Beck

use libm::*;

type TRange = (i32, i32);
type Accuracy = (f32, f32);

// implementation of t_range only for film resistors, accurracy is independent of resistor type
struct Class {
  F01: (TRange, Accuracy),
  F015: (TRange, Accuracy),
  F03: (TRange, Accuracy),
  F06: (TRange, Accuracy),
}

const CLASS: Class = Class {
  F01: (
    (0, 150),
    (0.1, 0.0017),
  ),
  F015: (
    (-30, 300),
    (0.15, 0.002),
  ),
  F03: (
    (-50, 500),
    (0.3, 0.005),
  ),
  F06: (
    (-50, 600),
    (0.6, 0.01),
  ),
};

struct VRange {
  V25: f32,
  V30: f32,
  V33: f32,
  V50: f32,
}

const V_RANGE: VRange = VRange {
  V25: 2.50,
  V30: 3.00,
  V33: 3.30,
  V50: 5.00,
};

// Data type for Pt thermo resistor configuration
struct Config {
  class: (TRange, Accuracy),
  r_0: f32,
}

// static config implementation for testing
const CONFIG: Config = Config {
  class: CLASS.F015,
  r_0: 100_f32,
};

// derive resistance from voltage
fn get_r_t (adc_counts: u16) -> f32 {
  // add variables parameters for ADC resolution and range?
  let adc_resolution: u16 = 4096;
  let v_range:f32 = V_RANGE.V33;

  (adc_counts / adc_resolution) as f32 * v_range
}

// calculate temp from resistance
pub fn get_temp (adc_counts: u16) -> f32 {
  let r_t = get_r_t(adc_counts);

  // implementation only for t>0 °C due to highly reduced complexity
  let a: f32 = 3.908 * powf(10_f32, -3_f32);
  let b: f32 = -5.775 * powf(10_f32, -7_f32);
  // let c: f32 = -4.183 * 10.pow(-12);

  // formula from https://techoverflow.net/2016/01/02/accurate-calculation-of-pt100pt1000-temperature-from-resistance/
  return (-CONFIG.r_0 * a + sqrtf(powf(CONFIG.r_0, 2_f32) * powf(a, 2_f32) - 4_f32 * CONFIG.r_0 * b * (CONFIG.r_0 - r_t)))/(2_f32 * CONFIG.r_0 * b)
}
