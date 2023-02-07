//! Calculation methods for platinum RTD temperature sensors.
//! All temperature related calculations are based on DIN EN 60751:2009-05.
//! No Method for temperature calculation at below 0°C is implemented due to much increased complexity.

use libm::{
    powf,
    sqrtf,
    floorf,
};

#[allow(dead_code)]
#[derive(Clone)]
#[derive(Copy)]
pub enum ADCRes {
    B8 = 255,
    B10 = 1_023,
    B12 = 4_095,
    B14 = 16_383,
    B16 = 65_535,
    B18 = 262_143,
    B20 = 1_048_575,
    B22 = 4_194_303,
    B24 = 16_777_215,
}

#[allow(dead_code)]
#[derive(Clone)]
#[derive(Copy)]
pub enum RTDType {
    PT100 = 100,
    PT200 = 200,
    PT500 = 500,
    PT1000 = 1000,
}

const A: f32 = 0.0039083;
const B: f32 = -0.0000005775;
const C: f32 = -0.000000000004183;

/// Calculate temperature of RTD from resistance value (for t >= 0°C).
#[allow(dead_code)]
pub fn calc_t(r: f32, r_0: RTDType) -> Result<f32, Error> {
    let r_max = floorf(calc_r(850_f32, r_0).unwrap()) as i32;
    let r_0 = r_0 as i32;

    if floorf(r) as i32 >= r_0 && floorf(r) as i32 <= r_max {
        let r_0 = r_0 as f32;
        Ok(( -r_0 * A + sqrtf( powf(r_0, 2_f32) * powf(A, 2_f32) - 4_f32 * r_0 * B * ( r_0 - r ) ) ) / ( 2_f32 * r_0 as f32 * B ))
    } else {
        Err(Error::OutOfRange)
    }
}

/// Calculate resistance of RTD for a specified temperature.
/// Allowed temperature range: -200–850°C.
#[allow(dead_code)]
pub fn calc_r(t: f32, r_0: RTDType) -> Result<f32, Error> {
    let r_0 = r_0 as i32;

    match floorf(t) as i32 {
        0..=850 => Ok(r_0 as f32 * ( 1_f32 + A * t + B * powf(t, 2_f32) )),
        -200..=-1 => Ok(r_0 as f32 * ( 1_f32 + A * t + B * powf(t, 2_f32) + C * ( t - 100_f32 ) * powf(t, 3_f32) )),
        _ => Err(Error::OutOfRange),
    }
}

/// Convert digital value for n bit ADC to resistance.
#[allow(dead_code)]
pub fn conv_d_val_to_r(d_val: i32, r_ref: i32, res: ADCRes, pga_gain: i32) -> Result<f32, Error> {
    let res = res as i32;

    Ok(d_val as f32 * r_ref as f32 / ( res as f32 * pga_gain as f32))
}

#[derive(Debug)]
pub enum Error {
    OutOfRange,
}
