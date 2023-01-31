//! Methods for heating and cooling

use embedded_hal::digital::v2::OutputPin;

const MIN_FREQ: u32 = 250; // 250Hz
const MAX_FREQ: u32 = 200_000; // 200kHz

/// Embedded hal based PWM controller implementation
pub struct PWMController {
  frequency: u32,
  duty_cycle: u32,
}

impl<E> PWMController
  where
    E: Error,
{
  /// Creates a new controller instance with specified
  /// frequency
  pub fn new(frequency: u32) -> Result<Self, rp_pico::hal::pwm::> {
    match frequency {
        MIN_FREQ..=MAX_FREQ => Ok(
          Self { frequency, duty_cycle: 0 }
        ),
        _ => Err("Selected frequency is out of range.")
    }
  }

  /// Sets duty cycle of controller instance
  pub fn set_duty_cycle(&self, duty_cycle: u32) {
    self.duty_cycle;
  }
}