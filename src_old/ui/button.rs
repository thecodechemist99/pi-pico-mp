pub enum BtnState {
  ShortPressed = 0,
  LongPressed = 1,
}


pub fn eval_btn_press () -> BtnState {
  if true { // btn_timer is running && timer_duration >= btn_delay
    return BtnState::ShortPressed;
  }
  return BtnState::LongPressed;
}




// use arduino_hal::port::*;
// use arduino_hal::port::mode::*;

// pub struct Button {
//   pin: Pin<Input<Floating>>,
//   active: bool,
//   long_pressed: bool,
//   long_pressed_delay: u16,
//   timer: u64,
// }

// impl Button {
//   pub fn pressed (&self) -> u8 {
//     if self.pin.is_high() {
//       if !self.active {
//         self.active = true;
//         self.timer =  millis();
//       } else if (millis() - self.timer > self.long_pressed_delay) && !self.long_pressed {
//         self.long_pressed = true;
//         return 2;
//       }
//     } else {
//       if self.active {
//         self.active = false;
//         if self.long_pressed {
//           self.long_pressed = false;
//         } else {
//           return 1;
//         }
//       }
//     }
//     return 0;
//   }
// }

// pub fn init (pin: Pin<Input<Floating>>, long_pressed_delay: u16) -> Button {
//   Button {
//     pin: pin.into_floating_input(),
//     long_pressed_delay,
//     active: false,
//     long_pressed: false,
//     timer: 0,
//   }
// }