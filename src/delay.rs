//! no_std implementation of DelayMs and DelayUs for raspberry pi pico
//! Based upon the cortex-m::delay source code and https://github.com/ChrisChrisLoLo/keezyboost40/blob/master/firmware/keezus/src/delay.rs.
//! cortex_m::delay cannot be used with RTIC, because SYST is already taken by it (https://github.com/rtic-rs/cortex-m-rtic/issues/523).

use rp_pico::hal::Timer;
use embedded_hal::blocking::delay::{
    DelayMs,
    DelayUs
};

/// ASM based delay implementation
pub struct Delay<'a> {
    timer: &'a Timer,
    frequency: u32,
}

impl Delay<'_> {
    /// Configures a rp2040-hal timer as a delay provider.
    /// `frequency` is the frequency of your clock source in Hz.
    pub fn new<'a>(timer: &'a Timer, frequency: u32) -> Delay {
        Delay {timer, frequency}
    }

    /// ASM based delay using a rp2040-hal timer for a certain duration, in µs.
    pub fn delay_us(&mut self, us: u32) {
        let ticks_per_second = self.frequency;
        let ticks_per_microsecond = ticks_per_second/1_000_000;

        // Iterate rather than multiply to prevent buffer overflow
        for _ in 0..us {
            cortex_m::asm::delay(ticks_per_microsecond);
        }
    }

    /// ASM based delay using a rp2040-hal timer for a certain duration, in ms.
    pub fn delay_ms(&mut self, ms: u32) {
        self.delay_us(ms * 1_000);
    }
}

impl DelayMs<u32> for Delay<'_> {
    fn delay_ms(&mut self, ms: u32) {
        Delay::delay_ms(self, ms);
    }
}

impl DelayUs<u32> for Delay<'_> {
    fn delay_us(&mut self, us: u32) {
        Delay::delay_us(self, us);
    }
}
