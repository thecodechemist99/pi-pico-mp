use fugit::MicrosDurationU32;

pub const DEBOUNCE_INTERVAL: MicrosDurationU32 = MicrosDurationU32::millis(20);
pub const BTN_STATE_INTERVAL: MicrosDurationU32 = MicrosDurationU32::millis(250);

fn btn_debounce_irq() {

}


// #[task(binds = TIMER_IRQ_3, priority = 2, shared = [ matrix, debouncer, timer, alarm, watchdog, usb_dev, usb_class])]
//     fn scan_timer_irq(mut c: scan_timer_irq::Context) {


//         c.shared.watchdog.feed();

//         for event in c.shared.debouncer.events(c.shared.matrix.get().unwrap()) {
//             handle_event::spawn(Some(event)).unwrap();
//         }

//         handle_event::spawn(None).unwrap();

//         let mut alarm = c.shared.alarm;

//         alarm.lock(|a| {
//             a.clear_interrupt();
//             let _ = a.schedule(SCAN_TIME_US.microseconds());
//         });
//     }