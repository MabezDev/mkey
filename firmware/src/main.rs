#![no_std]
#![no_main]

use esp32s3_hal::{
    clock::ClockControl,
    Delay,
    gpio::{AnyPin, Input, Output, PullDown, PushPull},
    peripherals::Peripherals,
    prelude::*,
    IO,
};
use esp_backtrace as _;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();

    esp_println::logger::init_logger_from_env();

    // V1.0 of the mKey has the USB DM and DP swapped.. oops. There is an EFUSE to swap the pins, yay! However,
    // it is bugged, and doesn't swap the pullups too. We can work around this by correcting the pullups early in the program,
    // as they are only used for signally to the host that we are a full speed device.
    // If flash is fully erased without programming again, the USB-SERIAL-JTAG will cease to work, firmware
    // will have to be flashed via UART0, at which point you can switch back.
    #[cfg(feature = "usb-pin-exchange")]
    {
        let usj = unsafe { &*esp32s3_hal::peripherals::USB_DEVICE::PTR };
        usj.conf0().modify(|_, w| {
            w.pad_pull_override()
                .set_bit()
                .dm_pullup()
                .clear_bit()
                .dp_pullup()
                .set_bit()
        });
    }

    let system = peripherals.SYSTEM.split();

    let clocks = ClockControl::max(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let columns: &mut [AnyPin<Output<PushPull>>] = &mut [
        io.pins.gpio2.into_push_pull_output().into(),  // 0
        io.pins.gpio43.into_push_pull_output().into(), // 1
        io.pins.gpio44.into_push_pull_output().into(), // 2
        io.pins.gpio38.into_push_pull_output().into(), // 3
        io.pins.gpio37.into_push_pull_output().into(), // 4
        io.pins.gpio36.into_push_pull_output().into(), // 5
        io.pins.gpio48.into_push_pull_output().into(), // 6
        io.pins.gpio47.into_push_pull_output().into(), // 7
        io.pins.gpio21.into_push_pull_output().into(), // 8
        io.pins.gpio14.into_push_pull_output().into(), // 9
        io.pins.gpio13.into_push_pull_output().into(), // 10
        io.pins.gpio12.into_push_pull_output().into(), // 11
        io.pins.gpio11.into_push_pull_output().into(), // 12
        io.pins.gpio10.into_push_pull_output().into(), // 13
    ];
    let rows: &mut [AnyPin<Input<PullDown>>] = &mut [
        io.pins.gpio1.into_pull_down_input().into(),  // 0
        io.pins.gpio35.into_pull_down_input().into(), // 1
        io.pins.gpio45.into_pull_down_input().into(), // 2
        io.pins.gpio9.into_pull_down_input().into(),  // 3
        io.pins.gpio46.into_pull_down_input().into(), // 4
    ];

    let mut last = (usize::MAX, usize::MAX);
    loop {
        for (x, c) in columns.iter_mut().enumerate() {
            for (y, r) in rows.iter_mut().enumerate() {
                c.set_high().unwrap();
                if r.is_high().unwrap() {
                    let current = (x, y);
                    if current != last {
                        last = current;
                        log::info!("({x}, {y}) is pressed!");
                    }
                }
                c.set_low().unwrap();
                // small debounce - is this needed? 
                // perhaps polling at too high of a rate we run into 
                // gpio switching frequency issues, or maybe even capacitance in the trace
                delay.delay_us(50u32); 
            }
        }
    }
}
