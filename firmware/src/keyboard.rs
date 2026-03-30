use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering;

use embassy_futures::join::join;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Duration;
use embassy_time::Timer;
use esp_hal::gpio::{Input, Output};
use esp_hal::otg_fs;
use esp_hal::otg_fs::asynch::{Config, Driver};

use embassy_usb::class::hid::{HidReaderWriter, ReportId, RequestHandler, State};
use embassy_usb::control::OutResponse;
use embassy_usb::{Builder, Handler};
use usbd_hid::descriptor::KeyboardReport;
use usbd_hid::descriptor::SerializedDescriptor;

use crate::keymap::{
    DEBOUNCE_SCANS, FN_COL, FN_ROW, KEY_BACKSPACE, KEY_D, KEY_ESC, KEYMAP, NUM_COLS, NUM_ROWS,
};

const DEBUG_MODE_MAGIC: u32 = 0xDBE6_F1A6;

fn rtc_cntl() -> &'static esp32s3::rtc_cntl::RegisterBlock {
    unsafe { &*esp32s3::RTC_CNTL::PTR }
}

/// Check if debug mode was requested (persisted in RTC store register across resets).
pub fn is_debug_mode() -> bool {
    rtc_cntl().store6().read().bits() == DEBUG_MODE_MAGIC
}

fn toggle_debug_mode() -> ! {
    let current = rtc_cntl().store6().read().bits();
    let new = if current == DEBUG_MODE_MAGIC { 0 } else { DEBUG_MODE_MAGIC };
    rtc_cntl().store6().write(|w| unsafe { w.bits(new) });
    if new == DEBUG_MODE_MAGIC {
        info!("Switching to debug mode...");
    } else {
        info!("Switching to normal mode...");
    }
    esp_hal::system::software_reset();
}

fn reset_to_download_mode() -> ! {
    info!("Resetting to download mode...");
    rtc_cntl()
        .option1()
        .modify(|_, w| w.force_download_boot().set_bit());
    esp_hal::system::software_reset();
}

/// Check pressed keys for Fn combos. Returns true if Fn is held (suppresses HID report).
fn handle_fn_combos(key_state: &[[bool; NUM_COLS]; NUM_ROWS]) -> bool {
    if !key_state[FN_ROW][FN_COL] {
        return false;
    }

    // Scan for combo keys by checking keycodes in the keymap
    for y in 0..NUM_ROWS {
        for x in 0..NUM_COLS {
            if (y, x) == (FN_ROW, FN_COL) {
                continue;
            }
            if !key_state[y][x] {
                continue;
            }
            let (_mod_bits, keycode) = KEYMAP[y][x];
            match keycode {
                KEY_ESC => esp_hal::system::software_reset(),
                KEY_BACKSPACE => reset_to_download_mode(),
                KEY_D => toggle_debug_mode(),
                _ => {}
            }
        }
    }

    // Fn held but no recognized combo — still suppress HID output
    true
}

#[embassy_executor::task]
pub async fn usb(
    usb: otg_fs::Usb<'static>,
    signal: &'static Signal<CriticalSectionRawMutex, KeyboardReport>,
) {
    // Create the driver, from the HAL.
    let mut ep_out_buffer = [0u8; 1024];
    let driver = Driver::new(usb, &mut ep_out_buffer, Config::default());

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    // You can also add a Microsoft OS descriptor.
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut request_handler = MyRequestHandler {};
    let mut device_handler = MyDeviceHandler::new();

    let mut state = State::new();

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0x16c0, 0x27dd);
    config.max_power = 100;
    config.max_packet_size_0 = 64;
    config.manufacturer = Some("Scott Mabin");
    config.product = Some("mKey");
    config.serial_number = Some("000001");

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    builder.handler(&mut device_handler);

    // Create classes on the builder.
    let config = embassy_usb::class::hid::Config {
        report_descriptor: KeyboardReport::desc(),
        request_handler: None,
        poll_ms: 60,
        max_packet_size: 64,
    };
    let hid = HidReaderWriter::<_, 1, 8>::new(&mut builder, &mut state, config);

    let mut usb = builder.build();

    let (reader, mut writer) = hid.split();

    let in_fut = async {
        loop {
            let report = signal.wait().await;
            match writer.write_serialize(&report).await {
                Ok(()) => {}
                Err(e) => warn!("Failed to send report: {:?}", e),
            };
        }
    };

    let out_fut = async {
        reader.run(false, &mut request_handler).await;
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join(usb.run(), join(in_fut, out_fut)).await;
}

#[embassy_executor::task]
pub async fn matrix(
    columns: &'static mut [Input<'static>],
    rows: &'static mut [Output<'static>],
    signal: &'static Signal<CriticalSectionRawMutex, KeyboardReport>,
) -> ! {
    let mut key_state = [[false; NUM_COLS]; NUM_ROWS];
    let mut debounce = [[0u8; NUM_COLS]; NUM_ROWS];

    loop {
        let mut state_changed = false;

        for (y, row) in rows.iter_mut().enumerate() {
            row.set_low();
            Timer::after(Duration::from_micros(5)).await;

            for (x, col) in columns.iter_mut().enumerate() {
                let pressed = col.is_low();

                if pressed != key_state[y][x] {
                    debounce[y][x] = debounce[y][x].saturating_add(1);
                    if debounce[y][x] >= DEBOUNCE_SCANS {
                        key_state[y][x] = pressed;
                        debounce[y][x] = 0;
                        state_changed = true;
                        #[cfg(feature = "debug-matrix")]
                        {
                            let (mod_bits, keycode) = KEYMAP[y][x];
                            if pressed {
                                info!("PRESS   row={} col={} modifier={:#04x} keycode={:#04x}", y, x, mod_bits, keycode);
                            } else {
                                info!("RELEASE row={} col={}", y, x);
                            }
                        }
                    }
                } else {
                    debounce[y][x] = 0;
                }
            }

            row.set_high();
        }

        if state_changed {
            // Check Fn combos first — resets never return, other combos suppress HID
            if handle_fn_combos(&key_state) {
                Timer::after(Duration::from_millis(1)).await;
                continue;
            }

            let mut modifier = 0u8;
            let mut keycodes = [0u8; 6];
            let mut keycode_idx = 0usize;

            for y in 0..NUM_ROWS {
                for x in 0..NUM_COLS {
                    if key_state[y][x] {
                        let (mod_bits, keycode) = KEYMAP[y][x];
                        modifier |= mod_bits;
                        if keycode != 0 && keycode_idx < 6 {
                            keycodes[keycode_idx] = keycode;
                            keycode_idx += 1;
                        }
                    }
                }
            }

            signal.signal(KeyboardReport {
                modifier,
                reserved: 0,
                leds: 0,
                keycodes,
            });
        }

        Timer::after(Duration::from_millis(1)).await;
    }
}

struct MyRequestHandler {}

impl RequestHandler for MyRequestHandler {
    fn get_report(&mut self, id: ReportId, _buf: &mut [u8]) -> Option<usize> {
        info!("Get report for {:?}", id);
        None
    }

    fn set_report(&mut self, id: ReportId, data: &[u8]) -> OutResponse {
        info!("Set report for {:?}: {:?}", id, data);
        OutResponse::Accepted
    }

    fn set_idle_ms(&mut self, id: Option<ReportId>, dur: u32) {
        info!("Set idle rate for {:?} to {:?}", id, dur);
    }

    fn get_idle_ms(&mut self, id: Option<ReportId>) -> Option<u32> {
        info!("Get idle rate for {:?}", id);
        None
    }
}

struct MyDeviceHandler {
    configured: AtomicBool,
}

impl MyDeviceHandler {
    fn new() -> Self {
        MyDeviceHandler {
            configured: AtomicBool::new(false),
        }
    }
}

#[embassy_executor::task]
pub async fn debug_consumer(
    signal: &'static Signal<CriticalSectionRawMutex, KeyboardReport>,
) -> ! {
    loop {
        let report = signal.wait().await;
        info!(
            "HID Report: modifier={:#04x} keycodes=[{:#04x}, {:#04x}, {:#04x}, {:#04x}, {:#04x}, {:#04x}]",
            report.modifier,
            report.keycodes[0], report.keycodes[1], report.keycodes[2],
            report.keycodes[3], report.keycodes[4], report.keycodes[5],
        );
    }
}

impl Handler for MyDeviceHandler {
    fn enabled(&mut self, enabled: bool) {
        self.configured.store(false, Ordering::Relaxed);
        if enabled {
            info!("Device enabled");
        } else {
            info!("Device disabled");
        }
    }

    fn reset(&mut self) {
        self.configured.store(false, Ordering::Relaxed);
        info!("Bus reset, the Vbus current limit is 100mA");
    }

    fn addressed(&mut self, addr: u8) {
        self.configured.store(false, Ordering::Relaxed);
        info!("USB address set to: {}", addr);
    }

    fn configured(&mut self, configured: bool) {
        self.configured.store(configured, Ordering::Relaxed);
        if configured {
            info!(
                "Device configured, it may now draw up to the configured current limit from Vbus."
            )
        } else {
            info!("Device is no longer configured, the Vbus current limit is 100mA.");
        }
    }
}
