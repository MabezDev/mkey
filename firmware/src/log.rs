use core::fmt::Write;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pipe::Pipe;

/// Pipe for CDC log output. The logger pushes formatted bytes here (non-blocking),
/// and the CDC writer task drains it over USB.
pub static LOG_PIPE: Pipe<CriticalSectionRawMutex, 512> = Pipe::new();

static DEBUG_MODE: AtomicBool = AtomicBool::new(false);
static LOGGER: MkeyLogger = MkeyLogger;

/// Initialize the logging backend.
/// - `debug = true`: logs go to USB-SERIAL-JTAG FIFO (no USB OTG)
/// - `debug = false`: logs go to CDC pipe (drained by USB task)
pub fn init(debug: bool) {
    DEBUG_MODE.store(debug, Ordering::Relaxed);
    unsafe {
        log::set_logger_racy(&LOGGER).ok();
        log::set_max_level_racy(log::LevelFilter::Info);
    }
}

struct MkeyLogger;

impl log::Log for MkeyLogger {
    fn enabled(&self, _metadata: &log::Metadata) -> bool {
        true
    }

    fn log(&self, record: &log::Record) {
        let mut buf = [0u8; 128];
        let mut w = BufWriter::new(&mut buf);
        let _ = write!(w, "[{}] {}\r\n", record.level(), record.args());
        let bytes = w.as_bytes();

        if DEBUG_MODE.load(Ordering::Relaxed) {
            jtag_serial_write(bytes);
        } else {
            let _ = LOG_PIPE.try_write(bytes);
        }
    }

    fn flush(&self) {}
}

// ---------------------------------------------------------------------------
// USB-SERIAL-JTAG FIFO writer (used in debug mode)
// ---------------------------------------------------------------------------

fn usb_device() -> &'static esp32s3::usb_device::RegisterBlock {
    unsafe { &*esp32s3::USB_DEVICE::PTR }
}

/// Write bytes to the USB-SERIAL-JTAG TX FIFO.
/// Blocks until all bytes are written. Max 64 bytes per flush.
pub fn jtag_serial_write(data: &[u8]) {
    let dev = usb_device();

    for chunk in data.chunks(64) {
        // Wait for FIFO space
        while !dev.ep1_conf().read().serial_in_ep_data_free().bit() {}

        for &byte in chunk {
            dev.ep1()
                .write(|w| unsafe { w.rdwr_byte().bits(byte) });
        }

        // Signal write complete
        dev.ep1_conf().write(|w| w.wr_done().set_bit());
    }
}

// ---------------------------------------------------------------------------
// CDC writer — drains LOG_PIPE to a USB CDC ACM sender
// ---------------------------------------------------------------------------


// ---------------------------------------------------------------------------
// Stack-based fmt::Write adapter (no alloc)
// ---------------------------------------------------------------------------

struct BufWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> BufWriter<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }
    fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.pos]
    }
}

impl Write for BufWriter<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let space = self.buf.len() - self.pos;
        let n = bytes.len().min(space);
        self.buf[self.pos..self.pos + n].copy_from_slice(&bytes[..n]);
        self.pos += n;
        Ok(())
    }
}
