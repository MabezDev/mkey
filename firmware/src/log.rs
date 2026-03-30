use core::fmt::Write;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pipe::Pipe;
use esp_hal::usb_serial_jtag::UsbSerialJtagTx;
use esp_hal::Blocking;

/// Pipe for CDC log output. The logger pushes formatted bytes here (non-blocking),
/// and the CDC writer task drains it over USB.
pub static LOG_PIPE: Pipe<CriticalSectionRawMutex, 512> = Pipe::new();

static DEBUG_MODE: AtomicBool = AtomicBool::new(false);
static LOGGER: MkeyLogger = MkeyLogger;

static mut JTAG_TX: Option<UsbSerialJtagTx<'static, Blocking>> = None;

/// Initialize the logging backend.
/// - `debug = true`: logs go to USB-SERIAL-JTAG (no USB OTG)
/// - `debug = false`: logs go to CDC pipe (drained by USB task)
///
/// The JTAG TX is always used for early boot output (before USB OTG takes over).
pub fn init(debug: bool, jtag_tx: UsbSerialJtagTx<'static, Blocking>) {
    DEBUG_MODE.store(debug, Ordering::Relaxed);
    unsafe { (*core::ptr::addr_of_mut!(JTAG_TX)) = Some(jtag_tx) };
    unsafe {
        ::log::set_logger_racy(&LOGGER).ok();
        ::log::set_max_level_racy(::log::LevelFilter::Info);
    }
}

/// Write directly to USB-SERIAL-JTAG. Used by panic handler for early output.
pub fn jtag_serial_write(data: &[u8]) {
    critical_section::with(|_| unsafe {
        if let Some(tx) = (*core::ptr::addr_of_mut!(JTAG_TX)).as_mut() {
            let _ = tx.write(data);
        }
    });
}

struct MkeyLogger;

impl ::log::Log for MkeyLogger {
    fn enabled(&self, _metadata: &::log::Metadata) -> bool {
        true
    }

    fn log(&self, record: &::log::Record) {
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
