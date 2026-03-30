use core::fmt::Write;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pipe::Pipe;

/// Log output pipe. The `log` facade pushes formatted bytes here (non-blocking).
/// A consumer task drains it — CDC writer in normal mode, JTAG writer in debug mode.
/// If no consumer is active or it falls behind, `try_write` silently drops data.
pub static LOG_PIPE: Pipe<CriticalSectionRawMutex, 512> = Pipe::new();

static LOGGER: MkeyLogger = MkeyLogger;

/// Set the global logger. No HAL types, no mode — just hooks up the `log` facade to the pipe.
pub fn init() {
    unsafe {
        ::log::set_logger_racy(&LOGGER).ok();
        ::log::set_max_level_racy(::log::LevelFilter::Info);
    }
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
        let _ = LOG_PIPE.try_write(w.as_bytes());
    }

    fn flush(&self) {}
}

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
