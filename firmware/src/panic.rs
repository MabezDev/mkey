use core::fmt::Write;
use core::panic::PanicInfo;
use core::sync::atomic::{AtomicBool, Ordering};

const PANIC_MAGIC: u32 = 0x50414E43; // "PANC"
const MAX_MSG_LEN: usize = 250;

// Layout: [magic: u32][len: u16][message: u8; MAX_MSG_LEN]
const BUF_SIZE: usize = 4 + 2 + MAX_MSG_LEN;

/// Panic buffer in main SRAM. Placed in `.noinit` so the bootloader won't
/// zero it on software reset — the previous contents survive across reboots.
#[used]
#[link_section = ".noinit"]
static mut PANIC_BUF: [u8; BUF_SIZE] = [0u8; BUF_SIZE];

/// Copy of the panic message for deferred CDC output.
static PANIC_AVAILABLE: AtomicBool = AtomicBool::new(false);
static mut PANIC_MSG_COPY: [u8; MAX_MSG_LEN] = [0u8; MAX_MSG_LEN];
static mut PANIC_MSG_LEN: usize = 0;

struct PanicWriter {
    offset: usize,
}

impl Write for PanicWriter {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let buf = unsafe { &mut *core::ptr::addr_of_mut!(PANIC_BUF) };
        for byte in s.bytes() {
            if self.offset >= MAX_MSG_LEN {
                break;
            }
            buf[6 + self.offset] = byte;
            self.offset += 1;
        }
        Ok(())
    }
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(PANIC_BUF) };

    let mut writer = PanicWriter { offset: 0 };
    let _ = write!(writer, "{}", info);

    // Write length then magic (magic last so partial writes aren't read as valid)
    buf[4..6].copy_from_slice(&(writer.offset as u16).to_le_bytes());
    buf[0..4].copy_from_slice(&PANIC_MAGIC.to_le_bytes());

    esp_hal::system::software_reset();
}

/// Check for a stored panic message from a previous boot.
/// If found, copies the message for later CDC output and prints over JTAG serial.
/// Returns `true` if a panic was found.
pub fn check_previous_panic() -> bool {
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(PANIC_BUF) };

    let magic = u32::from_le_bytes(buf[0..4].try_into().unwrap());
    if magic != PANIC_MAGIC {
        return false;
    }

    let len = u16::from_le_bytes(buf[4..6].try_into().unwrap()) as usize;
    let len = len.min(MAX_MSG_LEN);

    // Clear magic early (safety: prevents recursive panic from re-reading this)
    buf[0..4].fill(0);

    // Copy message for deferred CDC output
    unsafe {
        PANIC_MSG_COPY[..len].copy_from_slice(&buf[6..6 + len]);
        PANIC_MSG_LEN = len;
    }
    PANIC_AVAILABLE.store(true, Ordering::Release);

    true
}

/// Take the saved panic message (one-shot). Returns `None` if no panic or already taken.
/// Used by the CDC writer to send the message over USB after enumeration.
pub fn take_panic_message() -> Option<&'static str> {
    if !PANIC_AVAILABLE.swap(false, Ordering::Acquire) {
        return None;
    }
    let len = unsafe { PANIC_MSG_LEN };
    let buf = unsafe { &*core::ptr::addr_of!(PANIC_MSG_COPY) };
    core::str::from_utf8(&buf[..len]).ok()
}
