use core::fmt::Write;
use core::panic::PanicInfo;

const PANIC_MAGIC: u32 = 0x50414E43; // "PANC"
const MAX_MSG_LEN: usize = 250;

// Layout: [magic: u32][len: u16][message: u8; MAX_MSG_LEN]
const BUF_SIZE: usize = 4 + 2 + MAX_MSG_LEN;

/// Panic buffer in main SRAM. Placed in `.noinit` so the bootloader won't
/// zero it on software reset — the previous contents survive across reboots.
#[used]
#[link_section = ".noinit"]
static mut PANIC_BUF: [u8; BUF_SIZE] = [0u8; BUF_SIZE];

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
/// If found, prints it over JTAG serial and clears it.
/// Call this early in main(), before USB OTG takes over the serial pins.
pub fn check_previous_panic() {
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(PANIC_BUF) };

    let magic = u32::from_le_bytes(buf[0..4].try_into().unwrap());
    if magic != PANIC_MAGIC {
        return;
    }

    let len = u16::from_le_bytes(buf[4..6].try_into().unwrap()) as usize;
    let len = len.min(MAX_MSG_LEN);

    // Clear magic before printing (in case printing itself panics)
    buf[0..4].fill(0);

    if let Ok(msg) = core::str::from_utf8(&buf[6..6 + len]) {
        log::error!("========== PREVIOUS PANIC ==========");
        log::error!("{}", msg);
        log::error!("====================================");
        // JTAG serial FIFO is small — give it time to drain before USB OTG takes over
        esp_hal::delay::Delay::new().delay_millis(100u32);
    }
}
