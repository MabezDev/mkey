use core::fmt::Write;
use core::panic::PanicInfo;

// RTC slow memory on ESP32-S3: 0x5000_0000 - 0x5000_1FFF (8KB)
// We use offset 0x1000 to avoid conflicting with ULP or bootloader usage
const PANIC_MEM: *mut u8 = 0x5000_1000 as *mut u8;
const PANIC_MAGIC: u32 = 0x50414E43; // "PANC"
const MAX_MSG_LEN: usize = 250;

// Layout: [magic: u32][len: u16][message: u8; MAX_MSG_LEN]

struct RtcWriter {
    offset: usize,
}

impl Write for RtcWriter {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for byte in s.bytes() {
            if self.offset >= MAX_MSG_LEN {
                break;
            }
            unsafe {
                PANIC_MEM.add(6 + self.offset).write_volatile(byte);
            }
            self.offset += 1;
        }
        Ok(())
    }
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    // Save panic message to RTC slow memory (survives software reset)
    let mut writer = RtcWriter { offset: 0 };
    let _ = write!(writer, "{}", info);

    unsafe {
        // Write length then magic (magic last so partial writes aren't read as valid)
        (PANIC_MEM.add(4) as *mut u16).write_volatile(writer.offset as u16);
        (PANIC_MEM as *mut u32).write_volatile(PANIC_MAGIC);
    }

    esp_hal::system::software_reset();
}

/// Check RTC slow memory for a stored panic message from a previous boot.
/// If found, prints it over JTAG serial and clears it.
/// Call this early in main(), before USB OTG takes over the serial pins.
pub fn check_previous_panic() {
    let magic = unsafe { (PANIC_MEM as *mut u32).read_volatile() };
    if magic != PANIC_MAGIC {
        return;
    }

    let len = unsafe { (PANIC_MEM.add(4) as *mut u16).read_volatile() } as usize;
    let len = len.min(MAX_MSG_LEN);

    let mut buf = [0u8; MAX_MSG_LEN];
    for i in 0..len {
        buf[i] = unsafe { PANIC_MEM.add(6 + i).read_volatile() };
    }

    // Clear before printing (in case printing itself panics)
    unsafe { (PANIC_MEM as *mut u32).write_volatile(0) };

    if let Ok(msg) = core::str::from_utf8(&buf[..len]) {
        log::error!("========== PREVIOUS PANIC ==========");
        log::error!("{}", msg);
        log::error!("====================================");
    }
}
