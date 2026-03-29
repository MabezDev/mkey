#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::cell::RefCell;
use core::ptr::addr_of_mut;
use core::sync::atomic::AtomicBool;

use core::sync::atomic::Ordering;
use critical_section::Mutex;
use embassy_futures::join::join;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Duration;
use embassy_time::Timer;
use embedded_graphics::framebuffer::buffer_size;
use embedded_graphics::framebuffer::Framebuffer;
use embedded_graphics::geometry::Point;
use embedded_graphics::image::Image;
use embedded_graphics::pixelcolor::raw::BigEndian;
use embedded_graphics::pixelcolor::raw::RawU16;
use embedded_graphics::Drawable;
use embedded_graphics_core::pixelcolor::Rgb565;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::InputConfig;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::system::Stack;
use esp_hal::dma::DmaRxBuf;
use esp_hal::dma::DmaTxBuf;
use esp_hal::dma_buffers;
use esp_hal::gpio::Level;
use esp_hal::interrupt;
use esp_hal::interrupt::Priority;
use esp_hal::otg_fs;
use esp_hal::otg_fs::asynch::{Config, Driver};
use esp_hal::otg_fs::Usb;
use esp_hal::peripherals::Interrupt;
use esp_hal::time::Rate;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::Blocking;
use esp_hal::{
    delay::Delay,
    gpio::Io,
    gpio::{Input, Output, Pull},
    handler, main, ram,
    spi::{
        master::{Address, Command, Spi, DataMode},
        Mode,
    },
};
use esp_rtos::{self, embassy::Executor};

use embassy_usb::class::hid::{HidReaderWriter, ReportId, RequestHandler, State};
use embassy_usb::control::OutResponse;
use embassy_usb::{Builder, Handler};
use usbd_hid::descriptor::KeyboardReport;
use usbd_hid::descriptor::SerializedDescriptor;

#[macro_export]
macro_rules! make_static {
    ($t:ty, $val:expr) => ($crate::make_static!($t, $val,));
    ($t:ty, $val:expr, $(#[$m:meta])*) => {{
        $(#[$m])*
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        STATIC_CELL.init_with(|| $val)
    }};
}

esp_bootloader_esp_idf::esp_app_desc!();

// MUST be the first module so other modules see the macros
pub mod fmt;

type QSpiDisplay<'a> = esp_hal::spi::master::SpiDmaBus<'a, Blocking>;

const MAX_DMA_TRANSFER: usize = 32736 / 4;
const WIDTH: usize = 356;
const HEIGHT: usize = 400;

static TE: Mutex<RefCell<Option<Input<'static>>>> = Mutex::new(RefCell::new(None));

static TE_READY: AtomicBool = AtomicBool::new(false);
static mut APP_CORE_STACK: Stack<4096> = Stack::new();
static mut FRAME_BUFFER: Framebuffer<
    Rgb565,
    RawU16,
    BigEndian,
    WIDTH,
    HEIGHT,
    { buffer_size::<Rgb565>(WIDTH, HEIGHT) },
> = Framebuffer::new();

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    esp_println::logger::init_logger_from_env();

    // V1.0 of the mKey has the USB DM and DP swapped.. oops. There is an EFUSE to swap the pins, yay! However,
    // it is bugged, and doesn't swap the pullups too. We can work around this by correcting the pullups early in the program,
    // as they are only used for signally to the host that we are a full speed device.
    // If flash is fully erased without programming again, the USB-SERIAL-JTAG will cease to work, firmware
    // will have to be flashed via UART0, at which point you can switch back.
    #[cfg(feature = "usb-pin-exchange")]
    {
        let usj = unsafe { &*esp_hal::peripherals::USB_DEVICE::PTR };
        usj.conf0().modify(|_, w| {
            w.pad_pull_override()
                .set_bit()
                .dm_pullup()
                .clear_bit()
                .dp_pullup()
                .set_bit()
        });
    }

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let delay = Delay::new();

    let mut io = Io::new(peripherals.IO_MUX);
    io.set_interrupt_handler(te);
    interrupt::enable(Interrupt::GPIO, Priority::Priority2).unwrap();

    let sclk = peripherals.GPIO4;
    let sio0 = peripherals.GPIO6;
    let sio1 = peripherals.GPIO5;
    let sio2 = peripherals.GPIO15;
    let sio3 = peripherals.GPIO7;
    let cs = peripherals.GPIO16;
    let mut reset = Output::new(peripherals.GPIO3, Level::Low, Default::default());

    let mut te_pin = Input::new(peripherals.GPIO17, InputConfig::default().with_pull(Pull::Down));
    te_pin.listen(esp_hal::gpio::Event::RisingEdge);
    critical_section::with(|cs| TE.borrow_ref_mut(cs).replace(te_pin));

    let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) =
        dma_buffers!(MAX_DMA_TRANSFER, 1);

    let mut spi = Spi::new(
        peripherals.SPI2,
        esp_hal::spi::master::Config::default()
            .with_frequency(Rate::from_mhz(80))
            .with_mode(Mode::_3),
    )
    .unwrap()
    .with_sck(sclk)
    .with_mosi(sio0)
    .with_miso(sio1)
    .with_sio2(sio2)
    .with_sio3(sio3)
    .with_cs(cs)
    .with_dma(peripherals.DMA_CH0)
    .with_buffers(
        DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap(),
        DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap(),
    );

    reset.set_low();
    delay.delay_millis(300u32);
    reset.set_high();
    delay.delay_millis(300u32);

    // initialization commands
    for (cmd, data) in WEA2012_INIT_CMDS {
        lcd_write_cmd(&mut spi, *cmd, data);
    }
    log::info!("Finished initializing display!");

    let pixels = unsafe { &mut *addr_of_mut!(FRAME_BUFFER) };
    /*
       Matrix
    */
    let columns = make_static!(
        [Input<'static>; 14],
        [
            Input::new(peripherals.GPIO2,  InputConfig::default().with_pull(Pull::Up)), // 0
            Input::new(peripherals.GPIO43, InputConfig::default().with_pull(Pull::Up)), // 1
            Input::new(peripherals.GPIO44, InputConfig::default().with_pull(Pull::Up)), // 2
            Input::new(peripherals.GPIO38, InputConfig::default().with_pull(Pull::Up)), // 3
            Input::new(peripherals.GPIO37, InputConfig::default().with_pull(Pull::Up)), // 4
            Input::new(peripherals.GPIO36, InputConfig::default().with_pull(Pull::Up)), // 5
            Input::new(peripherals.GPIO48, InputConfig::default().with_pull(Pull::Up)), // 6
            Input::new(peripherals.GPIO47, InputConfig::default().with_pull(Pull::Up)), // 7
            Input::new(peripherals.GPIO21, InputConfig::default().with_pull(Pull::Up)), // 8
            Input::new(peripherals.GPIO14, InputConfig::default().with_pull(Pull::Up)), // 9
            Input::new(peripherals.GPIO13, InputConfig::default().with_pull(Pull::Up)), // 10
            Input::new(peripherals.GPIO12, InputConfig::default().with_pull(Pull::Up)), // 11
            Input::new(peripherals.GPIO11, InputConfig::default().with_pull(Pull::Up)), // 12
            Input::new(peripherals.GPIO10, InputConfig::default().with_pull(Pull::Up)), // 13
        ]
    );
    let rows = make_static!(
        [Output<'static>; 5],
        [
            Output::new(peripherals.GPIO1, Level::Low, Default::default()), // 0
            Output::new(peripherals.GPIO35, Level::Low, Default::default()), // 1
            Output::new(peripherals.GPIO45, Level::Low, Default::default()), // 2
            Output::new(peripherals.GPIO9, Level::Low, Default::default()), // 3
            Output::new(peripherals.GPIO46, Level::Low, Default::default()), // 4
        ]
    );

    let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start_second_core(
        peripherals.CPU_CTRL,
        sw_ints.software_interrupt0,
        sw_ints.software_interrupt1,
        unsafe { &mut *addr_of_mut!(APP_CORE_STACK) },
        || {
            let device = Usb::new(peripherals.USB0, peripherals.GPIO20, peripherals.GPIO19);
            let signal = &*make_static!(Signal<CriticalSectionRawMutex, u8>, Signal::new());
            let executor = make_static!(Executor, Executor::new());
            executor.run(|spawner| {
                spawner.must_spawn(matrix(columns, rows, signal));
                spawner.must_spawn(usb(device, signal));
            });
        },
    );

    let image =
        tinygif::Gif::<Rgb565>::from_slice(include_bytes!("../Ferris-240x240.gif")).unwrap();
    let mut start = SystemTimer::unit_value(esp_hal::timer::systimer::Unit::Unit0);
    let mut frames = 0;
    loop {
        for frame in image.frames() {
            let frame = Image::with_center(
                &frame,
                Point::new(WIDTH as i32 / 2, (HEIGHT as i32 / 2) - 40),
            );
            frame.draw(pixels).unwrap();

            // TE_READY won't get set until we mark that we're ready to flush a buffer
            critical_section::with(|cs| {
                TE_READY.store(false, Ordering::SeqCst);
                TE.borrow_ref_mut(cs).as_mut().unwrap().clear_interrupt();
            });
            // wait for next sync
            while !TE_READY.load(Ordering::SeqCst) {}

            let pixels = unsafe {
                core::slice::from_raw_parts(
                    pixels.data().as_ptr() as *const u8,
                    pixels.data().len(),
                )
            };
            let now = SystemTimer::unit_value(esp_hal::timer::systimer::Unit::Unit0);
            lcd_fill(&mut spi, pixels);
            log::trace!(
                "Time to fill display: {}ms",
                (SystemTimer::unit_value(esp_hal::timer::systimer::Unit::Unit0) - now)
                    / (SystemTimer::ticks_per_second() / 1024)
            );
            frames += 1;
            let now = SystemTimer::unit_value(esp_hal::timer::systimer::Unit::Unit0);
            if now.wrapping_sub(start) > SystemTimer::ticks_per_second() {
                start = now;
                log::info!("FPS: {}", frames);
                frames = 0;
            }
        }
    }
}

#[embassy_executor::task]
async fn usb(usb: otg_fs::Usb<'static>, signal: &'static Signal<CriticalSectionRawMutex, u8>) {
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
            let code = signal.wait().await;
            // Create a report with the A key pressed. (no shift modifier)
            let report = KeyboardReport {
                keycodes: [code, 0, 0, 0, 0, 0],
                leds: 0,
                modifier: 0,
                reserved: 0,
            };
            // Send the report.
            match writer.write_serialize(&report).await {
                Ok(()) => {}
                Err(e) => warn!("Failed to send report: {:?}", e),
            };
            Timer::after_micros(500).await; // simulate a release
            let report = KeyboardReport {
                keycodes: [0, 0, 0, 0, 0, 0],
                leds: 0,
                modifier: 0,
                reserved: 0,
            };
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
async fn matrix(
    columns: &'static mut [Input<'static>],
    rows: &'static mut [Output<'static>],
    signal: &'static Signal<CriticalSectionRawMutex, u8>,
) -> ! {
    let mut last = (usize::MAX, usize::MAX);
    loop {
        for (y, row) in rows.iter_mut().enumerate() {
            row.set_low();
            Timer::after(Duration::from_micros(1)).await;
            for (x, col) in columns.iter_mut().enumerate() {
                if col.is_low() {
                    let current = (x, y);
                    if current != last {
                        last = current;
                        log::info!("({x}, {y}) is pressed!");
                        signal.signal(4); // always send "a" for now
                    }
                }
            }
            row.set_high();
        }
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

#[handler]
fn te() {
    unsafe {
        static mut LAST: u64 = 0;
        let now = SystemTimer::unit_value(esp_hal::timer::systimer::Unit::Unit0);
        log::trace!(
            "TE fired! Interval: {}ms",
            (now - LAST) / (SystemTimer::ticks_per_second() / 1024)
        );
        LAST = now;
    }
    critical_section::with(|cs| TE.borrow_ref_mut(cs).as_mut().unwrap().clear_interrupt());
    TE_READY.store(true, Ordering::SeqCst);
}

#[ram]
fn lcd_fill<'a>(spi: &mut QSpiDisplay<'a>, pixels: &'static [u8]) {
    set_draw_area(spi, 0, 0, WIDTH as u16 - 1, HEIGHT as u16 - 1);
    for (i, pixels) in pixels.chunks(MAX_DMA_TRANSFER).enumerate() {
        // fifo size
        spi.half_duplex_write(
            DataMode::Quad,
            Command::_8Bit(0x32, DataMode::SingleTwoDataLines),
            Address::_24Bit(if i > 0 { 0x002C00 } else { 0x003C00 }, DataMode::SingleTwoDataLines),
            0,
            &pixels,
        )
        .unwrap();
    }
}

#[ram]
fn lcd_write_cmd<'a>(spi: &mut QSpiDisplay<'a>, cmd: u32, data: &[u8]) {
    static mut BUF: [u8; 4] = [0u8; 4];
    let buf = unsafe { &mut *addr_of_mut!(BUF) };
    if data.len() > 0 {
        buf[..data.len()].copy_from_slice(&data);
    }
    log::trace!("Sending: {} => {:?}", cmd, &buf[..data.len()]);
    spi.half_duplex_write(
        DataMode::Single,
        Command::_8Bit(0x02, DataMode::Single),
        Address::_24Bit(cmd << 8, DataMode::Single),
        0,
        &&buf[..data.len()],
    )
    .unwrap();
}

#[ram]
fn set_draw_area<'a>(spi: &mut QSpiDisplay<'a>, x1: u16, y1: u16, x2: u16, y2: u16) {
    let cmds = [
        (
            0x2a,
            &[(x1 >> 8) as u8, x1 as u8, (x2 >> 8) as u8, x2 as u8][..],
        ),
        (
            0x2b,
            &[(y1 >> 8) as u8, y1 as u8, (y2 >> 8) as u8, y2 as u8][..],
        ),
    ];

    for (cmd, data) in cmds {
        lcd_write_cmd(spi, cmd, data);
    }
}

// CMD, data
const WEA2012_INIT_CMDS: &[(u32, &[u8])] = &[
    (0xFF, &[0x20, 0x10, 0x43]),
    (0x04, &[0x70]),
    (0xFF, &[0x20, 0x10, 0x10]),
    (0x0C, &[0x11]),
    (0x10, &[0x02]),
    (0x11, &[0x11]),
    (0x15, &[0x42]),
    (0x16, &[0x11]),
    (0x1A, &[0x02]),
    (0x1B, &[0x11]),
    (0x61, &[0x80]),
    (0x62, &[0x80]),
    (0x54, &[0x44]),
    (0x58, &[0x88]),
    (0x5C, &[0xcc]),
    (0x20, &[0x80]),
    (0x21, &[0x81]),
    (0x22, &[0x31]),
    (0x23, &[0x20]),
    (0x24, &[0x11]),
    (0x25, &[0x11]),
    (0x26, &[0x12]),
    (0x27, &[0x12]),
    (0x30, &[0x80]),
    (0x31, &[0x81]),
    (0x32, &[0x31]),
    (0x33, &[0x20]),
    (0x34, &[0x11]),
    (0x35, &[0x11]),
    (0x36, &[0x12]),
    (0x37, &[0x12]),
    (0x41, &[0x11]),
    (0x42, &[0x22]),
    (0x43, &[0x33]),
    (0x49, &[0x11]),
    (0x4A, &[0x22]),
    (0x4B, &[0x33]),
    (0xFF, &[0x20, 0x10, 0x15]),
    (0x00, &[0x00]),
    (0x01, &[0x00]),
    (0x02, &[0x00]),
    (0x03, &[0x00]),
    (0x04, &[0x10]),
    (0x05, &[0x0C]),
    (0x06, &[0x23]),
    (0x07, &[0x22]),
    (0x08, &[0x21]),
    (0x09, &[0x20]),
    (0x0A, &[0x33]),
    (0x0B, &[0x32]),
    (0x0C, &[0x34]),
    (0x0D, &[0x35]),
    (0x0E, &[0x01]),
    (0x0F, &[0x01]),
    (0x20, &[0x00]),
    (0x21, &[0x00]),
    (0x22, &[0x00]),
    (0x23, &[0x00]),
    (0x24, &[0x0C]),
    (0x25, &[0x10]),
    (0x26, &[0x20]),
    (0x27, &[0x21]),
    (0x28, &[0x22]),
    (0x29, &[0x23]),
    (0x2A, &[0x33]),
    (0x2B, &[0x32]),
    (0x2C, &[0x34]),
    (0x2D, &[0x35]),
    (0x2E, &[0x01]),
    (0x2F, &[0x01]),
    (0xFF, &[0x20, 0x10, 0x16]),
    (0x00, &[0x00]),
    (0x01, &[0x00]),
    (0x02, &[0x00]),
    (0x03, &[0x00]),
    (0x04, &[0x08]),
    (0x05, &[0x04]),
    (0x06, &[0x19]),
    (0x07, &[0x18]),
    (0x08, &[0x17]),
    (0x09, &[0x16]),
    (0x0A, &[0x33]),
    (0x0B, &[0x32]),
    (0x0C, &[0x34]),
    (0x0D, &[0x35]),
    (0x0E, &[0x01]),
    (0x0F, &[0x01]),
    (0x20, &[0x00]),
    (0x21, &[0x00]),
    (0x22, &[0x00]),
    (0x23, &[0x00]),
    (0x24, &[0x04]),
    (0x25, &[0x08]),
    (0x26, &[0x16]),
    (0x27, &[0x17]),
    (0x28, &[0x18]),
    (0x29, &[0x19]),
    (0x2A, &[0x33]),
    (0x2B, &[0x32]),
    (0x2C, &[0x34]),
    (0x2D, &[0x35]),
    (0x2E, &[0x01]),
    (0x2F, &[0x01]),
    (0xFF, &[0x20, 0x10, 0x12]),
    (0x00, &[0x99]),
    (0x2A, &[0x28]),
    (0x2B, &[0x0f]),
    (0x2C, &[0x16]),
    (0x2D, &[0x28]),
    (0x2E, &[0x0f]),
    (0xFF, &[0x20, 0x10, 0xA0]),
    (0x08, &[0xdc]),
    (0xFF, &[0x20, 0x10, 0x45]),
    (0x03, &[0x64]),
    (0xFF, &[0x20, 0x10, 0x40]),
    (0x86, &[0x00]),
    (0xFF, &[0x20, 0x10, 0x00]),
    (0x2A, &[0x00, 0x00, 0x01, 0x63]),
    (0xFF, &[0x20, 0x10, 0x42]),
    (0x05, &[0x2c]),
    (0xFF, &[0x20, 0x10, 0x11]),
    (0x50, &[0x01]),
    (0xFF, &[0x20, 0x10, 0x12]),
    (0x0D, &[0x66]),
    (0xFF, &[0x20, 0x10, 0x17]),
    (0x39, &[0x3c]),
    (0xFF, &[0x20, 0x10, 0x31]),
    (0x00, &[0x00]),
    (0x01, &[0x00]),
    (0x02, &[0x00]),
    (0x03, &[0x09]),
    (0x04, &[0x00]),
    (0x05, &[0x1c]),
    (0x06, &[0x00]),
    (0x07, &[0x36]),
    (0x08, &[0x00]),
    (0x09, &[0x3d]),
    (0x0a, &[0x00]),
    (0x0b, &[0x54]),
    (0x0c, &[0x00]),
    (0x0d, &[0x62]),
    (0x0e, &[0x00]),
    (0x0f, &[0x72]),
    (0x10, &[0x00]),
    (0x11, &[0x79]),
    (0x12, &[0x00]),
    (0x13, &[0xa6]),
    (0x14, &[0x00]),
    (0x15, &[0xd0]),
    (0x16, &[0x01]),
    (0x17, &[0x0e]),
    (0x18, &[0x01]),
    (0x19, &[0x3d]),
    (0x1a, &[0x01]),
    (0x1b, &[0x7b]),
    (0x1c, &[0x01]),
    (0x1d, &[0xcf]),
    (0x1e, &[0x02]),
    (0x1f, &[0x0E]),
    (0x20, &[0x02]),
    (0x21, &[0x53]),
    (0x22, &[0x02]),
    (0x23, &[0x80]),
    (0x24, &[0x02]),
    (0x25, &[0xC2]),
    (0x26, &[0x02]),
    (0x27, &[0xFA]),
    (0x28, &[0x03]),
    (0x29, &[0x3E]),
    (0x2a, &[0x03]),
    (0x2b, &[0x52]),
    (0x2c, &[0x03]),
    (0x2d, &[0x70]),
    (0x2e, &[0x03]),
    (0x2f, &[0x8E]),
    (0x30, &[0x03]),
    (0x31, &[0xA2]),
    (0x32, &[0x03]),
    (0x33, &[0xBA]),
    (0x34, &[0x03]),
    (0x35, &[0xCF]),
    (0x36, &[0x03]),
    (0x37, &[0xe8]),
    (0x38, &[0x03]),
    (0x39, &[0xf0]),
    (0xFF, &[0x20, 0x10, 0x32]),
    (0x00, &[0x00]),
    (0x01, &[0x00]),
    (0x02, &[0x00]),
    (0x03, &[0x09]),
    (0x04, &[0x00]),
    (0x05, &[0x1c]),
    (0x06, &[0x00]),
    (0x07, &[0x36]),
    (0x08, &[0x00]),
    (0x09, &[0x3d]),
    (0x0a, &[0x00]),
    (0x0b, &[0x54]),
    (0x0c, &[0x00]),
    (0x0d, &[0x62]),
    (0x0e, &[0x00]),
    (0x0f, &[0x72]),
    (0x10, &[0x00]),
    (0x11, &[0x79]),
    (0x12, &[0x00]),
    (0x13, &[0xa6]),
    (0x14, &[0x00]),
    (0x15, &[0xd0]),
    (0x16, &[0x01]),
    (0x17, &[0x0e]),
    (0x18, &[0x01]),
    (0x19, &[0x3d]),
    (0x1a, &[0x01]),
    (0x1b, &[0x7b]),
    (0x1c, &[0x01]),
    (0x1d, &[0xcf]),
    (0x1e, &[0x02]),
    (0x1f, &[0x0E]),
    (0x20, &[0x02]),
    (0x21, &[0x53]),
    (0x22, &[0x02]),
    (0x23, &[0x80]),
    (0x24, &[0x02]),
    (0x25, &[0xC2]),
    (0x26, &[0x02]),
    (0x27, &[0xFA]),
    (0x28, &[0x03]),
    (0x29, &[0x3E]),
    (0x2a, &[0x03]),
    (0x2b, &[0x52]),
    (0x2c, &[0x03]),
    (0x2d, &[0x70]),
    (0x2e, &[0x03]),
    (0x2f, &[0x8E]),
    (0x30, &[0x03]),
    (0x31, &[0xA2]),
    (0x32, &[0x03]),
    (0x33, &[0xBA]),
    (0x34, &[0x03]),
    (0x35, &[0xCF]),
    (0x36, &[0x03]),
    (0x37, &[0xe8]),
    (0x38, &[0x03]),
    (0x39, &[0xf0]),
    (0xFF, &[0x20, 0x10, 0x11]),
    (0x60, &[0x01]),
    (0x65, &[0x03]),
    (0x66, &[0x38]),
    (0x67, &[0x04]),
    (0x68, &[0x34]),
    (0x69, &[0x03]),
    (0x61, &[0x03]),
    (0x62, &[0x38]),
    (0x63, &[0x04]),
    (0x64, &[0x34]),
    (0x0A, &[0x11]),
    (0x0B, &[0x14]),
    (0x0c, &[0x14]),
    (0x55, &[0x06]),
    (0xFF, &[0x20, 0x10, 0x42]),
    (0x05, &[0x3D]),
    (0x06, &[0x03]),
    (0xFF, &[0x20, 0x10, 0x12]),
    (0x1f, &[0xdc]),
    (0xFF, &[0x20, 0x10, 0x17]),
    (0x11, &[0xAA]),
    (0x16, &[0x12]),
    (0x0B, &[0xC3]),
    (0x10, &[0x0E]),
    (0x14, &[0xAA]),
    (0x18, &[0xA0]),
    (0x1A, &[0x80]),
    (0x1F, &[0x80]),
    (0xFF, &[0x20, 0x10, 0x11]),
    (0x30, &[0xEE]),
    (0xFF, &[0x20, 0x10, 0x12]),
    (0x15, &[0x0F]),
    (0x10, &[0x0F]),
    (0xFF, &[0x20, 0x10, 0x40]),
    (0x83, &[0xC4]),
    (0xFF, &[0x20, 0x10, 0x2d]),
    (0x01, &[0x3e]),
    (0xFF, &[0x20, 0x10, 0x12]),
    (0x2B, &[0x1e]),
    (0x2C, &[0x26]),
    (0x2E, &[0x1e]),
    (0xFF, &[0x20, 0x10, 0x18]),
    (0x01, &[0x01]),
    (0x00, &[0x1E]),
    (0xFF, &[0x20, 0x10, 0x43]),
    (0x03, &[0x04]),
    // touch For I2C
    (0xFF, &[0x20, 0x10, 0x50]),
    (0x05, &[0x00]),
    (0x00, &[0xA6]),
    (0x01, &[0xA6]),
    (0x08, &[0x55]),
    (0xFF, &[0x20, 0x10, 0x12]),
    (0x21, &[0xB4]), // set vcom
    (0xFF, &[0x20, 0x10, 0x00]),
    // ADD FOR QSPI/TP TEST
    (0x3A, &[0x05]), // Set 65K colour mode
    (0x11, &[0x00]),
    (0x29, &[0x00]),
];
