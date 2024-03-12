#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::cell::RefCell;
use core::sync::atomic::AtomicBool;

use core::sync::atomic::Ordering;
use critical_section::Mutex;
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::framebuffer::buffer_size;
use embedded_graphics::framebuffer::Framebuffer;
use embedded_graphics::geometry::Point;
use embedded_graphics::image::Image;
use embedded_graphics::pixelcolor::raw::BigEndian;
use embedded_graphics::pixelcolor::raw::RawU16;
use embedded_graphics::pixelcolor::RgbColor;
use embedded_graphics::Drawable;
use embedded_graphics_core::pixelcolor::Rgb565;
use esp_backtrace as _;
use esp_hal::cpu_control::CpuControl;
use esp_hal::cpu_control::Stack;
use esp_hal::dma::{Dma, DmaPriority};
use esp_hal::dma_descriptors;
use esp_hal::interrupt;
use esp_hal::interrupt::Priority;
use esp_hal::peripherals::Interrupt;
use esp_hal::systimer::SystemTimer;
use esp_hal::{
    clock::ClockControl,
    gpio::{AnyPin, Input, Output, PullDown, PushPull},
    peripherals::Peripherals,
    prelude::*,
    spi::{
        master::{prelude::*, Address, Command, Spi},
        HalfDuplexMode, SpiDataMode, SpiMode,
    },
    Delay, IO,
};

type QSpiDisplay<'a> = esp_hal::spi::master::dma::SpiDma<
    'a,
    esp_hal::peripherals::SPI2,
    esp_hal::dma::Channel0,
    HalfDuplexMode,
>;

const MAX_DMA_TRANSFER: usize = 32736;
const WIDTH: usize = 356;
const HEIGHT: usize = 400;

static TE: Mutex<RefCell<Option<esp_hal::gpio::Gpio17<Input<PullDown>>>>> =
    Mutex::new(RefCell::new(None));

static TE_READY: AtomicBool = AtomicBool::new(false);
static mut APP_CORE_STACK: Stack<16384> = Stack::new();
static mut FRAME_BUFFER: Framebuffer<
    Rgb565,
    RawU16,
    BigEndian,
    WIDTH,
    HEIGHT,
    { buffer_size::<Rgb565>(WIDTH, HEIGHT) },
> = Framebuffer::new();

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

    let system = peripherals.SYSTEM.split();

    let clocks = ClockControl::max(system.clock_control).freeze();
    let mut cpu_control = CpuControl::new(system.cpu_control);

    let mut delay = Delay::new(&clocks);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let sclk = io.pins.gpio4;
    let sio0 = io.pins.gpio6;
    let sio1 = io.pins.gpio5;
    let sio2 = io.pins.gpio15;
    let sio3 = io.pins.gpio7;
    let cs = io.pins.gpio16.into_push_pull_output();
    let mut reset = io.pins.gpio3.into_push_pull_output();

    let mut te = io.pins.gpio17.into_pull_down_input();
    te.listen(esp_hal::gpio::Event::RisingEdge);
    critical_section::with(|cs| TE.borrow_ref_mut(cs).replace(te));

    interrupt::enable(Interrupt::GPIO, Priority::Priority2).unwrap();

    let dma = Dma::new(peripherals.DMA);

    let (mut tx_descriptors, mut rx_descriptors) =
        dma_descriptors!(MAX_DMA_TRANSFER, MAX_DMA_TRANSFER);

    // Half-Duplex because the display will only send a response once the master has finished
    let mut spi = Spi::new_half_duplex(peripherals.SPI2, 80u32.MHz(), SpiMode::Mode3, &clocks)
        .with_pins(
            Some(sclk),
            Some(sio0),
            Some(sio1),
            Some(sio2),
            Some(sio3),
            Some(cs),
        )
        .with_dma(dma.channel0.configure(
            false,
            &mut tx_descriptors,
            &mut rx_descriptors,
            DmaPriority::Priority0,
        ));
    reset.set_low().unwrap();
    delay.delay_ms(300u32);
    reset.set_high().unwrap();
    delay.delay_ms(300u32);

    // initialization commands
    for (cmd, data) in WEA2012_INIT_CMDS {
        lcd_write_cmd(&mut spi, *cmd, data);
    }
    log::info!("Finished initializing display!");

    let pixels = unsafe { &mut FRAME_BUFFER };
    /*
       Matrix
    */
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

    let _guard = cpu_control
        .start_app_core(unsafe { &mut APP_CORE_STACK }, move || {
            // TODO why is this a closure, but crashes if we put code in here directly?
            // for some reason we have to call a function for it to not to get a `InstrProhibited` exception (and a broken stack trace)
            // could be `set_stack_pointer` not being inlined?
            core1_task(rows, columns, delay)
        })
        .unwrap();

    let image =
        tinygif::Gif::<Rgb565>::from_slice(include_bytes!("../Ferris-240x240.gif")).unwrap();
    let mut start = SystemTimer::now();
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
            let now = SystemTimer::now();
            lcd_fill(&mut spi, pixels);
            log::trace!(
                "Time to fill display: {}ms",
                (SystemTimer::now() - now) / (SystemTimer::TICKS_PER_SECOND / 1024)
            );
            frames += 1;
            let now = SystemTimer::now();
            if now.wrapping_sub(start) > SystemTimer::TICKS_PER_SECOND {
                start = now;
                log::info!("FPS: {}", frames);
                frames = 0;
            }
        }
    }
}

fn core1_task(
    rows: &mut [AnyPin<Input<PullDown>>],
    columns: &mut [AnyPin<Output<PushPull>>],
    mut delay: Delay,
) {
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
                delay.delay_us(1u32);
            }
        }
    }
}

#[ram]
#[interrupt]
unsafe fn GPIO() {
    static mut LAST: u64 = 0;
    let now = SystemTimer::now();
    log::trace!(
        "TE fired! Interval: {}ms",
        (now - LAST) / (SystemTimer::TICKS_PER_SECOND / 1024)
    );
    LAST = now;
    critical_section::with(|cs| TE.borrow_ref_mut(cs).as_mut().unwrap().clear_interrupt());
    TE_READY.store(true, Ordering::SeqCst);
}

fn lcd_fill<'a>(spi: &mut QSpiDisplay<'a>, pixels: &'static [u8]) {
    set_draw_area(spi, 0, 0, WIDTH as u16 - 1, HEIGHT as u16 - 1);
    for (i, pixels) in pixels.chunks(MAX_DMA_TRANSFER).enumerate() {
        // fifo size
        spi.write(
            SpiDataMode::Quad,
            Command::Command8(0x32, SpiDataMode::Single),
            Address::Address24(if i > 0 { 0x002C00 } else { 0x003C00 }, SpiDataMode::Single),
            0,
            &pixels,
        )
        .unwrap()
        .wait()
        .unwrap();
    }
}
fn lcd_write_cmd<'a>(spi: &mut QSpiDisplay<'a>, cmd: u32, data: &[u8]) {
    static mut BUF: [u8; 4] = [0u8; 4];
    let buf = unsafe { &mut BUF };
    if data.len() > 0 {
        buf[..data.len()].copy_from_slice(&data);
    }
    log::trace!("Sending: {} => {:?}", cmd, &buf[..data.len()]);
    spi.write(
        SpiDataMode::Single,
        Command::Command8(0x02, SpiDataMode::Single),
        Address::Address24(cmd << 8, SpiDataMode::Single),
        0,
        &&buf[..data.len()],
    )
    .unwrap()
    .wait()
    .unwrap();
}

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
