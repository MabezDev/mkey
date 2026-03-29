#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::ptr::addr_of_mut;
use core::sync::atomic::Ordering;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embedded_graphics::geometry::Point;
use embedded_graphics::image::Image;
use embedded_graphics::Drawable;
use embedded_graphics_core::pixelcolor::Rgb565;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::dma::DmaRxBuf;
use esp_hal::dma::DmaTxBuf;
use esp_hal::dma_buffers;
use esp_hal::gpio::InputConfig;
use esp_hal::gpio::Level;
use esp_hal::interrupt;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::interrupt::Priority;
#[cfg(not(feature = "debug-matrix"))]
use esp_hal::otg_fs::Usb;
use esp_hal::peripherals::Interrupt;
use esp_hal::system::Stack;
use esp_hal::time::Rate;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{
    delay::Delay,
    gpio::Io,
    gpio::{Input, Output, Pull},
    main,
    spi::{master::Spi, Mode},
};
use esp_rtos::{self, embassy::Executor};
use usbd_hid::descriptor::KeyboardReport;

use display::{
    lcd_fill, lcd_write_cmd, FRAME_BUFFER, HEIGHT, MAX_DMA_TRANSFER, TE, TE_READY,
    WEA2012_INIT_CMDS, WIDTH,
};

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

mod display;
mod keyboard;
mod keymap;

static mut APP_CORE_STACK: Stack<4096> = Stack::new();

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
    io.set_interrupt_handler(display::te);
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
            #[cfg(not(feature = "debug-matrix"))]
            let device = Usb::new(peripherals.USB0, peripherals.GPIO20, peripherals.GPIO19);
            let signal = &*make_static!(Signal<CriticalSectionRawMutex, KeyboardReport>, Signal::new());
            let executor = make_static!(Executor, Executor::new());
            executor.run(|spawner| {
                spawner.must_spawn(keyboard::matrix(columns, rows, signal));
                #[cfg(not(feature = "debug-matrix"))]
                spawner.must_spawn(keyboard::usb(device, signal));
                #[cfg(feature = "debug-matrix")]
                spawner.must_spawn(keyboard::debug_consumer(signal));
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
