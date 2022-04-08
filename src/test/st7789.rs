#![no_std]
#![no_main]

// The macro for our start-up function
use cortex_m_rt::entry;

use embedded_graphics::image::Image;
use embedded_graphics::pixelcolor::Rgb555;
use embedded_graphics::primitives::PrimitiveStyleBuilder;
use panic_halt as _;
use tinybmp::Bmp;
// Pull in any important traits
use rp_pico::hal::prelude::*;

// Embed the `Hz` function/trait:
use embedded_time::rate::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// Import the SPI abstraction:
use rp_pico::hal::spi;

// Import the GPIO abstraction:
use rp_pico::hal::gpio;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::*;
use rp_pico::hal;
use ssd1306::prelude::SPIInterfaceNoCS;
use st7789::Orientation;
use st7789::ST7789;
use tinybmp::DynamicBmp;

#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set the LED to be an output
    let mut led_pin = pins.led.into_push_pull_output();

    // Setup a delay for the LED blink signals:
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio10.into_mode::<gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio11.into_mode::<gpio::FunctionSpi>();
    let _spi_miso = pins.gpio8.into_mode::<gpio::FunctionSpi>();
    let disp_res = pins.gpio12.into_push_pull_output();

    let disp_dc = pins.gpio13.into_push_pull_output();

    // Create an SPI driver instance for the SPI0 device
    let spi = spi::Spi::<_, _, 8>::new(pac.SPI1);

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_3,
    );

    let di = SPIInterfaceNoCS::new(spi, disp_dc);
    let mut display = ST7789::new(di, disp_res, 240, 240);

    display.init(&mut delay).unwrap();
    display.set_orientation(Orientation::Landscape).unwrap();

    let circle1 =
        Circle::new(Point::new(128, 64), 64).into_styled(PrimitiveStyle::with_fill(Rgb565::RED));
    let circle2 = Circle::new(Point::new(64, 64), 64)
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::GREEN, 1));

    let blue_with_red_outline = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::BLUE)
        .stroke_color(Rgb565::RED)
        .stroke_width(1) // > 1 is not currently supported in embedded-graphics on triangles
        .build();

    let triangle = Triangle::new(
        Point::new(40, 120),
        Point::new(40, 220),
        Point::new(140, 120),
    )
    .into_styled(blue_with_red_outline);

    let line = Line::new(Point::new(180, 160), Point::new(239, 239))
        .into_styled(PrimitiveStyle::with_stroke(RgbColor::WHITE, 10));

    display.clear(Rgb565::BLACK).unwrap();
    circle1.draw(&mut display).unwrap();
    circle2.draw(&mut display).unwrap();
    triangle.draw(&mut display).unwrap();
    line.draw(&mut display).unwrap();

    let bmp_data = include_bytes!("testsss.bmp");
    let bmp = DynamicBmp::from_slice(bmp_data).unwrap();
    let data = Image::new(&bmp, Point::new(0, 0));
    display.clear(Rgb565::BLACK).unwrap();
    data.draw(&mut display).unwrap();

    loop {
        delay.delay_ms(1000);
    }
}
