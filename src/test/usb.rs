#![no_std]
#![no_main]

// The macro for our start-up function
use cortex_m_rt::entry;

use embedded_hal::digital::v2::{InputPin, OutputPin, ToggleableOutputPin};
use embedded_time::rate::Extensions;
use hal::{
    gpio::{
        bank0::{Gpio12, Gpio13},
        Output, Pin, PushPull,
    },
    spi::Enabled,
    Spi,
};
// The macro for marking our interrupt functions
use core::str::from_utf8;
use core::{fmt::Write, str::FromStr};
use rp_pico::{hal::pac::interrupt, pac::SPI1};
// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use display_interface_spi::SPIInterfaceNoCS;
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, Line, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, Triangle},
    text::{Baseline, Text},
};
use panic_halt as _;
use st7789::Orientation;
use st7789::ST7789;
use tinybmp::DynamicBmp;
// Pull in any important traits
use embedded_time::fixed_point::FixedPoint;
use rp_pico::hal::prelude::*;
// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::{gpio, pac, spi};

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

use usbd_serial::SerialPort;
// USB Human Interface Device (HID) Class support
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::descriptor::MouseReport;
use usbd_hid::hid_class::HIDClass;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
static mut DISPLAY: Option<
    ST7789<
        SPIInterfaceNoCS<Spi<Enabled, SPI1, 8>, Pin<Gpio13, Output<PushPull>>>,
        Pin<Gpio12, Output<PushPull>>,
    >,
> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<hal::usb::UsbBus>> = None;

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

    let sio = hal::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_BUS = Some(usb_bus);
    }

    // Grab a reference to the USB Bus allocator. We are promising to the
    // compiler not to take mutable access to this global variable whilst this
    // reference exists!
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    // Set up the USB HID Class Device driver, providing Mouse Reports
    let usb_hid = HIDClass::new(bus_ref, MouseReport::desc(), 60);
    let serial = SerialPort::new(bus_ref);
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet.
        USB_HID = Some(usb_hid);
        USB_SERIAL = Some(serial);
    }

    // Create a USB device with a fake VID and PID
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27da))
        .product("Twitchy Mousey")
        .device_class(0xef)
        .composite_with_iads()
        .build();
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());
    let mut led = pins.led.into_push_pull_output();

    let up_pin = pins.gpio26.into_pull_up_input();
    let down_pin = pins.gpio27.into_pull_up_input();
    let left_pin = pins.gpio16.into_pull_up_input();
    let right_pin = pins.gpio17.into_pull_up_input();
    let mid_pin = pins.gpio22.into_pull_up_input();

    let _spi_sclk = pins.gpio10.into_mode::<gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio11.into_mode::<gpio::FunctionSpi>();
    let _spi_imso = pins.gpio8.into_mode::<gpio::FunctionSpi>();
    let disp_res = pins.gpio12.into_push_pull_output();
    let disp_dc = pins.gpio13.into_push_pull_output();

    let spi = spi::Spi::<_, _, 8>::new(pac.SPI1);

    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        32_000_000u32.Hz(),
        &embedded_hal::spi::MODE_3,
    );
    let di = SPIInterfaceNoCS::new(spi, disp_dc);
    let mut display = ST7789::new(di, disp_res, 240, 240);
    display.init(&mut delay).unwrap();
    display.set_orientation(Orientation::Portrait).unwrap();
    display.clear(Rgb565::BLACK).unwrap();
    unsafe {
        DISPLAY = Some(display);
    }
    unsafe {
        // Enable the USB interrupt
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };

    let rep_up = MouseReport {
        x: 0,
        y: -4,
        buttons: 0,
        wheel: 0,
        pan: 0,
    };
    let rep_down = MouseReport {
        x: 0,
        y: 4,
        buttons: 0,
        wheel: 0,
        pan: 0,
    };
    let rep_left = MouseReport {
        x: -4,
        y: 0,
        buttons: 0,
        wheel: 0,
        pan: 0,
    };
    let rep_right = MouseReport {
        x: 4,
        y: 0,
        buttons: 0,
        wheel: 0,
        pan: 0,
    };
    let rep_mid = MouseReport {
        x: 0,
        y: 0,
        buttons: 1,
        wheel: 0,
        pan: 0,
    };
    let rep_release = MouseReport {
        x: 0,
        y: 0,
        buttons: 0,
        wheel: 0,
        pan: 0,
    };
    let mut led_count = 0;
    let mut num = 0;
    // Move the cursor up and down every 200ms
    loop {
        led_count += 1;
        if led_count == 100 {
            led.toggle().unwrap();
            led_count = 0;
            num += 1;
        }
        delay.delay_ms(5);
        if up_pin.is_low().unwrap() {
            push_mouse_movement(rep_up).ok().unwrap_or(0);
        }
        if down_pin.is_low().unwrap() {
            push_mouse_movement(rep_down).ok().unwrap_or(0);
        }
        if left_pin.is_low().unwrap() {
            push_mouse_movement(rep_left).ok().unwrap_or(0);
        }
        if right_pin.is_low().unwrap() {
            push_mouse_movement(rep_right).ok().unwrap_or(0);
        }
        if mid_pin.is_low().unwrap() {
            push_mouse_movement(rep_mid).ok().unwrap_or(0);
        } else {
            push_mouse_movement(rep_release).ok().unwrap_or(0);
        }
    }
}

/// Submit a new mouse movement report to the USB stack.
///
/// We do this with interrupts disabled, to avoid a race hazard with the USB IRQ.
fn push_mouse_movement(report: MouseReport) -> Result<usize, usb_device::UsbError> {
    cortex_m::interrupt::free(|_| unsafe {
        // Now interrupts are disabled, grab the global variable and, if
        // available, send it a HID report
        USB_HID.as_mut().map(|hid| hid.push_input(&report))
    })
    .unwrap()
}

/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    use core::sync::atomic::{AtomicBool, Ordering};
    static SAID_HELLO: AtomicBool = AtomicBool::new(false);
    // Handle USB request
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let usb_hid = USB_HID.as_mut().unwrap();
    let serial = USB_SERIAL.as_mut().unwrap();
    let display = DISPLAY.as_mut().unwrap();

    if !SAID_HELLO.load(Ordering::Relaxed) {
        SAID_HELLO.store(true, Ordering::Relaxed);
        let _ = serial.write(b"Hello, World!\r\r");
    }

    // usb_dev.poll(&mut [usb_hid]);
    if usb_dev.poll(&mut [usb_hid, serial]) {
        let mut buf = [0u8; 64];
        match serial.read(&mut buf) {
            Ok(0) => {}
            Err(_e) => {}
            Ok(count) => {
                buf.iter_mut().take(count).for_each(|b| {
                    b.make_ascii_uppercase();
                });

                let mut wr_ptr = &buf[..count];
                while !wr_ptr.is_empty() {
                    let _ = serial.write(wr_ptr).map(|len| {
                        wr_ptr = &wr_ptr[len..];
                    });
                    let text_style = MonoTextStyleBuilder::new()
                        .font(&FONT_10X20)
                        .text_color(Rgb565::CYAN)
                        .build();
                    let mut buff = FmtBuf::new();
                    buff.reset();
                    write!(&mut buff, "receive: {}", from_utf8(&buf[..count]).unwrap()).unwrap();
                    display
                        .fill_solid(
                            &Rectangle::new(
                                Point { x: 0, y: 0 },
                                Size {
                                    width: 240,
                                    height: 20,
                                },
                            ),
                            Rgb565::BLACK,
                        )
                        .unwrap();
                    Text::with_baseline(buff.as_str(), Point::zero(), text_style, Baseline::Top)
                        .draw(display)
                        .unwrap();
                }
            }
        }
    }
}
struct FmtBuf {
    buf: [u8; 64],
    ptr: usize,
}
impl FmtBuf {
    fn new() -> Self {
        Self {
            buf: [0; 64],
            ptr: 0,
        }
    }
    fn reset(&mut self) {
        self.ptr = 0;
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[0..self.ptr]).unwrap()
    }
}
impl core::fmt::Write for FmtBuf {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let rest_len = self.buf.len() - self.ptr;
        let len = if rest_len < s.len() {
            rest_len
        } else {
            s.len()
        };
        self.buf[self.ptr..(self.ptr + len)].copy_from_slice(&s.as_bytes()[0..len]);
        self.ptr += len;
        Ok(())
    }
}
