#![no_std]
#![no_main]

// The macro for our start-up function
use cortex_m_rt::entry;

use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;
use embedded_time::rate::*;
// The macro for marking our interrupt functions
use rp_pico::hal::pac::interrupt;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use embedded_time::fixed_point::FixedPoint;
use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;
use rp_pico::hal::gpio;

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

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<hal::usb::UsbBus>> = None;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then submits cursor movement
/// updates periodically.
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

    unsafe {
        // Enable the USB interrupt
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());
    let mut led = pins.led.into_push_pull_output();

    let up_pin = pins.gpio26.into_pull_up_input();
    let down_pin = pins.gpio27.into_pull_up_input();
    let left_pin = pins.gpio16.into_pull_up_input();
    let right_pin = pins.gpio17.into_pull_up_input();
    let mid_pin = pins.gpio22.into_pull_up_input();

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
    // Move the cursor up and down every 200ms
    loop {
        led_count += 1;
        if led_count == 25 {
            led.toggle().unwrap();
            led_count = 0;
        }
        delay.delay_ms(20);
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
                }
            }
        }
    }
    // usb_dev.poll(&mut [usb_hid, serial]);
}

// End of file
