#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_time::fixed_point::FixedPoint;
use panic_halt as _;
use rp_pico::hal::pac::interrupt;
use rp_pico::hal::prelude::*;

use rp_pico::hal::pac;

use rp_pico::hal;

use usb_device::{class_prelude::*, prelude::*};
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::descriptor::MouseReport;
use usbd_hid::descriptor::SerializedDescriptor;
use usbd_hid::hid_class::HIDClass;

static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

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

    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    unsafe {
        USB_BUS = Some(usb_bus);
    }

    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    let usb_hid = HIDClass::new(bus_ref, MouseReport::desc(), 60);

    unsafe {
        USB_HID = Some(usb_hid);
    }

    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27da))
        .manufacturer("lapis company")
        .product("Mouse")
        .serial_number("TEST")
        .device_class(0xEF)
        .build();

    unsafe {
        USB_DEVICE = Some(usb_dev);
    }

    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };
    let core = pac::CorePeripherals::take().unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    loop {
        delay.delay_ms(50);
        let rep_up = MouseReport {
            x: 0,
            y: 4,
            buttons: 0,
            wheel: 0,
            pan: 0,
        };
        push_mouse_movement(rep_up).ok().unwrap_or(0);
    }
}
fn push_mouse_movement(report: MouseReport) -> Result<usize, usb_device::UsbError> {
    cortex_m::interrupt::free(|_| unsafe { USB_HID.as_mut().map(|hid| hid.push_input(&report)) })
        .unwrap()
}

#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let usb_hid = USB_HID.as_mut().unwrap();
    usb_dev.poll(&mut [usb_hid]);
}
