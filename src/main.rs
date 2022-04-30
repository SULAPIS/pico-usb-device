#![no_std]
#![no_main]
use core::fmt::Write;
use cortex_m::delay::Delay;
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::duration::Extensions;
use embedded_time::rate::*;
use hal::gpio::{bank0::*, Output, Pin, PushPull};
use heapless::Vec;
use micromath::F32Ext;
use panic_halt as _;
use rp_pico::hal;
use rp_pico::hal::pac;
use rp_pico::hal::pac::interrupt;
use rp_pico::hal::prelude::*;
use serde::{Deserialize, Serialize};
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

#[derive(Serialize, Deserialize)]
struct Connection {
    x: f32,
    y: f32,
    z: f32,
}

static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

static mut USB_SERIAL: Option<SerialPort<hal::usb::UsbBus>> = None;
static mut ANGLE: f32 = 0.0;
static mut TARGET: f32 = 0.0;
static mut TARGET_CHANGE: bool = false;
static mut DURATION: usize = 0;
static mut VEC: Vec<f32, 2> = Vec::<_, 2>::new();

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
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

    let serial = SerialPort::new(bus_ref);
    unsafe {
        USB_SERIAL = Some(serial);
    }

    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("LAPIS")
        .product("LAPIS")
        .serial_number("LAPIS")
        .device_class(2)
        .build();
    unsafe {
        USB_DEVICE = Some(usb_dev);
    }

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let sio = hal::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();
    let mut dir_pin = pins.gpio21.into_push_pull_output();
    dir_pin.set_high().unwrap();
    let mut pul_pin = pins.gpio20.into_push_pull_output();
    //  unsafe {
    //      DIR_PIN = Some(dir_pin);
    //      PUL_PIN = Some(pul_pin);
    //  }

    let calc_round = |rotation: f32| (rotation.abs() / 0.225).round() as usize;
    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };

    let mut r = 0;
    let mut rotation = 0.0;
    let mut d = 0.225;
    let mut time = 0;
    // Blink the LED at 1 Hz
    loop {
        unsafe {
            if TARGET_CHANGE {
                TARGET_CHANGE = false;
                // let mut rotation = TARGET - ANGLE;
                // rotation = TARGET - ANGLE;
                // if rotation >= 0.0 {
                //     dir_pin.set_high().unwrap();
                //     d = 0.225;
                // } else {
                //     dir_pin.set_low().unwrap();
                //     d = -0.225;
                // }
                // if ANGLE > 270.0 && TARGET < 90.0 {
                //     rotation = 360.0 - ANGLE + TARGET;
                //     d = 0.225;
                // }
                // if ANGLE < 90.0 && TARGET > 270.0 {
                //     rotation = 360.0 - ANGLE + TARGET;
                //     d = -0.225;
                // }

                let (rotation, a) = calc_move();

                r = calc_round(rotation) as i32;
                // sent_message(4.0, r as _);
                if a {
                    dir_pin.set_high().unwrap();
                    d = 0.225;
                } else {
                    dir_pin.set_low().unwrap();
                    d = -0.225;
                }
                time = (DURATION * 1000) / r as usize;
            }
        }

        unsafe {
            if r > 0 {
                // for _ in 0..r {
                r -= 1;
                pul_pin.set_high().unwrap();
                delay.delay_us(120);
                pul_pin.set_low().unwrap();
                ANGLE += d;
                ANGLE = (ANGLE * 1000.0).round() / 1000.0;
                if ANGLE < 0.0 {
                    ANGLE = 360.0 + ANGLE;
                } else if ANGLE > 360.0 {
                    ANGLE = ANGLE - 360.0;
                }

                delay.delay_ms(time as _);
                // delay.delay_us(600);
            }
            // if r > 0 {
            //     sent_message(ANGLE, r as _);
            // }
        }
    }
}

fn calc_move() -> (f32, bool) {
    let mut t_angle = 0.0;
    let mut t_target = 0.0;
    unsafe {
        t_angle = ANGLE;
        t_target = TARGET;
    }

    let mut a1 = 360.0 - t_angle + t_target;
    if a1 > 360.0 {
        a1 = a1 - 360.0;
    }
    let a2 = 360.0 - a1;
    if a1 < a2 {
        return (a1, true);
    } else {
        return (a2, false);
    }
}

#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let serial = USB_SERIAL.as_mut().unwrap();

    if usb_dev.poll(&mut [serial]) {
        let mut buf = [0u8; 64];
        match serial.read(&mut buf) {
            Err(_e) => {}
            Ok(0) => {}
            Ok(count) => {
                // 纯串口测试
                let wr_ptr = core::str::from_utf8(&buf[..count]).unwrap();
                VEC = wr_ptr
                    .split(' ')
                    .into_iter()
                    .map(|c| core::str::FromStr::from_str(c).unwrap())
                    .collect();

                let mut fmt = FmtBuf::new();
                fmt.reset();
                write!(&mut fmt, "{:?}", VEC).unwrap();

                serial.write(fmt.as_str().as_bytes()).unwrap();

                // let mut angle: f32 = core::str::FromStr::from_str(wr_ptr).unwrap();
                let mut angle = VEC[0];
                let d = VEC[1] as usize;

                // // godot串口通信
                // let (connection, _): (Connection, usize) =
                //     serde_json_core::from_slice(&buf[..count]).unwrap();
                // let mut angle = connection.x;

                // angle = (angle * 1000.0).round() / 1000.0;
                // if angle > 360.0 {
                //     angle = 360.0;
                // } else if angle < 0.0 {
                //     angle = 0.0;
                // }

                TARGET = angle;
                DURATION = d;
                TARGET_CHANGE = true;
            }
        }
    }
}

fn sent_message(angle: f32, r: usize) {
    let mut buf = FmtBuf::new();
    buf.reset();
    write!(&mut buf, "counter: {} , timer: {}", angle, r).unwrap();
    let mut wr_ptr = buf.as_str();
    cortex_m::interrupt::free(|_| unsafe {
        let serial = USB_SERIAL.as_mut().unwrap();
        let _ = serial.write(wr_ptr.as_bytes()).map(|len| {
            wr_ptr = &wr_ptr[len..];
        });
    });
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
