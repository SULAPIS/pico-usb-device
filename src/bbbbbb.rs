#![no_std]
#![no_main]

mod decode;
mod fmt;
mod movement;
mod sensor;
use core::fmt::Write;
use cortex_m::delay::Delay;
use cortex_m::prelude::_embedded_hal_blocking_i2c_Write;
use cortex_m::prelude::_embedded_hal_blocking_i2c_WriteRead;
use cortex_m_rt::entry;
use decode::ikcode;
use decode::ikcode::Angle;
use embedded_hal::blocking::i2c::{Operation, Read, Transactional};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;
use embedded_time::duration::*;
use embedded_time::rate::*;
use fmt::fmt::FmtBuf;
use hal::gpio::Function;
use hal::gpio::FunctionSpi;
use hal::gpio::I2C;
use hal::gpio::{bank0::*, Output, Pin, PushPull};
use hal::i2c::peripheral::I2CPeripheralEventIterator;
use heapless::Vec;
use movement::movement::*;
use panic_halt as _;
use rp_pico::hal;
use rp_pico::hal::pac;
use rp_pico::hal::pac::interrupt;
use rp_pico::hal::prelude::*;
use rp_pico::pac::I2C0;
use sensor::sensor::As5600;
// use serde::{Deserialize, Serialize};
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<hal::usb::UsbBus>> = None;

static mut TANGLE: Option<([decode::ikcode::Angle; 600], usize)> = None;
static mut TANGLES: Option<decode::ikcode::TAngle> = None;

static mut AS_ANGLE: f64 = 0.0;
static mut IS_PLAY: bool = true;
static mut LED_ERR: bool = false;

static mut BUEF: [u8; 20000] = [0u8; 20000];
static mut BUEF_COUNT: usize = 0;
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
    //led
    let mut led = pins.led.into_push_pull_output();
    led.set_high().unwrap();

    //as5600
    let sda_pin = pins.gpio16.into_mode::<hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio17.into_mode::<hal::gpio::FunctionI2C>();

    let i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        clocks.peripheral_clock,
    );

    let as5600 = As5600::new(i2c);

    // 驱动器
    let mut ena = pins.gpio22.into_push_pull_output();
    ena.set_low().unwrap();
    let mut dir = pins.gpio21.into_push_pull_output();
    dir.set_low().unwrap();
    let mut pul = pins.gpio20.into_push_pull_output();
    pul.set_low().unwrap();
    let mut pulse = Pulse::new(ena, dir, pul);

    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };

    let mut movement = MoveMent::new(as5600, &mut pulse, &mut delay);
    movement.update();

    let mut fmt = FmtBuf::new();
    fmt.reset();
    write!(
        &mut fmt,
        "{} {} {}",
        movement.angle, movement.circle, movement.as_angle
    )
    .unwrap();
    sent_message(&fmt);
    delay.delay_ms(2000);

    // pulse.disable();

    // let mut n = 0;
    // delay.delay_ms(2000);

    //匀变速  50 deg/s^2   2秒
    //匀速   100 deg/s    2秒
    //匀变速 -50 deg/s^2  2秒
    //匀变速 -50 deg/s^2  2秒
    //匀速   -100 deg/s   2秒
    //匀变速  50 deg/s^2  2秒
    //共12秒

    // movement
    //     .add_point(RPoint::new(0.0, 100.0, -50.0, 2.0, 2))
    //     .unwrap();
    // movement
    //     .add_point(RPoint::new(0.0, 0.0, -50.0, 5.0, 2))
    //     .unwrap();
    // movement
    //     .add_point(RPoint::new(0.0, -250.0, 0.0, 2.0, 1))
    //     .unwrap();
    // movement
    //     .add_point(RPoint::new(0.0, -250.0, 50.0, 5.0, 2))
    //     .unwrap();

    // movement
    //     .add_point(RPoint::new(360.0, 0.0, 0.0, 5.0, 0))
    //     .unwrap();
    // movement.start(&mut pulse, &mut delay).unwrap();
    // movement.clean_path();

    // movement
    //     .add_point(RPoint::new(0.0, 0.0, 0.0, 2.0, 0))
    //     .unwrap();
    // movement.start(&mut pulse, &mut delay).unwrap();
    // movement.clean_path();

    // movement
    //     .add_point(RPoint::new(90.0, 0.0, 0.0, 2.0, 0))
    //     .unwrap();
    // movement.start(&mut pulse, &mut delay).unwrap();
    // movement.clean_path();

    // delay.delay_ms(3000);
    let mut n: i32 = 0;
    let mut speed = 300;
    let mut k = 5;
    // pulse.disable();

    loop {
        //----------------------------------------------
        movement.update();
        // led.set_high();
        delay.delay_ms(5);
        // led.set_low();
        // delay.delay_ms(500);
        unsafe {
            AS_ANGLE = movement.as_angle;
            if LED_ERR == true {
                led.set_low().unwrap();
            } else {
                led.set_high().unwrap();
            }
        }
        unsafe {
            if IS_PLAY == false {
                IS_PLAY = true;
                movement.clean_path();
                let tangle = &TANGLE.unwrap();

                movement
                    .add_point(RPoint::new(tangle.0[0].angle, 0.0, 0.0, 1.0, 0))
                    .unwrap();
                movement.start(&mut pulse, &mut delay).unwrap();
                movement.clean_path();

                for i in 0..tangle.1 {
                    movement
                        .add_point(RPoint::new(
                            tangle.0[i].angle,
                            0.0,
                            0.0,
                            tangle.0[i].time,
                            0,
                        ))
                        .unwrap();
                }
            }
            movement.start(&mut pulse, &mut delay).unwrap();
            movement.clean_path();
        }
        //----------------------------------------------------------------

        //
        //     }
        // }
        // n += 1;
        // if n > 100 {
        //     n = 0;
        //     fmt.reset();
        //     write!(
        //         &mut fmt,
        //         "{} {} {}",
        //         movement.angle, movement.circle, movement.as_angle
        //     )
        //     .unwrap();
        //     sent_message(&fmt);
        // }

        // n += 1;
        // pulse.sent_pulse(&mut delay);
        // n += 1;
        // if n >= 8288 {
        //     n = 0;

        //     delay.delay_ms(500);
        //     // if speed < 30 {
        //     //     k = 5;
        //     // } else if speed > 300 {
        //     //     k = -5;
        //     // }
        //     // speed = (speed as i32 + k) as u32;
        // }

        // // // // pulse.sent_pulse(&mut delay);

        // delay.delay_ms(1);
    }
}

#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    pac::NVIC::mask(hal::pac::Interrupt::USBCTRL_IRQ);

    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let serial = USB_SERIAL.as_mut().unwrap();

    if usb_dev.poll(&mut [serial]) {
        let mut buf = [0u8; 65];
        match serial.read(&mut buf) {
            Err(_e) => {}
            Ok(0) => {}
            Ok(count) => {
                for i in 0..count {
                    BUEF[BUEF_COUNT + i] = buf[i];
                }
                BUEF_COUNT += count;
                // let mut fmt = FmtBuf::new();
                // fmt.reset();
                // write!(&mut fmt, "    {}  \n", &BUEF_COUNT);
                // serial.write(fmt.as_str().as_bytes()).unwrap();

                if count < 64 {
                    let mut kkk = &BUEF[..BUEF_COUNT];
                    let a: Result<
                        (([decode::ikcode::Angle; 600], usize), usize),
                        bincode::error::DecodeError,
                    > = bincode::decode_from_slice(kkk, bincode::config::standard());
                    match a {
                        Ok((k, _)) => {
                            LED_ERR = false;
                            TANGLE = Some(k);
                            IS_PLAY = false;
                        }
                        Err(_) => {
                            LED_ERR = true;
                        }
                    }

                    // while !kkk.is_empty() {
                    //     let _ = serial.write(kkk).map(|len| {
                    //         kkk = &kkk[len..];
                    //     });
                    // }
                    BUEF_COUNT = 0;
                }
            } // let mut n = 1;
              // while n > 0 {
              //     match serial.read(&mut buf) {
              //         Err(e) => {
              //             let mut fmt = FmtBuf::new();
              //             fmt.reset();
              //             write!(&mut fmt, "    {:?} 2 \n", &e);
              //             serial.write(fmt.as_str().as_bytes()).unwrap();
              //             // break;
              //         }
              //         Ok(0) => {
              //             n = 0;
              //             break;
              //         }
              //         Ok(count) => {
              //             n = count;
              //             let mut fmt = FmtBuf::new();
              //             fmt.reset();
              //             write!(&mut fmt, "    {} 1 \n", &n);
              //             serial.write(fmt.as_str().as_bytes()).unwrap();
              //             if count < 64 {
              //                 break;
              //             }
              //         }
              //     }
              // n = serial.read(&mut buf).unwrap();
              // num += n;
              // for i in 0..count {
              //     buef[num + i] = buf[i];
              // }

              // serial.write(&mut buef[..num]).unwrap();

              // let a: Result<(decode::ikcode::TAngle, usize), bincode::error::DecodeError> =
              //     bincode::decode_from_slice(&buf[..count], bincode::config::standard());

              // let a = [Angle::new(0.0); 30];
              // let mut buff = [0u8; 5000];

              // let n = bincode::encode_into_slice(
              //     ([Angle::new(0.0); 100], 3),
              //     &mut buff,
              //     bincode::config::standard(),
              // )
              // .unwrap();
              // let mut kkk = &buff[..n];

              // serial.write(&mut buf[..count]).unwrap();

              // match a {
              //     Ok((k, _)) => {
              //         LED_ERR = false;
              //         TANGLE = Some(k);
              //         IS_PLAY = false;
              //     }
              //     Err(_) => {
              //         LED_ERR = true;
              //     }
              // }

              // match buf[0] {
              //     0x01 => {
              //         let angle = ikcode::Angle::new(AS_ANGLE, 1.0);
              //         let mut buff = [0u8; 1000];
              //         let len = bincode::encode_into_slice(
              //             angle,
              //             &mut buff,
              //             bincode::config::standard(),
              //         )
              //         .unwrap();
              //         serial.write(&buff[..len]).unwrap();
              //     }
              //     _ => {}
              // }

              //测试用
              // let mut fmt = FmtBuf::new();
              // fmt.reset();
              // write!(&mut fmt, "jlkk").unwrap();
        }
    }

    pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
}

fn sent_message(buf: &FmtBuf) {
    let mut wr_ptr = buf.as_str();
    cortex_m::interrupt::free(|_| unsafe {
        let serial = USB_SERIAL.as_mut().unwrap();
        let _ = serial.write(wr_ptr.as_bytes()).map(|len| {
            wr_ptr = &wr_ptr[len..];
        });
    });
}

//传输信息结构体
// #[derive(Serialize, Deserialize, Debug)]
// struct Angle {
//     angle: u16,
//     //1:lift 0:right
//     dir: bool,
//     time: f64,
// }
// impl Angle {
//     const fn new(angle: u16, dir: bool) -> Self {
//         Self {
//             angle,
//             dir,
//             time: 0.0,
//         }
//     }
// }
