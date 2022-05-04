#![no_std]
#![no_main]
use core::fmt::Write;
use cortex_m::delay::Delay;
use cortex_m::prelude::_embedded_hal_blocking_i2c_Write;
use cortex_m::prelude::_embedded_hal_blocking_i2c_WriteRead;
use cortex_m_rt::entry;
use embedded_hal::blocking::i2c::{Operation, Read, Transactional};
use embedded_hal::digital::v2::OutputPin;
use embedded_time::duration::*;
use embedded_time::rate::*;
use hal::gpio::Function;
use hal::gpio::I2C;
use hal::gpio::{bank0::*, Output, Pin, PushPull};
use heapless::Vec;
use panic_halt as _;
use rp_pico::hal;
use rp_pico::hal::pac;
use rp_pico::hal::pac::interrupt;
use rp_pico::hal::prelude::*;
use rp_pico::pac::I2C0;
use serde::{Deserialize, Serialize};
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

static mut USB_SERIAL: Option<SerialPort<hal::usb::UsbBus>> = None;

//上一次记录角度
static mut LAST_NUM: u16 = 0;
//当前角度
static mut NUM: u16 = 0;
//旋转方向
static mut DIR: bool = true;

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
    let mut movement = MoveMent::new(as5600);

    //驱动器
    let mut ena = pins.gpio22.into_push_pull_output();
    ena.set_low().unwrap();
    let mut dir = pins.gpio21.into_push_pull_output();
    dir.set_low().unwrap();
    let mut pul = pins.gpio20.into_push_pull_output();
    pul.set_low().unwrap();
    let mut pulse = Pulse::new(ena, dir, pul);

    let mut led = pins.led.into_push_pull_output();
    led.set_low().unwrap();
    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };

    let mut n = 0;

    loop {
        // pulse.sent_pulse(&mut delay);
        n += 1;
        if n >= 8288 {
            n = 0;
            delay.delay_ms(500);
        }

        pulse.sent_pulse(&mut delay);

        delay.delay_us(200);
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
                let mut sent_a = Angle::new(0, false);
                unsafe {
                    if LAST_NUM > NUM {
                        DIR = true;
                    } else {
                        DIR = false;
                    }
                    LAST_NUM = NUM;
                    sent_a = Angle::new(NUM, DIR);
                }

                let mut buf: [u8; 64] = [0; 64];
                let n = serde_json_core::to_slice(&sent_a, &mut buf).unwrap();

                //测试用
                // let mut fmt = FmtBuf::new();
                // fmt.reset();
                // write!(&mut fmt, "{:?}", sent_a).unwrap();

                // serial.write(&fmt.as_str().as_bytes()).unwrap();

                serial.write(&buf[..n]).unwrap();
            }
        }
    }
}

fn sent_message(buf: FmtBuf) {
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

///传输信息结构体
#[derive(Serialize, Deserialize, Debug)]
struct Angle {
    angle: u16,
    //1:lift 0:right
    dir: bool,
    time: f64,
}
impl Angle {
    const fn new(angle: u16, dir: bool) -> Self {
        Self {
            angle,
            dir,
            time: 0.0,
        }
    }
}

///编码器结构体
struct As5600 {
    addr: u8,
    raw_angle_high: u8,
    raw_angle_low: u8,
    i2c: hal::I2C<I2C0, (Pin<Gpio16, Function<I2C>>, Pin<Gpio17, Function<I2C>>)>,
}
impl As5600 {
    fn new(i2c: hal::I2C<I2C0, (Pin<Gpio16, Function<I2C>>, Pin<Gpio17, Function<I2C>>)>) -> Self {
        Self {
            addr: 0x36,
            raw_angle_high: 0x0c,
            raw_angle_low: 0x0d,
            i2c,
        }
    }

    ///return: 编码器12位bit数值
    fn get_bit(&mut self) -> u16 {
        let mut buff_l = [0; 1];
        let mut buff_h = [0; 1];
        self.i2c
            .write_read(self.addr, &[self.raw_angle_low], &mut buff_l)
            .unwrap();
        self.i2c
            .write_read(self.addr, &[self.raw_angle_high], &mut buff_h)
            .unwrap();
        u16::from_be_bytes([buff_h[0], buff_l[0]])
    }

    ///return: 当前绝对角度
    fn get_angle(&mut self) -> f64 {
        let u = self.get_bit();
        360.0 / 4096.0 * u as f64
    }
}

///驱动器结构体
struct Pulse {
    ena: Pin<Gpio22, Output<PushPull>>,
    dir: Pin<Gpio21, Output<PushPull>>,
    pul: Pin<Gpio20, Output<PushPull>>,
}

impl Pulse {
    fn new(
        ena: Pin<Gpio22, Output<PushPull>>,
        dir: Pin<Gpio21, Output<PushPull>>,
        pul: Pin<Gpio20, Output<PushPull>>,
    ) -> Self {
        Self { ena, dir, pul }
    }

    fn sent_pulse(&mut self, delay: &mut Delay) {
        self.pul.set_high().unwrap();
        delay.delay_us(1);
        self.pul.set_low().unwrap();
    }

    //1: 0->360 2:360->0
    fn change_dir(&mut self, dir: usize) {
        if dir == 0 {
            self.dir.set_low().unwrap();
        } else {
            self.dir.set_high().unwrap();
        }
    }
}

#[derive(Debug)]
struct RPoint {
    angle: f64,
    speed: f64,
    a: f64,
    time: f64,
}
impl RPoint {
    fn new(angle: f64, speed: f64, a: f64, time: f64) -> Self {
        Self {
            angle,
            speed,
            a,
            time,
        }
    }

    fn start(&self, now_angle: f64, bit_angle: f64, pulse: &mut Pulse, delay: &mut Delay) -> i32 {
        let n = ((self.angle - now_angle) / bit_angle) as i32;
        let speed = (self.time * 1000000.0 / n as f64) as u32;
        for i in 0..n {
            pulse.sent_pulse(delay);
            delay.delay_us(speed);
        }

        n
    }
}

//运动控制
struct MoveMent {
    angle: f64,
    bit_angle: f64,
    as5600: As5600,
    r_path: Vec<RPoint, 10>,
    capacity: usize,
    now_capacity: usize,
}
impl MoveMent {
    fn new(mut as5600: As5600) -> Self {
        let now_angle = as5600.get_angle();
        Self {
            angle: now_angle,
            bit_angle: 0.0434363,
            as5600,
            r_path: Vec::new(),
            capacity: 10,
            now_capacity: 0,
        }
    }

    //添加路径
    fn add_point(&mut self, point: RPoint) -> Result<(), ()> {
        if self.now_capacity < self.capacity {
            self.r_path.push(point).unwrap();
            self.now_capacity += 1;
            Ok(())
        } else {
            Err(())
        }
    }

    fn clean_path(&mut self) {
        self.r_path = Vec::new();
        self.now_capacity = 0;
    }

    fn start(&mut self, pulse: &mut Pulse, delay: &mut Delay) -> Result<(), ()> {
        Ok(())
    }
}
