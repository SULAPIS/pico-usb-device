#![no_std]
#![no_main]
use core::fmt::Write;
use cortex_m_rt::entry;
use display_interface_spi::SPIInterfaceNoCS;
use embedded_graphics::mono_font::{ascii::FONT_9X18_BOLD, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::*;
use embedded_graphics::text::{Baseline, Text};
use embedded_hal::{adc::OneShot, digital::v2::ToggleableOutputPin};
use embedded_time::{fixed_point::FixedPoint, rate::Extensions};
use hal::{
    gpio::{
        self,
        bank0::{Gpio12, Gpio13, Gpio25, Gpio26},
        Floating, Input, Output, Pin, PushPull,
    },
    spi::Enabled,
    Adc, Clock, Spi,
};
use panic_halt as _;
use rp_pico::hal::pac;
use rp_pico::hal::pac::interrupt;
use rp_pico::hal::spi;
use rp_pico::{hal, pac::SPI1};
use serde::{Deserialize, Serialize};
use serde_json_core;
use st7789::{Orientation, ST7789};
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

#[derive(Serialize, Deserialize)]
struct Angle {
    x: f32,
    y: f32,
    z: f32,
}
impl Angle {
    fn new() -> Self {
        Angle {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }
}

static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<hal::usb::UsbBus>> = None;
static mut ADC_PIN: Option<Pin<Gpio26, Input<Floating>>> = None;
static mut LED_PIN: Option<Pin<Gpio25, Output<PushPull>>> = None;
static mut ADC: Option<Adc> = None;
static mut DISPLAY: Option<
    ST7789<
        SPIInterfaceNoCS<Spi<Enabled, SPI1, 8>, Pin<Gpio13, Output<PushPull>>>,
        Pin<Gpio12, Output<PushPull>>,
    >,
> = None;
static mut DISPLAY_COUNT: i32 = 0;

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
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());
    //adc
    let adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);

    unsafe { ADC = Some(adc) }

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

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
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27da))
        .product("Twitchy Mousey")
        .device_class(2)
        .build();
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    let adc_pin = pins.gpio26.into_floating_input();
    unsafe { ADC_PIN = Some(adc_pin) }
    let led_pin = pins.led.into_push_pull_output();
    unsafe { LED_PIN = Some(led_pin) }

    let _spi_sclk = pins.gpio10.into_mode::<gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio11.into_mode::<gpio::FunctionSpi>();
    let _spi_miso = pins.gpio8.into_mode::<gpio::FunctionSpi>();
    let disp_res = pins.gpio12.into_push_pull_output();
    let disp_dc = pins.gpio13.into_push_pull_output();
    let spi = spi::Spi::<_, _, 8>::new(pac.SPI1);
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_3,
    );
    let di = SPIInterfaceNoCS::new(spi, disp_dc);
    let mut display = ST7789::new(di, disp_res, 240, 240);

    display.init(&mut delay).unwrap();
    display.set_orientation(Orientation::Portrait).unwrap();
    display.clear(Rgb565::BLACK).unwrap();
    unsafe { DISPLAY = Some(display) }

    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    }

    loop {
        //   let ad: u16 = adc.read(&mut adc_pin).unwrap();
        //   angle_num = (-13.0 + ad as f32) / (4096.0 - 13.0) * 180.0;
        //   let mut angle_num = (angle_num * 10.0) as i32;
        //   if angle_num % 10 <= 4 {
        //       angle_num = angle_num / 10 * 10 + 5;
        //   } else {
        //       angle_num = angle_num / 10 * 10 + 10;
        //   }
        //   let angle_num = angle_num as f32 / 10.0;
    }
}

#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let serial = USB_SERIAL.as_mut().unwrap();
    let adc_pin = ADC_PIN.as_mut().unwrap();
    let led_pin = LED_PIN.as_mut().unwrap();
    let adc = ADC.as_mut().unwrap();

    if usb_dev.poll(&mut [serial]) {
        let mut buf = [0u8; 64];
        match serial.read(&mut buf) {
            Ok(0) => {}
            Err(_e) => {}
            Ok(_count) => {
                led_pin.toggle().unwrap();
                let ad: u16 = adc.read(adc_pin).unwrap();
                let angle_num = (-13.0 + ad as f32) / (4096.0 - 13.0) * 180.0;
                let mut angle_num = (angle_num * 10.0) as i32;
                if angle_num % 10 <= 4 {
                    angle_num = angle_num / 10 * 10 + 5;
                } else {
                    angle_num = angle_num / 10 * 10 + 10;
                }
                let angle_num = angle_num as f32 / 10.0;
                let mut angle = Angle::new();
                angle.x = angle_num;
                let serialized = serde_json_core::to_string::<Angle, 60>(&angle).unwrap();
                let _ = serial.write(serialized.as_bytes());
                DISPLAY_COUNT += 1;
                if DISPLAY_COUNT >= 5 {
                    DISPLAY_COUNT = 0;
                    display_something(&angle);
                }
            }
        }
    }
}

fn display_something(angle: &Angle) {
    unsafe {
        let display = DISPLAY.as_mut().unwrap();
        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_9X18_BOLD)
            .text_color(Rgb565::CYAN)
            .build();
        display
            .fill_solid(
                &Rectangle::new(
                    Point { x: 0, y: 0 },
                    Size {
                        width: 130,
                        height: 20,
                    },
                ),
                Rgb565::BLACK,
            )
            .unwrap();
        let mut buff = FmtBuf::new();
        buff.reset();
        write!(&mut buff, "angle: {}", angle.x).unwrap();
        Text::with_baseline(buff.as_str(), Point::zero(), text_style, Baseline::Top)
            .draw(display)
            .unwrap();
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
