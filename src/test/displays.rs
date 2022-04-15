#![no_std]
#![no_main]

use core::fmt::Write;
use cortex_m_rt::entry;
use embedded_hal::timer::CountDown;
use embedded_time::duration::*;
use embedded_time::rate::Extensions;

use embedded_graphics::{
    mono_font::{ascii::FONT_6X13_ITALIC, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Line, PrimitiveStyle},
    text::{Baseline, Text},
};
use micromath::F32Ext;
use panic_halt as _;
use rp_pico::hal;
use rp_pico::hal::pac;
use ssd1306::{prelude::*, Ssd1306};

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

    let sio = hal::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
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
    let interface = ssd1306::I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X13_ITALIC)
        .text_color(BinaryColor::On)
        .build();

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut delay = timer.count_down();

    let mut buf = FmtBuf::new();

    let mut x1 = 50.;
    let mut y1 = 50.;
    let mut x2 = 32.;
    let mut y2 = 32.;
    let fi: f32 = 3.14 / 180.0;
    let fi_cos = fi.cos();
    let fi_sin = fi.sin();

    loop {
        display.clear();

        x1 = (x1 - x2) * fi_cos - (y1 - y2) * fi_sin + x2;
        y1 = (x1 - x2) * fi_sin + (y1 - y2) * fi_cos + y2;

        let p1 = Point::new(x1 as i32, y1 as i32);
        // p1.x
        let p2 = Point::new(x2 as i32, x2 as i32);
        Line::new(p1, p2)
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(&mut display)
            .unwrap();

        display.flush().unwrap();
        delay.start(10.milliseconds());
        let _ = nb::block!(delay.wait());
    }
}

fn rotation(p: &mut Point, coordinate: &str) {}

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
