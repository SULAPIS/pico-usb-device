#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::adc::OneShot;
use embedded_time::rate::*;
use panic_halt as _;
use rp2040_hal::{adc::Adc, gpio::Pins, pac, Sio};
use rp_pico::hal;
use rp_pico::hal::prelude::*;

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

    let sio = hal::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    let mut adc_pin_0 = pins.gpio26.into_floating_input();
    let pin_adc_counts: u16 = adc.read(&mut adc_pin_0).unwrap();
    loop {}
}
