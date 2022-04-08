#![no_std]
#![no_main]
use cortex_m_rt::entry;

use embedded_hal::digital::v2::{InputPin, ToggleableOutputPin};
use panic_halt as _;
use rp_pico::hal;
use rp_pico::hal::pac;
use rp_pico::hal::pac::interrupt;

use core::borrow::Borrow;
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

use rp2040_hal::gpio;
use rp2040_hal::gpio::Interrupt::EdgeLow;
type LedPin = gpio::Pin<gpio::bank0::Gpio25, gpio::PushPullOutput>;
type ButtonPin = gpio::Pin<gpio::bank0::Gpio26, gpio::PullUpInput>;
type LedAndButton = (LedPin, ButtonPin);
static GLOBAL_PINS: Mutex<RefCell<Option<LedAndButton>>> = Mutex::new(RefCell::new(None));
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

    let led = pins.led.into_mode();
    let in_pin = pins.gpio26.into_mode();

    in_pin.set_interrupt_enabled(EdgeLow, true);

    cortex_m::interrupt::free(|cs| GLOBAL_PINS.borrow(cs).replace(Some((led, in_pin))));

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }
    loop {
        cortex_m::asm::nop();
    }
}
#[interrupt]
fn IO_IRQ_BANK0() {
    static mut LED_AND_BUTTON: Option<LedAndButton> = None;
    if LED_AND_BUTTON.is_none() {
        cortex_m::interrupt::free(|cs| {
            *LED_AND_BUTTON = GLOBAL_PINS.borrow(cs).take();
        });
    }

    if let Some(gpios) = LED_AND_BUTTON {
        let (led, button) = gpios;
        for _ in 0..500 {
            cortex_m::asm::nop();
        }
        if button.is_low().unwrap() {
            let _ = led.toggle();
        }

        button.clear_interrupt(EdgeLow);
    }
}
