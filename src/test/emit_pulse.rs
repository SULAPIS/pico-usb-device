//! # Pico PWM Blink Example
//!
//! Fades the LED on a Pico board using the PWM peripheral.
//!
//! This will fade in/out the LED attached to GP25, which is the pin the Pico
//! uses for the on-board LED.
//!
//! See the `Cargo.toml` file for Copyright and licence details.

#![no_std]
#![no_main]

use cortex_m::delay::Delay;
// The macro for our start-up function
use cortex_m_rt::entry;

// GPIO traits
use embedded_hal::PwmPin;

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;
// Time handling traits
use embedded_time::rate::*;

use hal::gpio::bank0::Gpio20;
use hal::gpio::*;
// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

// The minimum PWM value (i.e. LED brightness) we want
const LOW: u16 = 0;

// The maximum PWM value (i.e. LED brightness) we want
const HIGH: u16 = 25000;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then fades the LED in an
/// infinite loop.
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

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let mut dir_pin = pins.gpio21.into_push_pull_output();
    dir_pin.set_low().unwrap();
    let mut pul_pin = pins.gpio20.into_push_pull_output();
    dir_pin.set_high().unwrap();
    // Init PWMs
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM4
    let pwm = &mut pwm_slices.pwm3;
    pwm.set_ph_correct();
    pwm.enable();

    // Output channel B on PWM4 to the LED pin
    let channel = &mut pwm.channel_a;
    channel.output_to(pins.gpio22);

    let mut timer = 80;
    let mut counter = 0;
    // Infinite loop, fading LED up and down
    let mut angle = 10;
    let mut now_angle = 0;
    let emit_pulse = |pul_pin: &mut Pin<Gpio20, Output<PushPull>>, delay: &mut Delay| {
        pul_pin.set_high().unwrap();
        delay.delay_us(120);
        pul_pin.set_low().unwrap();
    };
    loop {
        // counter += 1;
        // if counter == 800 {
        //     counter = 0;
        //     delay.delay_ms(1000);
        // }
        let num = calc_emit_number(angle);
        for _ in 0..num {
            emit_pulse(&mut pul_pin, &mut delay);
            delay.delay_us(1);
        }
        counter += 1;
        if counter == 36 {
            counter = 0;
            delay.delay_ms(1000);
        }

        // dir_pin.toggle().unwrap();
    }
}
fn calc_emit_number(angle: i32) -> u32 {
    (angle as f32 / 0.45) as u32
}
// End of file
