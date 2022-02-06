// //! # Pico WS2812 RGB LED Example
// //!
// //! Drives 3 WS2812 LEDs connected directly to the Raspberry Pi Pico.
// //! This assumes you drive the Raspberry Pi Pico via USB power, so that VBUS
// //! delivers the 5V and at least enough amperes to drive the LEDs.
// //!
// //! For a more large scale and longer strips you should use an extra power
// //! supply for the LED strip (or know what you are doing ;-) ).
// //!
// //! The example also comes with an utility function to calculate the colors
// //! from HSV color space. It also limits the brightness a bit to save a
// //! few milliamperes - be careful if you increase the strip length you will
// //! quickly get into power consumption of multiple amperes.
// //!
// //! The example assumes you connected the data input to pin 6 of the
// //! Raspberry Pi Pico, which is GPIO4 of the rp2040. Here is a circuit
// //! diagram that shows the assumed setup:
// //!
// //! ```text
// //!  _______________      /----------------------\
// //! |+5V  /---\  +5V|----/         _|USB|_       |
// //! |DO <-|LED|<- DI|-\           |1  R 40|-VBUS-/
// //! |GND  \---/  GND|--+---\      |2  P 39|
// //!  """""""""""""""   |    \-GND-|3    38|
// //!                    |          |4  P 37|
// //!                    |          |5  I 36|
// //!                    \------GP4-|6  C   |
// //!                               |7  O   |
// //!                               |       |
// //!                               .........
// //!                               |20   21|
// //!                                """""""
// //! Symbols:
// //!     - (+) crossing lines, not connected
// //!     - (o) connected lines
// //! ```
// //!
// //! See the `Cargo.toml` file for Copyright and licence details.

// #![no_std]
// #![no_main]

// // The macro for our start-up function
// use cortex_m_rt::entry;

// // Ensure we halt the program on panic (if we don't mention this crate it won't
// // be linked)
// use panic_halt as _;

// // Pull in any important traits
// use rp_pico::hal::prelude::*;

// // Embed the `Hz` function/trait:
// use embedded_time::rate::*;

// // A shorter alias for the Peripheral Access Crate, which provides low-level
// // register access
// use rp_pico::hal::pac;

// // Import the Timer for Ws2812:
// use rp_pico::hal::timer::Timer;

// // A shorter alias for the Hardware Abstraction Layer, which provides
// // higher-level drivers.
// use rp_pico::hal;

// // PIOExt for the split() method that is needed to bring
// // PIO0 into useable form for Ws2812:
// use rp_pico::hal::pio::PIOExt;

// // Import useful traits to handle the ws2812 LEDs:
// use smart_leds::{brightness, SmartLedsWrite, RGB8};

// // Import the actual crate to handle the Ws2812 protocol:
// use ws2812_pio::Ws2812;

// // Currently 3 consecutive LEDs are driven by this example
// // to keep the power draw compatible with USB:
// const STRIP_LEN: usize = 3;

// #[entry]
// fn main() -> ! {
//     // Grab our singleton objects
//     let mut pac = pac::Peripherals::take().unwrap();
//     let core = pac::CorePeripherals::take().unwrap();

//     // Set up the watchdog driver - needed by the clock setup code
//     let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

//     // Configure the clocks
//     //
//     // The default is to generate a 125 MHz system clock
//     let clocks = hal::clocks::init_clocks_and_plls(
//         rp_pico::XOSC_CRYSTAL_FREQ,
//         pac.XOSC,
//         pac.CLOCKS,
//         pac.PLL_SYS,
//         pac.PLL_USB,
//         &mut pac.RESETS,
//         &mut watchdog,
//     )
//     .ok()
//     .unwrap();

//     // The single-cycle I/O block controls our GPIO pins
//     let sio = hal::Sio::new(pac.SIO);

//     // Set the pins up according to their function on this particular board
//     let pins = rp_pico::Pins::new(
//         pac.IO_BANK0,
//         pac.PADS_BANK0,
//         sio.gpio_bank0,
//         &mut pac.RESETS,
//     );

//     // Setup a delay for the LED blink signals:
//     let mut frame_delay =
//         cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

//     // Import the `sin` function for a smooth hue animation from the
//     // Pico rp2040 ROM:
//     let sin = rp_pico::hal::rom_data::float_funcs::fsin();

//     // Create a count down timer for the Ws2812 instance:
//     let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

//     // Split the PIO state machine 0 into individual objects, so that
//     // Ws2812 can use it:
//     let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

//     // Instanciate a Ws2812 LED strip:
//     let mut ws = Ws2812::new(
//         // Use pin 6 on the Raspberry Pi Pico (which is GPIO4 of the rp2040 chip)
//         // for the LED data output:
//         pins.gpio16.into_mode(),
//         &mut pio,
//         sm0,
//         clocks.peripheral_clock.freq(),
//         timer.count_down(),
//     );

//     let mut leds: [RGB8; STRIP_LEN] = [(0, 0, 0).into(); STRIP_LEN];
//     let mut t = 0.0;

//     // Bring down the overall brightness of the strip to not blow
//     // the USB power supply: every LED draws ~60mA, RGB means 3 LEDs per
//     // ws2812 LED, for 3 LEDs that would be: 3 * 3 * 60mA, which is
//     // already 540mA for just 3 white LEDs!
//     let strip_brightness = 64u8; // Limit brightness to 64/256

//     // Slow down timer by this factor (0.1 will result in 10 seconds):
//     let animation_speed = 0.1;

//     loop {
//         for (i, led) in leds.iter_mut().enumerate() {
//             // An offset to give 3 consecutive LEDs a different color:
//             let hue_offs = match i % 3 {
//                 1 => 0.25,
//                 2 => 0.5,
//                 _ => 0.0,
//             };

//             let sin_11 = sin((t + hue_offs) * 2.0 * core::f32::consts::PI);
//             // Bring -1..1 sine range to 0..1 range:
//             let sin_01 = (sin_11 + 1.0) * 0.5;

//             let hue = 360.0 * sin_01;
//             let sat = 1.0;
//             let val = 1.0;

//             let rgb = hsv2rgb_u8(hue, sat, val);
//             *led = rgb.into();
//         }

//         // Here the magic happens and the `leds` buffer is written to the
//         // ws2812 LEDs:
//         ws.write(brightness(leds.iter().copied(), strip_brightness))
//             .unwrap();

//         // Wait a bit until calculating the next frame:
//         frame_delay.delay_ms(16); // ~60 FPS

//         // Increase the time counter variable and make sure it
//         // stays inbetween 0.0 to 1.0 range:
//         t += (16.0 / 1000.0) * animation_speed;
//         while t > 1.0 {
//             t -= 1.0;
//         }
//     }
// }

// pub fn hsv2rgb(hue: f32, sat: f32, val: f32) -> (f32, f32, f32) {
//     let c = val * sat;
//     let v = (hue / 60.0) % 2.0 - 1.0;
//     let v = if v < 0.0 { -v } else { v };
//     let x = c * (1.0 - v);
//     let m = val - c;
//     let (r, g, b) = if hue < 60.0 {
//         (c, x, 0.0)
//     } else if hue < 120.0 {
//         (x, c, 0.0)
//     } else if hue < 180.0 {
//         (0.0, c, x)
//     } else if hue < 240.0 {
//         (0.0, x, c)
//     } else if hue < 300.0 {
//         (x, 0.0, c)
//     } else {
//         (c, 0.0, x)
//     };
//     (r + m, g + m, b + m)
// }

// pub fn hsv2rgb_u8(h: f32, s: f32, v: f32) -> (u8, u8, u8) {
//     let r = hsv2rgb(h, s, v);

//     (
//         (r.0 * 255.0) as u8,
//         (r.1 * 255.0) as u8,
//         (r.2 * 255.0) as u8,
//     )
// }
//! # Pico USB 'Twitchy' Mouse Example
//!
//! Creates a USB HID Class Poiting device (i.e. a virtual mouse) on a Pico
//! board, with the USB driver running in the main thread.
//!
//! It generates movement reports which will twitch the cursor up and down by a
//! few pixels, several times a second.
//!
//! See the `Cargo.toml` file for Copyright and licence details.
//!
//! This is a port of
//! https://github.com/atsamd-rs/atsamd/blob/master/boards/itsybitsy_m0/examples/twitching_usb_mouse.rs

#![no_std]
#![no_main]

// The macro for our start-up function
use cortex_m_rt::entry;

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

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Human Interface Device (HID) Class support
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::descriptor::KeyboardReport;
use usbd_hid::descriptor::MouseReport;
use usbd_hid::hid_class::HIDClass;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;

// define my shit
const MOUSE_LEFT: u8 = 1;
const MOUSE_RIGHT: u8 = 2;
const MOUSE_MIDDLE: u8 = 3;

static SHIT: &[u8] = &[
    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x06, // Usage (Keyboard)
    0xa1, 0x01, // Collection (Application)
    0x05, 0x07, // Usage Page (Key Codes)
    0x19, 0xe0, // Usage Minimum (224)
    0x29, 0xe7, // Usage Maximum (231)
    0x15, 0x00, // Logical Minimum (0)
    0x25, 0x01, // Logical Maximum (1)
    0x75, 0x01, // Report Size (1)
    0x95, 0x08, // Report count (8)
    0x81, 0x02, // Input (Data, Variable, Absolute)
    0x19, 0x00, // Usage Minimum (0)
    0x29, 0xFF, // Usage Maximum (255)
    0x26, 0xFF, 0x00, // Logical Maximum (255)
    0x75, 0x08, // Report Size (8)
    0x95, 0x01, // Report Count (1)
    0x81, 0x03, // Input (Const, Variable, Absolute)
    0x05, 0x08, // Usage Page (LEDs)
    0x19, 0x01, // Usage Minimum (1)
    0x29, 0x05, // Usage Maximum (5)
    0x25, 0x01, // Logical Maximum (1)
    0x75, 0x01, // Report Size (1)
    0x95, 0x05, // Report Count (5)
    0x91, 0x02, // Output (Data, Variable, Absolute)
    0x95, 0x03, // Report Count (3)
    0x91, 0x03, // Output (Constant, Variable, Absolute)
    0x05, 0x07, // Usage Page (Key Codes)
    0x19, 0x00, // Usage Minimum (0)
    0x29, 0xDD, // Usage Maximum (221)
    0x26, 0xFF, 0x00, // Logical Maximum (255)
    0x75, 0x08, // Report Size (8)
    0x95, 0x06, // Report Count (6)
    0x81, 0x00, // Input (Data, Array, Absolute)
    0xc0, // End Collection
];

static CUSTOM_KEEB_REPORT: [u8; 21] = [
    0x6, 0x0, 0xff, // USEGE PAGE Vendor
    0x9, 0x1, // USAGE Vendor
    0xa1, 0x1, // Collection (Application)
    0x15, 0x0, // Logical minimum
    0x26, 0xff, 0x0, // Logical maximum
    0x75, 0x8, // Report size 8
    0x95, 0x1, // Report count 1
    0x09, 0x01, // USAGE Vendor <---- manually added here
    0x81, 0x0,  // Input
    0xc0, // End application collection
];

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
    // let usb_hid = HIDClass::new(bus_ref, &CUSTOM_KEEB_REPORT, 10);
    let usb_hid = HIDClass::new(bus_ref, KeyboardReport::desc(), 10);
    // let usb_hid = HIDClass::new(bus_ref, MouseReport::desc(), 60);
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet.
        USB_HID = Some(usb_hid);
    }

    // Create a USB device with a fake VID and PID
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27dd))
        // let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27da))
        .manufacturer("Test")
        .product("Test Keeb")
        .serial_number("TEST")
        // .device_class(0xEF) // misc
        .device_class(0)
        //.device_protocol(01)
        // .composite_with_iads()
        .build();
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    unsafe {
        // Enable the USB interrupt
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };
    let core = pac::CorePeripherals::take().unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // Move the cursor up and down every 200ms
    loop {
        // delay.delay_ms(100);

        // let click_down_report = MouseReport {
        //     x: 0,
        //     y: 4,
        //     buttons: 0,
        //     wheel: 0,
        //     pan: 0,
        // };
        // push_mouse_movement(click_down_report).ok().unwrap_or(0);

        // delay.delay_ms(25);

        // let click_up_report = MouseReport {
        //     x: 0,
        //     y: -4,
        //     buttons: 0,
        //     wheel: 0,
        //     pan: 0,
        // };
        // push_mouse_movement(click_up_report).ok().unwrap_or(0);

        let key_down_report = KeyboardReport {
            modifier: 0,
            reserved: 0,
            leds: 0,
            keycodes: [9, 0, 0, 0, 0, 0],
        };
        push_key_report(key_down_report).ok().unwrap_or(0);
        let key_up_report = KeyboardReport {
            modifier: 0,
            reserved: 0,
            leds: 0,
            keycodes: [0, 0, 0, 0, 0, 0],
        };
        push_key_report(key_up_report).ok().unwrap_or(0);
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

fn push_key_report(report: KeyboardReport) -> Result<usize, usb_device::UsbError> {
    cortex_m::interrupt::free(|_| unsafe { USB_HID.as_mut().map(|hid| hid.push_input(&report)) })
        .unwrap()
}

/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    // Handle USB request
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let usb_hid = USB_HID.as_mut().unwrap();
    usb_dev.poll(&mut [usb_hid]);
}
