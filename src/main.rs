#![no_std]
#![no_main]

// if panic, go into infinite loop
use panic_halt as _;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app {
    use rp_pico::{
        hal::{self, clocks::init_clocks_and_plls, watchdog::Watchdog},
        XOSC_CRYSTAL_FREQ,
    };
    use usb_device::device::UsbDevice;
    use usb_device::{class_prelude::*, prelude::*};
    use usbd_hid::descriptor::generator_prelude::*;
    use usbd_hid::{descriptor::KeyboardReport, hid_class::HIDClass};
    // use usbd_hid::descriptor::generator_prelude::*;
    // use usbd_hid::descriptor::KeyboardReport;
    // use usbd_hid::descriptor::MouseReport;

    #[shared]
    struct Shared {
        // timer: hal::Timer,
        usb_dev: UsbDevice<'static, hal::usb::UsbBus>,
        usb_hid: HIDClass<'static, hal::usb::UsbBus>,
    }

    #[local]
    struct Local {}

    #[init(local = [ usb_bus: Option<UsbBusAllocator<hal::usb::UsbBus>> = None,])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut resets = cx.device.RESETS;
        let mut watchdog = Watchdog::new(cx.device.WATCHDOG);
        let clocks_manager = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            cx.device.XOSC,
            cx.device.CLOCKS,
            cx.device.PLL_SYS,
            cx.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let og_usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
            cx.device.USBCTRL_REGS,
            cx.device.USBCTRL_DPRAM,
            clocks_manager.usb_clock,
            true,
            &mut resets,
        ));

        // fucky static borrowing?
        *cx.local.usb_bus = Some(og_usb_bus);
        let usb_bus = cx.local.usb_bus.as_ref().unwrap();
        let usb_hid = HIDClass::new(&usb_bus, KeyboardReport::desc(), 10);
        let usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("TTY Keyboards")
            .product("Keebchain")
            .serial_number("1")
            .device_class(0)
            .build();

        // let mut timer = hal::Timer::new(cx.device.TIMER, &mut resets);

        (
            Shared {
                // timer,
                usb_dev,
                usb_hid,
            },
            Local {},
            init::Monotonics(),
        )
    }
}
