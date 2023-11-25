#![no_std]
#![no_main]

use defmt as _;
use defmt_rtt as _;
use embedded_hal::{blocking::delay::DelayMs, digital::v2::OutputPin};
use panic_probe as _;
use rp2040_hal::{dma::DMAExt, pac, pio::PIOExt};
use usbh::UsbHost;
use usbh_defmt::UsbhDefmt;
use usbh_pio::{PioUsbConfiguration, UsbhPio};

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = rp2040_hal::Watchdog::new(pac.WATCHDOG);
    let clocks = rp2040_hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut timer = rp2040_hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let sio = rp2040_hal::Sio::new(pac.SIO);
    let pins = rp2040_hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let pin_dp = pins.gpio16.into_push_pull_output();
    let pin_dm = pins.gpio17.into_push_pull_output();

    let tx_ch = pac.DMA.split(&mut pac.RESETS).ch9;
    let (mut pio_tx, _, _, _, sm_tx) = pac.PIO0.split(&mut pac.RESETS);
    let (mut pio_rx, _, _, sm_rx, sm_eop) = pac.PIO1.split(&mut pac.RESETS);

    let config = PioUsbConfiguration {
        skip_alarm_pool: false,
        pin_dp,
        pin_dm,
        pio_rx,
        pio_tx,
        sm_eop,
        sm_tx,
        sm_rx,
        tx_ch,
    };
    usbh_pio::pio_usb::pio_usb_bus_init(config);
    let usb_host = UsbHost::new(UsbhDefmt::new(UsbhPio));
    let mut led_pin = pins.gpio25.into_push_pull_output();
    loop {
        led_pin.set_high().unwrap();
        timer.delay_ms(500);
        led_pin.set_low().unwrap();
        timer.delay_ms(500);
    }
}
