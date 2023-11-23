use crate::pio_usb_configuration::PioUsbConfiguration;
use crate::usb_definitions::RootPort;

use rp_pico::hal::gpio::{Function, PinId, PullType};

use crate::pio_usb_ll;

pub fn pio_usb_host_init<P, F, DP, DM, PIO_RX, PIO_TX>(
    _pp: &mut pio_usb_ll::PioPort<PIO_RX, PIO_TX>,
    _c: &PioUsbConfiguration<P, DP, DM, F, PIO_RX, PIO_TX>,
    root: &mut RootPort<P, F, DP, DM>,
) where
    P: PullType,
    DP: PinId,
    DM: PinId,
    F: Function,
    PIO_RX: rp_pico::hal::pio::PIOExt,
    PIO_TX: rp_pico::hal::pio::PIOExt,
{
    // pio_usb_bus_init(pp, c, root);
    root.mode = pio_usb_ll::PIO_USB_MODE_HOST;

    // float const cpu_freq = (float)clock_get_hz(clk_sys);
    // pio_calculate_clkdiv_from_float(cpu_freq / 48000000,
    //                                 &pp->clk_div_fs_tx.div_int,
    //                                 &pp->clk_div_fs_tx.div_frac);
    // pio_calculate_clkdiv_from_float(cpu_freq / 6000000,
    //                                 &pp->clk_div_ls_tx.div_int,
    //                                 &pp->clk_div_ls_tx.div_frac);

    // pio_calculate_clkdiv_from_float(cpu_freq / 96000000,
    //                                 &pp->clk_div_fs_rx.div_int,
    //                                 &pp->clk_div_fs_rx.div_frac);
    // pio_calculate_clkdiv_from_float(cpu_freq / 12000000,
    //                                 &pp->clk_div_ls_rx.div_int,
    //                                 &pp->clk_div_ls_rx.div_frac);

    // if (!c->skip_alarm_pool) {
    //   _alarm_pool = c->alarm_pool;
    //   if (!_alarm_pool) {
    //     _alarm_pool = alarm_pool_create(2, 1);
    //   }
    // }
    // start_timer(_alarm_pool);

    // return &pio_usb_device[0];
}
