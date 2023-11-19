use crate::pio_usb_configuration::PioUsbConfiguration;
use crate::usb_definitions::RootPort;

use rp_pico::hal::{
    gpio::{Function, PinId, PullType},
    pio::ValidStateMachine,
};

use crate::{pio_usb::pio_usb_bus_init, pio_usb_ll};

pub fn pio_usb_host_init<T, A, B, C, P, F, DPDM, DMDP>(
    pp: &mut pio_usb_ll::PioPort<T, A, B, C>,
    c: &PioUsbConfiguration<A, B, C>,
    root: &mut RootPort<P, F, DPDM, DMDP>,
) where
    A: ValidStateMachine,
    B: ValidStateMachine,
    C: ValidStateMachine,
    P: PullType,
    DPDM: PinId,
    DMDP: PinId,
    F: Function,
{
    pio_usb_bus_init(pp, c, root);
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
