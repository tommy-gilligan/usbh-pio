use crate::pio_usb_configuration::PioUsbConfiguration;
use crate::usb_definitions::RootPort;
use pio_usb_ll::{PioPort, IRQ_TX_COMP_MASK};
use crate::usb_definitions::{Endpoint, USB_PID_IN, USB_PID_OUT, USB_PID_ACK, USB_PID_DATA0, USB_PID_DATA1};

use rp2040_hal::gpio::{Function, PinId, PullType};

use crate::pio_usb_ll;

pub fn pio_usb_host_init<P, F, DP, DM, PioRx, PioTx>(
    _pp: &mut pio_usb_ll::PioPort<PioRx, PioTx>,
    _c: &PioUsbConfiguration<P, DP, DM, F, PioRx, PioTx>,
    root: &mut RootPort<P, F, DP, DM>,
) where
    P: PullType,
    DP: PinId,
    DM: PinId,
    F: Function,
    PioRx: rp2040_hal::pio::PIOExt,
    PioTx: rp2040_hal::pio::PIOExt,
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

    // #[inline]
    // fn _find_ep(root_idx: u8, device_address: u8, ep_address: u8) -> Endpoint {
    //     for ep_pool_idx in 0..crate::pio_usb_configuration::PIO_USB_EP_POOL_CNT {
    //         endpoint_t *ep = PIO_USB_ENDPOINT(ep_pool_idx);

    //         // note 0x00 and 0x80 are matched as control endpoint of opposite direction
    //         if (ep.root_idx == root_idx) && (ep.dev_addr == device_address) && ep.size && ((ep.ep_num == ep_address) || (((ep_address & 0x7f) == 0) && ((ep.ep_num & 0x7f) == 0))) {
    //           return ep;
    //         }
    //     }

    //     return NULL;
    // }

    // fn pio_usb_host_send_setup(uint8_t root_idx, uint8_t device_address, uint8_t const setup_packet[8]) -> bool {
    //   endpoint_t *ep = _find_ep(root_idx, device_address, 0);

    //   ep.ep_num = 0; // setup is is OUT
    //   ep.data_id = USB_PID_SETUP;
    //   ep.is_tx = true;

    //   return pio_usb_ll_transfer_start(ep, (uint8_t *)setup_packet, 8);
    // }

    // fn pio_usb_host_endpoint_transfer(uint8_t root_idx, uint8_t device_address, uint8_t ep_address, uint8_t *buffer, uint16_t buflen) -> bool {
    //   endpoint_t *ep = _find_ep(root_idx, device_address, ep_address);
    //   // Control endpoint, address may switch between 0x00 <-> 0x80
    //   // therefore we need to update ep_num and is_tx
    //   if (ep_address & 0x7f) == 0 {
    //     ep.ep_num = ep_address;
    //     ep.is_tx = ep_address == 0;
    //     ep.data_id = 1; // data and status always start with DATA1
    //   }

    //   return pio_usb_ll_transfer_start(ep, buffer, buflen);
    // }

    fn usb_in_transaction<PioRx, PioTx, P, F, DP, DM>(
        pp: &mut PioPort<PioRx, PioTx>,
        ep: &mut Endpoint,
        root: &RootPort<P, F, DP, DM>,
    ) where PioRx: rp2040_hal::pio::PIOExt, PioTx: rp2040_hal::pio::PIOExt, DP: PinId, DM: PinId, F: Function, P: PullType {
      let expect_pid: u8 = if ep.data_id.get() == 1 {
          USB_PID_DATA1
      } else {
          USB_PID_DATA0
      };

      crate::pio_usb::pio_usb_bus_prepare_receive(pp);
      crate::pio_usb::pio_usb_bus_send_token(
          pp,
          USB_PID_IN,
          ep.dev_addr,
          ep.ep_num
      );
      crate::pio_usb::pio_usb_bus_start_receive(pp);

      let receive_len: usize = 0; // pio_usb_bus_receive_packet_and_handshake(pp, USB_PID_ACK);
      let receive_pid: u8 = pp.rx_buffer[1];

      if receive_len >= 0 && receive_pid == expect_pid {
        // memcpy(ep.app_buf, &pp.usb_rx_buffer[2], receive_len);
        crate::pio_usb::pio_usb_ll_transfer_continue(ep, receive_len, root);
      }

      // pio_sm_set_enabled(pp->pio_rx, pp->sm_rx, false);
      pp.rx_buffer[0] = 0;
      pp.rx_buffer[1] = 0;
    }

    fn usb_out_transaction<PioRx, PioTx, P, F, DP, DM>(
        pp: &mut PioPort<PioRx, PioTx>,
        ep: &mut Endpoint,
        root: &RootPort<P, F, DP, DM>,
    ) where PioRx: rp2040_hal::pio::PIOExt, PioTx: rp2040_hal::pio::PIOExt, DP: PinId, DM: PinId, F: Function, P: PullType {
        let res: u32 = 0;

        let xact_len: usize = pio_usb_ll::pio_usb_ll_get_transaction_len(ep);

        crate::pio_usb::pio_usb_bus_prepare_receive(pp);
        crate::pio_usb::pio_usb_bus_send_token(
            pp,
            USB_PID_OUT,
            ep.dev_addr,
            ep.ep_num
        );
        // ensure previous tx complete
        while (pp.pio_tx.get_irq_raw() & IRQ_TX_COMP_MASK) == 0 {
          continue;
        }

        crate::pio_usb::pio_usb_bus_usb_transfer(pp, &ep.buffer, xact_len + 4);
        crate::pio_usb::pio_usb_bus_start_receive(pp);

        // crate::pio_usb::pio_usb_bus_wait_handshake(pp);

        let receive_token: u8 = pp.rx_buffer[1];

        if receive_token == USB_PID_ACK {
            crate::pio_usb::pio_usb_ll_transfer_continue(ep, xact_len, root);
        }

        // pio_sm_set_enabled(pp.pio_rx, pp.sm_rx, false);
        pp.rx_buffer[0] = 0;
        pp.rx_buffer[1] = 0;
    }
}
