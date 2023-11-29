use core::ffi::c_int;

use rp2040_hal::{
    dma::{Channel, CH9},
    gpio::{
        Function, InputOverride, OutputDriveStrength, OutputSlewRate, Pin, PinId, PullDown,
        PullType,
    },
    pio::{StateMachine, Stopped, UninitStateMachine, ValidStateMachine, PIO, SM3},
};

use crate::{pio_usb_configuration, pio_usb_ll, usb_definitions};

use pio_usb_configuration::PioUsbConfiguration;

use pio_usb_ll::{PioPort, IRQ_RX_COMP_MASK};
use usb_definitions::{Endpoint, EndpointDescriptor, RootPort, USB_PID_ACK};

//--------------------------------------------------------------------+
// Bus functions
//--------------------------------------------------------------------+

// pub fn send_pre<PioRx, PioTx>(
//     pp: &mut PioPort<PioRx, PioTx>,
// ) where PioRx: rp2040_hal::pio::PIOExt, PioTx: rp2040_hal::pio::PIOExt {
//     let data = [
//         usb_definitions::USB_SYNC,
//         usb_definitions::USB_PID_PRE,
//     ];
// 
//     // send PRE token in full-speed
//     pp.sm_tx.set_enabled(false);
//     for i in 0..USB_TX_EOP_DISABLER_LEN {
//       let instr: u16 = pp.fs_tx_pre_program.instructions[i + USB_TX_EOP_OFFSET];
//       pp.pio_tx.instr_mem[pp.offset_tx + i + USB_TX_EOP_OFFSET] = instr;
//     }
// 
//     SM_SET_CLKDIV(pp.pio_tx, pp.sm_tx, pp.clk_div_fs_tx);
// 
//     dma_channel_transfer_from_buffer_now(pp.tx_ch, data, 2);
// 
//     pp.sm_tx.set_enabled(true);
//     pp.pio_tx.irq |= pio_usb_ll::IRQ_TX_ALL_MASK;       // clear complete flag
//     pp.pio_tx.irq_force |= pio_usb_ll::IRQ_TX_EOP_MASK; // disable eop
// 
//     while (pp.pio_tx.get_irq_raw() & pio_usb_ll::IRQ_TX_COMP_MASK) == 0 {
//       continue;
//     }
// 
//     // change bus speed to low-speed
//     pp.sm_tx.set_enabled(false);
//     for i in 0..USB_TX_EOP_DISABLER_LEN {
//       let instr: u16 = pp.fs_tx_program.instructions[i + USB_TX_EOP_OFFSET];
//       pp.pio_tx.instr_mem[pp.offset_tx + i + USB_TX_EOP_OFFSET] = instr;
//     }
//     SM_SET_CLKDIV(pp.pio_tx, pp.sm_tx, pp.clk_div_ls_tx);
// 
//     pp.sm_rx.set_enabled(false);
//     SM_SET_CLKDIV_MAXSPEED(pp.pio_usb_rx, pp.sm_rx);
// 
//     pp.sm_eop.set_enabled(false);
//     SM_SET_CLKDIV(pp.pio_usb_rx, pp.sm_eop, pp.clk_div_ls_rx);
//     pp.sm_eop.set_enabled(true);
// }

pub fn pio_usb_bus_usb_transfer<PioRx, PioTx>(
    pp: &PioPort<PioRx, PioTx>,
    data: &[u8],
    len: usize,
) where PioRx: rp2040_hal::pio::PIOExt, PioTx: rp2040_hal::pio::PIOExt {
  if pp.need_pre {
      // send_pre(pp);
  }

  // dma_channel_transfer_from_buffer_now(pp.tx_ch, data, len);

  // pio_sm_set_enabled(pp.pio_usb_tx, pp.sm_tx, true);
  pp.pio_tx.clear_irq(pp.pio_tx.get_irq_raw() | pio_usb_ll::IRQ_TX_ALL_MASK); // clear complete flag

  while (pp.pio_tx.get_irq_raw() & pio_usb_ll::IRQ_TX_ALL_MASK) == 0 {
    continue;
  }
}

pub fn pio_usb_bus_send_handshake<PioRx, PioTx>(
    pp: &PioPort<PioRx, PioTx>,
    pid: u8
) where PioRx: rp2040_hal::pio::PIOExt, PioTx: rp2040_hal::pio::PIOExt {
  let data = [
      usb_definitions::USB_SYNC,
      pid,
  ];
  pio_usb_bus_usb_transfer(pp, &data, data.len());
}

pub fn pio_usb_bus_send_token<PioRx, PioTx>(
    pp: &PioPort<PioRx, PioTx>,
    token: u8,
    addr: u8,
    ep_num: u8
) where PioRx: rp2040_hal::pio::PIOExt, PioTx: rp2040_hal::pio::PIOExt {
  let mut packet: [u8; 4] = [usb_definitions::USB_SYNC, token, 0, 0];
  let dat: u16 = ((ep_num & 0xf) << 7) as u16 | (addr & 0x7f) as u16;
  let crc: u8 = usb_crc::calc_usb_crc5(dat);
  packet[2] = dat as u8;
  packet[3] = (crc << 3) | ((dat >> 8) as u8 & 0x1f);

  pio_usb_bus_usb_transfer(pp, &packet, packet.len());
}

pub fn pio_usb_bus_prepare_receive<PioRx, PioTx>(
    pp: &PioPort<PioRx, PioTx>,
) where PioRx: rp2040_hal::pio::PIOExt, PioTx: rp2040_hal::pio::PIOExt {
//   pio_sm_set_enabled(pp.pio_usb_rx, pp.sm_rx, false);
//     pp.sm_rx.clear_fifos();
//     sm_rx.restart();
//      sm_rx.set_instruction(pp.rx_reset_instr);
//     sm_rx.set_instruction(pp.rx_reset_instr2);
}

pub fn pio_usb_bus_start_receive<PioRx, PioTx>(
    pp: &mut PioPort<PioRx, PioTx>,
) where PioRx: rp2040_hal::pio::PIOExt, PioTx: rp2040_hal::pio::PIOExt  {
  // pp.pio_rx.ctrl |= 1 << pp.sm_rx;
  pp.pio_rx.force_irq(pio_usb_ll::IRQ_RX_ALL_MASK);
}

pub fn pio_usb_bus_wait_handshake<PioRx, PioTx, SM>(
    pp: &mut PioPort<PioRx, PioTx>,
    mut rx: rp2040_hal::pio::Rx<SM>
) -> u8 where PioRx: rp2040_hal::pio::PIOExt, PioTx: rp2040_hal::pio::PIOExt, SM: rp2040_hal::pio::ValidStateMachine {
  let mut t: u8 = 240;
  let mut idx: usize = 0;

  while t > 0 {
    t -= 1;
    if !rx.is_empty() {
        pp.rx_buffer[idx] = (rx.read().unwrap() >> 24) as u8;
        idx += 1;
        if idx == 2 {
            break;
        }
    }
  }

  if t > 0 {
    while (pp.pio_rx.get_irq_raw() & pio_usb_ll::IRQ_RX_COMP_MASK) == 0 {
      continue;
    }
  }

  // pio_sm_set_enabled(pp.pio_rx, pp.sm_rx, false);

  return pp.rx_buffer[1];
}

pub fn configure_tx_channel<P>(
    _ch: &mut Channel<CH9>,
    _pio: &mut PIO<P>,
    _sm: &mut UninitStateMachine<(P, SM3)>,
) where
    P: rp2040_hal::pio::PIOExt,
{
    //    let tx_transfer = rp_pico::hal::dma::single_buffer::Config::new(ch, tx_buf, tx).start();

    //     dma_channel_config conf = dma_channel_get_default_config(ch);
    //
    //     channel_config_set_read_increment(&conf, true);
    //     channel_config_set_write_increment(&conf, false);
    //     channel_config_set_transfer_data_size(&conf, DMA_SIZE_8);
    //     channel_config_set_dreq(&conf, pio_get_dreq(pio, sm, true));
    //
    //     dma_channel_set_config(ch, &conf, false);
    //     dma_channel_set_write_addr(ch, &pio->txf[sm], false);
    //    #[allow(clippy::unusual_byte_groupings)]
    //    let message = [
    //        0b10101010_00100010_11101010_00101110,
    //        0b10100011_10111011_10000000_10111011,
    //        0b10001110_11101110_00101110_10001011,
    //        0b10101000_11101010_00000000_00000000,
    //        0b10101010_00100010_11101010_00101110,
    //        0b10100011_10111011_10000000_10111011,
    //        0b10001110_11101110_00101110_10001011,
    //        0b10101000_11101010_00000000_00000000,
    //    ];
    //
    //    let tx_buf = singleton!(: [u32; 8] = message).unwrap();
    //
    //  let tx_transfer = rp_pico::hal::dma::single_buffer::Config::new(ch0, tx_buf, tx).start();
    //
    //  let (ch0, tx_buf, tx) = tx_transfer.wait();
    //  for i in 0..rx_buf.len() {
    //      if rx_buf[i] != tx_buf[i] {
    //          // The data did not match, abort.
    //          #[allow(clippy::empty_loop)]
    //          loop {}
    //      }
    //  }
}

pub fn pio_usb_bus_init<P, F, DP, DM, PioRx, PioTx>(
    c: PioUsbConfiguration<P, DP, DM, F, PioRx, PioTx>,
) where
    P: PullType,
    DP: PinId,
    DM: PinId,
    F: Function,
    PioRx: rp2040_hal::pio::PIOExt,
    PioTx: rp2040_hal::pio::PIOExt,
{
    let mut root = RootPort {
        initialized: vcell::VolatileCell::new(true),
        addr0_exists: vcell::VolatileCell::new(false),
        is_fullspeed: vcell::VolatileCell::new(false),
        connected: vcell::VolatileCell::new(false),
        suspended: vcell::VolatileCell::new(false),
        mode: pio_usb_ll::PIO_USB_MODE_HOST,
        pin_dp: c.pin_dp,
        pin_dm: c.pin_dm,
        dev_addr: 0,
        ep_complete: vcell::VolatileCell::new(0),
        ep_error: vcell::VolatileCell::new(0),
        ep_stalled: vcell::VolatileCell::new(0),
        ints: vcell::VolatileCell::new(0),
    };

    root.pin_dp.set_slew_rate(OutputSlewRate::Fast);
    root.pin_dm.set_slew_rate(OutputSlewRate::Fast);
    root.pin_dp
        .set_drive_strength(OutputDriveStrength::TwelveMilliAmps);
    root.pin_dm
        .set_drive_strength(OutputDriveStrength::TwelveMilliAmps);

    let mut pp = PioPort {
        sm_tx: c.sm_tx,
        sm_rx: c.sm_rx,
        sm_eop: c.sm_eop,
        pio_tx: c.pio_tx,
        tx_ch: c.tx_ch,
        pio_rx: c.pio_rx,
        rx_reset_instr2: (pio::InstructionOperands::SET {
            destination: pio::SetDestination::X,
            data: 0,
        })
        .encode(),
        rx_reset_instr: None,
        offset_tx: None,
        offset_rx: None,
        offset_eop: None,
        rx_buffer: [0; 128],
        need_pre: false,
    };

    let fs_tx_program = pio_proc::pio_file!(
        "src/usb_tx.pio",
        select_program("usb_tx_dpdm"),
        options(max_program_size = 32)
    );
    let _usb_tx_eop_disabler_len = fs_tx_program.public_defines.USB_TX_EOP_DISABLER_LEN;
    let _usb_tx_eop_offset = fs_tx_program.public_defines.USB_TX_EOP_OFFSET;

    let _fs_tx_pre_program = pio_proc::pio_file!(
        "src/usb_tx.pio",
        select_program("usb_tx_pre_dpdm"),
        options(max_program_size = 32)
    );
    let _ls_tx_program = pio_proc::pio_file!(
        "src/usb_tx.pio",
        select_program("usb_tx_dmdp"),
        options(max_program_size = 32)
    );

    let usb_nrzi_decoder_program = pio_proc::pio_file!(
        "src/usb_rx.pio",
        select_program("usb_nrzi_decoder"),
        options(max_program_size = 32)
    );

    let usb_edge_detector_program = pio_proc::pio_file!(
        "src/usb_rx.pio",
        select_program("usb_edge_detector"),
        options(max_program_size = 32)
    );

    pp.offset_tx = Some(pp.pio_tx.install(&fs_tx_program.program).unwrap());

    pp.offset_rx = Some(
        pp.pio_rx
            .install(&usb_nrzi_decoder_program.program)
            .unwrap(),
    );

    pp.rx_reset_instr = Some(
        (pio::InstructionOperands::JMP {
            condition: pio::JmpCondition::Always,
            address: pp.offset_rx.as_ref().unwrap().offset(),
        })
        .encode(),
    );

    pp.offset_eop = Some(
        pp.pio_rx
            .install(&usb_edge_detector_program.program)
            .unwrap(),
    );
}

//--------------------------------------------------------------------+
// Application API
//--------------------------------------------------------------------+

fn pio_usb_get_endpoint(
    device: &crate::usb_definitions::UsbDevice,
    idx: usize
) -> Option<&'static Endpoint> {
  let ep_id: u8 = device.endpoint_id[idx];

  // if ep_id >= 1 {
  //   Some(&pio_usb_ep_pool[ep_id - 1])
  // } else {
    None
  // }
}

fn pio_usb_get_in_data<'a>(
    ep: &'a mut Endpoint<'a>,
    buffer: &'a mut [u8],
    mut len: usize
) -> Option<usize> {
  if ep.has_transfer.get() || ep.is_tx {
    return None;
  }

  if ep.new_data_flag.get() {
    len = if len < ep.actual_len {
        len
    } else {
        ep.actual_len
    };

    unsafe {
        let dst_ptr = buffer.as_mut_ptr();
        let src_ptr = ep.buffer.as_ptr();

        copy_nonoverlapping(src_ptr, dst_ptr, len);
    }

    ep.new_data_flag.set(false);

    let size = ep.size;
    // return if pio_usb_ll_transfer_start(ep, ep.buffer, size) {
    //     Some(len)
    // } else {
    //     None
    // };
  }

  return None;
}

fn pio_usb_set_out_data<'a>(
    ep: &'a mut Endpoint<'a>,
    buffer: &'a mut [u8],
    len: usize
) -> bool {
  if !(ep.has_transfer.get() || !ep.is_tx) && pio_usb_ll_transfer_start(ep, buffer, len) {
      true
  } else {
      false
  }
}

//--------------------------------------------------------------------+
// Low Level Function
//--------------------------------------------------------------------+

fn pio_usb_ll_configure_endpoint(ep: &mut Endpoint, d: &EndpointDescriptor) {
    ep.size = u16::from_le_bytes(d.max_size) as usize;
    ep.ep_num = d.epaddr;
    ep.attr.set(d.attr);
    ep.interval.set(d.interval);
    ep.interval_counter.set(0);
    ep.data_id.set(0);
}

use core::ptr::copy_nonoverlapping;

fn prepare_tx_data(ep: &mut Endpoint) {
    let xact_len: usize = pio_usb_ll::pio_usb_ll_get_transaction_len(ep);
    ep.buffer[0] = usb_definitions::USB_SYNC;
    ep.buffer[1] = if ep.data_id.get() == 1 {
        usb_definitions::USB_PID_DATA1
    } else {
        usb_definitions::USB_PID_DATA0
    };

    unsafe {
        let dst_ptr = ep.buffer.as_mut_ptr().add(2);
        let src_ptr = ep.app_buf.as_ptr();

        copy_nonoverlapping(src_ptr, dst_ptr, xact_len);
    }

    let crc16: u16 = usb_crc::calc_usb_crc16(ep.app_buf, xact_len);
    ep.buffer[2 + xact_len] = crc16 as u8;
    ep.buffer[2 + xact_len + 1] = (crc16 >> 8) as u8;
}

fn pio_usb_ll_transfer_start<'a>(
    ep: &'a mut Endpoint<'a>,
    buffer: &'a mut [u8],
    buflen: usize,
) -> bool {
    if ep.has_transfer.get() {
        false
    } else {
        ep.app_buf = buffer;
        ep.total_len = buflen;
        ep.actual_len = 0;

        if ep.is_tx {
            prepare_tx_data(ep);
        } else {
            ep.new_data_flag.set(false);
        }

        ep.transfer_started.set(false);
        ep.transfer_aborted.set(false);
        ep.has_transfer.set(true);

        true
    }
}

pub fn pio_usb_ll_transfer_continue<F, DPDM, DMDP, P>(
    ep: &mut Endpoint,
    xferred_bytes: usize,
    root: &RootPort<P, F, DPDM, DMDP>,
) -> bool
where
    DPDM: PinId,
    DMDP: PinId,
    F: Function,
    P: PullType,
{
    ep.app_buf = &ep.app_buf[xferred_bytes..];
    ep.actual_len += xferred_bytes;
    ep.data_id.set(ep.data_id.get() ^ 1);

    if (xferred_bytes < ep.size) || (ep.actual_len >= ep.total_len) {
        // complete if all bytes transferred or short packet
        pio_usb_ll_transfer_complete::<F, DPDM, DMDP, P>(
            ep,
            pio_usb_ll::PIO_USB_INTS_ENDPOINT_COMPLETE_BITS,
            root,
        );
        false
    } else {
        if ep.is_tx {
            prepare_tx_data(ep);
        }

        true
    }
}

pub fn pio_usb_ll_transfer_complete<F, DPDM, DMDP, P>(
    ep: &Endpoint,
    flag: u32,
    root: &RootPort<P, F, DPDM, DMDP>,
) where
    DPDM: PinId,
    DMDP: PinId,
    F: Function,
    P: PullType,
{
    let ep_mask: u32 = 1;

    root.ints.set(root.ints.get() | flag);

    if flag == pio_usb_ll::PIO_USB_INTS_ENDPOINT_COMPLETE_BITS {
        root.ep_complete.set(root.ep_complete.get() | ep_mask);
        if !ep.is_tx {
            ep.new_data_flag.set(true);
        }
    } else if flag == pio_usb_ll::PIO_USB_INTS_ENDPOINT_ERROR_BITS {
        root.ep_error.set(root.ep_error.get() | ep_mask);
    } else if flag == pio_usb_ll::PIO_USB_INTS_ENDPOINT_STALLED_BITS {
        root.ep_stalled.set(root.ep_stalled.get() | ep_mask);
    } else {
        // something wrong
    }

    ep.has_transfer.set(false);
}

fn pio_usb_host_add_port<F, DPDM, DMDP, SM, P>(
    _pin_dp: Pin<DPDM, F, PullDown>,
    _pin_dm: Pin<DMDP, F, PullDown>,
    mut sm: StateMachine<SM, Stopped>,
    root: &mut RootPort<P, F, DPDM, DMDP>,
) -> bool
where
    DPDM: PinId,
    DMDP: PinId,
    F: Function,
    SM: ValidStateMachine,
    P: PullType,
{
    if !root.initialized.get() {
        root.pin_dp.set_input_override(InputOverride::Invert);
        root.pin_dm.set_input_override(InputOverride::Invert);

        let pin_dm_id = root.pin_dm.id().num;
        let pin_dp_id = root.pin_dp.id().num;

        sm.set_pindirs([
            (pin_dm_id, rp2040_hal::pio::PinDir::Input),
            (pin_dp_id, rp2040_hal::pio::PinDir::Input),
        ]);

        root.pin_dp.set_slew_rate(OutputSlewRate::Fast);
        root.pin_dm.set_slew_rate(OutputSlewRate::Fast);
        root.pin_dp
            .set_drive_strength(OutputDriveStrength::TwelveMilliAmps);
        root.pin_dm
            .set_drive_strength(OutputDriveStrength::TwelveMilliAmps);
        root.initialized.set(true);
        false
    } else {
        true
    }
}

pub fn pio_usb_bus_receive_packet_and_handshake<PioRx, PioTx, SM, State>(
    pp: &mut PioPort<PioRx, PioTx>,
    handshake: u8,
    mut rx: rp2040_hal::pio::Rx<SM>,
    mut sm_rx: rp2040_hal::pio::StateMachine<SM, State>
) -> Option<usize> where
    PioRx: rp2040_hal::pio::PIOExt,
    PioTx: rp2040_hal::pio::PIOExt,
    SM: rp2040_hal::pio::ValidStateMachine
{
  let mut crc: u16 = 0xffff;
  let mut crc_prev: u16 = 0xffff;
  let mut crc_prev2: u16 = 0xffff;
  let mut crc_receive: u16 = 0xffff;
  let mut crc_receive_inverse: u16 = 0;
  let mut crc_match: bool = false;
  let mut t: u8 = 240;
  let mut idx: usize = 0;

  while t > 0 {
    t -= 1;
    if !rx.is_empty() {
        pp.rx_buffer[idx] = (rx.read().unwrap() >> 24) as u8;
        idx += 1;
        if idx == 2 {
          break;
        }
    }
  }

  // timing critical start
  if t > 0 {
      if handshake == USB_PID_ACK {
          while (pp.pio_rx.get_irq_raw() & IRQ_RX_COMP_MASK) == 0 {
              if !rx.is_empty() {
                let data: u8 = (rx.read().unwrap() >> 24) as u8;
                crc_prev2 = crc_prev;
                crc_prev = crc;
                crc = usb_crc::update_usb_crc16(crc, data);
                pp.rx_buffer[idx] = data;
                idx += 1;
                crc_receive = (crc_receive >> 8) | ((data as u16) << 8);
                crc_receive_inverse = crc_receive ^ 0xffff;
                crc_match = crc_receive_inverse == crc_prev2;
              }
          }

          if idx >= 4 && crc_match {
              pio_usb_bus_send_handshake(pp, USB_PID_ACK);
              // timing critical end
              return Some(idx - 4);
          }
      } else {
        // just discard received data since we NAK/STALL anyway
        while (pp.pio_rx.get_irq_raw() & IRQ_RX_COMP_MASK) == 0 {
          continue;
        }
        sm_rx.clear_fifos();

        pio_usb_bus_send_handshake(pp, handshake);
      }
  }

  None
}
