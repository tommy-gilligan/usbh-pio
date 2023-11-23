use core::ffi::c_int;

use rp_pico::hal::{
    dma::{Channel, CH9},
    gpio::{
        Function, InputOverride, OutputDriveStrength, OutputSlewRate, Pin, PinId, PullDown,
        PullType,
    },
    pio::{StateMachine, Stopped, UninitStateMachine, ValidStateMachine, PIO, SM3},
};

use crate::{pio_usb_configuration, pio_usb_ll, usb_definitions};

use pio_usb_configuration::PioUsbConfiguration;

use usb_definitions::{Endpoint, EndpointDescriptor, RootPort};

//--------------------------------------------------------------------+
// Bus functions
//--------------------------------------------------------------------+

// pub fn send_pre<T, A, B, C>(
//     pp: &mut pio_usb_ll::PioPort<T, A, B, C>
// ) where A: ValidStateMachine, B: ValidStateMachine, C: ValidStateMachine {
//     let data = [
//         usb_definitions::USB_SYNC,
//         usb_definitions::USB_PID_PRE,
//     ];
//
//     // send PRE token in full-speed
//     pp.sm_tx.set_enabled(false);
//     for i in 0..USB_TX_EOP_DISABLER_LEN {
//       let instr: u16 = pp.fs_tx_pre_program.instructions[i + USB_TX_EOP_OFFSET];
//       pp.pio_usb_tx.instr_mem[pp.offset_tx + i + USB_TX_EOP_OFFSET] = instr;
//     }
//
//     SM_SET_CLKDIV(pp.pio_usb_tx, pp.sm_tx, pp.clk_div_fs_tx);
//
//     dma_channel_transfer_from_buffer_now(pp.tx_ch, data, 2);
//
//     pp.sm_tx.set_enabled(true);
//     pp.pio_usb_tx.irq |= pio_usb_ll::IRQ_TX_ALL_MASK;       // clear complete flag
//     pp.pio_usb_tx.irq_force |= pio_usb_ll::IRQ_TX_EOP_MASK; // disable eop
//
//     while (pp.pio_usb_tx.get_irq_raw() & pio_usb_ll::IRQ_TX_COMP_MASK) == 0 {
//       continue;
//     }
//
//     // change bus speed to low-speed
//     pp.sm_tx.set_enabled(false);
//     for i in 0..USB_TX_EOP_DISABLER_LEN {
//       let instr: u16 = pp.fs_tx_program.instructions[i + USB_TX_EOP_OFFSET];
//       pp.pio_usb_tx.instr_mem[pp.offset_tx + i + USB_TX_EOP_OFFSET] = instr;
//     }
//     SM_SET_CLKDIV(pp.pio_usb_tx, pp.sm_tx, pp.clk_div_ls_tx);
//
//     pp.sm_rx.set_enabled(false);
//     SM_SET_CLKDIV_MAXSPEED(pp.pio_usb_rx, pp.sm_rx);
//
//     pp.sm_eop.set_enabled(false);
//     SM_SET_CLKDIV(pp.pio_usb_rx, pp.sm_eop, pp.clk_div_ls_rx);
//     pp.sm_eop.set_enabled(true);
// }
//
// pub fn pio_usb_bus_usb_transfer<T, A, B, C>(
//     pp: &pio_usb_ll::PioPort<T, A, B, C>,
//     data: &[u8],
//     len: usize,
// ) where A: ValidStateMachine, B: ValidStateMachine, C: ValidStateMachine {
//   if pp.need_pre {
//     send_pre(pp);
//   }
//
//   dma_channel_transfer_from_buffer_now(pp.tx_ch, data, len);
//
//   pio_sm_set_enabled(pp.pio_usb_tx, pp.sm_tx, true);
//   pp.pio_usb_tx.clear_irq(pp.pio_usb_tx.get_irq_raw() | pio_usb_ll::IRQ_TX_ALL_MASK); // clear complete flag
//
//   while (pp.pio_usb_tx.get_irq_raw() & pio_usb_ll::IRQ_TX_ALL_MASK) == 0 {
//     continue;
//   }
// }
//
// pub fn pio_usb_bus_send_handshake<T, A, B, C>(
//     pp: &pio_usb_ll::PioPort<T, A, B, C>,
//     pid: u8
// ) where A: ValidStateMachine, B: ValidStateMachine, C: ValidStateMachine {
//   let data = [
//       usb_definitions::USB_SYNC,
//       pid,
//   ];
//   pio_usb_bus_usb_transfer(pp, &data, data.len());
// }
//
// fn pio_usb_bus_send_token<T, A, B, C>(
//     pp: &pio_usb_ll::PioPort<T, A, B, C>,
//     token: u8,
//     addr: u8,
//     ep_num: u8
// ) where A: ValidStateMachine, B: ValidStateMachine, C: ValidStateMachine {
//   let packet: [u8; 4] = [usb_definitions::USB_SYNC, token, 0, 0];
//   let dat: u16 = ((ep_num & 0xf) << 7) as u16 | (addr & 0x7f) as u16;
//   let crc: u8 = usb_crc::calc_usb_crc5(dat);
//   packet[2] = dat as u8;
//   packet[3] = (crc << 3) | ((dat >> 8) as u8 & 0x1f);
//
//   pio_usb_bus_usb_transfer(pp, &packet, packet.len());
// }
//
// fn pio_usb_bus_prepare_receive<T, A, B, C>(
//     pp: &pio_usb_ll::PioPort<T, A, B, C>,
//     mut sm_rx: StateMachine<A, rp_pico::hal::pio::Running>
// ) where A: ValidStateMachine, B: ValidStateMachine, C: ValidStateMachine {
// //   pio_sm_set_enabled(pp.pio_usb_rx, pp.sm_rx, false);
// //    sm_rx.clear_fifos();
//     sm_rx.restart();
//     sm_rx.set_instruction(pp.rx_reset_instr);
//     sm_rx.set_instruction(pp.rx_reset_instr2);
// }
//
// fn pio_usb_bus_start_receive<T, A, B, C>(
//     pp: &pio_usb_ll::PioPort<T, A, B, C>
// ) where A: ValidStateMachine, B: ValidStateMachine, C: ValidStateMachine {
//   pp.pio_usb_rx.ctrl |= (1 << pp.sm_rx);
//   pp.pio_usb_rx.force_irq(pio_usb_ll::IRQ_RX_ALL_MASK);
// }
//
// fn pio_usb_bus_wait_handshake<T, A, B, C>(
//     pp: &pio_usb_ll::PioPort<T, A, B, C>
// ) -> u8 where A: ValidStateMachine, B: ValidStateMachine, C: ValidStateMachine {
//   let t: i16 = 240;
//   let idx: usize = 0;
//
//   while t -= 1 {
//     if pio_sm_get_rx_fifo_level(pp.pio_usb_rx, pp.sm_rx) {
//       let data: u8 = pio_sm_get(pp.pio_usb_rx, pp.sm_rx) >> 24;
//       pp.usb_rx_buffer[idx] = data;
//       idx += 1;
//       if idx == 2 {
//         break;
//       }
//     }
//   }
//
//   if t > 0 {
//     while (pp.pio_usb_rx.get_irq_raw() & pio_usb_ll::IRQ_RX_COMP_MASK) == 0 {
//       continue;
//     }
//   }
//
//   pio_sm_set_enabled(pp.pio_usb_rx, pp.sm_rx, false);
//
//   return pp.usb_rx_buffer[1];
// }
//
// fn pio_usb_bus_receive_packet_and_handshake<T, A, B, C>(
//     pp: &mut pio_usb_ll::PioPort<T, A, B, C>,
//     handshake: u8,
// ) -> isize where A: ValidStateMachine, B: ValidStateMachine, C: ValidStateMachine {
//   let crc: u16 = 0xffff;
//   let crc_prev: u16 = 0xffff;
//   let crc_prev2: u16 = 0xffff;
//   let crc_receive: u16 = 0xffff;
//
//   let mut crc_receive_inverse: u16 = 0;
//   let crc_match: bool = false;
//   let t: i16 = 240;
//   let idx: usize = 0;
//
//   while t > 0 {
//       t -= 1;
//       if pio_sm_get_rx_fifo_level(pp.pio_usb_rx, pp.sm_rx) {
//           let data: u8 = pio_sm_get(pp.pio_usb_rx, pp.sm_rx) >> 24;
//           pp.usb_rx_buffer[idx] = data;
//           idx += 1;
//           if idx == 2 {
//             break;
//           }
//       }
//   }
//
//   // timing critical start
//   if t > 0 {
//     if handshake == usb_definitions::USB_PID_ACK {
//       while (pp.pio_usb_rx.get_irq_raw() & pio_usb_ll::IRQ_RX_COMP_MASK) == 0 {
//         if pio_sm_get_rx_fifo_level(pp.pio_usb_rx, pp.sm_rx) {
//           let data: u8 = pio_sm_get(pp.pio_usb_rx, pp.sm_rx) >> 24;
//           crc_prev2 = crc_prev;
//           crc_prev = crc;
//           crc = usb_crc::update_usb_crc16(crc, data);
//           pp.usb_rx_buffer[idx] = data;
//           idx += 1;
//           crc_receive = (crc_receive >> 8) | (data << 8);
//           crc_receive_inverse = crc_receive ^ 0xffff;
//           crc_match = crc_receive_inverse == crc_prev2;
//         }
//       }
//
//       if idx >= 4 && crc_match {
//         pio_usb_bus_send_handshake(pp, usb_definitions::USB_PID_ACK);
//         // timing critical end
//         return (idx - 4) as isize;
//       }
//     } else {
//       // just discard received data since we NAK/STALL anyway
//       while (pp.pio_usb_rx.get_irq_raw() & pio_usb_ll::IRQ_RX_COMP_MASK) == 0 {
//         continue;
//       }
//       pio_sm_clear_fifos(pp.pio_usb_rx, pp.sm_rx);
//
//       pio_usb_bus_send_handshake(pp, handshake);
//     }
//   }
//
//   return -1;
// }

fn initialize_host_programs<P, F, DP, DM, PIO_RX, PIO_TX>(
    pp: &mut pio_usb_ll::PioPort<PIO_RX, PIO_TX>,
    _port: &RootPort<P, F, DP, DM>,
) where
    P: PullType,
    DP: PinId,
    DM: PinId,
    F: Function,
    PIO_RX: rp_pico::hal::pio::PIOExt,
    PIO_TX: rp_pico::hal::pio::PIOExt,
{
    let fs_tx_program = pio_proc::pio_file!(
        "src/usb_tx.pio",
        select_program("usb_tx_dpdm"),
        options(max_program_size = 32)
    );
    let _USB_TX_EOP_DISABLER_LEN = fs_tx_program.public_defines.USB_TX_EOP_DISABLER_LEN;
    let _USB_TX_EOP_OFFSET = fs_tx_program.public_defines.USB_TX_EOP_OFFSET;

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
//

pub fn configure_tx_channel<P>(
    _ch: &mut Channel<CH9>,
    _pio: &mut PIO<P>,
    _sm: &mut UninitStateMachine<(P, SM3)>,
) where
    P: rp_pico::hal::pio::PIOExt,
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

fn port_pin_drive_setting<P, F, DPDM, DMDP>(port: &mut RootPort<P, F, DPDM, DMDP>)
where
    P: PullType,
    DPDM: PinId,
    DMDP: PinId,
    F: Function,
{
    port.pin_dp.set_slew_rate(OutputSlewRate::Fast);
    port.pin_dm.set_slew_rate(OutputSlewRate::Fast);
    port.pin_dp
        .set_drive_strength(OutputDriveStrength::TwelveMilliAmps);
    port.pin_dm
        .set_drive_strength(OutputDriveStrength::TwelveMilliAmps);
}

use pio_usb_ll::PioPort;
// return root and port but for now just root
pub fn apply_config<P, F, DP, DM, PIO_RX, PIO_TX>(
    c: PioUsbConfiguration<P, DP, DM, F, PIO_RX, PIO_TX>,
) -> (RootPort<P, F, DP, DM>, PioPort<PIO_RX, PIO_TX>)
where
    P: PullType,
    DP: PinId,
    DM: PinId,
    F: Function,
    PIO_RX: rp_pico::hal::pio::PIOExt,
    PIO_TX: rp_pico::hal::pio::PIOExt,
{
    let root = RootPort {
        initialized: vcell::VolatileCell::new(false),
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

    let pp = PioPort {
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
    };

    (root, pp)
}

pub fn pio_usb_bus_init<P, F, DP, DM, PIO_RX, PIO_TX>(
    c: PioUsbConfiguration<P, DP, DM, F, PIO_RX, PIO_TX>,
) where
    P: PullType,
    DP: PinId,
    DM: PinId,
    F: Function,
    PIO_RX: rp_pico::hal::pio::PIOExt,
    PIO_TX: rp_pico::hal::pio::PIOExt,
{
    let (mut root, mut pp) = apply_config(c);

    initialize_host_programs(&mut pp, &root);
    port_pin_drive_setting(&mut root);

    root.initialized.set(true);
    root.dev_addr = 0;
}

//--------------------------------------------------------------------+
// Application API
//--------------------------------------------------------------------+

// fn pio_usb_get_endpoint(
//     device: &crate::usb_definitions::UsbDevice,
//     idx: usize
// ) -> Option<&'static Endpoint> {
//   let ep_id: u8 = device.endpoint_id[idx];
//
//   // if ep_id >= 1 {
//   //   Some(&pio_usb_ep_pool[ep_id - 1])
//   // } else {
//     None
//   // }
// }
//
// fn pio_usb_get_in_data(
//     ep: &mut Endpoint,
//     buffer: &[u8],
//     len: usize
// ) -> isize {
//   if ep.has_transfer.get() || ep.is_tx {
//     return -1;
//   }
//
//   if ep.new_data_flag.get() {
//     len = if len < ep.actual_len {
//         len
//     } else {
//         ep.actual_len
//     };
//
//     unsafe {
//         let dst_ptr = buffer.as_mut_ptr();
//         let src_ptr = ep.buffer.as_ptr();
//
//         copy_nonoverlapping(src_ptr, dst_ptr, len);
//     }
//
//     ep.new_data_flag.set(false);
//
//     return if pio_usb_ll_transfer_start(ep, &ep.buffer, ep.size.get()) {
//         len as isize
//     } else {
//         -1
//     };
//   }
//
//   return -1;
// }

// fn pio_usb_set_out_data(
//     ep: &mut Endpoint,
//     buffer: &[u8],
//     len: usize
// ) -> bool {
//   if !(ep.has_transfer.get() || !ep.is_tx) && pio_usb_ll_transfer_start(ep, buffer, len) {
//       true
//   } else {
//       false
//   }
// }

//--------------------------------------------------------------------+
// Low Level Function
//--------------------------------------------------------------------+

fn pio_usb_ll_configure_endpoint(ep: &Endpoint, d: &EndpointDescriptor) {
    ep.size.set(u16::from_le_bytes(d.max_size) as usize);
    ep.ep_num.set(d.epaddr);
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
    buffer: &'a [u8],
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
    root: &mut RootPort<P, F, DPDM, DMDP>,
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

    if (xferred_bytes < ep.size.get()) || (ep.actual_len >= ep.total_len) {
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
    root: &mut RootPort<P, F, DPDM, DMDP>,
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
) -> c_int
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
            (pin_dm_id, rp_pico::hal::pio::PinDir::Input),
            (pin_dp_id, rp_pico::hal::pio::PinDir::Input),
        ]);

        port_pin_drive_setting(root);
        root.initialized.set(true);

        0
    } else {
        -1
    }
}
