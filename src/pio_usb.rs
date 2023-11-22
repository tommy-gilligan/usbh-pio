use core::ffi::c_int;

use rp_pico::hal::{
    gpio::{
        Function, InputOverride, OutputDriveStrength, OutputSlewRate, Pin, PinId, PullDown,
        PullType,
    },
    pac::PIO0,
    pio::{StateMachine, Stopped, ValidStateMachine, PIO},
};

use crate::{pio_usb_configuration, pio_usb_ll, usb_definitions};
use pio::ProgramWithDefines;
use pio_usb_configuration::PioUsbConfiguration;

use usb_definitions::{Endpoint, EndpointDescriptor, RootPort};

pub fn pio_usb_bus_init<T, A, B, C, P, F, DPDM, DMDP>(
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
    // memset(root, 0, sizeof(root_port_t));

    // pp.pio_usb_tx = if c.pio_tx_num == 0 {
    //     pio0
    // } else {
    //     pio1
    // };
    // dma_claim_mask(1<<c->tx_ch);
    // configure_tx_channel(c.tx_ch, pp.pio_usb_tx, c.sm_tx);

    // apply_config(pp, c, root);
    initialize_host_programs(pp, c, root);
    port_pin_drive_setting(root);
    root.initialized.set(true);
    root.dev_addr = 0;
}

fn add_pio_host_rx_program<T, U>(
    pio: &mut PIO<PIO0>,
    program: &ProgramWithDefines<T, 32>,
    debug_program: &ProgramWithDefines<U, 32>,
    debug_pin: c_int,
) {
    if debug_pin < 0 {
        pio.install(&program.program).unwrap();
    } else {
        pio.install(&debug_program.program).unwrap();
    }
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

fn initialize_host_programs<T, A, B, C, P, F, DPDM, DMDP>(
    pp: &mut pio_usb_ll::PioPort<T, A, B, C>,
    c: &PioUsbConfiguration<A, B, C>,
    _port: &RootPort<P, F, DPDM, DMDP>,
) where
    A: ValidStateMachine,
    B: ValidStateMachine,
    C: ValidStateMachine,
    P: PullType,
    DPDM: PinId,
    DMDP: PinId,
    F: Function,
{
    let usb_tx_dpdm = pio_proc::pio_file!(
        "src/usb_tx.pio",
        select_program("usb_tx_dpdm"),
        options(max_program_size = 32)
    );
    pp.offset_tx = pp.pio_usb_tx.install(&usb_tx_dpdm.program).unwrap();

    // usb_tx_fs_program_init(
    //     pp->pio_usb_tx,
    //     pp->sm_tx,
    //     pp->offset_tx,
    //     port->pin_dp,
    //     port->pin_dm
    // );

    add_pio_host_rx_program(
        &mut pp.pio_usb_rx,
        &pio_proc::pio_file!(
            "src/usb_rx.pio",
            select_program("usb_nrzi_decoder"),
            options(max_program_size = 32)
        ),
        &pio_proc::pio_file!(
            "src/usb_rx.pio",
            select_program("usb_nrzi_decoder_debug"),
            options(max_program_size = 32)
        ),
        c.debug_pin_rx,
    );
    // usb_rx_fs_program_init(pp->pio_usb_rx, pp->sm_rx, pp->offset_rx, port->pin_dp, port->pin_dm, c->debug_pin_rx);

    pp.rx_reset_instr = (pio::InstructionOperands::JMP {
        condition: pio::JmpCondition::Always,
        address: pp.offset_rx,
    })
    .encode();
    pp.rx_reset_instr2 = (pio::InstructionOperands::SET {
        destination: pio::SetDestination::X,
        data: 0,
    })
    .encode();

    // add_pio_host_rx_program(pp->pio_usb_rx, &usb_edge_detector_program, &usb_edge_detector_debug_program, &pp->offset_eop, c->debug_pin_eop);
    // eop_detect_fs_program_init(pp->pio_usb_rx, c->sm_eop, pp->offset_eop, port->pin_dp, port->pin_dm, true, c->debug_pin_eop);

    // usb_tx_configure_pins(pp->pio_usb_tx, pp->sm_tx, port->pin_dp, port->pin_dm);

    // pio_sm_set_jmp_pin(pp->pio_usb_rx, pp->sm_rx, port->pin_dp);
    // pio_sm_set_jmp_pin(pp->pio_usb_rx, pp->sm_eop, port->pin_dm);
    // pio_sm_set_in_pins(pp->pio_usb_rx, pp->sm_eop, port->pin_dp);
}

// pub fn configure_tx_channel<T>(ch0: Channel<CH0>, pio: PIO<PIO0>, sm: T) where T: ValidStateMachine {
//     #[allow(clippy::unusual_byte_groupings)]
//     let message = [
//         0b10101010_00100010_11101010_00101110,
//         0b10100011_10111011_10000000_10111011,
//         0b10001110_11101110_00101110_10001011,
//         0b10101000_11101010_00000000_00000000,
//         0b10101010_00100010_11101010_00101110,
//         0b10100011_10111011_10000000_10111011,
//         0b10001110_11101110_00101110_10001011,
//         0b10101000_11101010_00000000_00000000,
//     ];
//
//     let tx_buf = singleton!(: [u32; 8] = message).unwrap();
//
//   let tx_transfer = rp_pico::hal::dma::single_buffer::Config::new(ch0, tx_buf, tx).start();
//
//   let (ch0, tx_buf, tx) = tx_transfer.wait();
//   for i in 0..rx_buf.len() {
//       if rx_buf[i] != tx_buf[i] {
//           // The data did not match, abort.
//           #[allow(clippy::empty_loop)]
//           loop {}
//       }
//   }
//
//   let conf = rp_pico::hal::dma::single_buffer::Config::new(ch0, tx_buf, tx).start();
//
//   baked into api types/names
//   channel_config_set_read_increment(&conf, true);
//   channel_config_set_write_increment(&conf, false);
//   channel_config_set_transfer_data_size(&conf, DMA_SIZE_8);
//   channel_config_set_dreq(&conf, pio_get_dreq(pio, sm, true));
//
//   dma_channel_set_config(ch, &conf, false);
//   dma_channel_set_write_addr(ch, &pio->txf[sm], false);

// }

fn apply_config<T, A, B, C>(
    pp: &mut pio_usb_ll::PioPort<T, A, B, C>,
    c: &pio_usb_configuration::PioUsbConfiguration<A, B, C>,
) where
    A: ValidStateMachine,
    B: ValidStateMachine,
    C: ValidStateMachine,
{
    // pp.pio_usb_tx = if c.pio_tx_num == 0 {
    //     rp_pico::hal::pac::PIO0
    // } else {
    //     rp_pico::hal::pac::PIO1
    // };
    // pp.sm_tx = c.sm_tx;
    pp.tx_ch = c.tx_ch;
    // pp.pio_usb_rx = if c.pio_rx_num == 0 {
    //     rp_pico::hal::pac::PIO0
    // } else {
    //     rp_pico::hal::pac::PIO1
    // };
    // pp.sm_rx = c.sm_rx;
    // pp.sm_eop = c.sm_eop;

    // pp.fs_tx_program = &pio_proc::pio_file!(
    //     "src/usb_tx.pio",
    //     select_program("usb_tx_dpdm"),
    //     options(max_program_size = 32)
    // );

    // if c.pinout == PIO_USB_PINOUT_DPDM {
    //     pp.fs_tx_program = &pio_proc::pio_file!("src/usb_tx.pio", select_program("usb_tx_dpdm"), options(max_program_size = 32)),
    //     pp.fs_tx_pre_program = &usb_tx_pre_dpdm_program;
    //     pp.ls_tx_program = &usb_tx_dmdp_program;
    // } else {
    //     pp.fs_tx_program = &usb_tx_dmdp_program;
    //     pp.fs_tx_pre_program = &usb_tx_pre_dmdp_program;
    //     pp.ls_tx_program = &usb_tx_dpdm_program;
    // }

    pp.debug_pin_rx = c.debug_pin_rx;
    pp.debug_pin_eop = c.debug_pin_eop;

    // pio_sm_claim(pp.pio_usb_tx, pp.sm_tx);
    // pio_sm_claim(pp.pio_usb_rx, pp.sm_rx);
    // pio_sm_claim(pp.pio_usb_rx, pp.sm_eop);
}

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
