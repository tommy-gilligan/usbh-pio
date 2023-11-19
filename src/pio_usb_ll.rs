use core::ffi::{c_int, c_uint};
use pio::ProgramWithDefines;

struct PioClkDiv {
    div_int: u16,
    div_frac: u8,
}

use rp_pico::hal::{
    pac::PIO0,
    pio::{InstalledProgram, UninitStateMachine, ValidStateMachine, PIO},
};

pub struct PioPort<'a, T, A, B, C>
where
    A: ValidStateMachine,
    B: ValidStateMachine,
    C: ValidStateMachine,
{
    pub sm_tx: UninitStateMachine<A>,
    pub sm_rx: UninitStateMachine<B>,
    pub sm_eop: UninitStateMachine<C>,

    pub pio_usb_tx: PIO<PIO0>,
    pub offset_tx: InstalledProgram<PIO0>,
    pub tx_ch: c_uint,

    pub pio_usb_rx: PIO<PIO0>,
    pub offset_rx: u8,
    pub offset_eop: c_uint,
    pub rx_reset_instr: u16,
    pub rx_reset_instr2: u16,
    pub device_rx_irq_num: c_uint,

    pub debug_pin_rx: c_int,
    pub debug_pin_eop: c_int,

    pub fs_tx_program: &'a ProgramWithDefines<T, 32>,
    //const pio_program_t *;
    //const pio_program_t *fs_tx_pre_program;
    //const pio_program_t *ls_tx_program;
    pub clk_div_fs_tx: PioClkDiv,
    pub clk_div_fs_rx: PioClkDiv,
    pub clk_div_ls_tx: PioClkDiv,
    pub clk_div_ls_rx: PioClkDiv,

    pub need_pre: bool,
    pub usb_rx_buffer: [u8; 128],
}

pub const PIO_USB_MODE_HOST: u8 = 2;

pub fn pio_usb_ll_get_transaction_len(ep: &crate::usb_definitions::Endpoint) -> usize {
    let remaining: usize = ep.total_len - ep.actual_len;

    if remaining < ep.size.get() {
        remaining
    } else {
        ep.size.get()
    }
}

const PIO_USB_INTS_CONNECT_POS: u32 = 0;
const PIO_USB_INTS_DISCONNECT_POS: u32 = 1;
const PIO_USB_INTS_RESET_END_POS: u32 = 2;
const PIO_USB_INTS_SETUP_REQ_POS: u32 = 3;
const PIO_USB_INTS_SOF_POS: u32 = 4;
const PIO_USB_INTS_ENDPOINT_COMPLETE_POS: u32 = 5;
const PIO_USB_INTS_ENDPOINT_ERROR_POS: u32 = 6;
const PIO_USB_INTS_ENDPOINT_STALLED_POS: u32 = 7;

pub const PIO_USB_INTS_CONNECT_BITS: u32 = 1 << PIO_USB_INTS_CONNECT_POS;
pub const PIO_USB_INTS_DISCONNECT_BITS: u32 = 1 << PIO_USB_INTS_DISCONNECT_POS;
pub const PIO_USB_INTS_RESET_END_BITS: u32 = 1 << PIO_USB_INTS_RESET_END_POS;
pub const PIO_USB_INTS_SETUP_REQ_BITS: u32 = 1 << PIO_USB_INTS_SETUP_REQ_POS;
pub const PIO_USB_INTS_SOF_BITS: u32 = 1 << PIO_USB_INTS_SOF_POS;
pub const PIO_USB_INTS_ENDPOINT_COMPLETE_BITS: u32 = 1 << PIO_USB_INTS_ENDPOINT_COMPLETE_POS;
pub const PIO_USB_INTS_ENDPOINT_ERROR_BITS: u32 = 1 << PIO_USB_INTS_ENDPOINT_ERROR_POS;
pub const PIO_USB_INTS_ENDPOINT_STALLED_BITS: u32 = 1 << PIO_USB_INTS_ENDPOINT_STALLED_POS;
