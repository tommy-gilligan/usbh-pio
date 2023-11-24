struct PioClkDiv {
    div_int: u16,
    div_frac: u8,
}

use rp2040_hal::{
    dma::{Channel, CH9},
    pio::{InstalledProgram, UninitStateMachine, PIO, SM2, SM3},
};

// use rp2040_hal::{
//     pio::{},
// };

pub struct PioPort<PIO_RX, PIO_TX>
where
    PIO_RX: rp2040_hal::pio::PIOExt,
    PIO_TX: rp2040_hal::pio::PIOExt,
{
    pub sm_eop: UninitStateMachine<(PIO_RX, SM3)>,
    pub sm_tx: UninitStateMachine<(PIO_TX, SM3)>,
    pub sm_rx: UninitStateMachine<(PIO_RX, SM2)>,
    pub tx_ch: Channel<CH9>,
    pub pio_rx: PIO<PIO_RX>,
    pub pio_tx: PIO<PIO_TX>,
    pub rx_reset_instr2: u16,
    pub rx_reset_instr: Option<u16>,
    pub offset_tx: Option<InstalledProgram<PIO_TX>>,
    pub offset_rx: Option<InstalledProgram<PIO_RX>>,
    pub offset_eop: Option<InstalledProgram<PIO_RX>>,
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

pub const usb_tx_dpdm_IRQ_COMP: u32 = 0;
pub const usb_tx_dpdm_IRQ_EOP: u32 = 1;

pub const IRQ_RX_BS_ERR: u8 = 1;
pub const IRQ_RX_EOP: u8 = 2;
pub const IRQ_RX_START: u8 = 3;
pub const DECODER_TRIGGER: u8 = 4;

pub const IRQ_TX_EOP_MASK: u8 = 1 << usb_tx_dpdm_IRQ_EOP;
pub const IRQ_TX_COMP_MASK: u8 = 1 << usb_tx_dpdm_IRQ_COMP;
pub const IRQ_TX_ALL_MASK: u8 = IRQ_TX_EOP_MASK | IRQ_TX_COMP_MASK;
pub const IRQ_RX_COMP_MASK: u8 = 1 << IRQ_RX_EOP;
pub const IRQ_RX_ALL_MASK: u8 =
    (1 << IRQ_RX_EOP) | (1 << IRQ_RX_BS_ERR) | (1 << IRQ_RX_START) | (1 << DECODER_TRIGGER);
