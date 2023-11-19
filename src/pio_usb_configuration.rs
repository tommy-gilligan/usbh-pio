use core::ffi::{c_int, c_uint};
use rp_pico::hal::pio::{UninitStateMachine, ValidStateMachine};

pub struct PioUsbConfiguration<A, B, C>
where
    A: ValidStateMachine,
    B: ValidStateMachine,
    C: ValidStateMachine,
{
    pub sm_tx: UninitStateMachine<A>,
    pub sm_rx: UninitStateMachine<B>,
    pub sm_eop: UninitStateMachine<C>,

    pub pio_tx_num: c_uint,
    pub tx_ch: c_uint,
    pub pio_rx_num: c_uint,
    pub debug_pin_rx: c_int,
    pub debug_pin_eop: c_int,
    pub skip_alarm_pool: bool,
}

const PIO_USB_TX_DEFAULT: c_uint = 0;
const PIO_USB_DMA_TX_DEFAULT: c_uint = 0;
const PIO_USB_RX_DEFAULT: c_uint = 1;

const PIO_SM_USB_TX_DEFAULT: c_uint = 0;
const PIO_SM_USB_RX_DEFAULT: c_uint = 0;
const PIO_SM_USB_EOP_DEFAULT: c_uint = 1;

const PIO_USB_DEBUG_PIN_NONE: c_int = -1;

// pub const PIO_USB_DEFAULT_CONFIG: PioUsbConfiguration = PioUsbConfiguration {
//     sm_tx: PIO_SM_USB_TX_DEFAULT,
//     sm_rx: PIO_SM_USB_RX_DEFAULT,
//     sm_eop: PIO_SM_USB_EOP_DEFAULT,
//
//     pio_tx_num: PIO_USB_TX_DEFAULT,
//     tx_ch: PIO_USB_DMA_TX_DEFAULT,
//     pio_rx_num: PIO_USB_RX_DEFAULT,
//     debug_pin_rx: PIO_USB_DEBUG_PIN_NONE,
//     debug_pin_eop: PIO_USB_DEBUG_PIN_NONE,
//     skip_alarm_pool: false,
// };

const PIO_USB_EP_POOL_CNT: u8 = 32;
const PIO_USB_DEV_EP_CNT: u8 = 16;
const PIO_USB_DEVICE_CNT: u8 = 4;
const PIO_USB_HUB_PORT_CNT: u8 = 8;
pub const PIO_USB_ROOT_PORT_CNT: u8 = 2;

const PIO_USB_EP_SIZE: u8 = 64;

pub const PIO_USB_PINOUT_DPDM: u8 = 0;
