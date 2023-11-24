use core::ffi::{c_int, c_uint};
use rp2040_hal::{
    dma::{Channel, CH9},
    gpio::{Function, Pin, PinId, PullType},
    pio::{UninitStateMachine, PIO, SM2, SM3},
};

pub struct PioUsbConfiguration<P, DP, DM, F, PIO_RX, PIO_TX>
where
    P: PullType,
    DP: PinId,
    DM: PinId,
    F: Function,
    PIO_RX: rp2040_hal::pio::PIOExt,
    PIO_TX: rp2040_hal::pio::PIOExt,
{
    pub skip_alarm_pool: bool,
    pub pin_dp: Pin<DP, F, P>,
    pub pin_dm: Pin<DM, F, P>,
    pub pio_rx: PIO<PIO_RX>,
    pub pio_tx: PIO<PIO_TX>,
    pub sm_eop: UninitStateMachine<(PIO_RX, SM3)>,
    pub sm_tx: UninitStateMachine<(PIO_TX, SM3)>,
    pub sm_rx: UninitStateMachine<(PIO_RX, SM2)>,
    pub tx_ch: Channel<CH9>,
}

const PIO_USB_TX_DEFAULT: c_uint = 0;
const PIO_USB_DMA_TX_DEFAULT: c_uint = 0;
const PIO_USB_RX_DEFAULT: c_uint = 1;

const PIO_SM_USB_TX_DEFAULT: c_uint = 0;
const PIO_SM_USB_RX_DEFAULT: c_uint = 0;
const PIO_SM_USB_EOP_DEFAULT: c_uint = 1;

const PIO_USB_DEBUG_PIN_NONE: c_int = -1;

pub const PIO_USB_EP_POOL_CNT: usize = 32;
pub const PIO_USB_DEV_EP_CNT: usize = 16;
pub const PIO_USB_DEVICE_CNT: usize = 4;
pub const PIO_USB_HUB_PORT_CNT: usize = 8;
pub const PIO_USB_ROOT_PORT_CNT: usize = 2;

const PIO_USB_EP_SIZE: u8 = 64;

pub const PIO_USB_PINOUT_DPDM: u8 = 0;
