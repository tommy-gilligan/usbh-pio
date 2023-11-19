use vcell::VolatileCell;

use rp_pico::hal::gpio::{Function, Pin, PinId, PullType};

pub enum ControlTransferOperation {
    ControlNone,
    ControlIn,
    ControlOut,
    ControlComplete,
    ControlError,
}

pub enum SetupTransferStage {
    StageSetup,
    StageData,
    StageIn,
    StageOut,
    StageStatus,
    StageComplete,
    StageError,
}

pub struct RootPort<P, F, DPDM, DMDP>
where
    P: PullType,
    DPDM: PinId,
    DMDP: PinId,
    F: Function,
{
    pub initialized: VolatileCell<bool>,
    pub addr0_exists: VolatileCell<bool>,
    pub is_fullspeed: VolatileCell<bool>,
    pub connected: VolatileCell<bool>,
    pub suspended: VolatileCell<bool>,
    pub mode: u8,
    pub dev_addr: u8,
    pub pin_dp: Pin<DPDM, F, P>,
    pub pin_dm: Pin<DMDP, F, P>,

    pub ints: VolatileCell<u32>,
    pub ep_complete: VolatileCell<u32>,
    pub ep_error: VolatileCell<u32>,
    pub ep_stalled: VolatileCell<u32>,
}

pub struct ControlPipe {
    pub data_in_num: VolatileCell<u8>,
    pub buffer_idx: VolatileCell<u16>,
    pub rx_buffer: VolatileCell<u8>,
    pub request_length: VolatileCell<i16>,
    pub operation: VolatileCell<ControlTransferOperation>,
    pub stage: VolatileCell<SetupTransferStage>,
}

pub struct Endpoint<'a> {
    pub dev_addr: u8,
    pub need_pre: bool,
    pub is_tx: bool,

    pub ep_num: VolatileCell<u8>,
    pub new_data_flag: VolatileCell<bool>,
    pub size: VolatileCell<usize>,

    pub attr: VolatileCell<u8>,
    pub interval: VolatileCell<u8>,
    pub interval_counter: VolatileCell<u8>,
    pub data_id: VolatileCell<u8>,

    pub stalled: VolatileCell<bool>,
    pub has_transfer: VolatileCell<bool>,
    pub transfer_started: VolatileCell<bool>,
    pub transfer_aborted: VolatileCell<bool>,

    pub buffer: [u8; 64 + 4],

    pub app_buf: &'a [u8],
    pub total_len: usize,
    pub actual_len: usize,
}

pub struct EndpointDescriptor {
    pub length: u8,
    pub r#type: u8,
    pub epaddr: u8,
    pub attr: u8,
    pub max_size: [u8; 2],
    pub interval: u8,
}

pub const USB_SYNC: u8 = 0x80;
pub const USB_PID_OUT: u8 = 0xe1;
pub const USB_PID_IN: u8 = 0x69;
pub const USB_PID_SOF: u8 = 0xa5;
pub const USB_PID_SETUP: u8 = 0x2d;
pub const USB_PID_DATA0: u8 = 0xc3;
pub const USB_PID_DATA1: u8 = 0x4b;
pub const USB_PID_ACK: u8 = 0xd2;
pub const USB_PID_NAK: u8 = 0x5a;
pub const USB_PID_STALL: u8 = 0x1e;
pub const USB_PID_PRE: u8 = 0x3c;
pub const USB_CRC16_PLACE: u8 = 0;
