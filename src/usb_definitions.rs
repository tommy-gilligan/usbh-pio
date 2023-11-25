use vcell::VolatileCell;

use rp2040_hal::gpio::{Function, Pin, PinId, PullType};

pub enum ControlTransferOperation {
    None,
    In,
    Out,
    Complete,
    Error,
}

pub enum EndpointType {
    In,
    Out,
}

pub enum SetupTransferStage {
    Setup,
    Data,
    In,
    Out,
    Status,
    Complete,
    Error,
}

pub enum EnumerationStage {
    Idle,
    Device,
    Config,
    Config2,
    Interface,
    Endpoint,
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

pub enum DeviceEvent {
    None,
    Connect,
    Disconnect,
    HubPortChange,
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

pub struct UsbDevice {
    pub connected: VolatileCell<bool>,
    pub enumerated: VolatileCell<bool>,
    pub address: VolatileCell<u8>,
    pub vid: VolatileCell<u16>,
    pub pid: VolatileCell<u16>,
    pub device_class: VolatileCell<u8>,
    pub is_fullspeed: VolatileCell<bool>,
    pub is_root: VolatileCell<bool>,
    pub endpoint_id: [u8; crate::pio_usb_configuration::PIO_USB_DEV_EP_CNT],
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

pub const DESC_TYPE_DEVICE: u8 = 0x01;
pub const DESC_TYPE_CONFIG: u8 = 0x02;
pub const DESC_TYPE_STRING: u8 = 0x03;
pub const DESC_TYPE_INTERFACE: u8 = 0x04;
pub const DESC_TYPE_ENDPOINT: u8 = 0x05;
pub const DESC_TYPE_HID: u8 = 0x21;
pub const DESC_TYPE_HID_REPORT: u8 = 0x22;

pub const CLASS_HID: u8 = 0x03;
pub const CLASS_HUB: u8 = 0x09;

pub const EP_ATTR_CONTROL: u8 = 0x00;
pub const EP_ATTR_ISOCHRONOUS: u8 = 0x01;
pub const EP_ATTR_BULK: u8 = 0x02;
pub const EP_ATTR_INTERRUPT: u8 = 0x03;
pub const EP_ATTR_ENUMERATING: u8 = 0x80;

pub struct UsbSetupPacket {
    request_type: u8,
    request: u8,
    value_lsb: u8,
    value_msb: u8,
    index_lsb: u8,
    index_msb: u8,
    length_lsb: u8,
    length_msb: u8,
}

pub struct DeviceDescriptor {
    length: u8,
    r#type: u8,
    bcd_usb: [u8; 2],
    device_class: u8,
    subclass: u8,
    protocol: u8,
    max_packet_size: u8,
    vid: [u8; 2],
    pid: [u8; 2],
    bcd_device: [u8; 2],
    manufacture: u8,
    product: u8,
    serial: u8,
    num_configu: u8,
}

pub struct InterfaceDescriptor {
    length: u8,
    r#type: u8,
    inum: u8,
    altsetting: u8,
    numep: u8,
    iclass: u8,
    isubclass: u8,
    iprotocol: u8,
    iface: u8,
}

pub struct EndpointDescriptor {
    pub length: u8,
    pub r#type: u8,
    pub epaddr: u8,
    pub attr: u8,
    pub max_size: [u8; 2],
    pub interval: u8,
}

pub struct HidDescriptor {
    length: u8,
    r#type: u8,
    bcd_hid: [u8; 2],
    contry_code: u8,
    num_desc: u8,
    desc_type: u8,
    desc_len: [u8; 2],
}

pub struct ConfigurationDescriptorTag {
    length: u8,
    r#type: u8,
    total_length_lsb: u8,
    total_length_msb: u8,
    num_interfaces: u8,
    configuration_value: u8,
    configuration: u8,
    attributes: u8,
    max_power: u8,
}

pub struct HubDescriptor {
    length: u8,
    r#type: u8,
    port_num: u8,
    chara_lsb: u8,
    chara_msb: u8,
    pow_on_time: u8,
    current: u8,
    removable: u8,
}

pub struct HubPortStatus {
    port_status: u16,
    port_change: u16,
}

pub const HUB_SET_PORT_RESET: u8 = 4;
pub const HUB_SET_PORT_POWER: u8 = 8;
pub const HUB_CLR_PORT_CONNECTION: u8 = 16;
pub const HUB_CLR_PORT_ENABLE: u8 = 17;
pub const HUB_CLR_PORT_SUSPEND: u8 = 18;
pub const HUB_CLR_PORT_RESET: u8 = 20;

pub const HUB_STAT_PORT_CONNECTION: u16 = 1 << 0;
pub const HUB_STAT_PORT_ENABLE: u16 = 1 << 1;
pub const HUB_STAT_PORT_SUSPEND: u16 = 1 << 2;
pub const HUB_STAT_PORT_OC: u16 = 1 << 3;
pub const HUB_STAT_PORT_RESET: u16 = 1 << 4;
pub const HUB_STAT_PORT_POWER: u16 = 1 << 8;
pub const HUB_STAT_PORT_LOWSPEED: u16 = 1 << 9;

pub const HUB_CHANGE_PORT_CONNECTION: u8 = 1 << 0;
pub const HUB_CHANGE_PORT_ENABLE: u8 = 1 << 1;
pub const HUB_CHANGE_PORT_SUSPEND: u8 = 1 << 2;
pub const HUB_CHANGE_PORT_OC: u8 = 1 << 3;
pub const HUB_CHANGE_PORT_RESET: u8 = 1 << 4;

pub const USB_REQ_DIR_IN: u8 = 0x80;
pub const USB_REQ_DIR_OUT: u8 = 0x00;
pub const USB_REQ_TYP_STANDARD: u8 = 0x00;
pub const USB_REQ_TYP_CLASS: u8 = 0x20;
pub const USB_REQ_TYP_VENDOR: u8 = 0x40;
pub const USB_REQ_REC_DEVICE: u8 = 0x00;
pub const USB_REQ_REC_IFACE: u8 = 0x01;
pub const USB_REQ_REC_EP: u8 = 0x02;
pub const USB_REQ_REC_OTHER: u8 = 0x03;

pub const GET_DEVICE_DESCRIPTOR_REQ_DEFAULT: UsbSetupPacket = UsbSetupPacket {
    request_type: USB_REQ_DIR_IN,
    request: 0x06,
    value_lsb: 0,
    value_msb: 0x01,
    index_lsb: 0,
    index_msb: 0,
    length_lsb: 0x12,
    length_msb: 0,
};
pub const GET_CONFIGURATION_DESCRIPTOR_REQ_DEFAULT: UsbSetupPacket = UsbSetupPacket {
    request_type: USB_REQ_DIR_IN,
    request: 0x06,
    value_lsb: 0,
    value_msb: 0x02,
    index_lsb: 0,
    index_msb: 0,
    length_lsb: 0x09,
    length_msb: 0,
};
pub const SET_CONFIGURATION_REQ_DEFAULT: UsbSetupPacket = UsbSetupPacket {
    request_type: USB_REQ_DIR_OUT,
    request: 0x09,
    value_lsb: 0,
    value_msb: 0,
    index_lsb: 0,
    index_msb: 0,
    length_lsb: 0,
    length_msb: 0,
};
pub const SET_ADDRESS_REQ_DEFAULT: UsbSetupPacket = UsbSetupPacket {
    request_type: USB_REQ_DIR_OUT,
    request: 0x5,
    value_lsb: 0x02,
    value_msb: 0,
    index_lsb: 0,
    index_msb: 0,
    length_lsb: 0,
    length_msb: 0,
};
pub const SET_HID_IDLE_REQ_DEFAULT: UsbSetupPacket = UsbSetupPacket {
    request_type: USB_REQ_TYP_CLASS | USB_REQ_REC_IFACE,
    request: 0x0A,
    value_lsb: 0,
    value_msb: 0,
    index_lsb: 0,
    index_msb: 0,
    length_lsb: 0,
    length_msb: 0,
};
pub const GET_HID_REPORT_DESCRIPTOR_DEFAULT: UsbSetupPacket = UsbSetupPacket {
    request_type: USB_REQ_DIR_IN | USB_REQ_REC_IFACE,
    request: 0x06,
    value_lsb: 0,
    value_msb: 0x22,
    index_lsb: 0,
    index_msb: 0,
    length_lsb: 0xff,
    length_msb: 0,
};
pub const GET_HUB_DESCRPTOR_REQUEST: UsbSetupPacket = UsbSetupPacket {
    request_type: USB_REQ_DIR_IN | USB_REQ_TYP_CLASS | USB_REQ_REC_DEVICE,
    request: 0x06,
    value_lsb: 0,
    value_msb: 0x29,
    index_lsb: 0,
    index_msb: 0,
    length_lsb: 8,
    length_msb: 0,
};
pub const GET_HUB_PORT_STATUS_REQUEST: UsbSetupPacket = UsbSetupPacket {
    request_type: USB_REQ_DIR_IN | USB_REQ_TYP_CLASS | USB_REQ_REC_OTHER,
    request: 0,
    value_lsb: 0,
    value_msb: 0,
    index_lsb: 0,
    index_msb: 0,
    length_lsb: 4,
    length_msb: 0,
};
pub const SET_HUB_FEATURE_REQUEST: UsbSetupPacket = UsbSetupPacket {
    request_type: USB_REQ_DIR_OUT | USB_REQ_TYP_CLASS | USB_REQ_REC_OTHER,
    request: 0x03,
    value_lsb: 0,
    value_msb: 0,
    index_lsb: 0,
    index_msb: 0,
    length_lsb: 0,
    length_msb: 0,
};
pub const CLEAR_HUB_FEATURE_REQUEST: UsbSetupPacket = UsbSetupPacket {
    request_type: USB_REQ_DIR_OUT | USB_REQ_TYP_CLASS | USB_REQ_REC_OTHER,
    request: 0x01,
    value_lsb: 0,
    value_msb: 0,
    index_lsb: 0,
    index_msb: 0,
    length_lsb: 0,
    length_msb: 0,
};
