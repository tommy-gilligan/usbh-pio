#![no_std]
use usbh::types::{DeviceAddress, SetupPacket, TransferType};
mod pio_usb;
mod pio_usb_configuration;
mod pio_usb_host;
mod pio_usb_ll;
mod usb_crc;
mod usb_definitions;

use rp_pico::hal::gpio::{Function, Pin, PinId, PullType};

pub struct UsbhPio<P, F, DPDM, DMDP>
where
    P: PullType,
    DPDM: PinId,
    DMDP: PinId,
    F: Function,
{
    root: usb_definitions::RootPort<P, F, DPDM, DMDP>,
}

impl<P, F, DPDM, DMDP> UsbhPio<P, F, DPDM, DMDP>
where
    P: PullType,
    DPDM: PinId,
    DMDP: PinId,
    F: Function,
{
    pub fn new(
        dpdm: Pin<DPDM, F, P>,
        dmdp: Pin<DMDP, F, P>,
        _pio0: rp_pico::hal::pio::PIO<rp_pico::hal::pac::PIO0>,
        _pio0sm0: rp_pico::hal::pio::UninitStateMachine<(
            rp_pico::hal::pac::PIO0,
            rp_pico::hal::pio::SM0,
        )>,
        _ch0: rp_pico::hal::dma::Channel<rp_pico::hal::dma::CH0>,
        _ch1: rp_pico::hal::dma::Channel<rp_pico::hal::dma::CH1>,
    ) -> Self {
        Self {
            root: usb_definitions::RootPort {
                initialized: vcell::VolatileCell::new(false),
                addr0_exists: vcell::VolatileCell::new(false),
                is_fullspeed: vcell::VolatileCell::new(false),
                connected: vcell::VolatileCell::new(false),
                suspended: vcell::VolatileCell::new(false),
                mode: pio_usb_ll::PIO_USB_MODE_HOST,
                pin_dp: dpdm,
                pin_dm: dmdp,
                dev_addr: 0,
                ep_complete: vcell::VolatileCell::new(0),
                ep_error: vcell::VolatileCell::new(0),
                ep_stalled: vcell::VolatileCell::new(0),
                ints: vcell::VolatileCell::new(0),
            },
        }
    }
}

impl<P, F, DPDM, DMDP> usbh::bus::HostBus for UsbhPio<P, F, DPDM, DMDP>
where
    P: PullType,
    DPDM: PinId,
    DMDP: PinId,
    F: Function,
{
    fn reset_controller(&mut self) {
        // first
        todo!()
    }
    fn reset_bus(&mut self) {
        todo!()
    }
    fn enable_sof(&mut self) {
        todo!()
    }
    fn sof_enabled(&self) -> bool {
        todo!()
    }
    fn set_recipient(&mut self, _: Option<DeviceAddress>, _: u8, _: TransferType) {
        todo!()
    }
    fn ls_preamble(&mut self, _: bool) {
        todo!()
    }
    fn stop_transaction(&mut self) {
        todo!()
    }
    fn write_setup(&mut self, _: SetupPacket) {
        todo!()
    }
    fn write_data_in(&mut self, _: u16, _: bool) {
        todo!()
    }
    fn prepare_data_out(&mut self, _: &[u8]) {
        todo!()
    }
    fn write_data_out_prepared(&mut self) {
        todo!()
    }
    fn poll(&mut self) -> Option<usbh::bus::Event> {
        todo!()
    }
    fn received_data(&self, _: usize) -> &[u8] {
        todo!()
    }
    fn create_interrupt_pipe(
        &mut self,
        _: DeviceAddress,
        _: u8,
        _: usb_device::UsbDirection,
        _: u16,
        _: u8,
    ) -> Option<usbh::bus::InterruptPipe> {
        todo!()
    }
    fn release_interrupt_pipe(&mut self, _: u8) {
        todo!()
    }
    fn pipe_continue(&mut self, _: u8) {
        todo!()
    }
    fn interrupt_on_sof(&mut self, _: bool) {
        todo!()
    }
}
