#![no_std]
use usbh::types::{DeviceAddress, SetupPacket, TransferType};

mod pio_usb;
mod pio_usb_configuration;
mod pio_usb_host;
mod pio_usb_ll;
mod usb_definitions;

pub use pio_usb_configuration::PioUsbConfiguration;

pub struct UsbhPio;
impl usbh::bus::HostBus for UsbhPio {
    fn reset_controller(&mut self) {
        defmt::println!("reset_controller");
    }
    fn reset_bus(&mut self) {
        defmt::println!("reset_bus");
    }
    fn enable_sof(&mut self) {
        defmt::println!("enable_sof");
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
