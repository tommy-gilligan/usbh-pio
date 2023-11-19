use usbh::types::{TransferType, SetupPacket, DeviceAddress};
use pio_proc::pio_file;

struct MyStruct;

fn compile() {
    let program_with_defines = pio_proc::pio_file!(
        "src/usb_rx.pio",
        select_program("usb_nrzi_decoder_debug"), // Optional if only one program in the file
        options(max_program_size = 32) // Optional, defaults to 32
    );
}

impl usbh::bus::HostBus for MyStruct {
    fn reset_controller(&mut self) { todo!() }
    fn reset_bus(&mut self) { todo!() }
    fn enable_sof(&mut self) { todo!() }
    fn sof_enabled(&self) -> bool { todo!() }
    fn set_recipient(&mut self, _: Option<DeviceAddress>, _: u8, _: TransferType) { todo!() }
    fn ls_preamble(&mut self, _: bool) { todo!() }
    fn stop_transaction(&mut self) { todo!() }
    fn write_setup(&mut self, _: SetupPacket) { todo!() }
    fn write_data_in(&mut self, _: u16, _: bool) { todo!() }
    fn prepare_data_out(&mut self, _: &[u8]) { todo!() }
    fn write_data_out_prepared(&mut self) { todo!() }
    fn poll(&mut self) -> Option<usbh::bus::Event> { todo!() }
    fn received_data(&self, _: usize) -> &[u8] { todo!() }
    fn create_interrupt_pipe(&mut self, _: DeviceAddress, _: u8, _: usb_device::UsbDirection, _: u16, _: u8) -> Option<usbh::bus::InterruptPipe> { todo!() }
    fn release_interrupt_pipe(&mut self, _: u8) { todo!() }
    fn pipe_continue(&mut self, _: u8) { todo!() }
    fn interrupt_on_sof(&mut self, _: bool) { todo!() }
}
