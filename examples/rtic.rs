#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#[rtic::app(
    device = rp2040_hal::pac,
    dispatchers = [TIMER_IRQ_1]
)]

mod app {
    use panic_probe as _;

    use usbh_pio::{
        pio_usb_ll::{self, PioPort},
        usb_definitions::RootPort,
        PioUsbConfiguration
    };

    use rp2040_hal::{
        pac::{self, PIO0, PIO1},
        pio::{PIOExt, PIO, SM2, SM3, UninitStateMachine, ShiftDirection, Rx, Tx, StateMachine, Stopped},
        dma::DMAExt,
        gpio::{OutputSlewRate, OutputDriveStrength, InputOverride}
    };

    fn init_tx<P>(pin_dp: u8, pio_tx: &mut PIO<P>, sm_tx: UninitStateMachine<(P, SM3)>) -> (
        StateMachine<(P, SM3), Stopped>,
        Rx<(P, SM3)>,
        Tx<(P, SM3)>
    ) where P: PIOExt {
        let fs_tx_program = pio_proc::pio_file!(
            "src/usb_tx.pio",
            select_program("usb_tx_dpdm"),
            options(max_program_size = 32)
        );
        rp2040_hal::pio::PIOBuilder::from_program(
            pio_tx.install(&fs_tx_program.program).unwrap()
        )
        .clock_divisor_fixed_point(0, 0)
        .autopull(true)
        .pull_threshold(8)
        .out_shift_direction(ShiftDirection::Right)
        .out_pins(pin_dp, 2)
        .set_pins(pin_dp, 2)
        .side_set_pin_base(pin_dp)
        .build(sm_tx)
    }

    fn init_rx<P>(pin_dp: u8, mut pio_tx: &mut PIO<P>, sm_tx: UninitStateMachine<(P, SM2)>) -> (
        StateMachine<(P, SM2), Stopped>,
        Rx<(P, SM2)>,
        Tx<(P, SM2)>
    ) where P: PIOExt {
        let fs_rx_program = pio_proc::pio_file!(
            "src/usb_rx.pio",
            select_program("usb_nrzi_decoder"),
            options(max_program_size = 32)
        );

        rp2040_hal::pio::PIOBuilder::from_program(
            pio_tx.install(&fs_rx_program.program).unwrap()
        )
        .clock_divisor_fixed_point(0, 0)
        .autopush(true)
        .push_threshold(8)
        .in_shift_direction(ShiftDirection::Right)
        .in_pin_base(pin_dp)
        .jmp_pin(pin_dp)
        .build(sm_tx)
    }

    fn init_eop<P>(pin_dp: u8, pin_dm: u8, pio_tx: &mut PIO<P>, sm_tx: UninitStateMachine<(P, SM3)>) -> (
        StateMachine<(P, SM3), Stopped>,
        Rx<(P, SM3)>,
        Tx<(P, SM3)>
    ) where P: PIOExt {
        let fs_rx_program = pio_proc::pio_file!(
            "src/usb_rx.pio",
            select_program("usb_edge_detector"),
            options(max_program_size = 32)
        );

        rp2040_hal::pio::PIOBuilder::from_program(
            pio_tx.install(&fs_rx_program.program).unwrap()
        )
        .clock_divisor_fixed_point(0, 0)
        .autopush(false)
        .push_threshold(8)
        .in_shift_direction(ShiftDirection::Left)
        .in_pin_base(pin_dp)
        .jmp_pin(pin_dm)
        .build(sm_tx)
    }

    const XTAL_FREQ_HZ: u32 = 12_000_000u32;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        tx_tuple: (StateMachine<(PIO0, SM3), Stopped>, Rx<(PIO0, SM3)>, Tx<(PIO0, SM3)>),
        rx_tuple: (StateMachine<(PIO1, SM2), Stopped>, Rx<(PIO1, SM2)>, Tx<(PIO1, SM2)>),
        eop_tuple: (StateMachine<(PIO1, SM3), Stopped>, Rx<(PIO1, SM3)>, Tx<(PIO1, SM3)>),
    }

    // doesn't let it have generics
    #[init]
    fn init(_ctx: init::Context) -> (Shared, Local) {
        let mut pac = pac::Peripherals::take().unwrap();
        let mut watchdog = rp2040_hal::Watchdog::new(pac.WATCHDOG);
        let clocks = rp2040_hal::clocks::init_clocks_and_plls(
            XTAL_FREQ_HZ,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();
        let sio = rp2040_hal::Sio::new(pac.SIO);
        let pins = rp2040_hal::gpio::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        let mut pin_dp = pins.gpio16.into_push_pull_output();
        let mut pin_dm = pins.gpio17.into_push_pull_output();

        pin_dp.set_input_override(InputOverride::Invert);
        pin_dp.set_slew_rate(OutputSlewRate::Fast);
        pin_dp.set_drive_strength(OutputDriveStrength::TwelveMilliAmps);

        pin_dm.set_input_override(InputOverride::Invert);
        pin_dm.set_slew_rate(OutputSlewRate::Fast);
        pin_dm.set_drive_strength(OutputDriveStrength::TwelveMilliAmps);

        let tx_ch = pac.DMA.split(&mut pac.RESETS).ch9;

        let (mut pio_tx, _, _, _, sm_tx) = pac.PIO0.split(&mut pac.RESETS);
        let (mut pio_rx, _, _, sm_rx, sm_eop) = pac.PIO1.split(&mut pac.RESETS);

        let pin_dp = pin_dp.id().num;
        let pin_dm = pin_dm.id().num;

        let tx_tuple = init_tx(pin_dp, &mut pio_tx, sm_tx);
        let rx_tuple = init_rx(pin_dp, &mut pio_rx, sm_rx);
        let eop_tuple = init_eop(pin_dp, pin_dm, &mut pio_rx, sm_eop);

        (
            Shared {},
            Local {
                tx_tuple,
                rx_tuple,
                eop_tuple,
            },
        )
    }
}
