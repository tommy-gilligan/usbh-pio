# Links

- [Original C Implementation](https://github.com/sekigon-gonnoc/Pico-PIO-USB)
- [My RP-HAL fork](https://github.com/tommy-gilligan/rp-hal)
- [usbh-defmt](https://github.com/tommy-gilligan/usbh-defmt)
- [usbh](https://github.com/nilclass/usbh)

## USB

- [USB in a Nutshell](https://www.beyondlogic.org/usbnutshell/usb1.shtml)
- [rusb](https://crates.io/crates/rusb)
- [libusb1-sys](https://crates.io/crates/libusb1-sys)

## PIO

- Run and understand C examples
- Run and understand Rust examples
- Work out a good way to test PIO programs in isolation
- [RP2040 Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)
- [RP2040 PIO Emulator](https://github.com/soundpaint/rp2040pio)
- [Adafruit Tutorial](https://learn.adafruit.com/intro-to-rp2040-pio-with-circuitpython/overview)

## Simplifying Reference Code
Simplifying reference code (`Pico-PIO-USB`) is just my way of reducing what I have to implement as an initial solution.  This simplification can be done manually by careful inspection of code.  I would like to make it easier for myself though.

Static analysis on embedded C code is tricky enough that I thought I might try to implement my own quicky and dirty dynamic analysis for eliminating unused code.  Any time a function is called, it pushes its identifier to a set.  Periodically the set is written to flash or UART.

The pushing to the set could live inside a macro.  This macro could also emit information during compilation to make it easier to then create the set Uncalled Instrumented Functions = (All Instrumented Functions - Called Instrumented Functions).
