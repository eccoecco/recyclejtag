# Recycle Jtag

Recycling old dev boards into JTAG dongles with [OpenOCD](https://openocd.org/).  Rather than implement a new protocol, let's recycle and use [Bus Pirate](http://dangerousprototypes.com/docs/Bus_Pirate) [support in OpenOCD](https://openocd.org/doc/html/Debug-Adapter-Configuration.html).

## Currently supported platforms

This is basically a list of small development boards that I have, which I think are fast enough and convenient enough to turn into JTAG dongles.
* Virtual development - [Environment setup and details](docs/virtualdev.md)
* [LPC845-BRK](https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/general-purpose-mcus/lpc800-arm-cortex-m0-plus-/lpc845-breakout-board-for-lpc84x-family-mcus:LPC845-BRK) - [Environment setup and details](docs/lpc845-brk.md)
* A Zephyr based adapter, with my initial implementation based on a random [STM32F411](https://www.st.com/en/microcontrollers-microprocessors/stm32f411ce.html) development kit (probably made by a 3rd party) - [Environment setup and details](docs/zephyr.md)

## Architecture

Further details about the architecture can be found [here](docs/architecture.md).

## Motivation

Over the years, I've picked up quite a few development boards, but oddly enough, never really bought a general purpose JTAG dongle.  I recently bought a few cheap dev kits that can be flashed over their onboard serial port, but can only be debugged via JTAG.

"Oh dear," I thought, "Gpio and printf debugging is nice, but... some days, I like breakpoints and examining processor peripheral registers directly.  I have all these dev boards all around the place.  I wonder if I can make a JTAG dongle out of it?"

And after some searching, I found that OpenOCD supports Bus Pirate over _serial port_ (important because some of my older dev boards only have serial ports - whether they be virtual, ftdi, or actual RS232), and so this is the protocol that I've chosen to be compatible with.

Now, if I were to be only debugging ARM Cortexes, then I'd probably just use [Black Magic Probe](https://black-magic.org/), or [DAPLink](https://github.com/ARMmbed/DAPLink/blob/main/README.md) itself.  However, there are a few other architectures that I've got lying around that I want JTAG debuggers for, hence this project.

## Random Notes

As at time of writing, OpenOCD for Windows (MinGW) does not support Bus Pirate.  I am testing a patch [here](misc/openocd-windows.patch) that, once I'm satisfied, I will submit to OpenOCD.  Hopefully, we'll get somewhere with this!

I personally use [WSL2 with usbipd](https://learn.microsoft.com/en-us/windows/wsl/connect-usb) to forward USB serial ports to Linux, and run OpenOCD there, which does support Bus Pirate.
