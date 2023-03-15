# Recycle JTAG: LPC845-BRK

The LPC845-BRK is a very cheap, but capable, development board for the NXP LPC845 microcontroller.

An advantage of the LPC845 is the onboard CMSIS-DAP debug probe, which also doubles as a virtual serial port.  Now, because the virtual serial port is hooked up to the LPC845's UART, as opposed to being an on-chip USB peripheral, this port only supports the "normal" baud rate of 115,200, as opposed to the fast serial port of 1Mbit/s.

In this application, another advantage of the LPC845 itself is that its SPI port allows transfers from 1-16 bits, which is very useful since Jtag is a bit oriented protocol, as opposed to being byte oriented.

## Initial Setup

### Obtain arm-none-eabi toolchain

Grab a recent version of the toolchain - I get it from [ARM](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain).  I believe that some Linux distributions also have it available.  The toolchain from ARM tends to be one of the most recent; sometimes distribution provided toolchains are a bit dated.

Note that the toolchain does not have to be in the path.

I personally installed the toolchain so that `arm-none-eabi-gcc` can be found in `${HOME}/opt/arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-gcc`

### Grab NXP's MCUXpresso SDK

You'll need to grab [NXP's MCUXpresso SDK](https://mcuxpresso.nxp.com/en/welcome).  NXP requires you to configure what you want to download, and here are the settings that I use:

* OS: Linux or Windows
* Middleware: None
* Toolchain: GCC ARM Embedded 10-2021.10 (even though the version of gcc we're using is newer, that should be okay)

In this SDK are useful files, such as LPC845 header files, startup scripts, and linker scripts.  It also integrates nicely with CMake.

I have installed this so that the SDK manifest can be found in `${HOME}/mcuxpresso/sdk-2.13-lpc845/LPC845_manifest_v3_10.xml`