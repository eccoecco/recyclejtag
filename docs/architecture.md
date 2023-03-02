# Recycle JTAG: Architecture

The firmware is divided into two halves:
1. The **Core** which does simple protocol handling, and provides common support functions that are useful across multiple platforms.
1. The **Platform Implementation** which contains platform specific calls (e.g. to receive/transmit data over the serial port).

In many ways, this is a common split.

## Core

The core handles talking with OpenOCD, and implements the [subset of the Bus Pirate protocol that OpenOCD uses](https://github.com/openocd-org/openocd/blob/master/src/jtag/drivers/buspirate.c).

For now, we only aim for JTAG, but in future, maybe SWD?

The platform may request the core to cache blocks of data for processing, or handle things on a byte by byte basis.

For example, on platforms with a real serial port (e.g. with an FTDI chip) that process data faster than the baud rate, it might keep things simple by processing data as fast as it comes.

On the other hand, on platforms with USB inbuilt that provide a CDC ACM driver (virtual COM ports), it would be better to process the chunks of data that are sent and send things back in bulk, rather than one byte at a time.

It really is up to the platform, so the core is designed with a bit of flexibility in mind to support these scenarios.

A lot of the platforms that I'm targetting support C++17 now (but not necessarily the standard C++ libraries), so I'll use that.

## Platform Implementation

The platform is responsible for setting up the hardware as appropriate, whether it is simple bit bashing on GPIOs, or using an exciting combination of hardware peripherals to achieve high JTAG speeds.