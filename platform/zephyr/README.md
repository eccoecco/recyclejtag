# Zephyr Based Board for RJtag

See [here](../../docs/zephyr.md) for more in-depth documentation.

## Details for Blackpill F411CE

### Implementation: Software Bit Bashing (Functional)

The generic Zephyr based GPIO was too slow.  I directly access the
GPIO registers in software and bit bash the JTAG signals.  This
actually works reliably well, and relatively fast as well.  I believe
it's around 600kb/s or so, which is a bit surprising.  It's fast enough
that GDB doesn't time out on some of the longer instructions.

This is in comparison to just using the Zephyr GPIO API, which is
guaranteed cross platform, but is slow enough that OpenOCD/GDB times
out waiting for the JTAG command to finish.

### Implementation: DMA to GPIO Registers (Non-Functional)

I tried using Timer 1 + DMA2 to write directly to the GPIO registers,
and a second timer + DMA2 to read directly from the GPIO registers.

However, it didn't seem reliable!  For example, the second timer
was scheduled to trigger the DMA read from GPIO *after* the first
timer had set the TCK pin (rising edge sample), but examining the
result of IDR showed that TCK was not always set.

TDI and TMS would also mostly be correct, but sometimes would be
corrupted as well.  I slowed down the timers, but this didn't make
as much of a difference as I had hoped.

I suspect some weird interaction with bus delays and the DMA
controller, but I'm unsure.  Either way, corrupted data was either
sent or received.  I found an errata detailing potential data
corruption when using DMA2 and GPIO, and implemented the suggested
workaround, but that while the workaround did indeed clear one of
the FIFO errors that the DMA flagged, it did not fix the overall
data transfer behaviour.

### Implementation: Mixing Timer and SPI Peripherals (Development)

I have resisted doing this, because even though it should be more
reliable, this is sort of heavy abuse of the peripherals, and makes
me wish that I had a CPLD/FPGA or some extra glue logic available.
But, I don't, so here we are.

The main issue is that the SPI peripherals on the STM32F411 are:

* Only have MOSI/MISO - no dual output (need two outputs for TMS
  and TDI).
* Restricted to 8-bit or 16-bit transfers (JTAG uses arbitrary
  bit lengths)

To deal with requiring two outputs (TMS/TDI) and one input (TDO)
alongside the clock, we can put two SPI peripherals in parallel,
and just configure one of them to ignore an input.  These SPI
peripherals are both in slave mode, because we need special
consideration when generating the clock signal.

```
                +--------+
                |       MISO --> TDI
SPI_SCK ----+---| SPI1   |
            |   |       MOSI <-- TDO
            |   +--------+
            |
            |   +--------+
            +---| SPI2  MISO --> TMS
                +--------+
```

SPI1 and SPI2 are chosen as their MISO/MOSI pins are both on PORTB
of the STM32 board, which allows the software bitbashing to work
(as it writes to `PORTB->BSRR`).

Since the SPI peripherals are restricted to 8 or 16 bits, the
clock that feeds them must also be in multiples of 8 or 16 pulses.
Probably 8, since OpenOCD sends data to the device in bytes.

Howevever, TCK needs an exact number of clock pulses that are
commonly *not* a multiple of 8.  Hence, the trickiness.

Short of bit bashing SCK and TCK and transferring it via DMA
like the previous solution (which was sort of unreliable...),
I am going to generate it via a timer.

Previously, I was going to cascade timers, but the extra complexity
isn't really worth it and I might as well just use TIMER1's
repetition counter (even though it is only 8 bits).

The concept is that TIMER1 can provide both TCK and SCK, and will
do so while while there are full bytes left to send.  When an
odd number of bits are left over, then TIMER1 will clock out only
the leftover number of bits, and then be reconfigured to only clock
out the number of bits necessary to make SCK round up to 8 (while
holding TCK idle), so that the SPI periperhals continue working.

For example, if we only want to send 43 bits of data:

* TIMER1 is configured to send 43 rising edges on both TCK and SCK
* TIMER1 is then reconfigured to send 5 rising edges on SCK, while
  TCK remains idle.

Since the repetition counter used to make this happen is only 8 bits
wide, larger transactions can only happen in 256 bit chunks.

The SPI periperhals will need DMA set up for them.  Hopefully we
won't experience the same data corruption as we did as in the previous
setup.

#### Provisional Peripheral Choices

Referring to the board pin mapping of the [Blackpill F411CE](https://docs.zephyrproject.org/latest/boards/arm/blackpill_f411ce/doc/index.html),
I'm going to try and keep all the JTAG pins on PORTB/left hand side of
the board.  Mainly so that I can easily swap it back to bitbashing mode
(which assumes all pins are on a single port) without much hassle.

The SCK pins are not considered JTAG signals, and I'm going to put the
SCK + Timer pulses on the right hand side.

This means using:
* TMS: SPI1 - MISO1 - PB4 (left hand side)
* TDI: SPI2 - MISO2 - PB14 (left hand side)
* TDO: SPI2 - MOSI2 - PB15 (left hand side)
* TCK: TIMER1 - CH1N - PB13 (left hand side)
* SPI1 - SCK1  - PA5 (right hand side)
* Source of SCK1: TIMER1 - CH2N - PB0 (right hand side, three pins up from PA5)
* SPI2 - SCK2 - PB10 (right hand side)
* Source of SCK2: TIMER1 - CH3N - PB1 (right hand side, two pins down from PB10)
