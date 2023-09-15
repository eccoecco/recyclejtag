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
I am going to generate it via cascaded timers.

I haven't yet assigned specific timers to the chain yet, but
the concept is:

```
        +-- TIMs1 -> TIMs2 --> SCK
        |
TIMa ---+
        |
        +-- TIMj1 -> TIMj2 --> TCK
```

Both SCK and TCK are *synchronised*

TIMs2 and TIMj2 are configured to have the exact same configuration
(same prescaler, same counter reset to 0 prior to start, etc) so
that if they are enabled at the same time, they will generate the
exact same waveforms via their channel compare outputs.  These are
configured in gated input mode, so that the TRGO of TIMs1/TIMj1
will enable/disable the TIMs2/TIMj2 counters.

TIMs1 and TIMj1 are configured in one shot mode, and configured to
only run as long as necessary for TIMs2/TIMj2 to generate the number
of pulses required.

For example, if SCK needed 32 pulses, then TIMs1 would be configured
to run for (32 x SCK period) cycles before stopping (or its TRGO would
deassert after that many cycles.  However, if TCK only needed 30 pulses,
then TIMj1 would be configured for (30 x TCK/SCK period) cycles.

TIMs1 and TIMj1 are both run in gated mode, where TIMa is used to
ensure that TIMs1 and TIMj1 both start at the exact same time, to ensure
that TIMs2 and TIMj2 both start at the exact same time.

This silly 5 timer and 2 SPI peripheral connection is used to try and
speed up JTAG transactions.

Obviously, the SPI periperhals will need DMA set up for them.  Hopefully
we won't experience the same data corruption as we did as in the previous
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
* TCK: TIMER4 - CH1 - PB6 (left hand side)
* SPI1 - SCK1  - PA5 (right hand side)
* Source of SCK1: TIMER3 - CH1 - PA6 (right hand side, neighbours PA5 - can use a jumper)
* SPI2 - SCK2 - PB10 (right hand side)
* Source of SCK2: TIMER3 - CH4 - PB1 (right hand side, two pins away from PB10 - have to use a wire)

Given the timer's internal connections of ITRn and TRIGO, we thus have:

```
          +-> TIMER5 --> TIMER3 -> SCK
          |   ITR0       ITR2
TIMER2 ---+
          |
          +-> TIMER1 --> TIMER4 -> TCK
              ITR1       ITR0
```

I'm constrained by pin mapping to use TIMER3 and TIMER4 as the outputs.
Remember I want to keep TCK on PORTB, and while I technically
could use PB3, that's hilarious the board's JTDO pin, which I think
would be hilarious to try and preserve.  I've had to replace JTRST with
MISO1 (because I can't find another MISO pin on PORTB), but I think that
JTRST is sometimes optional, so not too fussed.

Timer 3 is the only timer out of Timers 2, 3, and 4 to accept Timer 5
as its input, and so Timer 5 was placed there due to the constraints.
However, Timer 5 does *not* accept Timer 1 as its inputs, which meant
that Timer 1 was linked to Timer 4.  Timer 5 and Timer 1 does both
accept Timer 2 as its trigger input though, and thus the layout was
decided.