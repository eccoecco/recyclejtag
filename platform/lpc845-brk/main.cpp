#include <cstdint>

#include <LPC845.h>

#include "gpio.h"
#include "init.h"
#include "systick.h"
#include "uart.h"

#include <rjcore/rjcore.h>

#define BENCHMARK_TAP_SHIFT 1

static void startRjCore();

int main()
{
    Init::InitSystem();

    startRjCore();

    return 0;
}

#if BENCHMARK_TAP_SHIFT

// Set as global, so that you can pause the debugger on the main loop and just
// examine it in the variable pane
struct
{
    int totalBitsShifted = 0;
    int debugShiftCounter = 0;
    uint32_t startTime = 0;
    uint32_t endTime = 0;
    bool inProgress = false;
} tapShiftBenchmark;

#endif

namespace
{
namespace PlatformImpl
{

uint32_t CurrentUptime(void *)
{
    return SystemTick::CurrentTick();
}

void TransmitData(void *, const void *buffer, size_t length)
{
    auto data = static_cast<const uint8_t *>(buffer);

    for (; length > 0; ++data, --length)
    {
        Uart::WriteCharacter(*data);
    }
}

bool SetSerialMode(void *, enum RJCoreSerialMode)
{
    // Does not support 1Mbps - only 115,200
    return false;
}

// Implementation that sets the port mode
bool SetPortMode(void *, enum RJCorePortMode portMode)
{
    switch (portMode)
    {
    case RJCorePortMode_HighImpedance:
        // TODO: Write to DIRCLR for the GPIO pins
        break;
    case RJCorePortMode_PushPull:
        // TODO: Write to DIRSET for the GPIO pins
        break;
    case RJCorePortMode_OpenDrain:
        // TODO: Support open drain mode
        break;
    default:
        return false;
    }

    return true;
}

bool SetFeature(void *, enum RJCoreFeature, int)
{
    // TODO: Set feature
    return true;
}

void ReadVoltages(void *, uint16_t *values)
{
    values[0] = 0;
    values[1] = 0;
    values[2] = 0;
    values[3] = 0;
}

void NewTapShift(void *, int totalBitsToShift)
{
#if BENCHMARK_TAP_SHIFT
    if (totalBitsToShift < tapShiftBenchmark.totalBitsShifted)
    {
        return;
    }

    tapShiftBenchmark.debugShiftCounter = 0;
    tapShiftBenchmark.totalBitsShifted = totalBitsToShift;
    tapShiftBenchmark.startTime = SystemTick::CurrentTick();
    tapShiftBenchmark.inProgress = true;
#else
    (void)totalBitsToShift;
#endif
}

void TapShiftComplete(void *)
{
#if BENCHMARK_TAP_SHIFT
    if (tapShiftBenchmark.inProgress)
    {
        tapShiftBenchmark.endTime = SystemTick::CurrentTick();
        tapShiftBenchmark.inProgress = false;
    }
#endif
}

#if LPC845_JTAG_USE_GPIO
// Generate a clock cycle (TCK low, TDI = tdi, TMS = tms, pause, TCK high, read TDO)
// Returns 0 if TDO low, 1 if TDO high
int TapShiftGPIOClockCycle(void *, int tdi, int tms)
{
    Gpio::SetState<Gpio::Mapping::JtagTck>(false);
    Gpio::SetState<Gpio::Mapping::JtagTdi>((tdi != 0));
    Gpio::SetState<Gpio::Mapping::JtagTms>((tms != 0));

    Gpio::SetState<Gpio::Mapping::JtagTck>(true);
    return Gpio::GetState<Gpio::Mapping::JtagTdo>();
}
#endif

#if LPC845_JTAG_USE_SPI

void ShiftViaSPI(uint8_t tdi, uint8_t tms, int bitsToShift)
{
    constexpr uint32_t defaultSpiConfigValue =
        SPI_CFG_ENABLE(1) | SPI_CFG_MASTER(1) | SPI_CFG_LSBF(1) | SPI_CFG_CPHA(1) | SPI_CFG_CPOL(1);

    int totalBitsShifted = 0;
    bool currentOutputTms = false;
    SPI0->CFG = defaultSpiConfigValue;
    uint8_t tdo = 0;

#if BENCHMARK_TAP_SHIFT
    if (tapShiftBenchmark.inProgress)
    {
        tapShiftBenchmark.debugShiftCounter += bitsToShift;
    }
#endif

    while (bitsToShift > 0)
    {
        int currentPolarity = tms & 1;
        int consecutiveBits = 0;

        for (int bitIndex = 0; (bitIndex < bitsToShift) && ((tms & 1) == currentPolarity);
             ++bitIndex, ++consecutiveBits)
        {
            tms >>= 1;
        }

        if ((currentPolarity != 0) != currentOutputTms)
        {
            currentOutputTms = !currentOutputTms;
            SPI0->CFG = defaultSpiConfigValue | (currentOutputTms ? SPI_CFG_SPOL0(1) : 0);
        }

        SPI0->TXDATCTL = SPI_TXDATCTL_TXDAT(tdi) | SPI_TXDATCTL_LEN(consecutiveBits);

        bitsToShift -= consecutiveBits;
        tdi >>= consecutiveBits;

        while ((SPI0->STAT & SPI_STAT_RXRDY_MASK) == 0)
        {
            // Shouldn't take too long, just spin a few times
            __NOP();
        }

        tdo |= static_cast<uint8_t>(SPI0->RXDAT & 0xFF) << totalBitsShifted;

        totalBitsShifted += consecutiveBits;
    }

    Uart::WriteCharacter(tdo);

    {
        auto tick = SystemTick::CurrentTick();

        while ((SystemTick::CurrentTick() - tick) < 10)
        {
            __NOP();
        }
    }
}

// Processes a packet of data - multiple clock cycles.
// Returns 0 if success, -1 if error.
int TapShiftPacketSPI(void *, const uint8_t *buffer, int bitsToShift)
{
    for (; bitsToShift > 0; bitsToShift -= 8, buffer += 2)
    {
        uint8_t tdi = buffer[0];
        uint8_t tms = buffer[1];
        int bitsForData = (bitsToShift > 8) ? 8 : bitsToShift;

        ShiftViaSPI(tdi, tms, bitsForData);
    }

    return 0;
}
#endif

} // namespace PlatformImpl
} // namespace

void startRjCore()
{
    constexpr RJCorePlatform_TapShiftGPIOClockCycle tapShiftGPIO =
#if LPC845_JTAG_USE_GPIO
        PlatformImpl::TapShiftGPIOClockCycle;
    constexpr RJCoreTapShiftMode preferredMode = RJCoreTapShiftMode_GPIO;
#else
        nullptr;
#endif
    constexpr RJCorePlatform_TapShiftPacket tapShiftPacket =
#if LPC845_JTAG_USE_SPI
        PlatformImpl::TapShiftPacketSPI;
    constexpr RJCoreTapShiftMode preferredMode = RJCoreTapShiftMode_Packet;
#else
        nullptr;
#endif

    RJCoreHandle rjcoreHandle;
    RJCorePlatform platform{
        .tapShiftMode = preferredMode,
        .currentUptime = PlatformImpl::CurrentUptime,
        .transmitData = PlatformImpl::TransmitData,
        .setSerialMode = PlatformImpl::SetSerialMode,
        .setPortMode = PlatformImpl::SetPortMode,
        .setFeature = PlatformImpl::SetFeature,
        .readVoltages = PlatformImpl::ReadVoltages,
        .newTapShift = PlatformImpl::NewTapShift,
        .tapShiftComplete = PlatformImpl::TapShiftComplete,
        .tapShiftGPIO = tapShiftGPIO,
        .tapShiftPacket = tapShiftPacket,
        .tapShiftCustom = nullptr, // TODO: Or maybe use custom mode?
    };

    RJCore_Init(&rjcoreHandle, &platform, nullptr);

    while (true)
    {
        int readCharacter = Uart::ReadCharacter();
        if (readCharacter < 0)
        {
            continue;
        }

        auto value = static_cast<uint8_t>(readCharacter);

        RJCore_NotifyDataReceived(&rjcoreHandle, &value, 1);
    }
}
