#include <cstdint>

#include <LPC845.h>

#include "gpio.h"
#include "init.h"
#include "systick.h"
#include "uart.h"

#include <rjcore/rjcore.h>

static void startRjCore();

int main()
{
    Init::InitSystem();

    startRjCore();

    return 0;
}

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
    // Since only doing GPIO for now, no need to track total bits to shift
    (void)totalBitsToShift;
}

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

} // namespace PlatformImpl
} // namespace

void startRjCore()
{
    RJCoreHandle rjcoreHandle;
    RJCorePlatform platform{
        .tapShiftMode = RJCoreTapShiftMode_GPIO,
        .currentUptime = PlatformImpl::CurrentUptime,
        .transmitData = PlatformImpl::TransmitData,
        .setSerialMode = PlatformImpl::SetSerialMode,
        .setPortMode = PlatformImpl::SetPortMode,
        .setFeature = PlatformImpl::SetFeature,
        .readVoltages = PlatformImpl::ReadVoltages,
        .newTapShift = PlatformImpl::NewTapShift,
        .tapShiftGPIO = PlatformImpl::TapShiftGPIOClockCycle,
        .tapShiftPacket = nullptr, // TODO: Use packet mode when doing hardware SPI?
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
