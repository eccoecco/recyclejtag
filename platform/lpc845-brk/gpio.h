/*!

    \file gpio.h
    \note Manual GPIO toggling

*/

#pragma once

#include <LPC845.h>

#include <array>
#include <cstdint>

namespace Gpio
{

enum class PinPolarity
{
    ActiveHigh,
    ActiveLow,
};

enum class PinDirection
{
    Output,
    Input,
};

namespace Mapping
{

static constexpr int MaxPorts = 2;        //!< Maximum number of ports on the device
static constexpr int MaxPinsPerPort = 32; //!< Maximum number of pins per port on the device

using PortPinBitMask = uint32_t; //!< The data type used for storing a bitmask of pins per port
using PortBitMasks = std::array<PortPinBitMask, MaxPorts>;

static_assert(MaxPinsPerPort <= (sizeof(PortPinBitMask) * 8),
              "Internal bitmask data type must be able to store MaxPinsPerPort bits of data");

struct PinMap
{
    const int Port;               // 0 or 1
    const int Pin;                // 0-31
    const PinPolarity Polarity;   // Low or High
    const PinDirection Direction; // Input or Output (no bidirectional at this time)

    constexpr PinMap(int port, int pin, PinPolarity polarity, PinDirection pinDirection)
        : Port{port}, Pin{pin}, Polarity{polarity}, Direction{pinDirection}
    {
    }

    constexpr bool IsActiveHigh() const
    {
        return Polarity == PinPolarity::ActiveHigh;
    }

    constexpr uint32_t BitMask() const
    {
        return 1 << Pin;
    }
};

constexpr PinMap JtagTck{0, 16, PinPolarity::ActiveHigh, PinDirection::Output};
constexpr PinMap JtagTdi{0, 17, PinPolarity::ActiveHigh, PinDirection::Output};
constexpr PinMap JtagTms{0, 18, PinPolarity::ActiveHigh, PinDirection::Output};
constexpr PinMap JtagTdo{0, 19, PinPolarity::ActiveHigh, PinDirection::Input};

constexpr PinMap DebugRed{1, 2, PinPolarity::ActiveLow, PinDirection::Output};
constexpr PinMap DebugGreen{1, 0, PinPolarity::ActiveLow, PinDirection::Output};
constexpr PinMap DebugBlue{1, 1, PinPolarity::ActiveLow, PinDirection::Output};

constexpr PinMap AllPins[] = {
    JtagTck, JtagTdi, JtagTms, JtagTdo, DebugRed, DebugGreen, DebugBlue,
};

} // namespace Mapping

template <Mapping::PinMap pinMap> inline void SetState(bool setActive)
{
    if (setActive == pinMap.IsActiveHigh())
    {
        GPIO->SET[pinMap.Port] = pinMap.BitMask();
    }
    else
    {
        GPIO->CLR[pinMap.Port] = pinMap.BitMask();
    }
}

template <Mapping::PinMap pinMap> inline void Toggle()
{
    GPIO->NOT[pinMap.Port] = pinMap.BitMask();
}

template <Mapping::PinMap pinMap> inline bool GetState()
{
    return (GPIO->PIN[pinMap.Port] & pinMap.BitMask()) != 0;
}

} // namespace Gpio