#include <zephyr/kernel.h>

#ifdef CONFIG_SOC_FAMILY_STM32

#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(rjtag, LOG_LEVEL_INF);

#include "platform_impl.h"

namespace
{

constexpr struct gpio_dt_spec rjTck = GPIO_DT_SPEC_GET(DT_NODELABEL(tck), gpios);
constexpr struct gpio_dt_spec rjTms = GPIO_DT_SPEC_GET(DT_NODELABEL(tms), gpios);
constexpr struct gpio_dt_spec rjTdo = GPIO_DT_SPEC_GET(DT_NODELABEL(tdo), gpios);
constexpr struct gpio_dt_spec rjTdi = GPIO_DT_SPEC_GET(DT_NODELABEL(tdi), gpios);

// For an MVP, let's just assume that all pins are on the same port.  There's no technical
// reason that they have to be, but it does just simplify the code a lot.
static_assert(rjTck.port == rjTms.port, "All jtag pins must be on the same port (TCK/TMS differs)");
static_assert(rjTck.port == rjTdo.port, "All jtag pins must be on the same port (TCK/TDO differs)");
static_assert(rjTck.port == rjTdi.port, "All jtag pins must be on the same port (TCK/TDI differs)");

// static_assert(rjTck.port
constexpr auto rjPortAddress = DT_REG_ADDR(DT_GPIO_CTLR(DT_NODELABEL(tck), gpios));

// TODO: Create a constexpr array of all the valid GPIO port addresses, and then ensure that
// rjPortAddress is within that array.
// Do something like:
//      constexpr uintptr_t address[] = {
//      #ifdef GPIOA_BASE
//          GPIOA_BASE,
//      #endif
//      #ifdef GPIOB_BASE
//          GPIOB_BASE,
//      #endif
//      ... etc ...
//      };
// because each STM32s has different GPIO ports, and not always consecutive

} // namespace

extern "C"
{

bool PlatformImpl_HasShiftPacket()
{
    return true;
}

int PlatformImpl_TapShiftPacket(void *privateData, const uint8_t *buffer, int bitsToShift)
{
    auto gpioController = reinterpret_cast<GPIO_TypeDef *>(rjPortAddress);
    constexpr uint32_t tckSetMask = (1 << rjTck.pin);
    constexpr uint32_t tckClearMask = (0x1'0000 << rjTck.pin);
    constexpr uint32_t tdiSetMask = (1 << rjTdi.pin);
    constexpr uint32_t tdiClearMask = (0x1'0000 << rjTdi.pin);
    constexpr uint32_t tmsSetMask = (1 << rjTms.pin);
    constexpr uint32_t tmsClearMask = (0x1'0000 << rjTms.pin);
    constexpr uint32_t tdoInputMask = (1 << rjTdo.pin);

    for (; bitsToShift > 0; bitsToShift -= 8)
    {
        uint8_t tdi = buffer[0];
        uint8_t tms = buffer[1];
        uint8_t tdo = 0;

        buffer += 2;

        const int totalBitsThisShift = (bitsToShift > 8) ? 8 : bitsToShift;

        for (int bitsLeft = totalBitsThisShift; bitsLeft > 0; --bitsLeft, tdi >>= 1, tms >>= 1)
        {
            uint32_t bsrr = tckClearMask;
            bsrr |= ((tdi & 1) != 0) ? tdiSetMask : tdiClearMask;
            bsrr |= ((tms & 1) != 0) ? tmsSetMask : tmsClearMask;

            gpioController->BSRR = bsrr;

            // just *some* delay, because most JTAG ports can't toggle at 50-100MHz
            for (int delay = 3; delay > 0; --delay)
            {
                __NOP();
            }

            gpioController->BSRR = tckSetMask;

            tdo >>= 1;
            tdo |= ((gpioController->IDR & tdoInputMask) != 0) ? 0x80 : 0x00;
        }

        tdo >>= (8 - totalBitsThisShift);

        PlatformImpl_TransmitData(privateData, &tdo, 1);
    }

    return 0;
}
}

#endif