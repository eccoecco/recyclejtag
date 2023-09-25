#include <zephyr/kernel.h>

#include "platform_stm32.h"

#ifdef RJTAG_STM32_USE_GPIO

#pragma message("Using STM32 specific GPIO bit bashing")

#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(rjtag, LOG_LEVEL_INF);

#include "platform_impl.h"

#include <cstddef>

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

// By assuming all pins are on the same port, we can just do this once:
constexpr auto rjPortAddress = DT_REG_ADDR(DT_GPIO_CTLR(DT_NODELABEL(tck), gpios));

// STM32s only define GPIOx_BASE if they're present on the device.  Use the BASE address
// before they're cast to GPIO_TypeDef * as we can directly compare ints with DT_REG_ADDR.
constexpr int validPortAddresses[] = {
#ifdef GPIOA_BASE
    GPIOA_BASE,
#endif
#ifdef GPIOB_BASE
    GPIOB_BASE,
#endif
#ifdef GPIOC_BASE
    GPIOC_BASE,
#endif
#ifdef GPIOD_BASE
    GPIOD_BASE,
#endif
#ifdef GPIOE_BASE
    GPIOE_BASE,
#endif
#ifdef GPIOF_BASE
    GPIOF_BASE,
#endif
#ifdef GPIOG_BASE
    GPIOG_BASE,
#endif
#ifdef GPIOH_BASE
    GPIOH_BASE,
#endif
#ifdef GPIOI_BASE
    GPIOI_BASE,
#endif
#ifdef GPIOJ_BASE
    GPIOJ_BASE,
#endif
#ifdef GPIOK_BASE
    GPIOK_BASE,
#endif
};

template <int testAddress> constexpr bool ensureValidPortAddress()
{
    for (auto portAddress : validPortAddresses)
    {
        if (testAddress == portAddress)
        {
            return true;
        }
    }

    return false;
}

// Just a sanity check to make sure that rjPortAddress points to a known GPIOx_BASE address
static_assert(ensureValidPortAddress<rjPortAddress>(), "Unable to find GPIO controller address in port addresses");

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
            // I need to tune this better, because different STM32s will run at
            // different speeds.  Maybe I should put this into a KConfig option?
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