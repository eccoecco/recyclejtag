#include "platform_impl.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(rjtag, LOG_LEVEL_INF);

static const struct gpio_dt_spec rjTck = GPIO_DT_SPEC_GET(DT_NODELABEL(tck), gpios);
static const struct gpio_dt_spec rjTms = GPIO_DT_SPEC_GET(DT_NODELABEL(tms), gpios);
static const struct gpio_dt_spec rjTdo = GPIO_DT_SPEC_GET(DT_NODELABEL(tdo), gpios);
static const struct gpio_dt_spec rjTdi = GPIO_DT_SPEC_GET(DT_NODELABEL(tdi), gpios);

void PlatformImpl_Init()
{
    // For now, initialise the gpio pins here, but really should be done in set port mode
    // (to pull it back into high impedance)

    gpio_pin_configure_dt(&rjTck, GPIO_OUTPUT_LOW);
    gpio_pin_configure_dt(&rjTms, GPIO_OUTPUT_LOW);
    gpio_pin_configure_dt(&rjTdi, GPIO_OUTPUT_LOW);
    gpio_pin_configure_dt(&rjTdo, GPIO_INPUT);
}

uint32_t PlatformImpl_CurrentUptime(void *)
{
    return k_uptime_get();
}

bool PlatformImpl_SetSerialMode(void *, enum RJCoreSerialMode mode)
{
    LOG_INF("Setting serial mode: %d", (int)mode);

    // USB interface accepts all bauds
    return true;
}

bool PlatformImpl_SetPortMode(void *, enum RJCorePortMode mode)
{
    switch (mode)
    {
    case RJCorePortMode_HighImpedance:
        break;
    case RJCorePortMode_PushPull:
        break;
    case RJCorePortMode_OpenDrain:
        break;
    default:
        LOG_ERR("Unknown port mode");
        return false;
    }

    return true;
}

bool PlatformImpl_SetFeature(void *, enum RJCoreFeature feature, int action)
{
    // No real feature supported at this prototype stage
    return true;
}

void PlatformImpl_ReadVoltages(void *, uint16_t *values)
{
    for (int i = 0; i < 4; ++i)
    {
        values[i] = 0;
    }
}

void PlatformImpl_NewTapShift(void *privateData, int totalBitsToShift)
{
    LOG_INF("Starting to shift %d bits", totalBitsToShift);
}

void PlatformImpl_TapShiftComplete(void *privateData)
{
    LOG_INF("Shift complete");
}

int PlatformImpl_TapShiftGPIOClock(void *, int tdi, int tms)
{
    gpio_pin_set_dt(&rjTck, 0);
    gpio_pin_set_dt(&rjTdi, tdi != 0);
    gpio_pin_set_dt(&rjTms, tms != 0);

    gpio_pin_set_dt(&rjTck, 1);

    return gpio_pin_get_dt(&rjTdo);
}
