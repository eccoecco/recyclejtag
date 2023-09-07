#include "platform_impl.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(rjtag, LOG_LEVEL_INF);

static const struct gpio_dt_spec rjTck = GPIO_DT_SPEC_GET(DT_NODELABEL(tck), gpios);
static const struct gpio_dt_spec rjTms = GPIO_DT_SPEC_GET(DT_NODELABEL(tms), gpios);
static const struct gpio_dt_spec rjTdo = GPIO_DT_SPEC_GET(DT_NODELABEL(tdo), gpios);
static const struct gpio_dt_spec rjTdi = GPIO_DT_SPEC_GET(DT_NODELABEL(tdi), gpios);

static struct RJCorePlatform rjcorePlatform = {
    .tapShiftMode = RJCoreTapShiftMode_GPIO,
    .currentUptime = PlatformImpl_CurrentUptime,
    .transmitData = PlatformImpl_TransmitData,
    .setSerialMode = PlatformImpl_SetSerialMode,
    .setPortMode = PlatformImpl_SetPortMode,
    .setFeature = PlatformImpl_SetFeature,
    .readVoltages = PlatformImpl_ReadVoltages,
    .newTapShift = PlatformImpl_NewTapShift,
    .tapShiftComplete = PlatformImpl_TapShiftComplete,
    .tapShiftGPIO = PlatformImpl_TapShiftGPIOClock,
    .tapShiftPacket = PlatformImpl_TapShiftPacket,
    .tapShiftCustom = NULL,
};

// Platforms that implement their own PlatformImpl_TapShiftPacket() must also implement PlatformImpl_HasShiftPacket()
// but have it return true.  The weak linkage will remove these stub versions.
__attribute__((weak)) bool PlatformImpl_HasShiftPacket()
{
    return false;
}

__attribute__((weak)) int PlatformImpl_TapShiftPacket(void *, const uint8_t *buffer, int bitsToShift)
{
    LOG_ERR("Tap shift not implemented for this platform!");

    return -1;
}

struct RJCorePlatform *PlatformImpl_Init()
{
    // For now, initialise the gpio pins here, but really should be done in set port mode
    // (to pull it back into high impedance)

    gpio_pin_configure_dt(&rjTck, GPIO_OUTPUT_LOW);
    gpio_pin_configure_dt(&rjTms, GPIO_OUTPUT_LOW);
    gpio_pin_configure_dt(&rjTdi, GPIO_OUTPUT_LOW);
    gpio_pin_configure_dt(&rjTdo, GPIO_INPUT);

    if (PlatformImpl_HasShiftPacket())
    {
        LOG_INF("Using tap shift packet mode");
        rjcorePlatform.tapShiftMode = RJCoreTapShiftMode_Packet;
    }
    else
    {
        LOG_INF("Using tap shift gpio mode");
    }

    return &rjcorePlatform;
}

uint32_t PlatformImpl_CurrentUptime(void *)
{
    return k_uptime_get();
}

bool PlatformImpl_SetSerialMode(void *, enum RJCoreSerialMode mode)
{
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
    LOG_DBG("Starting to shift %d bits", totalBitsToShift);
}

void PlatformImpl_TapShiftComplete(void *privateData)
{
    LOG_DBG("Shift complete");
}

int PlatformImpl_TapShiftGPIOClock(void *, int tdi, int tms)
{
    gpio_pin_set_dt(&rjTck, 0);
    gpio_pin_set_dt(&rjTdi, tdi != 0);
    gpio_pin_set_dt(&rjTms, tms != 0);

    gpio_pin_set_dt(&rjTck, 1);

    return gpio_pin_get_dt(&rjTdo);
}
