#include "platform_impl.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(rjtag, LOG_LEVEL_INF);

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
}

int PlatformImpl_TapShiftGPIOClock(void *, int tdi, int tms)
{
    (void)tdi;
    (void)tms;
    return 0;
}
