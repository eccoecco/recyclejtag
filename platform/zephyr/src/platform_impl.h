
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>

#include <rjcore/rjcore.h>

uint32_t PlatformImpl_CurrentUptime(void *);
bool PlatformImpl_SetSerialMode(void *privateData, enum RJCoreSerialMode mode);
bool PlatformImpl_SetPortMode(void *, enum RJCorePortMode mode);
bool PlatformImpl_SetFeature(void *, enum RJCoreFeature feature, int action);
void PlatformImpl_ReadVoltages(void *, uint16_t *values);
void PlatformImpl_NewTapShift(void *privateData, int totalBitsToShift);
int PlatformImpl_TapShiftGPIOClock(void *, int tdi, int tms);