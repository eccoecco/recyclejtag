/*!
    \file platform_stm32.h
    \brief Examines the device tree to see which stm32 implementation (if any)
           should be used
*/

#pragma once

#include <zephyr/kernel.h>

#ifdef CONFIG_SOC_FAMILY_STM32
// Only knows how to deal with STM32s

#if DT_NODE_EXISTS(DT_NODELABEL(tck)) && DT_NODE_EXISTS(DT_NODELABEL(tms)) && DT_NODE_EXISTS(DT_NODELABEL(tdi)) &&     \
    DT_NODE_EXISTS(DT_NODELABEL(tdo))
// Device tree has the jtag pins hopefully assigned
#define RJTAG_STM32_USE_GPIO
#elif DT_NODE_EXISTS(DT_NODELABEL(rjtagtck)) && DT_NODE_EXISTS(DT_NODELABEL(rjtagsck))
// Device tree has both tck and sck defined - hopefully as timers
#define RJTAG_STM32_USE_TIMER_SPI
#endif

#endif