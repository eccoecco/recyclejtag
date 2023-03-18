#include <LPC845.h>

#include "init.h"
#include "systick.h"

namespace
{
uint32_t Uptime_ms = 0;
}

namespace SystemTick
{

uint32_t CurrentTick()
{
    return Uptime_ms;
}

} // namespace SystemTick

extern "C"
{

void SysTick_Handler(void)
{
    constexpr uint32_t msIncrement = 1000 / Init::Clocks::SysTickFrequency_Hz;
    Uptime_ms += msIncrement;
}
}