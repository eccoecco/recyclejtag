/*!

    \file init.h
    \note System initialisation routines

*/

#pragma once

namespace Init
{

void InitSystem(void);

namespace Clocks
{

static constexpr uint32_t FROFrequency_Hz = 30000000;

// No need for higher resolution systick - only really used for a timeout
static constexpr uint32_t SysTickFrequency_Hz = 10;

} // namespace Clocks

} // namespace Init