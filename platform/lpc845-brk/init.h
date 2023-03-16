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

}

} // namespace Init