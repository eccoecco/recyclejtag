#include <cstdint>

#include <LPC845.h>

#include "init.h"

namespace
{

namespace Internal::LPC845
{

/*! \brief Hardware ROM address for changing FRO clock frequency */
static constexpr uint32_t clockFROSettingAPIROMAddress = 0x0F0026F5U;

/*! \brief Change the internal free running oscillator (FRO) clock to a new frequency */
template <uint32_t clockFrequency_Hz> inline void InitFROClock()
{
    static_assert((clockFrequency_Hz == 18000000) || (clockFrequency_Hz == 24000000) || (clockFrequency_Hz == 30000000),
                  "FRO clock frequency must be one of 18MHz, 24MHz, or 30MHz");

    // The ROM expects that the frequency be in kHz
    constexpr uint32_t freq_kHz = clockFrequency_Hz / 1000;

    typedef void (*clockFROSettingAPICall)(uint32_t);
    auto call = reinterpret_cast<clockFROSettingAPICall>(clockFROSettingAPIROMAddress);
    (*call)(freq_kHz);

    // I'm not really interested in the further divide by 2 or 16 functionality that is there
    // for this application, so I don't implement it.  But it is there, if needed.

    // Need to logically OR this, because the reserved bits in this register need to retain
    // their value, as opposed to being "reserved, shouldn't write 1 to this field"
    SYSCON->FROOSCCTRL = SYSCON->FROOSCCTRL | (1 << SYSCON_FROOSCCTRL_FRO_DIRECT_SHIFT);

    // To enable the new clock setting (default is to divide by 2), we have to toggle
    // the FRO direct clock source update register
    SYSCON->FRODIRECTCLKUEN = SYSCON->FRODIRECTCLKUEN & ~0x1u;
    SYSCON->FRODIRECTCLKUEN = SYSCON->FRODIRECTCLKUEN | (1 << SYSCON_FRODIRECTCLKUEN_ENA_SHIFT);

    while ((SYSCON->FRODIRECTCLKUEN & 1) == 0)
    {
        // Spin until ready
        __NOP();
    }

    // The CPU core defaults to FRO, so we should hopefully be running at 30MHz now
}

/*!
    \brief Change the internal flash access cycles to 1, 2, or 3 clock cycles
    */
template <int accessCycles> inline void InitFlashAccessCycles()
{
    static_assert((accessCycles >= 1) && (accessCycles <= 3),
                  "Flash access cycles must be either 1, 2, or 3 clock cycles");

    uint32_t flashCfg;

    flashCfg = FLASH_CTRL->FLASHCFG & ~FLASH_CTRL_FLASHCFG_FLASHTIM_MASK;
    flashCfg |= (accessCycles - 1);
    FLASH_CTRL->FLASHCFG = flashCfg;
}
} // namespace Internal::LPC845

} // namespace

namespace Init
{

void InitSystem(void)
{
    // Up internal PLL to high speed
    Internal::LPC845::InitFROClock<Clocks::FROFrequency_Hz>();

    // According to
    // https://community.nxp.com/t5/LPC-Microcontrollers/LPC8xx-Flash-access-time-aka-setting-FLASHTIM/m-p/1186229
    // setting the access cycle to 1 cycle is not guaranteed across all temperature conditions when FRO is 30MHz
    // (because the core speed is almost equal to flash speed), but 2 cycles is okay.
    Internal::LPC845::InitFlashAccessCycles<2>();
}

} // namespace Init