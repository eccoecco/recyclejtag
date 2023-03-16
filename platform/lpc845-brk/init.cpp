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

/*!
    \brief Initialises USART0 for asynchronous 115,200 baud rate
*/
inline void InitUSART0()
{
    // The LPC845 breakout board wires RX to P0.24, and TX to P0.25
    // P0.24 and P0.25 have no special functions in SWM->PINENABLE{0,1}, so nothing needs to be done there

    constexpr uint32_t ioconMode_PullUp = 2;
    // Pull up, hysteresis enabled, not inverted, disable open drain, bypass input digital filter
    constexpr uint32_t ioconPinMode = (ioconMode_PullUp << IOCON_PIO_MODE_SHIFT) | (1 << IOCON_PIO_HYS_SHIFT);

    IOCON->PIO[IOCON_INDEX_PIO0_24] = ioconPinMode;
    IOCON->PIO[IOCON_INDEX_PIO0_25] = ioconPinMode;

    constexpr uint32_t uart0PinAssignment = SWM_PINASSIGN0_U0_RTS_O_MASK | SWM_PINASSIGN0_U0_CTS_I_MASK |
                                            SWM_PINASSIGN0_U0_TXD_O(25) | SWM_PINASSIGN0_U0_RXD_I(24);
    SWM0->PINASSIGN.PINASSIGN0 = uart0PinAssignment;

    SYSCON->FRG[0].FRGDIV = 0xFF; // As per datasheet, must be 0xFF, to represent / 256 (no other values supported)
    SYSCON->FRG[0].FRGMULT = 207; // Divide 30MHz FRO to 16587473, close to a nominal 16588800
    SYSCON->FRG[0].FRGCLKSEL = 0; // Select FRO (should be 30MHz)

    // UART0CLKSEL is index 0, as per 8.6.26
    SYSCON->FCLKSEL[0] = 2; // Select FRG0 clock

    // Register values are off by one to actual values
    // Divide by 9, coupled with oversample value of 16 means a clock divisor of 9 * 16 = 144
    // to get the final baud rate from the fractional baud clock:
    //   16587473 / 144 = 115190.78, which is close enough to 115200
    USART0->BRG = 8;
    USART0->OSR = 15;

    USART0->CTL = 0; // Default control is good
    // Enable, 8N1 data, asynchronous, no flow, no loopback, not inverted

    // Enable receive ready interrupt - wait until data is ready to transmit before interested in txrdy
    // USART0->INTENSET = USART_INTENCLR_RXRDYCLR(1); // | USART_INTENCLR_TXRDYCLR(1);

    USART0->CFG = USART_CFG_ENABLE(1) | USART_CFG_DATALEN(1);
}

} // namespace Internal::LPC845

} // namespace

namespace Init
{

void InitSystem(void)
{
    constexpr uint32_t sysAHBClkCtrl0Flags = (1 << SYSCON_SYSAHBCLKCTRL0_SWM_SHIFT) |
                                             (1 << SYSCON_SYSAHBCLKCTRL0_UART0_SHIFT) |
                                             (1 << SYSCON_SYSAHBCLKCTRL0_IOCON_SHIFT);

    SYSCON->SYSAHBCLKCTRL0 = SYSCON->SYSAHBCLKCTRL0 | sysAHBClkCtrl0Flags;

    constexpr uint32_t presetCtrl0Flags = SYSCON_PRESETCTRL0_IOCON_RST_N(1);
    SYSCON->PRESETCTRL0 = SYSCON->PRESETCTRL0 & ~presetCtrl0Flags;
    SYSCON->PRESETCTRL0 = SYSCON->PRESETCTRL0 | presetCtrl0Flags;

    // Up internal PLL to high speed
    Internal::LPC845::InitFROClock<Clocks::FROFrequency_Hz>();

    // According to
    // https://community.nxp.com/t5/LPC-Microcontrollers/LPC8xx-Flash-access-time-aka-setting-FLASHTIM/m-p/1186229
    // setting the access cycle to 1 cycle is not guaranteed across all temperature conditions when FRO is 30MHz
    // (because the core speed is almost equal to flash speed), but 2 cycles is okay.
    Internal::LPC845::InitFlashAccessCycles<2>();

    NVIC_EnableIRQ(USART0_IRQn);

    Internal::LPC845::InitUSART0();

    // Turn off SWM clock once configured
    SYSCON->SYSAHBCLKCTRL0 = SYSCON->SYSAHBCLKCTRL0 & ~static_cast<uint32_t>(1u << SYSCON_SYSAHBCLKCTRL0_SWM_SHIFT);
}

} // namespace Init