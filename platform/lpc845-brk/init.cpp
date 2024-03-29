#include <cstdint>

#include <LPC845.h>

#include "gpio.h"
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
    SYSCON->FROOSCCTRL = SYSCON->FROOSCCTRL | SYSCON_FROOSCCTRL_FRO_DIRECT(1);

    // To enable the new clock setting (default is to divide by 2), we have to toggle
    // the FRO direct clock source update register
    SYSCON->FRODIRECTCLKUEN = SYSCON->FRODIRECTCLKUEN & ~0x1u;
    SYSCON->FRODIRECTCLKUEN = SYSCON->FRODIRECTCLKUEN | SYSCON_FRODIRECTCLKUEN_ENA(1);

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
    constexpr uint32_t ioconPinMode = IOCON_PIO_MODE(ioconMode_PullUp) | IOCON_PIO_HYS(1);

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
    USART0->INTENSET = USART_INTENSET_RXRDYEN(1);

    USART0->CFG = USART_CFG_ENABLE(1) | USART_CFG_DATALEN(1);
}

#if LPC845_JTAG_USE_SPI
inline void InitSPI()
{
    // Map TCK/TDI/TMS/TDO pins to the same as the GPIO mode:
    //      TCK P0.16   SCK
    //      TDI P0.17   MOSI
    //      TMS P0.18   \CS0
    //      TDO P0.19   MISO

    // P0.17-P0.19 are potential ADC pins in SWM->PINENABLE0, but ADC is disabled by default, so no action
    // required there.

    // However, do set the pull down and hystersis filter for MISO
    IOCON->PIO[IOCON_INDEX_PIO0_19] = IOCON_PIO_MODE(1) | IOCON_PIO_HYS(1);

    // Pin assignment 3 also handles USART2, but this project does not use USART2 (only USART0), so ignore it
    constexpr uint32_t pinAssignment3 = SWM_PINASSIGN3_U2_RTS_O_MASK | SWM_PINASSIGN3_U2_CTS_I_MASK |
                                        SWM_PINASSIGN3_U2_SCLK_IO_MASK | SWM_PINASSIGN3_SPI0_SCK_IO(16);
    constexpr uint32_t pinAssignment4 = SWM_PINASSIGN4_SPI0_MOSI_IO(17) | SWM_PINASSIGN4_SPI0_MISO_IO(19) |
                                        SWM_PINASSIGN4_SPI0_SSEL0_IO_MASK | SWM_PINASSIGN4_SPI0_SSEL1_IO_MASK;
    // SWM_PINASSIGN4_SPI0_SSEL0_IO(18) | SWM_PINASSIGN4_SPI0_SSEL1_IO_MASK;

    SWM0->PINASSIGN.PINASSIGN3 = pinAssignment3;
    SWM0->PINASSIGN.PINASSIGN4 = pinAssignment4;

    // SPI0CLKSEL is index 9, as per 8.6.26
    SYSCON->FCLKSEL[9] = 0; // Select FRO clock (30 MHz)

    // SPI is configured to change data on falling edge, clock data in on rising edge
    // It could either be CPHA=CPOL=0, or CPHA=CPOL=1
    // Now, since the GPIO bit bashing leaves TCK in idle high, we'll try CPHA=CPOL=1

    // It seems that the UART -> USB VCOM driver on the LPC845BRK drops data if we clock SPI
    // data back at full data rate.  So rather than make things complicated and have the delay
    // introduced via UART handling, slow down SPI... so that it ends up not much faster than
    // GPIO bit bashing =(
    SPI0->DIV = 399; // 299 too fast?

    // Don't actually configure it here, because SPOL0 in the control register is set to
    // adjust \CS0 to the proper value

    // TODO: Actually configure SPI
    // TODO: When sending data, RLE the TMS (and invert it) so that we can use \CS0
    // TODO: Change CPOL/CPHA to match JTAG, and load up TDI with MOSI
}
#endif

template <uint32_t systemFrequency_Hz, uint32_t sysTickFrequency_Hz, bool enableInterrupt> inline void InitSysTick()
{
    // Use the divide by 2 clock
    constexpr uint32_t reloadValue = systemFrequency_Hz / 2 / sysTickFrequency_Hz - 1;
    // The reload value must fit into 24 bits
    static_assert((reloadValue & SysTick_LOAD_RELOAD_Msk) == reloadValue,
                  "System frequency too high, or sys tick frequency too low");

    // Start the systick
    SysTick->CTRL = 0;
    SysTick->LOAD = reloadValue;
    SysTick->VAL = reloadValue;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | (enableInterrupt ? SysTick_CTRL_TICKINT_Msk : 0);
    // If want source clock (not divide by 2), then add this: | (1 << SysTick_CTRL_CLKSOURCE_Pos);

    // Note: No need to explicitly enable the NVIC - SysTick will work without __enable_irq()
}

template <int port> constexpr uint32_t IsPortInUse()
{
    // int pinsInPort = 0;

    for (const auto &pinMap : Gpio::Mapping::AllPins)
    {
        if (pinMap.Port == port)
        {
            return 1;
        }
    }

    return 0;
}

template <int port> constexpr uint32_t GenerateDIRSET()
{
    uint32_t result = 0;
    for (const auto &pinMap : Gpio::Mapping::AllPins)
    {
        if ((pinMap.Port == port) && (pinMap.Direction == Gpio::PinDirection::Output))
        {
            result |= (1u << pinMap.Pin);
        }
    }

    return result;
}

} // namespace Internal::LPC845

} // namespace

namespace Init
{

void InitSystem(void)
{
    // Temporarily enable GPIO0, until SPI0 is used to replace it
    constexpr uint32_t sysAHBClkCtrl0Flags = SYSCON_SYSAHBCLKCTRL0_SWM(1) | SYSCON_SYSAHBCLKCTRL0_UART0(1) |
                                             SYSCON_SYSAHBCLKCTRL0_IOCON(1) |
                                             SYSCON_SYSAHBCLKCTRL0_GPIO0(Internal::LPC845::IsPortInUse<0>()) |
                                             SYSCON_SYSAHBCLKCTRL0_GPIO1(Internal::LPC845::IsPortInUse<1>())
#ifdef LPC845_JTAG_USE_SPI
                                             | SYSCON_SYSAHBCLKCTRL0_SPI0(1)
#endif
        ;

    SYSCON->SYSAHBCLKCTRL0 = SYSCON->SYSAHBCLKCTRL0 | sysAHBClkCtrl0Flags;

    constexpr uint32_t presetCtrl0Flags = SYSCON_PRESETCTRL0_IOCON_RST_N(1)
#ifdef LPC845_JTAG_USE_SPI
                                          | SYSCON_PRESETCTRL0_SPI0_RST_N(1)
#endif
        ;
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

#ifdef LPC845_JTAG_USE_GPIO
    static_assert((Gpio::Mapping::JtagTdo.Port == 0) && (Gpio::Mapping::JtagTdo.Pin == 19),
                  "Assumed that TDO was P0.19");
    IOCON->PIO[IOCON_INDEX_PIO0_19] = IOCON_PIO_MODE(1) | IOCON_PIO_HYS(1);
#endif

#ifdef LPC845_JTAG_USE_SPI
    Internal::LPC845::InitSPI();
#endif

    // Turn off SWM clock once configured
    SYSCON->SYSAHBCLKCTRL0 = SYSCON->SYSAHBCLKCTRL0 & ~static_cast<uint32_t>(SYSCON_SYSAHBCLKCTRL0_SWM(1));

    constexpr auto gpioDirset0 = Internal::LPC845::GenerateDIRSET<0>();
    constexpr auto gpioDirset1 = Internal::LPC845::GenerateDIRSET<1>();
    GPIO->DIRSET[0] = gpioDirset0;
    GPIO->DIRSET[1] = gpioDirset1;

    // Initialise a nice 1kHz clock
    Internal::LPC845::InitSysTick<Clocks::FROFrequency_Hz, Clocks::SysTickFrequency_Hz, true>();
}

} // namespace Init