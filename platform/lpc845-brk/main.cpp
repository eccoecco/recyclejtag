#include <cstdint>

#include <LPC845.h>

#include "init.h"

static void serialEcho();

int main()
{
    Init::InitSystem();

    // TODO: Turn serial port into interrupt based

    serialEcho();

    return 0;
}

void serialEcho()
{
    while (1)
    {
        uint32_t status = USART0->STAT;
        if ((status & USART_STAT_RXRDY(1)) != 0)
        {
            USART0->STAT = UINT32_MAX;

            // Data ready!
            // If using parity for extra data, then use RXDATSTAT instead
            uint32_t rxData = USART0->RXDAT;

            ++rxData;
            rxData &= 0xFF;

            USART0->TXDAT = rxData;
        }
    }
}