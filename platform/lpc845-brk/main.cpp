#include <cstdint>

#include <LPC845.h>

#include "init.h"
#include "uart.h"

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
        int readCharacter = Uart::ReadCharacter();
        if (readCharacter < 0)
        {
            continue;
        }

        ++readCharacter;
        Uart::WriteCharacter(readCharacter & 0xFF);
    }
}