#include <cstdint>

#include <LPC845.h>

#include "gpio.h"
#include "init.h"
#include "systick.h"
#include "uart.h"

static void serialEcho();

int main()
{
    Init::InitSystem();

    // TODO: RJCore

    serialEcho();

    return 0;
}

void serialEcho()
{
    constexpr uint32_t testDelay_ms = 1000;

    uint32_t lastSend_ms = SystemTick::CurrentTick();
    uint8_t lastSentChar = '0';

    while (1)
    {
        auto currentTick = SystemTick::CurrentTick();
        if ((currentTick - lastSend_ms) >= testDelay_ms)
        {
            lastSend_ms += testDelay_ms;

            Uart::WriteCharacter(lastSentChar);

            if (lastSentChar == '9')
            {
                lastSentChar = '0';
            }
            else
            {
                ++lastSentChar;
            }
        }

        int readCharacter = Uart::ReadCharacter();
        if (readCharacter < 0)
        {
            continue;
        }

        Uart::WriteCharacter(readCharacter);

        switch (readCharacter)
        {
        case 'r':
        case 'R':
            Gpio::SetState<Gpio::Mapping::DebugRed>(readCharacter == 'R');
            break;
        case 'g':
        case 'G':
            Gpio::SetState<Gpio::Mapping::DebugGreen>(readCharacter == 'G');
            break;
        case 'b':
        case 'B':
            Gpio::SetState<Gpio::Mapping::DebugBlue>(readCharacter == 'B');
            break;
        case 'i':
        case 'I':
            Uart::WriteCharacter(Gpio::GetState<Gpio::Mapping::JtagTdo>() ? 'H' : 'L');
            break;
        }
    }
}