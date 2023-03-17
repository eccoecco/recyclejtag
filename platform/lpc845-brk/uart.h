/*!

    \file uart.h

    \brief Simple interrupt based uart with a circular buffer

*/

#pragma once

#include <cstdint>

namespace Uart
{

void WriteCharacter(uint8_t value);

/*!
    \retval <0 no character
    \retval 0-255 the character
*/
int ReadCharacter(void);

} // namespace Uart
