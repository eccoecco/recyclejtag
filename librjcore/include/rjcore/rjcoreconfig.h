/**

    @file rjcore/rjcoreconfig.h
    @brief Recycle JTAG Core constants/configuration

*/

#pragma once

#ifndef RJCORE_STATE_MAXIMUM_BUFFERED_BYTES
// Maximum number of bytes that can be buffered prior to entering a state
#define RJCORE_STATE_MAXIMUM_BUFFERED_BYTES 4
#endif

#ifndef RJCORE_UART_IDLE_TIMEOUT
// Timeout on the UART idle.  If the UART is idle for this long, assume that OpenOCD
// has disconnected, and reset to the wait for handshake/init state.  These units
// are in whatever the platform currentUptime callback returns.
#define RJCORE_UART_IDLE_TIMEOUT 0
// #define RJCORE_UART_IDLE_TIMEOUT 1000
#endif

#ifndef RJCORE_MAXIMUM_TAP_SHIFT_BIT_COUNT
// Maximum number of bits that can be done in a jtag tap shift.
// This value is chosen because it is both in OpenOCD and Bus Pirate.
#define RJCORE_MAXIMUM_TAP_SHIFT_BIT_COUNT 0x2000
#endif