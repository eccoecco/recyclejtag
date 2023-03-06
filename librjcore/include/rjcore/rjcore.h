/**

    @file rjcore/rjcore.h
    @brief Recycle JTAG Core file

*/

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "rjcoreplatform.h"

#ifndef RJCORE_TIMEOUT_HANDSHAKE
// Timeout on the handshake, in whatever units the platform timer provides
// The default is to assume ms, and have a short timeout
#define RJCORE_TIMEOUT_HANDSHAKE 50
#endif

#ifndef RJCORE_TIMEOUT_BINARY_MODE
// Timeout when in binary mode for any request (e.g. configure serial port, jtag shift)
// Defaults to a long 30s
#define RJCORE_TIMEOUT_BINARY_MODE 30000
#endif

#ifndef RJCORE_TIMEOUT_SERIAL_MODE_ACK
// How long to wait for an ack message when the serial mode has changed
#define RJCORE_TIMEOUT_SERIAL_MODE_ACK 250
#endif

#ifndef RJCORE_MAXIMUM_TAP_SHIFT_BIT_COUNT
#define RJCORE_MAXIMUM_TAP_SHIFT_BIT_COUNT 0x2000
#endif

#ifdef __cplusplus
extern "C"
{
#endif

struct RJCoreState;

// All callbacks must return:
// <0 - Internal error - go back to initial state (will flush read buffer)
// else number of bytes processed (can be 0)
typedef int (*RJCoreStateCallback)(struct RJCoreState *);

/// @brief Recycle JTAG core state machine
struct RJCoreState
{
    struct RJCorePlatform *platform; //!< Platform implementation
    void *privateData;               //!< User provided private data

    RJCoreStateCallback stateCallback; //!< Current state in the state machine

    union {
        struct
        {
            int validBytes;
        } handshake;
        struct
        {
            uint8_t counter;
            uint8_t mode;
        } serial; //!< Used to keep track when setting serial mode
        struct
        {
            uint8_t counter;
            uint8_t feature;
        } feature; //!< Used to keep track of what feature request is being changed
        struct
        {
            uint16_t bitsToShift;
            uint8_t counter;
        } tapShift;
    } state;

    uint32_t timeSinceLastRxActivity; //!< How long since last activity time was detected
    uint32_t lastRxActivityTime;      //!< When receive activity was last detected

    int bytesReceived;            // Number of bytes that have been received this event
    const uint8_t *receiveBuffer; // Pointer to the received data
};

/// @brief Initialises the core state machine to start
/// @param state Pointer to the state to initialise
/// @param platform Pointer to the platform callbacks
/// @param privateData Pointer to optional provide data for this implementation
void RJCore_Init(struct RJCoreState *state, struct RJCorePlatform *platform, void *privateData);

/// @brief Notifies the core that data has been received on the uart
/// @param data Pointer to the data received (or NULL if a timeout)
/// @param bytesReceived How many bytes have been received (or 0 if a timeout)
void RJCore_NotifyDataReceived(struct RJCoreState *state, const uint8_t *data, size_t bytesReceived);

#ifdef __cplusplus
}
#endif
