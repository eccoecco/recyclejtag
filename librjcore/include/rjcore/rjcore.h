/**

    @file rjcore/rjcore.h
    @brief Recycle JTAG Core file

*/

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "rjcoreplatform.h"

#ifndef RJCORE_MAX_READ_BUFFER_SIZE
// Different platforms can redefine this for optimum performance
#define RJCORE_MAX_READ_BUFFER_SIZE 64
#endif

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

#ifdef __cplusplus
extern "C"
{
#endif

struct RJCoreState;
typedef void (*RJCoreStateCallback)(struct RJCoreState *);

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
    } state;

    uint32_t currentRxActivityTime; //!< Current receive activity
    uint32_t lastRxActivityTime;    //!< When receive activity was last detected
    int bytesBeforeCallback;        //!< Bytes that are needed in the read buffer before invoking the state
    int bytesInBuffer;              //!< Number of valid bytes in the read buffer
    char readBuffer[RJCORE_MAX_READ_BUFFER_SIZE]; //!< Used to buffer data from the uart
};

/// @brief Initialises the core state machine to start
/// @param state Pointer to the state to initialise
/// @param platform Pointer to the platform callbacks
/// @param privateData Pointer to optional provide data for this implementation
void RJCore_Init(struct RJCoreState *state, struct RJCorePlatform *platform, void *privateData);

/// @brief Notifies the core that data has been received on the uart
/// @param data Pointer to the data received
/// @param bytesReceived How many bytes have been received
void RJCore_NotifyDataReceived(const char *data, size_t bytesReceived);

/// @brief Notifies the core that a timer event has occurred (used to check for timeouts)
void RJCore_NotifyTimer(void);

#ifdef __cplusplus
}
#endif
