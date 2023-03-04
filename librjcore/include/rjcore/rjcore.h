/**

    \file rjcore/rjcore.h
    \brief Recycle JTAG Core file

*/

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/// @brief The platform implementation
struct RJCorePlatform
{
    uint32_t (*currentMillisecondUptime)();    //!< What the current uptime in millisecond is
    int (*transmitData)(const char *, size_t); //!< Transmits a buffer of data
};

/// @brief Recycle JTAG core state machine
struct RJCoreState
{
    char void *privateData;
};

/// @brief Initialises the core state machine to start
/// @param state Pointer to the state to initialise
/// @param privateData Pointer to optional provide data for this implementation
void RJCoreInit(struct RJCoreState *state, void *privateData);

enum RJCoreEventType
{
    RJCoreEventType_ReceivedData, //!< Data has been received over uart
    RJCoreEventType_Timer,        //!< A timer has been received
};

struct RJCoreEventReceivedData
{
    const char *data;
    size_t length;
};

/// @brief An event that has just occurred
struct RJCoreEvent
{
    RJCoreEventType type;
    union {
        RJCoreEventReceivedData receivedData;
    };
};

void RJCoreNotifyEvent(

#ifdef __cplusplus
}
#endif