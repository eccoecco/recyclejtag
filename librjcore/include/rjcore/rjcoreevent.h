/**

    @file rjcore/rjcoreevent.h
    @brief Recycle JTAG Core events

*/

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

enum RJCoreEventType
{
    RJCoreEventType_ReceivedData, //!< Data has been received over uart
    RJCoreEventType_Timer,        //!< A timer event (used to check for timeouts)
};

struct RJCoreEventReceivedData
{
    const char *data;
    size_t length;
};

/// @brief An event that has just occurred
struct RJCoreEvent
{
    enum RJCoreEventType type; //!< What type of event just occurred

    union {
        struct RJCoreEventReceivedData receivedData; //!< What data was received
    };
};

#ifdef __cplusplus
}
#endif
