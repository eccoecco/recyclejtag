/**

    @file rjcore/rjcore.h
    @brief Recycle JTAG Core file

*/

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "rjcoreevent.h"
#include "rjcoreplatform.h"

#ifdef __cplusplus
extern "C"
{
#endif

/// @brief Recycle JTAG core state machine
struct RJCoreState
{
    struct RJCorePlatform *platform; //!< Platform implementation
    void *privateData;               //!< User provided private data
};

/// @brief Initialises the core state machine to start
/// @param state Pointer to the state to initialise
/// @param platform Pointer to the platform callbacks
/// @param privateData Pointer to optional provide data for this implementation
void RJCoreInit(struct RJCoreState *state, struct RJCorePlatform *platform, void *privateData);

/// @brief Notify the core that an event has occurred
/// @param event The event that occurred
void RJCoreNotifyEvent(const struct RJCoreEvent *event);

#ifdef __cplusplus
}
#endif
