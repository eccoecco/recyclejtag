/**

    @file rjcore/rjcore.h
    @brief Recycle JTAG Core file

*/

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "rjcoreconfig.h"
#include "rjcoreplatform.h"
#include "rjcorestate.h"

#ifdef __cplusplus
extern "C"
{
#endif

/// @brief Recycle JTAG core state machine
struct RJCoreHandle
{
    struct RJCorePlatform *platform; //!< Platform implementation
    void *privateData;               //!< User provided private data

    const struct RJCoreStateDescription *currentState; //!< Current state
    struct RJCoreStateData stateData;                  //!< Persistant data for the current state

    uint32_t lastRxActivityTime; //!< When receive activity was last detected on the UART

    struct
    {
        int bytesValid;                                    //!< Number of bytes valid
        uint8_t data[RJCORE_STATE_MAXIMUM_BUFFERED_BYTES]; //!< Actual data
    } buffered;                                            //!< Buffered data for state
};

/// @brief Initialises the core state machine to start
/// @param handle Pointer to the handle to initialise.  Memory for the handle is allocated by the caller.
/// @param platform Pointer to the platform callbacks
/// @param privateData Pointer to optional provide data for this implementation
void RJCore_Init(struct RJCoreHandle *handle, struct RJCorePlatform *platform, void *privateData);

/// @brief Notifies the core that data has been received on the uart
/// @param handle Handle of the core
/// @param data Pointer to the data received (or NULL if a timeout)
/// @param bytesReceived How many bytes have been received (or 0 if a timeout)
void RJCore_NotifyDataReceived(struct RJCoreHandle *handle, const uint8_t *data, int bytesReceived);

#ifdef __cplusplus
}
#endif
