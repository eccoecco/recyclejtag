/**

    @file rjcore/rjcorestate.h
    @brief State machine details

    @note This is exposed so that in future, platforms can introduce new states

*/
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "rjcoreplatform.h"

#ifdef __cplusplus
extern "C"
{
#endif

/// @brief Persistant data for each state
struct RJCoreStateData
{
    struct
    {
        int value;
        void *ptr;
    } generic; //!< Generic state data

    union {
        struct
        {
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
};

/// @brief Returned by a state machine function
struct RJCoreStateReply
{
    int bytesProcessed; //!< Bytes processed
    const struct RJCoreStateDescription *nextState;
};

struct RJCoreHandle;

/// @brief Function to call when a state is being entered (optional)
typedef void (*RJCoreStateEnter)(struct RJCoreHandle *);

/// @brief Function to call to process the state
typedef struct RJCoreStateReply (*RJCoreStateProcess)(struct RJCoreHandle *, const uint8_t *, int);

/// @brief Describes a state machine function
struct RJCoreStateDescription
{
    RJCoreStateEnter stateEnter;     //!< Optional function to call upon entering this state
    RJCoreStateProcess stateProcess; //!< The actual function that processes this state
    int bytesRequired;               //!< Bytes to receive and buffer before entering this state (can be >= 0).
    //!< Maximum value of RJCORE_STATE_MAXIMUM_BUFFERED_BYTES
    //!< If -1, then whatever has been received (can be 0 bytes)
};

#ifdef __cplusplus
}
#endif