/**

    @file rjcore/rjcoreevent.h
    @brief Recycle JTAG Core platform implementation

*/

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/// Implementation that returns the current uptime in platform dependent units.  It is
/// up to the platform to configure RJCore to indicate how long a timeout should be, in
/// terms of its timing units.
typedef uint32_t (*RJCorePlatform_CurrentUptime)(void *);

/// Implementation that transmits data over the uart.  Only returns after all data has
/// been transmitted.
typedef void (*RJCorePlatform_TransmitData)(void *, const char *, size_t);

enum RJCoreSerialMode
{
    RJCoreSerialMode_Normal = 0, //!< 115,200 baud
    RJCoreSerialMode_Fast = 1,   //!< 1,000,000 baud
};

typedef bool (*RJCorePlatform_SetSerialMode)(void *, enum RJCoreSerialMode);

/// @brief The platform implementation
struct RJCorePlatform
{
    RJCorePlatform_CurrentUptime currentUptime; //!< Current uptime
    RJCorePlatform_TransmitData transmitData;   //!< Used to transmit data
    RJCorePlatform_SetSerialMode setSerialMode; //!< Set the serial mode
};

#ifdef __cplusplus
}
#endif
