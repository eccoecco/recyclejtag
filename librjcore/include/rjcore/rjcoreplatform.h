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
typedef void (*RJCorePlatform_TransmitData)(void *, const void *, size_t);

enum RJCoreSerialMode
{
    RJCoreSerialMode_Normal = 0, //!< 115,200 baud
    RJCoreSerialMode_Fast = 1,   //!< 1,000,000 baud
};

typedef bool (*RJCorePlatform_SetSerialMode)(void *, enum RJCoreSerialMode);

enum RJCorePortMode
{
    RJCorePortMode_HighImpedance = 0,
    RJCorePortMode_PushPull = 1,
    RJCorePortMode_OpenDrain = 2,
    RJCorePortMode_Size,
};

// Implementation that sets the port mode
typedef bool (*RJCorePlatform_SetPortMode)(void *, enum RJCorePortMode);

enum RJCoreFeature
{
    RJCoreFeature_LED = 0x01,
    RJCoreFeature_VReg = 0x02,
    RJCoreFeature_TRst = 0x04,
    RJCoreFeature_SRst = 0x08,
    RJCoreFeature_Pullup = 0x10,
};

typedef bool (*RJCorePlatform_SetFeature)(void *, enum RJCoreFeature, int action);

// Implementation that reads the four voltages
// TODO: Document ADC ratios etc
typedef void (*RJCorePlatform_ReadVoltages)(void *, uint16_t *values);

/// @brief The platform implementation
struct RJCorePlatform
{
    RJCorePlatform_CurrentUptime currentUptime; //!< Current uptime
    RJCorePlatform_TransmitData transmitData;   //!< Used to transmit data
    RJCorePlatform_SetSerialMode setSerialMode; //!< Set the serial mode
    RJCorePlatform_SetPortMode setPortMode;     //!< Sets the port mode
    RJCorePlatform_SetFeature setFeature;       //!< Sets features
    RJCorePlatform_ReadVoltages readVoltages;   //!< Reads the voltages
};

#ifdef __cplusplus
}
#endif
