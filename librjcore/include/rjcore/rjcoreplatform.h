/**

    @file rjcore/rjcoreevent.h
    @brief Functions that platforms must provide to run the Recycle JTAG Core.

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

typedef void (*RJCorePlatform_NewTapShift)(void *, int totalBitsToShift);

enum RJCoreTapShiftMode
{
    RJCoreTapShiftMode_GPIO,   //!< Jtag done via manual bit bashing
    RJCoreTapShiftMode_Packet, //!< Jtag done with packets of data via callback
};

// Generate a clock cycle (TCK low, TDI = tdi, TMS = tms, pause, TCK high, read TDO)
// Returns 0 if TDO low, 1 if TDO high
typedef int (*RJCorePlatform_TapShiftGPIOClockCycle)(void *, int tdi, int tms);

// Given a packet of data, try to tap shift in bulk.  Note that you *MUST* handle
// the case where the number of bytes received is odd, and thus you must potentially
// be able to buffer a tdi byte until the next packet of data comes.
// NOTE: The return value is a bit funny.
//   This returns 0 to indicate that all bytes in the buffer are processed, and that we want
// to keep processing packet data.
//   If all data has been processed for a tap shift, then this returns the exact number of
// bytes processed in the buffer - just in case extra commands are present in the serial buffer.
//   If an error occurs, return < 0.
//
// Note: It is acceptable for this callback to take control of the serial port, and just block
// until all data has been processed, as opposed to having this state machine get notified by
// serial events.  In that case, just block, and when complete, return bufferLength.
typedef int (*RJCorePlatform_TapShiftPacket)(void *, const uint8_t *buffer, int bufferLength);

/// @brief The platform implementation
struct RJCorePlatform
{
    enum RJCoreTapShiftMode tapShiftMode; //!< Tells us which tap shift callback to use

    RJCorePlatform_CurrentUptime currentUptime; //!< Current uptime
    RJCorePlatform_TransmitData transmitData;   //!< Used to transmit data
    RJCorePlatform_SetSerialMode setSerialMode; //!< Set the serial mode
    RJCorePlatform_SetPortMode setPortMode;     //!< Sets the port mode
    RJCorePlatform_SetFeature setFeature;       //!< Sets features
    RJCorePlatform_ReadVoltages readVoltages;   //!< Reads the voltages
    RJCorePlatform_NewTapShift newTapShift;     //!< New tap shift incoming
    RJCorePlatform_TapShiftGPIOClockCycle
        tapShiftGPIO; //!< Executes one clock of the tap shift (tapShiftMode == RJCoreTapShiftMode_GPIO)
    RJCorePlatform_TapShiftPacket tapShiftPacket; //!< Executes a packet (tapShiftMode == RJCoreTapShiftMode_Packet)
};

#ifdef __cplusplus
}
#endif
