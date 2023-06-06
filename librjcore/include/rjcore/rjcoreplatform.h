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

typedef void (*RJCorePlatform_TapShiftComplete)(void *);

enum RJCoreTapShiftMode
{
    RJCoreTapShiftMode_GPIO,   //!< Jtag done via manual bit bashing
    RJCoreTapShiftMode_Packet, //!< Jtag done with packets of data via callback
    RJCoreTapShiftMode_Custom, //!< Jtag done with a full custom callback
};

// Generate a clock cycle (TCK low, TDI = tdi, TMS = tms, pause, TCK high, read TDO)
// Returns 0 if TDO low, 1 if TDO high
typedef int (*RJCorePlatform_TapShiftGPIOClockCycle)(void *, int tdi, int tms);

// Callback for when mode is RJCoreTapShiftMode_Packet
// This callback returns 0 for success, -1 to reset to handshake mode.
// This callback is invoked with even packets of data, so that you're always guaranteed to
// have tdi/tms come in a pair.
// === Note: The length is in *bits* to shift, not *bytes*. ===
// e.g. If bitsToShift == 13, then you would expect { buffer[0], buffer[1] } to represent
// the first 8 bits to shift of {tdi, tms}, and then { buffer[2], buffer[3] } to only contain
// the remaining 5 bits to shift of {tdi, tms} starting from the least significant bit.
// Note: This callback is responsible for sending tdo back via serial port.
typedef int (*RJCorePlatform_TapShiftPacket)(void *, const uint8_t *buffer, int bitsToShift);

// Callback for when mode is RJCoreTapShiftMode_Custom
// This callback returns 0 for success, -1 to reset to handshake mode.
// This callback is invoked exactly once.  If the internal buffer has data that should be
// processed as part of the tap shift, then it will be placed in buffer and marked as bytesAvailable.
//  After that is processed, the callback should assume control of the uart, and *only read enough bytes*
// from the uart to complete this tap shift.
//
// Note: Keep in mind, due to timing/serial port, it is entirely possible that an odd number
// of bytes are available, and thus a tdi is present without a corresponding tms.  This callback
// will need to handle this edge case correctly.
typedef int (*RJCorePlatform_TapShiftCustom)(void *, const uint8_t *buffer, int bytesAvailable, int totalBitsToShift);

/// @brief The platform implementation
struct RJCorePlatform
{
    enum RJCoreTapShiftMode tapShiftMode; //!< Tells us which tap shift callback to use

    RJCorePlatform_CurrentUptime currentUptime;       //!< Current uptime
    RJCorePlatform_TransmitData transmitData;         //!< Used to transmit data
    RJCorePlatform_SetSerialMode setSerialMode;       //!< Set the serial mode
    RJCorePlatform_SetPortMode setPortMode;           //!< Sets the port mode
    RJCorePlatform_SetFeature setFeature;             //!< Sets features
    RJCorePlatform_ReadVoltages readVoltages;         //!< Reads the voltages
    RJCorePlatform_NewTapShift newTapShift;           //!< New tap shift incoming
    RJCorePlatform_TapShiftComplete tapShiftComplete; //!< Tap shift complete

    RJCorePlatform_TapShiftGPIOClockCycle
        tapShiftGPIO; //!< Executes one clock of the tap shift (tapShiftMode == RJCoreTapShiftMode_GPIO)
    RJCorePlatform_TapShiftPacket tapShiftPacket; //!< Executes a packet for tap shifting in packet mode
    RJCorePlatform_TapShiftCustom tapShiftCustom; //!< Executes a fully custom callback for tap shifting
};

#ifdef __cplusplus
}
#endif
