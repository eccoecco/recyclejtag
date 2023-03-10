#include <string.h>

#include "rjcore/rjcore.h"

enum BusPirateCommand
{
    BusPirateCommand_Unknown = 0x00,
    BusPirateCommand_PortMode = 0x01,
    BusPirateCommand_Feature = 0x02,
    BusPirateCommand_ReadADCs = 0x03,
    BusPirateCommand_TapShift = 0x05,
    BusPirateCommand_EnterRWire = 0x05,
    BusPirateCommand_EnterOpenOCD = 0x06,
    BusPirateCommand_UartSpeed = 0x07,
    BusPirateCommand_JtagSpeed = 0x08,
};

static void RJCoreState_Handshake_Enter(struct RJCoreHandle *);
static struct RJCoreStateReply RJCoreState_Handshake_Process(struct RJCoreHandle *, const uint8_t *, int);
static struct RJCoreStateReply RJCoreState_BinaryMode_Process(struct RJCoreHandle *, const uint8_t *, int);
static struct RJCoreStateReply RJCoreState_SetSerialSpeed_Process(struct RJCoreHandle *, const uint8_t *, int);
static struct RJCoreStateReply RJCoreState_AwaitSerialAck_Process(struct RJCoreHandle *, const uint8_t *, int);
static struct RJCoreStateReply RJCoreState_SetPortMode_Process(struct RJCoreHandle *, const uint8_t *, int);
static struct RJCoreStateReply RJCoreState_SetFeatures_Process(struct RJCoreHandle *, const uint8_t *, int);
static struct RJCoreStateReply RJCoreState_ReadVoltages_Process(struct RJCoreHandle *, const uint8_t *, int);
static struct RJCoreStateReply RJCoreState_TapShiftInitialise_Process(struct RJCoreHandle *, const uint8_t *, int);
static struct RJCoreStateReply RJCoreState_TapShiftGPIO_Process(struct RJCoreHandle *, const uint8_t *, int);
static struct RJCoreStateReply RJCoreState_TapShiftPacket_Process(struct RJCoreHandle *, const uint8_t *, int);
static struct RJCoreStateReply RJCoreState_TapShiftCustom_Process(struct RJCoreHandle *, const uint8_t *, int);

static struct RJCoreStateDescription RJCoreState_Handshake = {
    .stateEnter = RJCoreState_Handshake_Enter,
    .stateProcess = RJCoreState_Handshake_Process,
    .bytesRequired = -1,
};
static struct RJCoreStateDescription RJCoreState_BinaryMode = {
    .stateEnter = NULL,
    .stateProcess = RJCoreState_BinaryMode_Process,
    .bytesRequired = 1,
};
static struct RJCoreStateDescription RJCoreState_SetSerialSpeed = {
    .stateEnter = NULL,
    .stateProcess = RJCoreState_SetSerialSpeed_Process,
    .bytesRequired = 1,
};
static struct RJCoreStateDescription RJCoreState_AwaitSerialAck = {
    .stateEnter = NULL,
    .stateProcess = RJCoreState_AwaitSerialAck_Process,
    .bytesRequired = 2,
};
static struct RJCoreStateDescription RJCoreState_SetPortMode = {
    .stateEnter = NULL,
    .stateProcess = RJCoreState_SetPortMode_Process,
    .bytesRequired = 1,
};
static struct RJCoreStateDescription RJCoreState_SetFeatures = {
    .stateEnter = NULL,
    .stateProcess = RJCoreState_SetFeatures_Process,
    .bytesRequired = 2,
};
static struct RJCoreStateDescription RJCoreState_ReadVoltages = {
    .stateEnter = NULL,
    .stateProcess = RJCoreState_ReadVoltages_Process,
    .bytesRequired = 0,
};
static struct RJCoreStateDescription RJCoreState_TapShiftInitialise = {
    .stateEnter = NULL,
    .stateProcess = RJCoreState_TapShiftInitialise_Process,
    .bytesRequired = 2,
};
static struct RJCoreStateDescription RJCoreState_TapShiftGPIO = {
    .stateEnter = NULL,
    .stateProcess = RJCoreState_TapShiftGPIO_Process,
    .bytesRequired = 2,
};
static struct RJCoreStateDescription RJCoreState_TapShiftPacket = {
    .stateEnter = NULL,
    .stateProcess = RJCoreState_TapShiftPacket_Process,
    .bytesRequired = -1,
};
static struct RJCoreStateDescription RJCoreState_TapShiftCustom = {
    .stateEnter = NULL,
    .stateProcess = RJCoreState_TapShiftCustom_Process,
    .bytesRequired = -1,
};

static inline void RJCore_ChangeToState(struct RJCoreHandle *handle, const struct RJCoreStateDescription *state)
{
    if (state->bytesRequired > RJCORE_STATE_MAXIMUM_BUFFERED_BYTES)
    {
        // Oh no - can't have this - internal bug!  Just go reset.
        // TODO: Notify the platform that an internal error occurred
        state = &RJCoreState_Handshake;
    }

    handle->currentState = state;
    handle->buffered.bytesValid = 0;

    if (state->stateEnter != NULL)
    {
        (*state->stateEnter)(handle);
    }
}

void RJCore_Init(struct RJCoreHandle *handle, struct RJCorePlatform *platform, void *privateData)
{
    memset(handle, 0, sizeof(*handle));

    handle->platform = platform;
    handle->privateData = privateData;

    RJCore_ChangeToState(handle, &RJCoreState_Handshake);
}

static void RJCore_CheckIdleTime(struct RJCoreHandle *handle, size_t bytesReceived)
{
    uint32_t currentTime = (*handle->platform->currentUptime)(handle->privateData);

    if (bytesReceived > 0)
    {
        handle->lastRxActivityTime = currentTime;
    }
#if RJCORE_UART_IDLE_TIMEOUT > 0
    else if (handle->currentState != &RJCoreState_Handshake)
    {
        uint32_t idleTime = currentTime - handle->lastRxActivityTime;

        if (idleTime >= RJCORE_UART_IDLE_TIMEOUT)
        {
            // Took too long to receive any data, just reset.
            RJCore_ChangeToState(handle, &RJCoreState_Handshake);
        }
    }
#endif
}

static int RJCore_CopyDataToBuffer(struct RJCoreHandle *handle, const uint8_t *data, int bytesAvailable)
{
    int totalBytesRequired = handle->currentState->bytesRequired;
    int bytesToCopy;

    if ((bytesAvailable == 0) || (totalBytesRequired <= 0))
    {
        // Either no data available, or the state is requesting as much data as available (< 0),
        // or not needing any data at all (== 0)
        return 0;
    }

    bytesToCopy = totalBytesRequired - handle->buffered.bytesValid;

    if (bytesToCopy > bytesAvailable)
    {
        bytesToCopy = bytesAvailable;
    }

    for (int offset = 0; offset < bytesToCopy; ++offset)
    {
        handle->buffered.data[handle->buffered.bytesValid + offset] = data[offset];
    }

    handle->buffered.bytesValid += bytesToCopy;

    return bytesToCopy;
}

static int RJCore_ProcessState(struct RJCoreHandle *handle, const uint8_t *data, int bytesAvailable)
{
    struct RJCoreStateReply reply;

    reply = (*handle->currentState->stateProcess)(handle, data, bytesAvailable);

    // Always clear buffered data after completion, so that a state can keep on requesting
    // more data for itself
    handle->buffered.bytesValid = 0;

    if (reply.bytesProcessed == -1)
    {
        // Oh no!  Failure!
        RJCore_ChangeToState(handle, &RJCoreState_Handshake);
    }
    else if (reply.nextState != NULL)
    {
        RJCore_ChangeToState(handle, reply.nextState);
    }

    return reply.bytesProcessed;
}

void RJCore_NotifyDataReceived(struct RJCoreHandle *handle, const uint8_t *data, int bytesReceived)
{
    const struct RJCoreStateDescription *startingState;

    RJCore_CheckIdleTime(handle, bytesReceived);

    do
    {
        startingState = handle->currentState;

        // If this state needs a certain number of bytes in the buffer before
        // entering, then see if that's available
        if (startingState->bytesRequired > 0)
        {
            int bytesProcessed = RJCore_CopyDataToBuffer(handle, data, bytesReceived);

            data += bytesProcessed;
            bytesReceived -= bytesProcessed;

            if (handle->buffered.bytesValid < startingState->bytesRequired)
            {
                continue;
            }

            // Can ignore return value of bytes processed, since we've already taken the
            // data from data/bytesReceived, and put them into the buffered data
            RJCore_ProcessState(handle, handle->buffered.data, handle->buffered.bytesValid);
        }
        else
        {
            // Either no data was requested, or as much data as possible.  Either way, should
            // be valid to pass the raw data across.
            int bytesProcessed = RJCore_ProcessState(handle, data, bytesReceived);

            data += bytesProcessed;
            bytesReceived -= bytesProcessed;
        }

        // Keep looping if there's still data to process.
        // If there's no more data, only keep looping if the state machine keeps changing state.
        // Once the state machine stays still, no point looping (can accidentally cause infinite loop otherwise)
    } while ((bytesReceived > 0) || (startingState != handle->currentState));
}

static void RJCoreState_Handshake_Enter(struct RJCoreHandle *handle)
{
    handle->stateData.generic.value = 0; // Use this as a counter for the number of zeroes received
    (*handle->platform->setSerialMode)(handle->privateData, RJCoreSerialMode_Normal);
}

static struct RJCoreStateReply RJCoreState_Handshake_Process(struct RJCoreHandle *handle, const uint8_t *data,
                                                             int bytesAvailable)
{
    int bytesProcessed;

    for (bytesProcessed = 0; bytesProcessed < bytesAvailable; ++bytesProcessed, ++data)
    {
        switch (*data)
        {
        case BusPirateCommand_Unknown:
            // Bus Pirate says that it should only send a response after 20 consecutive zeroes
            // but that errant zeroes can happen (e.g. during startup), so it is possible that
            // more than 20 consecutive zeroes come streaming in
            if (handle->stateData.generic.value < 20)
            {
                ++handle->stateData.generic.value;

                if (handle->stateData.generic.value == 20)
                {
                    // Send the Bus Pirate identifier
                    (*handle->platform->transmitData)(handle->privateData, "BBIO1", 5);
                }
            }
            // Okay to keep on receiving lots of zeroes - it might be that some were errant
            break;
        case BusPirateCommand_EnterOpenOCD:
            if (handle->stateData.generic.value == 20)
            {
                (*handle->platform->transmitData)(handle->privateData, "OCD1", 4);

                return (struct RJCoreStateReply){
                    .bytesProcessed = bytesProcessed + 1, // +1, have processed the current byte
                    .nextState = &RJCoreState_BinaryMode,
                };
            }
            else
            {
                return (struct RJCoreStateReply){
                    .bytesProcessed = -1,
                    .nextState = NULL,
                };
            }
            break;
        default:
            // Don't know how to handle this command - just reset to initial state so that
            // we don't get into an odd state
            return (struct RJCoreStateReply){
                .bytesProcessed = -1,
                .nextState = NULL,
            };
        }
    }

    return (struct RJCoreStateReply){
        .bytesProcessed = bytesProcessed,
        .nextState = NULL,
    };
}

static struct RJCoreStateReply RJCoreState_BinaryMode_Process(struct RJCoreHandle *handle, const uint8_t *data,
                                                              int bytesAvailable)
{
    (void)handle;

    // bytesAvailable should be equal to 1, since the state description requested exactly 1 byte
    if (bytesAvailable == 1)
    {
        switch (*data)
        {
        case BusPirateCommand_UartSpeed:
            return (struct RJCoreStateReply){
                .bytesProcessed = 0,
                .nextState = &RJCoreState_SetSerialSpeed,
            };
        case BusPirateCommand_PortMode:
            return (struct RJCoreStateReply){
                .bytesProcessed = 0,
                .nextState = &RJCoreState_SetPortMode,
            };
        case BusPirateCommand_Feature:
            return (struct RJCoreStateReply){
                .bytesProcessed = 0,
                .nextState = &RJCoreState_SetFeatures,
            };
        case BusPirateCommand_ReadADCs:
            return (struct RJCoreStateReply){
                .bytesProcessed = 0,
                .nextState = &RJCoreState_ReadVoltages,
            };
        case BusPirateCommand_TapShift:
            return (struct RJCoreStateReply){
                .bytesProcessed = 0,
                .nextState = &RJCoreState_TapShiftInitialise,
            };
        }
    }

    // Don't know how to handle other commands
    return (struct RJCoreStateReply){
        .bytesProcessed = -1,
        .nextState = NULL,
    };
}

static struct RJCoreStateReply RJCoreState_SetSerialSpeed_Process(struct RJCoreHandle *handle, const uint8_t *data,
                                                                  int bytesAvailable)
{
    if (bytesAvailable == 1)
    {
        uint8_t mode = *data;

        if ((*handle->platform->setSerialMode)(handle->privateData, mode))
        {
            handle->stateData.state.serial.mode = mode;

            // Successfully changed baud rate - go wait for the ack
            return (struct RJCoreStateReply){
                .bytesProcessed = 0,
                .nextState = &RJCoreState_AwaitSerialAck,
            };
        }
    }

    // Unable to support requested serial port mode (or missing data)
    return (struct RJCoreStateReply){
        .bytesProcessed = -1,
        .nextState = NULL,
    };
}

static struct RJCoreStateReply RJCoreState_AwaitSerialAck_Process(struct RJCoreHandle *handle, const uint8_t *data,
                                                                  int bytesAvailable)
{
    // Should I actually check for this?  If some boards use a physical UART
    // then we might be too slow, and 0xAA/0x55 might be corrupted or not received.
    // But having said that, a lot of the high speed boards are USB CDC ACM anyway,
    // so changing the baud rate doesn't really do much, and we won't lose data...

    if (bytesAvailable == 2)
    {
        if ((data[0] == 0xAA) && (data[1] == 0x55))
        {
            uint8_t reply[2] = {
                BusPirateCommand_UartSpeed,
                handle->stateData.state.serial.mode,
            };

            (*handle->platform->transmitData)(handle->privateData, reply, 2);

            return (struct RJCoreStateReply){
                .bytesProcessed = 0,
                .nextState = &RJCoreState_BinaryMode,
            };
        }
    }

    return (struct RJCoreStateReply){
        .bytesProcessed = -1,
        .nextState = NULL,
    };
}

static struct RJCoreStateReply RJCoreState_SetPortMode_Process(struct RJCoreHandle *handle, const uint8_t *data,
                                                               int bytesAvailable)
{
    if ((bytesAvailable == 1) && (*data < RJCorePortMode_Size))
    {
        if ((*handle->platform->setPortMode)(handle->privateData, *data))
        {
            return (struct RJCoreStateReply){
                .bytesProcessed = 0,
                .nextState = &RJCoreState_BinaryMode,
            };
        }
    }

    return (struct RJCoreStateReply){
        .bytesProcessed = -1,
        .nextState = NULL,
    };
}

static struct RJCoreStateReply RJCoreState_SetFeatures_Process(struct RJCoreHandle *handle, const uint8_t *data,
                                                               int bytesAvailable)
{
    if (bytesAvailable == 2)
    {
        uint8_t feature = data[0];
        uint8_t action = data[1];

        if ((*handle->platform->setFeature)(handle->privateData, feature, action))
        {
            return (struct RJCoreStateReply){
                .bytesProcessed = 0,
                .nextState = &RJCoreState_BinaryMode,
            };
        }
    }

    return (struct RJCoreStateReply){
        .bytesProcessed = -1,
        .nextState = NULL,
    };
}

static struct RJCoreStateReply RJCoreState_ReadVoltages_Process(struct RJCoreHandle *handle, const uint8_t *data,
                                                                int bytesAvailable)
{
    (void)data;
    (void)bytesAvailable;

    uint8_t result[10];
    uint16_t voltages[4];

    (*handle->platform->readVoltages)(handle->privateData, voltages);

    result[0] = BusPirateCommand_ReadADCs;
    result[1] = 8; // This constant is what the Bus Pirate firmware provides

    uint8_t *dest = result + 2;

    for (int i = 0; i < 4; ++i)
    {
        *dest = (voltages[i] >> 8) & 0xFF;
        ++dest;
        *dest = (voltages[i] & 0xFF);
        ++dest;
    }

    (*handle->platform->transmitData)(handle->privateData, result, 10);

    return (struct RJCoreStateReply){
        .bytesProcessed = 0,
        .nextState = &RJCoreState_BinaryMode,
    };
}

static struct RJCoreStateReply RJCoreState_TapShiftInitialise_Process(struct RJCoreHandle *handle, const uint8_t *data,
                                                                      int bytesAvailable)
{
    if (bytesAvailable == 2)
    {
        int bitsToShift = (data[0] << 8) | data[1];
        uint8_t reply[3];

        if (bitsToShift > RJCORE_MAXIMUM_TAP_SHIFT_BIT_COUNT)
        {
            bitsToShift = RJCORE_MAXIMUM_TAP_SHIFT_BIT_COUNT;
        }

        handle->stateData.state.tapShift.bitsToShift = bitsToShift;
        handle->stateData.state.tapShift.bufferUsed = 0;
        handle->stateData.state.tapShift.packetBuffer[0] = 0;
        handle->stateData.state.tapShift.packetBuffer[1] = 0;

        if (handle->platform->newTapShift)
        {
            (*handle->platform->newTapShift)(handle->privateData, bitsToShift);
        }

        reply[0] = BusPirateCommand_TapShift;
        reply[1] = (bitsToShift >> 8) & 0xFF;
        reply[2] = bitsToShift & 0xFF;

        (*handle->platform->transmitData)(handle->privateData, reply, 3);

        return (struct RJCoreStateReply){
            .bytesProcessed = 0,
            .nextState = (handle->platform->tapShiftMode == RJCoreTapShiftMode_GPIO)     ? &RJCoreState_TapShiftGPIO
                         : (handle->platform->tapShiftMode == RJCoreTapShiftMode_Packet) ? &RJCoreState_TapShiftPacket
                                                                                         : &RJCoreState_TapShiftCustom,
        };
    }

    return (struct RJCoreStateReply){
        .bytesProcessed = -1,
        .nextState = NULL,
    };
}

static struct RJCoreStateReply RJCoreState_TapShiftGPIO_Process(struct RJCoreHandle *handle, const uint8_t *data,
                                                                int bytesAvailable)
{
    if (bytesAvailable == 2)
    {
        int bitsToToggle = 8;
        uint8_t tdi = data[0];
        uint8_t tms = data[1];
        uint8_t tdo = 0;

        if (handle->stateData.state.tapShift.bitsToShift < 8)
        {
            bitsToToggle = handle->stateData.state.tapShift.bitsToShift;
        }

        for (int bitIndex = 0; bitIndex < bitsToToggle; ++bitIndex)
        {
            tdo |= (*handle->platform->tapShiftGPIO)(handle->privateData, tdi & 1, tms & 1) ? 0x80 : 0x00;

            tdi >>= 1;
            tms >>= 1;
        }

        tdo >>= (8 - bitsToToggle);
        (*handle->platform->transmitData)(handle->privateData, &tdo, 1);

        handle->stateData.state.tapShift.bitsToShift -= bitsToToggle;
    }

    if (handle->stateData.state.tapShift.bitsToShift == 0)
    {
        // All done!  Back to the binary mode selection
        return (struct RJCoreStateReply){
            .bytesProcessed = 0,
            .nextState = &RJCoreState_BinaryMode,
        };
    }

    return (struct RJCoreStateReply){
        .bytesProcessed = 0,
        .nextState = NULL,
    };
}

static int RJCoreState_TapShiftPacket_ProcessInvoke(struct RJCoreHandle *handle, const uint8_t *data,
                                                    int bytesAvailable)
{
    // bytesAvailable should always be even
    if ((bytesAvailable & 0x01) != 0)
    {
        // Internal error, whoops...
        return -1;
    }

    // TDI/TMS comes in pairs, so every 2 bytes available means 8 bits of data, hence * 4
    int bitsAvailable = bytesAvailable << 2;

    if (bitsAvailable > handle->stateData.state.tapShift.bitsToShift)
    {
        bitsAvailable = handle->stateData.state.tapShift.bitsToShift;
    }

    if ((*handle->platform->tapShiftPacket)(handle->privateData, data, bitsAvailable) < 0)
    {
        return -1;
    }

    handle->stateData.state.tapShift.bitsToShift -= bitsAvailable;

    // Return number of actual bytes consumed
    return ((bitsAvailable + 7) & ~0x7u) >> 2;
}

static struct RJCoreStateReply RJCoreState_TapShiftPacket_Process(struct RJCoreHandle *handle, const uint8_t *data,
                                                                  int bytesAvailable)
{
    int bytesProcessed = 0;
    int res = 0;

    if (bytesAvailable == 0)
    {
        // Need data to process
        return (struct RJCoreStateReply){
            .bytesProcessed = 0,
            .nextState = NULL,
        };
    }

    if (handle->stateData.state.tapShift.bufferUsed != 0)
    {
        // Implies that a tdi without a matching tms is being buffered
        handle->stateData.state.tapShift.packetBuffer[1] = data[0];

        res = RJCoreState_TapShiftPacket_ProcessInvoke(handle, handle->stateData.state.tapShift.packetBuffer, 2);

        handle->stateData.state.tapShift.bufferUsed = 0;

        ++data;
        ++bytesProcessed;
        --bytesAvailable;
    }

    // Requires an even number of bytes in a packet
    if ((res == 0) && (bytesAvailable > 1) && (handle->stateData.state.tapShift.bitsToShift > 0))
    {
        // Only ever send even amounts of data, so that tdi/tms can be easily extracted
        res = RJCoreState_TapShiftPacket_ProcessInvoke(handle, data, bytesAvailable & ~0x1u);

        if (res > 0)
        {
            data += res;
            bytesProcessed += res;
            bytesAvailable -= res;
        }
    }

    if ((res == 0) && (bytesAvailable == 1) && (handle->stateData.state.tapShift.bitsToShift > 0))
    {
        // Odd byte out - just buffer it
        handle->stateData.state.tapShift.bufferUsed = 1;
        handle->stateData.state.tapShift.packetBuffer[0] = data[0];

        ++data;
        ++bytesProcessed;
        --bytesAvailable;
    }

    if (res < 0)
    {
        return (struct RJCoreStateReply){
            .bytesProcessed = -1,
            .nextState = NULL,
        };
    }

    return (struct RJCoreStateReply){
        .bytesProcessed = bytesProcessed,
        .nextState = (handle->stateData.state.tapShift.bitsToShift == 0) ? &RJCoreState_BinaryMode : NULL,
    };
}

static struct RJCoreStateReply RJCoreState_TapShiftCustom_Process(struct RJCoreHandle *handle, const uint8_t *data,
                                                                  int bytesAvailable)
{
    int bytesProcessed = bytesAvailable;
    int bitsToShift = handle->stateData.state.tapShift.bitsToShift;

    int bytesRequired = ((bitsToShift + 7) & ~0x7u) >> 2;

    if (bytesProcessed > bytesRequired)
    {
        // Edge case: The entire tap shift is already in the buffer, but has extra data behind it!
        // Ensure that we don't accidentally mark that data as processed.
        bytesProcessed = bytesRequired;
    }

    if ((*handle->platform->tapShiftCustom)(handle->privateData, data, bytesProcessed, bitsToShift) < 0)
    {
        bytesProcessed = -1;
    }

    return (struct RJCoreStateReply){
        .bytesProcessed = bytesProcessed,
        .nextState = (bytesProcessed < 0) ? NULL : &RJCoreState_BinaryMode,
    };
}