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

#if 0
static int RJCoreState_SetSerialSpeed(struct RJCoreState *state);
static int RJCoreState_AwaitAck(struct RJCoreState *state);
static int RJCoreState_SetPortState(struct RJCoreState *state);
static int RJCoreState_SetFeature(struct RJCoreState *state);
static int RJCoreState_ReadVoltages(struct RJCoreState *state);
static int RJCoreState_InitTapShift(struct RJCoreState *state);
static int RJCoreState_ExecuteTapShift(struct RJCoreState *state);
#endif

static void RJCoreState_Handshake_Enter(struct RJCoreHandle *);
static struct RJCoreStateReply RJCoreState_Handshake_Process(struct RJCoreHandle *, const uint8_t *, int);
static struct RJCoreStateReply RJCoreState_BinaryMode_Process(struct RJCoreHandle *, const uint8_t *, int);
static struct RJCoreStateReply RJCoreState_SetSerialSpeed_Process(struct RJCoreHandle *, const uint8_t *, int);
static struct RJCoreStateReply RJCoreState_AwaitSerialAck_Process(struct RJCoreHandle *, const uint8_t *, int);

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
    else if (handle->currentState != &RJCoreState_Handshake)
    {
        uint32_t idleTime = currentTime - handle->lastRxActivityTime;

        if (idleTime >= RJCORE_UART_IDLE_TIMEOUT)
        {
            // Took too long to receive any data, just reset.
            RJCore_ChangeToState(handle, &RJCoreState_Handshake);
        }
    }
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
#if 0
    case BusPirateCommand_PortMode:
        RJCore_ChangeToState(state, RJCoreState_SetPortState);
        return 1;
    case BusPirateCommand_Feature:
        RJCore_ChangeToState(state, RJCoreState_SetFeature);
        return 1;
    case BusPirateCommand_ReadADCs:
        RJCore_ChangeToState(state, RJCoreState_ReadVoltages);
        return 1;
    case BusPirateCommand_TapShift:
        RJCore_ChangeToState(state, RJCoreState_InitTapShift);
        return 1;
#endif
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

#if 0

static int RJCoreState_SetPortState(struct RJCoreState *state)
{
    if (state->stateCallback != RJCoreState_SetPortState)
    {
        return 0;
    }

    if (state->bytesReceived == 0)
    {
        return 0;
    }

    if (state->receiveBuffer[0] >= RJCorePortMode_Size)
    {
        // Unknown port mode
        return -1;
    }

    if (!(*state->platform->setPortMode)(state->privateData, state->receiveBuffer[0]))
    {
        return -1;
    }

    RJCore_ChangeToState(state, RJCoreState_BinaryMode);

    return 1;
}

static int RJCoreState_SetFeature(struct RJCoreState *state)
{
    if (state->stateCallback != RJCoreState_SetFeature)
    {
        state->state.feature.counter = 0;
        state->state.feature.feature = 0;

        return 0;
    }

    if (state->bytesReceived == 0)
    {
        return 0;
    }

    const uint8_t *buffer = state->receiveBuffer;
    int bytesProcessed = 0;

    if (state->state.feature.counter == 0)
    {
        state->state.feature.feature = *buffer;

        ++state->state.feature.counter;
        ++buffer;
        ++bytesProcessed;
    }

    if ((state->state.feature.counter == 1) && (bytesProcessed < state->bytesReceived))
    {
        if (!(*state->platform->setFeature)(state->privateData, state->state.feature.feature, *buffer))
        {
            return -1;
        }

        RJCore_ChangeToState(state, RJCoreState_BinaryMode);

        ++bytesProcessed;
    }

    return bytesProcessed;
}

static int RJCoreState_ReadVoltages(struct RJCoreState *state)
{
    if (state->stateCallback != RJCoreState_ReadVoltages)
    {
        return 0;
    }

    uint8_t result[10];
    uint16_t voltages[4];

    (*state->platform->readVoltages)(state->privateData, voltages);

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

    (*state->platform->transmitData)(state->privateData, result, 10);

    RJCore_ChangeToState(state, RJCoreState_BinaryMode);

    return 0;
}

static int RJCoreState_InitTapShift(struct RJCoreState *state)
{
    if (state->stateCallback != RJCoreState_InitTapShift)
    {
        state->state.tapShift.bitsToShift = 0;
        state->state.tapShift.counter = 0;

        return 0;
    }

    int bytesProcessed = 0;
    const uint8_t *buffer = state->receiveBuffer;

    while (state->state.tapShift.counter < 2)
    {
        if (bytesProcessed >= state->bytesReceived)
        {
            break;
        }

        state->state.tapShift.bitsToShift <<= 8;
        state->state.tapShift.bitsToShift += *buffer;

        ++state->state.tapShift.counter;
        ++bytesProcessed;
        ++buffer;
    }

    if (state->state.tapShift.counter == 2)
    {
        uint8_t reply[3];
        reply[0] = BusPirateCommand_TapShift;

        if (state->state.tapShift.bitsToShift > RJCORE_MAXIMUM_TAP_SHIFT_BIT_COUNT)
        {
            state->state.tapShift.bitsToShift = RJCORE_MAXIMUM_TAP_SHIFT_BIT_COUNT;
        }

        // Bus Pirate proper echoes back what was received, but this sends back the actual bits shifted
        // just to determine the limit.  It doesn't actually matter, though, because OpenOCD ignores the
        // reply... I could just fill this with 0s.
        reply[1] = (state->state.tapShift.bitsToShift >> 8) & 0xFF;
        reply[2] = state->state.tapShift.bitsToShift & 0xFF;

        (*state->platform->transmitData)(state->privateData, reply, 3);

        RJCore_ChangeToState(state, RJCoreState_ExecuteTapShift);
    }

    return bytesProcessed;
}

static int RJCoreState_ExecuteTapShift(struct RJCoreState *state)
{
    if (state->stateCallback != RJCoreState_ExecuteTapShift)
    {
        // TODO: Query platform if it wants to handle all tap shift commands
        // or if it wants to let us manually bit bash

        return 0;
    }

    if (state->bytesReceived == 0)
    {
        return 0;
    }

    return -1;
}

#endif