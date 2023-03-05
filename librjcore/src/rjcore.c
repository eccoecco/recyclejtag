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

static int RJCoreState_Handshake(struct RJCoreState *state);
static int RJCoreState_BinaryMode(struct RJCoreState *state);
static int RJCoreState_SetSerialSpeed(struct RJCoreState *state);
static int RJCoreState_AwaitAck(struct RJCoreState *state);

static inline void RJCore_ChangeToState(struct RJCoreState *state, RJCoreStateCallback callback)
{
    // Each state gets notified that it is entering this state.
    (*callback)(state);

    state->stateCallback = callback;
}

void RJCore_Init(struct RJCoreState *state, struct RJCorePlatform *platform, void *privateData)
{
    memset(state, 0, sizeof(*state));

    state->platform = platform;
    state->privateData = privateData;

    RJCore_ChangeToState(state, RJCoreState_Handshake);
}

void RJCore_NotifyDataReceived(struct RJCoreState *state, const uint8_t *data, size_t bytesReceived)
{
    uint32_t currentTime = state->platform->currentUptime(state->privateData);

    if (bytesReceived > 0)
    {
        state->timeSinceLastRxActivity = 0;
        state->lastRxActivityTime = currentTime;
    }
    else
    {
        state->timeSinceLastRxActivity = currentTime - state->lastRxActivityTime;
    }

    do
    {
        int bytesProcessed;
        bool resetToInit = false;

        RJCoreStateCallback currentCallback = state->stateCallback;
        state->bytesReceived = bytesReceived;
        state->receiveBuffer = data;

        bytesProcessed = state->stateCallback(state);
        if (bytesProcessed < 0)
        {
            resetToInit = true;
        }
        else if (bytesProcessed == 0)
        {
            // bytesReceived == 0, if checking for timeout
            if ((bytesReceived > 0) && (state->stateCallback == currentCallback))
            {
                // Why is this state not processing any data and remaining in the current
                // state?  This is considered an internal bug, but I don't want to lock up
                // the system, so just reset back to init.
                resetToInit = true;
            }
        }
        else if (bytesProcessed > (int)bytesReceived)
        {
            // How can you process more data than was received?!
            resetToInit = true;
        }
        else
        {
            bytesReceived -= bytesProcessed;
            data += bytesProcessed;
        }

        if (resetToInit)
        {
            // Reset state callback to null, so that the handshake state thinks
            // that it's entering a fresh state.
            state->stateCallback = NULL;
            RJCore_ChangeToState(state, RJCoreState_Handshake);

            bytesReceived = 0;
        }

    } while (bytesReceived > 0);
}

static int RJCoreState_Handshake(struct RJCoreState *state)
{
    if (state->stateCallback != RJCoreState_Handshake)
    {
        // Entering this state, so just clear our state
        state->state.handshake.validBytes = 0;
        return 0;
    }

    // If nothing has happened for a while, reset everything
    if (state->timeSinceLastRxActivity > RJCORE_TIMEOUT_HANDSHAKE)
    {
        state->state.handshake.validBytes = 0;
    }

    int bytesProcessed;

    for (bytesProcessed = 0; bytesProcessed < state->bytesReceived; ++bytesProcessed)
    {
        switch (state->receiveBuffer[bytesProcessed])
        {
        case BusPirateCommand_Unknown:
            // Bus Pirate says that it should only send a response after 20 consecutive zeroes
            // but that errant zeroes can happen (e.g. during startup), so it is possible that
            // more than 20 consecutive zeroes come streaming in
            if (state->state.handshake.validBytes < 20)
            {
                ++state->state.handshake.validBytes;

                if (state->state.handshake.validBytes == 20)
                {
                    // Send the Bus Pirate identifier
                    (*state->platform->transmitData)(state->privateData, "BBIO1", 5);
                }
            }
            // Okay to keep on receiving lots of zeroes - it might be that some were errant
            break;
        case BusPirateCommand_EnterOpenOCD:
            if (state->state.handshake.validBytes == 20)
            {
                (*state->platform->transmitData)(state->privateData, "OCD1", 4);
                RJCore_ChangeToState(state, RJCoreState_BinaryMode);
                // +1, have processed the current byte
                return bytesProcessed + 1;
            }
            else
            {
                return -1;
            }
            break;
        default:
            // Don't know how to handle this command - just reset to initial state so that
            // we don't get into an odd state
            return -1;
        }
    }

    // Maybe more zeroes are coming in future?
    return bytesProcessed;
}

static int RJCoreState_BinaryMode(struct RJCoreState *state)
{
    if (state->stateCallback != RJCoreState_BinaryMode)
    {
        // Entering this state
        return 0;
    }

    // If nothing has happened for a while, go back to handshake mode
    if (state->timeSinceLastRxActivity > RJCORE_TIMEOUT_BINARY_MODE)
    {
        return -1;
    }

    if (state->bytesReceived == 0)
    {
        return 0;
    }

    switch (state->receiveBuffer[0])
    {
    case BusPirateCommand_UartSpeed:
        RJCore_ChangeToState(state, RJCoreState_SetSerialSpeed);
        return 1;
    }

    // TODO: Handle binary mode commands

    return -1;
}

static int RJCoreState_SetSerialSpeed(struct RJCoreState *state)
{
    if (state->stateCallback != RJCoreState_SetSerialSpeed)
    {
        state->state.serial.mode = 0;
        state->state.serial.value = 0;
        return 0;
    }

    if (state->bytesReceived == 0)
    {
        return 0;
    }

    uint8_t newMode = state->receiveBuffer[0];
    state->state.serial.mode = newMode;

    if (!(*state->platform->setSerialMode)(state->privateData, newMode))
    {
        // Unable to support requested serial port mode
        return -1;
    }

    RJCore_ChangeToState(state, RJCoreState_AwaitAck);

    return 1;
}

static int RJCoreState_AwaitAck(struct RJCoreState *state)
{
    if (state->stateCallback != RJCoreState_AwaitAck)
    {
        return 0;
    }

    if (state->timeSinceLastRxActivity >= RJCORE_TIMEOUT_SERIAL_MODE_ACK)
    {
        // Timed out!  Just go back to handshake mode
        return -1;
    }

    if (state->bytesReceived == 0)
    {
        return 0;
    }

    int bytesProcessed = 0;
    const uint8_t *unprocessedByte = state->receiveBuffer;

    if (state->state.serial.value == 0)
    {
        if (*unprocessedByte != 0xAA)
        {
            return -1;
        }

        ++state->state.serial.value;
        ++bytesProcessed;
        ++unprocessedByte;
    }

    if ((state->state.serial.value == 1) && (bytesProcessed < state->bytesReceived))
    {
        if (*unprocessedByte != 0x55)
        {
            return -1;
        }

        char reply[2] = {
            BusPirateCommand_UartSpeed,
            state->state.serial.mode,
        };

        (*state->platform->transmitData)(state->privateData, reply, 2);

        RJCore_ChangeToState(state, RJCoreState_BinaryMode);
        ++bytesProcessed;
    }

    return bytesProcessed;
}
