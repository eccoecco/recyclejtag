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

void RJCore_NotifyDataReceived(struct RJCoreState *state, const char *data, size_t bytesReceived)
{
    bool rxActivity = (bytesReceived > 0);
    state->currentTime = state->platform->currentUptime(state->privateData);

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

    if (rxActivity)
    {
        state->lastRxActivityTime = state->currentTime;
    }
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
    if ((state->currentTime - state->lastRxActivityTime) > RJCORE_TIMEOUT_HANDSHAKE)
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
                    state->platform->transmitData(state->privateData, "BBIO1", 5);
                }
            }
            // Okay to keep on receiving lots of zeroes - it might be that some were errant
            break;
        case BusPirateCommand_EnterOpenOCD:
            if (state->state.handshake.validBytes == 20)
            {
                state->platform->transmitData(state->privateData, "OCD1", 4);
                RJCore_ChangeToState(state, RJCoreState_BinaryMode);
                return bytesProcessed;
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
    if ((state->currentTime - state->lastRxActivityTime) > RJCORE_TIMEOUT_BINARY_MODE)
    {
        return -1;
    }

    if (state->bytesReceived == 0)
    {
        return 0;
    }

    // TODO: Handle binary mode commands

    return -1;
}
