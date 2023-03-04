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

static void RJCoreState_Handshake(struct RJCoreState *state);
static void RJCoreState_BinaryMode(struct RJCoreState *state);

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

void RJCore_NotifyDataReceived(const char *data, size_t bytesReceived)
{
}

void RJCore_NotifyTimer(void)
{
    // TODO: Check the last time activity was detected, and time out if too long
}

static void RJCoreState_Handshake(struct RJCoreState *state)
{
    if (state->stateCallback != RJCoreState_Handshake)
    {
        // Entering this state
        state->bytesBeforeCallback = 1;
        state->state.handshake.validBytes = 0;

        return;
    }

    // If nothing has happened for a while, reset everything
    if ((state->currentRxActivityTime - state->lastRxActivityTime) > RJCORE_TIMEOUT_HANDSHAKE)
    {
        state->state.handshake.validBytes = 0;
    }

    if (state->bytesInBuffer == 0)
    {
        // Can occur if a timer check was invoked
        return;
    }

    switch (state->readBuffer[0])
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
        break;
    case BusPirateCommand_EnterOpenOCD:
        if (state->state.handshake.validBytes == 20)
        {
            state->platform->transmitData(state->privateData, "OCD1", 4);
            RJCore_ChangeToState(state, RJCoreState_BinaryMode);
            return;
        }
        break;
    default:
        // Unknown command - just reset to initial state
        state->state.handshake.validBytes = 0;
        break;
    }
}

static void RJCoreState_BinaryMode(struct RJCoreState *state)
{
    if (state->stateCallback != RJCoreState_BinaryMode)
    {
        // Entering this state
        state->bytesBeforeCallback = 1;
        return;
    }

    // If nothing has happened for a while, go back to handshake mode
    if ((state->currentRxActivityTime - state->lastRxActivityTime) > RJCORE_TIMEOUT_BINARY_MODE)
    {
        RJCore_ChangeToState(state, RJCoreState_Handshake);
        return;
    }

    if (state->bytesInBuffer == 0)
    {
        return;
    }
}
