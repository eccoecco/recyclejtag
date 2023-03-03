/**

    \file rjcore/rjcore.h
    \brief Recycle JTAG Core file

*/

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

/// @brief Recycle JTAG core state machine
struct RJCoreState
{
    int JustForBuilding;
};

/// @brief Initialises the core state machine to start
/// @param state Pointer to the state to initialise
void RJCoreInit(struct RJCoreState *state);

#ifdef __cplusplus
}
#endif