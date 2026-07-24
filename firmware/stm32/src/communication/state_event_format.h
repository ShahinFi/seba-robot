#ifndef STATE_EVENT_FORMAT_H
#define STATE_EVENT_FORMAT_H

#include "control/state_estimation/state_estimation.h"

/*
 * Appends compact RobotState key/value fields to the current
 * serial line for event diagnostics.
 */
void StateEventFormat_WriteState(
    const RobotState *state
);

#endif
