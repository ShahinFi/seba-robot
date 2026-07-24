#ifndef STATE_EVENT_FORMAT_H
#define STATE_EVENT_FORMAT_H

#include "control/state_estimation/state_estimation.h"

/*
 * Appends compact RobotState key/value fields to the current
 * serial line for event diagnostics.
 *
 * Units match RobotState:
 *   ages in ms
 *   theta in rad
 *   theta_dot in rad/s
 */
void StateEventFormat_WriteState(
    const RobotState *state
);

#endif
