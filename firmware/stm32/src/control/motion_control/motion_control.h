#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "control/state_estimation/state_estimation.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MOTION_CONTROL_FAULT_NONE = 0,

    /*
     * The estimator snapshot is not valid for control.
     */
    MOTION_CONTROL_FAULT_STATE_INVALID,

    /*
     * IMU reports exist but are older than the configured
     * freshness limit.
     */
    MOTION_CONTROL_FAULT_IMU_STALE,

    /*
     * Pitch exceeded the configured fall angle.
     */
    MOTION_CONTROL_FAULT_FALL,

    /*
     * The actuator layer rejected or faulted the torque command.
     */
    MOTION_CONTROL_FAULT_ACTUATOR
} MotionControlFault;

typedef struct
{
    /*
     * Status is copied by command and telemetry paths while the
     * controller runs from the estimator interrupt.
     */
    bool enabled;
    bool fault_active;
    bool state_invalid;
    bool fall_detected;
    MotionControlFault fault;
    RobotState fault_state;

    /*
     * Operator command freshness. The command watchdog zeros
     * stale commands without disabling balance.
     */
    uint32_t command_age_ms;
    uint32_t command_update_count;
    float forward_velocity_command_mps;
    float yaw_rate_command_rads;

    /*
     * Torque commands are wheel-side millinewton-meters.
     * Torque rates are millinewton-meters per second.
     */
    float left_torque_command_mnm;
    float right_torque_command_mnm;
    float left_torque_rate_mnm_s;
    float right_torque_rate_mnm_s;
    float max_wheel_torque_mnm;
    float motion_gain_scale;
} MotionControlStatus;

void MotionControl_Init(void);

/*
 * Enable starts balance control from zero motion command and
 * zero stored wheel torque. Disable stops actuator output.
 */
void MotionControl_Enable(void);
void MotionControl_Disable(void);
bool MotionControl_IsEnabled(void);

/*
 * Maximum absolute wheel-side torque command in millinewton-
 * meters.
 */
bool MotionControl_SetMaxWheelTorque(
    float max_wheel_torque_mnm
);

/*
 * Multiplier applied to the full RSLQR gain matrix.
 */
bool MotionControl_SetGainScale(
    float gain_scale
);

/*
 * Commands are forward velocity in m/s and yaw rate in rad/s.
 * The command watchdog returns both commands to zero if updates
 * stop arriving.
 */
bool MotionControl_SetCommand(
    float forward_velocity_command_mps,
    float yaw_rate_command_rads
);

/*
 * Updates one element of the 2x6 RSLQR gain matrix.
 *
 * row:
 *   0 = left wheel
 *   1 = right wheel
 *
 * column:
 *   0 = v error
 *   1 = psi_dot error
 *   2 = v_dot
 *   3 = theta_dot
 *   4 = theta_ddot
 *   5 = psi_ddot
 */
bool MotionControl_SetGain(
    uint32_t row,
    uint32_t column,
    float gain
);

/*
 * Runs one controller update from a completed RobotState
 * snapshot. Called from the fixed-rate state-estimator tick.
 */
void MotionControl_Update(
    const RobotState *state
);

/*
 * Copies the latest motion-control status snapshot.
 */
void MotionControl_GetStatus(
    MotionControlStatus *output
);

/*
 * Returns the stable text token used in telemetry and events.
 */
const char *MotionControl_FaultName(
    MotionControlFault fault
);

#endif
