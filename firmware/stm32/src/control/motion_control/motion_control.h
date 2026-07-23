#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "control/state_estimation/state_estimation.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    bool enabled;
    bool fault_active;
    bool state_invalid;
    bool fall_detected;
    float forward_velocity_command_mps;
    float yaw_rate_command_rads;
    float left_torque_command_mnm;
    float right_torque_command_mnm;
    float left_torque_rate_mnm_s;
    float right_torque_rate_mnm_s;
    float max_wheel_torque_mnm;
    float motion_gain_scale;
} MotionControlStatus;

void MotionControl_Init(void);
void MotionControl_Enable(void);
void MotionControl_Disable(void);
bool MotionControl_IsEnabled(void);
bool MotionControl_SetMaxWheelTorque(
    float max_wheel_torque_mnm
);

bool MotionControl_SetGainScale(
    float gain_scale
);

bool MotionControl_SetCommand(
    float forward_velocity_command_mps,
    float yaw_rate_command_rads
);

bool MotionControl_SetGain(
    uint32_t row,
    uint32_t column,
    float gain
);

void MotionControl_Update(
    const RobotState *state
);

void MotionControl_GetStatus(
    MotionControlStatus *output
);

#endif
