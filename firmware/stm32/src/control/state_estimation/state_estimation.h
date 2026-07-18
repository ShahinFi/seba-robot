#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    bool valid;
    bool imu_valid;
    bool encoder_valid;
    uint32_t update_count;
    float forward_velocity_mps;
    float pitch_rad;
    float pitch_rate_rads;
    float yaw_rate_rads;
    float forward_acceleration_mps2;
    float pitch_acceleration_rads2;
    float yaw_acceleration_rads2;
} RobotState;

void StateEstimation_Init(void);
void StateEstimation_Reset(void);

void StateEstimation_GetState(
    RobotState *output
);

#endif
