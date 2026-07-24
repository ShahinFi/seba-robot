#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    /*
     * valid is true only when the IMU and encoder state are
     * both usable for closed-loop control.
     */
    bool valid;
    bool imu_valid;
    bool imu_stale;
    bool encoder_valid;

    /*
     * Update counters and ages expose estimator health to
     * telemetry, event logs, and start-up checks.
     */
    uint32_t update_count;
    uint32_t orientation_update_count;
    uint32_t gyroscope_update_count;
    uint32_t orientation_age_ms;
    uint32_t gyroscope_age_ms;

    /*
     * Control-model state:
     *   v         = forward velocity
     *   theta     = body pitch
     *   theta_dot = body pitch rate
     *   psi_dot   = yaw rate
     */
    float forward_velocity_mps;
    float pitch_rad;
    float pitch_rate_rads;
    float yaw_rate_rads;

    /*
     * Derivatives used by the RSLQR augmented state.
     */
    float forward_acceleration_mps2;
    float pitch_acceleration_rads2;
    float yaw_acceleration_rads2;
} RobotState;

/*
 * Starts the fixed-rate estimator timer. IMU and encoders must
 * already be initialized.
 */
void StateEstimation_Init(void);

/*
 * Resets encoder-derived velocity state without changing the
 * current IMU orientation.
 */
void StateEstimation_Reset(void);

/*
 * Copies the latest estimator snapshot.
 */
void StateEstimation_GetState(
    RobotState *output
);

#endif
