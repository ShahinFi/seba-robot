#ifndef CONTROL_PARAMETERS_H
#define CONTROL_PARAMETERS_H

#include <stdint.h>

/*
 * Physical model, estimator timing, safety limits, and RSLQR
 * defaults shared by the STM32 control modules.
 */
typedef struct
{
    /*
     * Field suffixes carry the physical units.
     *
     * RSLQR gain rows:
     *   0 = left wheel torque rate
     *   1 = right wheel torque rate
     *
     * RSLQR gain columns:
     *   0 = v error
     *   1 = psi_dot error
     *   2 = v_dot
     *   3 = theta_dot
     *   4 = theta_ddot
     *   5 = psi_ddot
     */
    float body_mass_kg;
    float wheel_radius_m;
    float wheel_separation_m;
    float body_com_height_m;
    float body_pitch_inertia_kg_m2;
    float body_yaw_inertia_kg_m2;
    float wheel_pitch_inertia_kg_m2;
    float wheel_yaw_inertia_kg_m2;
    float wheel_damping_nms_rad;
    uint32_t encoder_counts_per_wheel_rev;
    uint32_t state_estimator_rate_hz;
    uint32_t motion_controller_rate_hz;
    uint32_t imu_orientation_timeout_ms;
    uint32_t imu_gyroscope_timeout_ms;
    uint32_t motion_command_timeout_ms;
    float velocity_filter_cutoff_hz;
    float derivative_filter_cutoff_hz;
    float max_wheel_torque_mnm;
    float start_angle_rad;
    float fall_angle_rad;
    float motion_gain_scale;
    float rslqr_gain_nm_s[2][6];
} ControlParameters;

/*
 * Copies the immutable default parameter set.
 */
void ControlParameters_Get(
    ControlParameters *parameters
);

#endif
