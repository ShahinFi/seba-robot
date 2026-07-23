#ifndef CONTROL_PARAMETERS_H
#define CONTROL_PARAMETERS_H

#include <stdint.h>

typedef struct
{
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
    float velocity_filter_cutoff_hz;
    float derivative_filter_cutoff_hz;
    float max_wheel_torque_mnm;
    float fall_angle_rad;
    float motion_gain_scale;
    float rslqr_gain_nm_s[2][6];
} ControlParameters;

void ControlParameters_Get(
    ControlParameters *parameters
);

#endif
