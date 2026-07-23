#include "control_parameters.h"

#include <stddef.h>

static const ControlParameters control_parameters =
{
    .body_mass_kg = 2.90F,
    .wheel_radius_m = 0.035F,
    .wheel_separation_m = 0.10F,
    .body_com_height_m = 0.125F,
    .body_pitch_inertia_kg_m2 = 0.01752F,
    .body_yaw_inertia_kg_m2 = 0.00483F,
    .wheel_pitch_inertia_kg_m2 = 3.0625e-5F,
    .wheel_yaw_inertia_kg_m2 = 1.90625e-5F,
    .wheel_damping_nms_rad = 0.0005F,
    .encoder_counts_per_wheel_rev = 2800U,
    .state_estimator_rate_hz = 200U,
    .motion_controller_rate_hz = 200U,
    .velocity_filter_cutoff_hz = 20.0F,
    .derivative_filter_cutoff_hz = 15.0F,
    .max_wheel_torque_mnm = 5000.0F,
    .fall_angle_rad = 1.047197551F,
    .motion_gain_scale = 2.5F,
    .rslqr_gain_nm_s =
    {
        {
            -2.0F,
            -1.00000000000000F,
            -2.8F,
            -10.0F,
            -1.2F,
            -0.16F
        },
        {
            -2.0F,
             1.00000000000000F,
            -2.8F,
            -10.0F,
            -1.2F,
             0.16F
        }
    }
};

void ControlParameters_Get(
    ControlParameters *parameters
)
{
    if (parameters == NULL)
    {
        return;
    }

    *parameters = control_parameters;
}
