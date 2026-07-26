#include "control_parameters.h"

#include <stddef.h>

/*
 * Central control constants.
 *
 * These values define the physical model, estimator timing,
 * safety limits, and controller defaults used by the STM32
 * runtime and Raspberry Pi tuner.
 */
static const ControlParameters control_parameters =
{
    /*
     * Rigid-body model parameters used by the RSLQR design.
     */
    .body_mass_kg = 2.32F,
    .wheel_radius_m = 0.0325F,
    .wheel_separation_m = 0.25F,
    .body_com_height_m = 0.1112F,
    .body_pitch_inertia_kg_m2 = 0.0153F,
    .body_yaw_inertia_kg_m2 = 0.0158F,
    .wheel_pitch_inertia_kg_m2 = 3.0e-5F,
    .wheel_yaw_inertia_kg_m2 = 2.0e-5F,
    .wheel_damping_nms_rad = 0.003F,
    .encoder_counts_per_wheel_rev = 2800U,

    /*
     * Estimator and controller rates are matched so motion
     * control consumes one completed state per control tick.
     */
    .state_estimator_rate_hz = 200U,
    .motion_controller_rate_hz = 200U,
    .imu_orientation_timeout_ms = 50U,
    .imu_gyroscope_timeout_ms = 50U,
    .motion_command_timeout_ms = 300U,
    .velocity_filter_cutoff_hz = 20.0F,
    .derivative_filter_cutoff_hz = 15.0F,

    /*
     * Runtime safety and tuning defaults. These values match the
     * Raspberry Pi tuner defaults unless changed over serial.
     */
    .max_wheel_torque_mnm = 1300.0F,
    .fall_angle_rad = 1.047197551F,
    .motion_gain_scale = 1.0F,

    /*
     * RSLQR torque-rate gain rows:
     *   row 0 = left wheel
     *   row 1 = right wheel
     */
    .rslqr_gain_nm_s =
    {
        {
            -2.0F,
            -1.00000000000000F,
            -2.3F,
            -8.0F,
            -0.7F,
            -0.16F
        },
        {
            -2.0F,
             1.00000000000000F,
            -2.3F,
            -8.0F,
            -0.7F,
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
