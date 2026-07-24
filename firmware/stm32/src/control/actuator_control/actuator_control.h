#ifndef ACTUATOR_CONTROL_H
#define ACTUATOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    /*
     * All limits and gains use physical actuator units. The
     * serial tuner sends these values directly.
     */
    uint32_t control_period_ms;
    int32_t battery_voltage_mv;
    int32_t proportional_gain_mv_per_a;
    int32_t integral_gain_mv_per_a_s;
    int32_t integral_limit_mv;
    int32_t max_current_reference_ma;
    int16_t max_command_percent;
    int32_t wheel_torque_constant_mnm_per_a;
} ActuatorControlConfig;

typedef struct
{
    /*
     * PWM command values are signed per-mille:
     * -1000 = full reverse
     *     0 = stopped
     * +1000 = full forward
     */
    bool enabled;
    bool fault_active;
    bool read_failed;
    int32_t left_current_reference_ma;
    int32_t right_current_reference_ma;
    int32_t left_current_measured_ma;
    int32_t right_current_measured_ma;
    int32_t left_error_ma;
    int32_t right_error_ma;
    int32_t left_integral_mv;
    int32_t right_integral_mv;
    int16_t left_command_permille;
    int16_t right_command_permille;
} ActuatorControlStatus;

/*
 * Initializes the actuator current loop and its timer. Current
 * sensors and motor GPIO/PWM must already be initialized.
 */
void ActuatorControl_Init(void);

/*
 * Enable wakes the motor drivers and starts closed-loop current
 * control from zero integrator state. Disable stops and sleeps
 * both motor drivers.
 */
void ActuatorControl_Enable(void);
void ActuatorControl_Disable(void);
bool ActuatorControl_IsEnabled(void);

/*
 * Current references are signed milliamps. Positive current
 * follows the robot-forward wheel torque convention.
 */
bool ActuatorControl_SetCurrentReferences(
    int32_t left_current_ma,
    int32_t right_current_ma
);

/*
 * Wheel torque references are signed millinewton-meters and are
 * converted to current using the configured torque constant.
 */
bool ActuatorControl_SetWheelTorqueReferences(
    int32_t left_torque_mnm,
    int32_t right_torque_mnm
);

/*
 * Control period in milliseconds. Valid range is 1 to 1000.
 */
bool ActuatorControl_SetControlPeriod(uint32_t control_period_ms);

/*
 * Battery voltage in millivolts. Must be positive.
 */
bool ActuatorControl_SetBatteryVoltage(int32_t battery_voltage_mv);

/*
 * Proportional gain in mV/A. Must be nonnegative.
 */
bool ActuatorControl_SetProportionalGain(int32_t gain_mv_per_a);

/*
 * Integral gain in mV/(A*s). Must be nonnegative.
 */
bool ActuatorControl_SetIntegralGain(int32_t gain_mv_per_a_s);

/*
 * Integral clamp in millivolts. Must be nonnegative.
 */
bool ActuatorControl_SetIntegralLimit(int32_t integral_limit_mv);

/*
 * Maximum absolute current reference in milliamps. Must be
 * nonnegative.
 */
bool ActuatorControl_SetMaxCurrentReference(int32_t max_current_ma);

/*
 * Maximum absolute PWM command in percent. Valid range is
 * 0 to 100.
 */
bool ActuatorControl_SetMaxCommandPercent(int16_t max_command_percent);

/*
 * Wheel torque constant in mN*m/A. Must be positive.
 */
bool ActuatorControl_SetWheelTorqueConstant(int32_t torque_constant_mnm_per_a);

/*
 * Copies the current actuator-loop configuration.
 */
void ActuatorControl_GetConfig(
    ActuatorControlConfig *output
);

/*
 * Copies the latest current-loop status snapshot.
 */
void ActuatorControl_GetStatus(
    ActuatorControlStatus *output
);

#endif
