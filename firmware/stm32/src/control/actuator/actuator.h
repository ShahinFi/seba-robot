#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t control_period_ms;
    int32_t battery_voltage_mv;
    int32_t proportional_gain_mv_per_a;
    int32_t integral_gain_mv_per_a_s;
    int32_t integral_limit_mv;
    int32_t max_current_reference_ma;
    int16_t max_command_percent;
    int32_t wheel_torque_constant_mnm_per_a;
} ActuatorConfig;

typedef struct
{
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
} ActuatorStatus;

void Actuator_Init(void);

void Actuator_Enable(void);
void Actuator_Disable(void);
bool Actuator_IsEnabled(void);

bool Actuator_SetCurrentReferences(
    int32_t left_current_ma,
    int32_t right_current_ma
);

bool Actuator_SetWheelTorqueReferences(
    int32_t left_torque_mnm,
    int32_t right_torque_mnm
);

bool Actuator_SetControlPeriod(uint32_t control_period_ms);
bool Actuator_SetBatteryVoltage(int32_t battery_voltage_mv);
bool Actuator_SetProportionalGain(int32_t gain_mv_per_a);
bool Actuator_SetIntegralGain(int32_t gain_mv_per_a_s);
bool Actuator_SetIntegralLimit(int32_t integral_limit_mv);
bool Actuator_SetMaxCurrentReference(int32_t max_current_ma);
bool Actuator_SetMaxCommandPercent(int16_t max_command_percent);
bool Actuator_SetWheelTorqueConstant(int32_t torque_constant_mnm_per_a);

void Actuator_GetConfig(
    ActuatorConfig *output
);

void Actuator_GetStatus(
    ActuatorStatus *output
);

#endif
