#include "telemetry_stream.h"

#include "control/actuator/actuator.h"
#include "control/motion_control/motion_control.h"
#include "control/state_estimation/state_estimation.h"
#include "serial/serial.h"

void TelemetryStream_WriteSnapshot(void)
{
    RobotState state;
    MotionControlStatus balance;
    ActuatorStatus actuator;

    StateEstimation_GetState(
        &state
    );

    MotionControl_GetStatus(
        &balance
    );

    Actuator_GetStatus(
        &actuator
    );

    Serial_Write("TEL valid=");
    Serial_Write(state.valid ? "1" : "0");

    Serial_Write(" imu=");
    Serial_Write(state.imu_valid ? "1" : "0");

    Serial_Write(" enc=");
    Serial_Write(state.encoder_valid ? "1" : "0");

    Serial_Write(" updates=");
    Serial_WriteUInt32(state.update_count);

    Serial_Write(" v=");
    Serial_WriteFloat3(state.forward_velocity_mps);

    Serial_Write(" theta=");
    Serial_WriteFloat3(state.pitch_rad);

    Serial_Write(" theta_dot=");
    Serial_WriteFloat3(state.pitch_rate_rads);

    Serial_Write(" psi_dot=");
    Serial_WriteFloat3(state.yaw_rate_rads);

    Serial_Write(" v_dot=");
    Serial_WriteFloat3(state.forward_acceleration_mps2);

    Serial_Write(" theta_ddot=");
    Serial_WriteFloat3(state.pitch_acceleration_rads2);

    Serial_Write(" psi_ddot=");
    Serial_WriteFloat3(state.yaw_acceleration_rads2);

    Serial_Write(" balance=");
    Serial_Write(balance.enabled ? "1" : "0");

    Serial_Write(" fault=");
    Serial_Write(balance.fault_active ? "1" : "0");

    Serial_Write(" fall=");
    Serial_Write(balance.fall_detected ? "1" : "0");

    Serial_Write(" v_cmd=");
    Serial_WriteFloat3(balance.forward_velocity_command_mps);

    Serial_Write(" yaw_cmd=");
    Serial_WriteFloat3(balance.yaw_rate_command_rads);

    Serial_Write(" left_T=");
    Serial_WriteFloat3(balance.left_torque_command_mnm);

    Serial_Write(" right_T=");
    Serial_WriteFloat3(balance.right_torque_command_mnm);

    Serial_Write(" left_dT=");
    Serial_WriteFloat3(balance.left_torque_rate_mnm_s);

    Serial_Write(" right_dT=");
    Serial_WriteFloat3(balance.right_torque_rate_mnm_s);

    Serial_Write(" max_T=");
    Serial_WriteFloat3(balance.max_wheel_torque_mnm);

    Serial_Write(" gain=");
    Serial_WriteFloat3(balance.motion_gain_scale);

    Serial_Write(" act=");
    Serial_Write(actuator.enabled ? "1" : "0");

    Serial_Write(" left_ref=");
    Serial_WriteInt32(actuator.left_current_reference_ma);

    Serial_Write(" right_ref=");
    Serial_WriteInt32(actuator.right_current_reference_ma);

    Serial_Write(" left_meas=");
    Serial_WriteInt32(actuator.left_current_measured_ma);

    Serial_Write(" right_meas=");
    Serial_WriteInt32(actuator.right_current_measured_ma);

    Serial_Write(" left_pwm=");
    Serial_WriteInt32(actuator.left_command_permille);

    Serial_Write(" right_pwm=");
    Serial_WriteInt32(actuator.right_command_permille);

    Serial_WriteLine("");
}
