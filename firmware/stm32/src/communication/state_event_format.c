#include "state_event_format.h"

#include "serial/serial.h"

#include <stddef.h>

/*
 * Event lines carry only the state fields needed to diagnose
 * arming and fault transitions. Full telemetry stays in TEL lines.
 */
void StateEventFormat_WriteState(
    const RobotState *state
)
{
    if (state == NULL)
    {
        return;
    }

    Serial_Write(" valid=");
    Serial_Write(state->valid ? "1" : "0");

    Serial_Write(" imu=");
    Serial_Write(state->imu_valid ? "1" : "0");

    Serial_Write(" imu_stale=");
    Serial_Write(state->imu_stale ? "1" : "0");

    Serial_Write(" enc=");
    Serial_Write(state->encoder_valid ? "1" : "0");

    Serial_Write(" updates=");
    Serial_WriteUInt32(state->update_count);

    Serial_Write(" ori_age=");
    Serial_WriteUInt32(state->orientation_age_ms);

    Serial_Write(" gyro_age=");
    Serial_WriteUInt32(state->gyroscope_age_ms);

    Serial_Write(" ori_count=");
    Serial_WriteUInt32(state->orientation_update_count);

    Serial_Write(" gyro_count=");
    Serial_WriteUInt32(state->gyroscope_update_count);

    Serial_Write(" theta=");
    Serial_WriteFloat3(state->pitch_rad);

    Serial_Write(" theta_dot=");
    Serial_WriteFloat3(state->pitch_rate_rads);
}
