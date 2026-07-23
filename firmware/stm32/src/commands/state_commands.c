#include "state_commands.h"
#include "control/state_estimation/state_estimation.h"
#include "serial/serial.h"

#include <string.h>

static void StateCommands_Print(void);

void StateCommands_Handle(
    int argument_count,
    char *arguments[]
)
{
    if (
        argument_count != 2 ||
        strcmp(arguments[1], "read") != 0
    )
    {
        Serial_WriteLine(
            "ERROR: usage: state read"
        );

        return;
    }

    StateCommands_Print();
}

static void StateCommands_Print(void)
{
    RobotState state;

    StateEstimation_GetState(
        &state
    );

    Serial_Write("State: valid=");
    Serial_Write(state.valid ? "yes" : "no");

    Serial_Write(" imu=");
    Serial_Write(state.imu_valid ? "yes" : "no");

    Serial_Write(" encoder=");
    Serial_Write(state.encoder_valid ? "yes" : "no");

    Serial_Write(" updates=");
    Serial_WriteUInt32(state.update_count);
    Serial_WriteLine("");

    Serial_Write("  v [m/s]: ");
    Serial_WriteFloat3(state.forward_velocity_mps);
    Serial_WriteLine("");

    Serial_Write("  theta [rad]: ");
    Serial_WriteFloat3(state.pitch_rad);
    Serial_WriteLine("");

    Serial_Write("  theta_dot [rad/s]: ");
    Serial_WriteFloat3(state.pitch_rate_rads);
    Serial_WriteLine("");

    Serial_Write("  psi_dot [rad/s]: ");
    Serial_WriteFloat3(state.yaw_rate_rads);
    Serial_WriteLine("");

    Serial_Write("  v_dot [m/s^2]: ");
    Serial_WriteFloat3(state.forward_acceleration_mps2);
    Serial_WriteLine("");

    Serial_Write("  theta_ddot [rad/s^2]: ");
    Serial_WriteFloat3(state.pitch_acceleration_rads2);
    Serial_WriteLine("");

    Serial_Write("  psi_ddot [rad/s^2]: ");
    Serial_WriteFloat3(state.yaw_acceleration_rads2);
    Serial_WriteLine("");
}
