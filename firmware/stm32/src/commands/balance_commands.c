#include "balance_commands.h"
#include "command_parser.h"
#include "communication/state_event_format.h"
#include "control/control_parameters.h"
#include "control/motion_control/motion_control.h"
#include "control/state_estimation/state_estimation.h"
#include "serial/serial.h"
#include "tests/motor_test.h"

#include <stdint.h>
#include <string.h>

static void BalanceCommands_PrintStatus(void);
static CommandResult BalanceCommands_ApplyConfig(
    int argument_count,
    char *arguments[]
);
static bool BalanceCommands_StateAllowsStart(
    const RobotState *state,
    const ControlParameters *parameters
);
static void BalanceCommands_PrintArmEvent(
    const char *result,
    const char *reason,
    const RobotState *state
);

CommandResult BalanceCommands_Handle(
    int argument_count,
    char *arguments[]
)
{
    int32_t value;
    int32_t column;
    uint32_t row;
    float float_value;
    float second_float_value;

    if (
        argument_count == 16 &&
        strcmp(arguments[1], "config") == 0
    )
    {
        return BalanceCommands_ApplyConfig(
            argument_count,
            arguments
        );
    }

    if (
        argument_count == 3 &&
        (
            strcmp(arguments[1], "max-torque") == 0 ||
            strcmp(arguments[1], "gain-scale") == 0
        )
    )
    {
        if (!CommandParser_ParseInt32(
                arguments[2],
                0,
                10000,
                &value
            ))
        {
            Serial_WriteLine(
                "ERROR: balance value must be 0 to 10000."
            );

            return COMMAND_RESULT_ERROR;
        }

        if (strcmp(arguments[1], "max-torque") == 0)
        {
            if (!MotionControl_SetMaxWheelTorque(
                    (float)value
                ))
            {
                Serial_WriteLine(
                    "ERROR: balance max torque update failed."
                );

                return COMMAND_RESULT_ERROR;
            }

            Serial_WriteLine(
                "OK: balance torque limit updated."
            );

            return COMMAND_RESULT_OK;
        }

        if (!MotionControl_SetGainScale(
                (float)value /
                100.0F
            ))
        {
            Serial_WriteLine(
                "ERROR: balance gain scale update failed."
            );

            return COMMAND_RESULT_ERROR;
        }

        Serial_WriteLine(
            "OK: balance gain scale updated."
        );

        return COMMAND_RESULT_OK;
    }

    if (
        argument_count == 4 &&
        strcmp(arguments[1], "command") == 0
    )
    {
        if (
            !CommandParser_ParseFloat(
                arguments[2],
                -10.0F,
                10.0F,
                &float_value
            ) ||
            !CommandParser_ParseFloat(
                arguments[3],
                -20.0F,
                20.0F,
                &second_float_value
            )
        )
        {
            Serial_WriteLine(
                "ERROR: balance command values are out of range."
            );

            return COMMAND_RESULT_ERROR;
        }

        (void)MotionControl_SetCommand(
            float_value,
            second_float_value
        );

        Serial_WriteLine(
            "OK: balance command updated."
        );

        return COMMAND_RESULT_OK;
    }

    if (
        argument_count == 5 &&
        strcmp(arguments[1], "gain") == 0
    )
    {
        if (strcmp(arguments[2], "left") == 0)
        {
            row = 0U;
        }
        else if (strcmp(arguments[2], "right") == 0)
        {
            row = 1U;
        }
        else
        {
            Serial_WriteLine(
                "ERROR: balance gain side must be 'left' or 'right'."
            );

            return COMMAND_RESULT_ERROR;
        }

        if (
            !CommandParser_ParseInt32(
                arguments[3],
                0,
                5,
                &column
            ) ||
            !CommandParser_ParseFloat(
                arguments[4],
                -100000.0F,
                100000.0F,
                &float_value
            )
        )
        {
            Serial_WriteLine(
                "ERROR: balance gain requires column 0 to 5 and numeric gain."
            );

            return COMMAND_RESULT_ERROR;
        }

        if (!MotionControl_SetGain(
                row,
                (uint32_t)column,
                float_value
            ))
        {
            Serial_WriteLine(
                "ERROR: balance gain update failed."
            );

            return COMMAND_RESULT_ERROR;
        }

        Serial_WriteLine(
            "OK: balance gain updated."
        );

        return COMMAND_RESULT_OK;
    }

    if (argument_count != 2)
    {
        Serial_WriteLine(
            "ERROR: usage: balance <start|stop|status> OR balance max-torque <mNm> OR balance gain-scale <percent> OR balance command <v> <yaw> OR balance gain <left|right> <0...5> <value> OR balance config <gain_percent> <max_mNm> <12 gains>"
        );

        return COMMAND_RESULT_ERROR;
    }

    if (strcmp(arguments[1], "start") == 0)
    {
        ControlParameters parameters;
        RobotState state;

        /*
         * Repeated start commands are idempotent. Restarting the
         * controller while balanced would reset the torque state.
         */
        if (MotionControl_IsEnabled())
        {
            Serial_WriteLine(
                "OK: balance control already running."
            );

            return COMMAND_RESULT_OK;
        }

        ControlParameters_Get(
            &parameters
        );

        StateEstimation_GetState(
            &state
        );

        /*
         * Emit both attempted and accepted/rejected arm events so
         * the Pi log records the exact state used for the decision.
         */
        BalanceCommands_PrintArmEvent(
            "attempt",
            "none",
            &state
        );

        if (!BalanceCommands_StateAllowsStart(
                &state,
                &parameters
            ))
        {
            return COMMAND_RESULT_ERROR;
        }

        MotorTest_Stop();
        MotionControl_Enable();

        BalanceCommands_PrintArmEvent(
            "accepted",
            "none",
            &state
        );

        Serial_WriteLine(
            "OK: balance control started."
        );

        return COMMAND_RESULT_OK;
    }

    if (strcmp(arguments[1], "stop") == 0)
    {
        MotionControl_Disable();

        Serial_WriteLine(
            "OK: balance control stopped."
        );

        return COMMAND_RESULT_OK;
    }

    if (strcmp(arguments[1], "status") == 0)
    {
        BalanceCommands_PrintStatus();
        return COMMAND_RESULT_OK;
    }

    Serial_WriteLine(
        "ERROR: usage: balance <start|stop|status> OR balance max-torque <mNm> OR balance gain-scale <percent> OR balance command <v> <yaw> OR balance gain <left|right> <0...5> <value> OR balance config <gain_percent> <max_mNm> <12 gains>"
    );

    return COMMAND_RESULT_ERROR;
}

static CommandResult BalanceCommands_ApplyConfig(
    int argument_count,
    char *arguments[]
)
{
    int32_t gain_percent;
    int32_t max_torque_mnm;
    float gains[2][6];
    uint32_t row;
    uint32_t column;

    if (argument_count != 16)
    {
        Serial_WriteLine(
            "ERROR: usage: balance config <gain_percent> <max_mNm> <12 gains>"
        );

        return COMMAND_RESULT_ERROR;
    }

    if (
        !CommandParser_ParseInt32(
            arguments[2],
            0,
            10000,
            &gain_percent
        ) ||
        !CommandParser_ParseInt32(
            arguments[3],
            0,
            10000,
            &max_torque_mnm
        )
    )
    {
        Serial_WriteLine(
            "ERROR: balance config limits must be 0 to 10000."
        );

        return COMMAND_RESULT_ERROR;
    }

    for (row = 0U; row < 2U; row++)
    {
        for (column = 0U; column < 6U; column++)
        {
            if (!CommandParser_ParseFloat(
                    arguments[4 + (int)(row * 6U + column)],
                    -100000.0F,
                    100000.0F,
                    &gains[row][column]
                ))
            {
                Serial_WriteLine(
                    "ERROR: balance config gains must be numeric."
                );

                return COMMAND_RESULT_ERROR;
            }
        }
    }

    /*
     * Parse the complete gain matrix before writing live motion
     * controller settings.
     */
    if (!MotionControl_SetGainScale(
            (float)gain_percent /
            100.0F
        ))
    {
        Serial_WriteLine(
            "ERROR: balance gain scale update failed."
        );

        return COMMAND_RESULT_ERROR;
    }

    if (!MotionControl_SetMaxWheelTorque(
            (float)max_torque_mnm
        ))
    {
        Serial_WriteLine(
            "ERROR: balance max torque update failed."
        );

        return COMMAND_RESULT_ERROR;
    }

    for (row = 0U; row < 2U; row++)
    {
        for (column = 0U; column < 6U; column++)
        {
            if (!MotionControl_SetGain(
                    row,
                    column,
                    gains[row][column]
                ))
            {
                Serial_WriteLine(
                    "ERROR: balance gain update failed."
                );

                return COMMAND_RESULT_ERROR;
            }
        }
    }

    Serial_WriteLine(
        "OK: balance config updated."
    );

    return COMMAND_RESULT_OK;
}

static void BalanceCommands_PrintStatus(void)
{
    MotionControlStatus status;

    MotionControl_GetStatus(
        &status
    );

    Serial_Write("Balance: enabled=");
    Serial_Write(status.enabled ? "yes" : "no");

    Serial_Write(" fault=");
    Serial_Write(status.fault_active ? "ACTIVE" : "normal");

    Serial_Write(" fault_code=");
    Serial_WriteUInt32((uint32_t)status.fault);

    Serial_Write(" fault_name=");
    Serial_Write(
        MotionControl_FaultName(status.fault)
    );

    Serial_Write(" cmd_age=");
    Serial_WriteUInt32(status.command_age_ms);

    Serial_Write(" cmd_count=");
    Serial_WriteUInt32(status.command_update_count);

    Serial_Write(" state_invalid=");
    Serial_Write(status.state_invalid ? "yes" : "no");

    Serial_Write(" fall=");
    Serial_Write(status.fall_detected ? "yes" : "no");

    Serial_Write(" v_cmd=");
    Serial_WriteFloat3(status.forward_velocity_command_mps);

    Serial_Write(" yaw_cmd=");
    Serial_WriteFloat3(status.yaw_rate_command_rads);

    Serial_Write(" left_T=");
    Serial_WriteFloat3(status.left_torque_command_mnm);

    Serial_Write(" right_T=");
    Serial_WriteFloat3(status.right_torque_command_mnm);

    Serial_Write(" left_dT=");
    Serial_WriteFloat3(status.left_torque_rate_mnm_s);

    Serial_Write(" right_dT=");
    Serial_WriteFloat3(status.right_torque_rate_mnm_s);

    Serial_Write(" max_T=");
    Serial_WriteFloat3(status.max_wheel_torque_mnm);

    Serial_Write(" gain=");
    Serial_WriteFloat3(status.motion_gain_scale);

    Serial_WriteLine("");
}

static bool BalanceCommands_StateAllowsStart(
    const RobotState *state,
    const ControlParameters *parameters
)
{
    if (
        state == NULL ||
        parameters == NULL
    )
    {
        return false;
    }

    if (state->imu_stale)
    {
        BalanceCommands_PrintArmEvent(
            "rejected",
            "imu_stale",
            state
        );

        Serial_WriteLine(
            "ERROR: IMU data is stale."
        );

        return false;
    }

    if (!state->imu_valid)
    {
        BalanceCommands_PrintArmEvent(
            "rejected",
            "imu_not_ready",
            state
        );

        Serial_WriteLine(
            "ERROR: IMU state is not ready."
        );

        return false;
    }

    if (!state->encoder_valid)
    {
        BalanceCommands_PrintArmEvent(
            "rejected",
            "encoder_not_ready",
            state
        );

        Serial_WriteLine(
            "ERROR: encoder state is not ready."
        );

        return false;
    }

    if (
        state->pitch_rad >
        parameters->fall_angle_rad ||
        state->pitch_rad <
        -parameters->fall_angle_rad
    )
    {
        BalanceCommands_PrintArmEvent(
            "rejected",
            "fall_angle",
            state
        );

        Serial_WriteLine(
            "ERROR: robot is past the fall angle."
        );

        return false;
    }

    if (!state->valid)
    {
        BalanceCommands_PrintArmEvent(
            "rejected",
            "state_not_ready",
            state
        );

        Serial_WriteLine(
            "ERROR: state is not ready."
        );

        return false;
    }

    return true;
}

static void BalanceCommands_PrintArmEvent(
    const char *result,
    const char *reason,
    const RobotState *state
)
{
    /*
     * EVT balance arm is parsed by the Raspberry Pi as a log
     * event, not as periodic telemetry.
     */
    Serial_Write("EVT balance arm result=");
    Serial_Write(result);
    Serial_Write(" reason=");
    Serial_Write(reason);
    StateEventFormat_WriteState(
        state
    );
    Serial_WriteLine("");
}
