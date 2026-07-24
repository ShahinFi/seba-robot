#include "balance_commands.h"
#include "command_parser.h"
#include "control/motion_control/motion_control.h"
#include "serial/serial.h"
#include "tests/motor_test.h"

#include <stdint.h>
#include <string.h>

static void BalanceCommands_PrintStatus(void);
static const char *BalanceCommands_FaultName(
    MotionControlFault fault
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
            "ERROR: usage: balance <start|stop|status> OR balance max-torque <mNm> OR balance gain-scale <percent> OR balance command <v> <yaw> OR balance gain <left|right> <0...5> <value>"
        );

        return COMMAND_RESULT_ERROR;
    }

    if (strcmp(arguments[1], "start") == 0)
    {
        MotorTest_Stop();
        MotionControl_Enable();

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
        "ERROR: usage: balance <start|stop|status> OR balance max-torque <mNm> OR balance gain-scale <percent> OR balance command <v> <yaw> OR balance gain <left|right> <0...5> <value>"
    );

    return COMMAND_RESULT_ERROR;
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
        BalanceCommands_FaultName(status.fault)
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

static const char *BalanceCommands_FaultName(
    MotionControlFault fault
)
{
    switch (fault)
    {
        case MOTION_CONTROL_FAULT_NONE:
            return "none";

        case MOTION_CONTROL_FAULT_STATE_INVALID:
            return "state_invalid";

        case MOTION_CONTROL_FAULT_IMU_STALE:
            return "imu_stale";

        case MOTION_CONTROL_FAULT_FALL:
            return "fall";

        case MOTION_CONTROL_FAULT_ACTUATOR:
            return "actuator";

        default:
            return "unknown";
    }
}
