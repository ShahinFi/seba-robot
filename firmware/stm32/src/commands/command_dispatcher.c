#include "command_dispatcher.h"

#include "communication/telemetry_stream.h"
#include "control/actuator/actuator.h"
#include "control/motion_control/motion_control.h"
#include "control/state_estimation/state_estimation.h"
#include "serial/serial.h"
#include "tests/current_sensor_test.h"
#include "tests/encoder_test.h"
#include "tests/imu_test.h"
#include "tests/motor_test.h"

#include "stm32g4xx_hal.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define COMMAND_DISPATCHER_MAX_ARGUMENTS  5

static int CommandDispatcher_Tokenize(
    char *line,
    char *arguments[],
    int maximum_arguments
);

static void CommandDispatcher_HandleMotor(
    int argument_count,
    char *arguments[]
);

static void CommandDispatcher_HandleEncoder(
    int argument_count,
    char *arguments[]
);

static void CommandDispatcher_HandleCurrent(
    int argument_count,
    char *arguments[]
);

static void CommandDispatcher_HandleActuator(
    int argument_count,
    char *arguments[]
);

static void CommandDispatcher_HandleState(
    int argument_count,
    char *arguments[]
);

static void CommandDispatcher_HandleBalance(
    int argument_count,
    char *arguments[]
);

static void CommandDispatcher_HandleTelemetry(
    int argument_count,
    char *arguments[]
);

static void CommandDispatcher_HandleSystem(
    int argument_count,
    char *arguments[]
);

static void CommandDispatcher_HandleIMU(
    int argument_count,
    char *arguments[]
);

static bool CommandDispatcher_ParseSpeed(
    const char *text,
    int16_t *speed
);

static bool CommandDispatcher_ParseInt32(
    const char *text,
    int32_t minimum,
    int32_t maximum,
    int32_t *value
);

static bool CommandDispatcher_ParseFloat(
    const char *text,
    float minimum,
    float maximum,
    float *value
);

static void CommandDispatcher_PrintActuatorConfig(void);
static void CommandDispatcher_PrintActuatorStatus(void);
static void CommandDispatcher_PrintBalanceStatus(void);
static void CommandDispatcher_PrintState(void);
static void CommandDispatcher_PrintHelp(void);

void CommandDispatcher_ExecuteLine(
    char *line
)
{
    char *arguments[COMMAND_DISPATCHER_MAX_ARGUMENTS];

    const int argument_count =
        CommandDispatcher_Tokenize(
            line,
            arguments,
            COMMAND_DISPATCHER_MAX_ARGUMENTS
        );

    if (argument_count == 0)
    {
        return;
    }

    if (strcmp(arguments[0], "help") == 0)
    {
        if (argument_count != 1)
        {
            Serial_WriteLine(
                "ERROR: usage: help"
            );

            return;
        }

        CommandDispatcher_PrintHelp();

        return;
    }

    if (strcmp(arguments[0], "motor") == 0)
    {
        CommandDispatcher_HandleMotor(
            argument_count,
            arguments
        );

        return;
    }

    if (strcmp(arguments[0], "encoder") == 0)
    {
        CommandDispatcher_HandleEncoder(
            argument_count,
            arguments
        );

        return;
    }

    if (strcmp(arguments[0], "current") == 0)
    {
        CommandDispatcher_HandleCurrent(
            argument_count,
            arguments
        );

        return;
    }

    if (strcmp(arguments[0], "actuator") == 0)
    {
        CommandDispatcher_HandleActuator(
            argument_count,
            arguments
        );

        return;
    }

    if (strcmp(arguments[0], "state") == 0)
    {
        CommandDispatcher_HandleState(
            argument_count,
            arguments
        );

        return;
    }

    if (strcmp(arguments[0], "balance") == 0)
    {
        CommandDispatcher_HandleBalance(
            argument_count,
            arguments
        );

        return;
    }

    if (strcmp(arguments[0], "telemetry") == 0)
    {
        CommandDispatcher_HandleTelemetry(
            argument_count,
            arguments
        );

        return;
    }

    if (strcmp(arguments[0], "system") == 0)
    {
        CommandDispatcher_HandleSystem(
            argument_count,
            arguments
        );

        return;
    }

    if (strcmp(arguments[0], "imu") == 0)
    {
        CommandDispatcher_HandleIMU(
            argument_count,
            arguments
        );

        return;
    }

    Serial_WriteLine(
        "ERROR: unknown command."
    );

    Serial_WriteLine(
        "Type 'help' for available commands."
    );
}

static int CommandDispatcher_Tokenize(
    char *line,
    char *arguments[],
    int maximum_arguments
)
{
    int argument_count = 0;

    char *token =
        strtok(line, " \t");

    while (
        token != NULL &&
        argument_count < maximum_arguments
    )
    {
        arguments[argument_count] =
            token;

        argument_count++;

        token =
            strtok(NULL, " \t");
    }

    return argument_count;
}

static void CommandDispatcher_HandleMotor(
    int argument_count,
    char *arguments[]
)
{
    int16_t speed;

    if (
        argument_count == 2 &&
        strcmp(arguments[1], "stop") == 0
    )
    {
        MotionControl_Disable();
        MotorTest_Stop();

        Serial_WriteLine(
            "OK: all motors stopped and disabled."
        );

        return;
    }

    if (argument_count != 3)
    {
        Serial_WriteLine(
            "ERROR: usage: motor <left|right> <-100...100> OR motor stop"
        );

        return;
    }

    if (!CommandDispatcher_ParseSpeed(
            arguments[2],
            &speed
        ))
    {
        Serial_WriteLine(
            "ERROR: speed must be an integer from -100 to 100."
        );

        return;
    }

    if (strcmp(arguments[1], "left") == 0)
    {
        MotionControl_Disable();

        if (!MotorTest_SetLeft(speed))
        {
            Serial_WriteLine(
                "ERROR: invalid left motor command."
            );

            return;
        }

        Serial_Write(
            "OK: left motor = "
        );

        Serial_Write(arguments[2]);

        Serial_WriteLine("%");

        return;
    }

    if (strcmp(arguments[1], "right") == 0)
    {
        MotionControl_Disable();

        if (!MotorTest_SetRight(speed))
        {
            Serial_WriteLine(
                "ERROR: invalid right motor command."
            );

            return;
        }

        Serial_Write(
            "OK: right motor = "
        );

        Serial_Write(arguments[2]);

        Serial_WriteLine("%");

        return;
    }

    Serial_WriteLine(
        "ERROR: motor must be 'left', 'right', or 'stop'."
    );
}

static void CommandDispatcher_HandleEncoder(
    int argument_count,
    char *arguments[]
)
{
    if (argument_count != 2)
    {
        Serial_WriteLine(
            "ERROR: usage: encoder <read|reset>"
        );

        return;
    }

    if (strcmp(arguments[1], "read") == 0)
    {
        EncoderTest_PrintPositions();

        return;
    }

    if (strcmp(arguments[1], "reset") == 0)
    {
        EncoderTest_ResetPositions();
        StateEstimation_Reset();

        return;
    }

    Serial_WriteLine(
        "ERROR: usage: encoder <read|reset>"
    );
}

static void CommandDispatcher_HandleCurrent(
    int argument_count,
    char *arguments[]
)
{
    CurrentSensorChannel channel;
    int32_t sample_count;

    if (
        argument_count == 2 &&
        strcmp(arguments[1], "read") == 0
    )
    {
        if (Actuator_IsEnabled())
        {
            Serial_WriteLine(
                "ERROR: stop actuator control before using current read."
            );

            return;
        }

        if (!CurrentSensorTest_PrintReadings())
        {
            Serial_WriteLine(
                "ERROR: current sensor read failed."
            );

            return;
        }

        return;
    }

    if (
        argument_count == 4 &&
        strcmp(arguments[1], "scope") == 0
    )
    {
        if (strcmp(arguments[2], "left") == 0)
        {
            channel = CURRENT_SENSOR_LEFT;
        }
        else if (strcmp(arguments[2], "right") == 0)
        {
            channel = CURRENT_SENSOR_RIGHT;
        }
        else
        {
            Serial_WriteLine(
                "ERROR: current scope channel must be 'left' or 'right'."
            );

            return;
        }

        if (!CommandDispatcher_ParseInt32(
                arguments[3],
                1,
                10000,
                &sample_count
            ))
        {
            Serial_WriteLine(
                "ERROR: sample count must be 1 to 10000."
            );

            return;
        }

        if (Actuator_IsEnabled())
        {
            Serial_WriteLine(
                "ERROR: stop actuator control before using current scope."
            );

            return;
        }

        if (!CurrentSensorTest_PrintCapture(
                channel,
                (uint32_t)sample_count
            ))
        {
            Serial_WriteLine(
                "ERROR: current sensor capture failed."
            );

            return;
        }

        return;
    }

    Serial_WriteLine(
        "ERROR: usage: current read OR current scope <left|right> <samples>"
    );
}

static void CommandDispatcher_HandleActuator(
    int argument_count,
    char *arguments[]
)
{
    int32_t value;
    int32_t second_value;
    ActuatorStatus status;

    if (argument_count == 2)
    {
        if (strcmp(arguments[1], "stop") == 0)
        {
            MotionControl_Disable();

            Serial_WriteLine(
                "OK: actuator current control stopped."
            );

            return;
        }

        if (strcmp(arguments[1], "status") == 0)
        {
            CommandDispatcher_PrintActuatorStatus();
            return;
        }

        if (strcmp(arguments[1], "config") == 0)
        {
            CommandDispatcher_PrintActuatorConfig();
            return;
        }
    }

    if (argument_count == 3)
    {
        if (!CommandDispatcher_ParseInt32(
                arguments[2],
                -100000L,
                100000L,
                &value
            ))
        {
            Serial_WriteLine(
                "ERROR: actuator value must be an integer."
            );

            return;
        }

        Actuator_GetStatus(
            &status
        );

        if (strcmp(arguments[1], "left") == 0)
        {
            if (!Actuator_SetCurrentReferences(
                    value,
                    status.right_current_reference_ma
                ))
            {
                Serial_WriteLine(
                    "ERROR: current reference exceeds configured limit."
                );

                return;
            }

            MotorTest_Stop();
            MotionControl_Disable();
            Actuator_Enable();

            Serial_WriteLine(
                "OK: left current reference updated."
            );

            return;
        }

        if (strcmp(arguments[1], "right") == 0)
        {
            if (!Actuator_SetCurrentReferences(
                    status.left_current_reference_ma,
                    value
                ))
            {
                Serial_WriteLine(
                    "ERROR: current reference exceeds configured limit."
                );

                return;
            }

            MotorTest_Stop();
            MotionControl_Disable();
            Actuator_Enable();

            Serial_WriteLine(
                "OK: right current reference updated."
            );

            return;
        }

        if (strcmp(arguments[1], "kp") == 0)
        {
            if (!Actuator_SetProportionalGain(value))
            {
                Serial_WriteLine(
                    "ERROR: kp must be nonnegative."
                );

                return;
            }

            Serial_WriteLine("OK: actuator kp updated.");
            return;
        }

        if (strcmp(arguments[1], "ki") == 0)
        {
            if (!Actuator_SetIntegralGain(value))
            {
                Serial_WriteLine(
                    "ERROR: ki must be nonnegative."
                );

                return;
            }

            Serial_WriteLine("OK: actuator ki updated.");
            return;
        }

        if (strcmp(arguments[1], "battery") == 0)
        {
            if (!Actuator_SetBatteryVoltage(value))
            {
                Serial_WriteLine(
                    "ERROR: battery voltage must be positive."
                );

                return;
            }

            Serial_WriteLine(
                "OK: actuator battery voltage updated."
            );

            return;
        }

        if (strcmp(arguments[1], "max-current") == 0)
        {
            if (!Actuator_SetMaxCurrentReference(value))
            {
                Serial_WriteLine(
                    "ERROR: max current must be nonnegative."
                );

                return;
            }

            Serial_WriteLine(
                "OK: actuator current limit updated."
            );

            return;
        }

        if (strcmp(arguments[1], "max-pwm") == 0)
        {
            if (
                value < 0 ||
                value > 100 ||
                !Actuator_SetMaxCommandPercent(
                    (int16_t)value
                )
            )
            {
                Serial_WriteLine(
                    "ERROR: max PWM must be 0 to 100 percent."
                );

                return;
            }

            Serial_WriteLine(
                "OK: actuator PWM limit updated."
            );

            return;
        }

        if (strcmp(arguments[1], "integral-limit") == 0)
        {
            if (!Actuator_SetIntegralLimit(value))
            {
                Serial_WriteLine(
                    "ERROR: integral limit must be nonnegative."
                );

                return;
            }

            Serial_WriteLine(
                "OK: actuator integral limit updated."
            );

            return;
        }

        if (strcmp(arguments[1], "period") == 0)
        {
            if (
                value <= 0 ||
                value > 1000 ||
                !Actuator_SetControlPeriod(
                    (uint32_t)value
                )
            )
            {
                Serial_WriteLine(
                    "ERROR: period must be 1 to 1000 ms."
                );

                return;
            }

            Serial_WriteLine(
                "OK: actuator control period updated."
            );

            return;
        }

        if (strcmp(arguments[1], "torque-constant") == 0)
        {
            if (!Actuator_SetWheelTorqueConstant(value))
            {
                Serial_WriteLine(
                    "ERROR: torque constant must be positive."
                );

                return;
            }

            Serial_WriteLine(
                "OK: actuator torque constant updated."
            );

            return;
        }
    }

    if (
        argument_count == 4 &&
        (
            strcmp(arguments[1], "both") == 0 ||
            strcmp(arguments[1], "torque") == 0
        )
    )
    {
        if (
            !CommandDispatcher_ParseInt32(
                arguments[2],
                -100000L,
                100000L,
                &value
            ) ||
            !CommandDispatcher_ParseInt32(
                arguments[3],
                -100000L,
                100000L,
                &second_value
            )
        )
        {
            Serial_WriteLine(
                "ERROR: current references must be integers."
            );

            return;
        }

        if (strcmp(arguments[1], "both") == 0)
        {
            if (!Actuator_SetCurrentReferences(
                    value,
                    second_value
                ))
            {
                Serial_WriteLine(
                    "ERROR: current reference exceeds configured limit."
                );

                return;
            }
        }
        else
        {
            if (!Actuator_SetWheelTorqueReferences(
                    value,
                    second_value
                ))
            {
                Serial_WriteLine(
                    "ERROR: torque reference exceeds configured current limit."
                );

                return;
            }
        }

        MotorTest_Stop();
        MotionControl_Disable();
        Actuator_Enable();

        if (strcmp(arguments[1], "both") == 0)
        {
            Serial_WriteLine(
                "OK: actuator current references updated."
            );
        }
        else
        {
            Serial_WriteLine(
                "OK: actuator torque references updated."
            );
        }

        return;
    }

    Serial_WriteLine(
        "ERROR: usage: actuator <left|right> <mA>, actuator both <left_mA> <right_mA>, actuator torque <left_mNm> <right_mNm>, actuator stop, actuator status, actuator config"
    );
}

static void CommandDispatcher_HandleIMU(
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
            "ERROR: usage: imu read"
        );

        return;
    }

    if (!IMUTest_PrintLatest())
    {
        Serial_WriteLine(
            "ERROR: IMU is not initialized."
        );

        return;
    }
}

static void CommandDispatcher_HandleState(
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

    CommandDispatcher_PrintState();
}

static void CommandDispatcher_HandleBalance(
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
        if (!CommandDispatcher_ParseInt32(
                arguments[2],
                0,
                10000,
                &value
            ))
        {
            Serial_WriteLine(
                "ERROR: balance value must be 0 to 10000."
            );

            return;
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

                return;
            }

            Serial_WriteLine(
                "OK: balance torque limit updated."
            );

            return;
        }

        if (!MotionControl_SetGainScale(
                (float)value /
                100.0F
            ))
        {
            Serial_WriteLine(
                "ERROR: balance gain scale update failed."
            );

            return;
        }

        Serial_WriteLine(
            "OK: balance gain scale updated."
        );

        return;
    }

    if (
        argument_count == 4 &&
        strcmp(arguments[1], "command") == 0
    )
    {
        if (
            !CommandDispatcher_ParseFloat(
                arguments[2],
                -10.0F,
                10.0F,
                &float_value
            ) ||
            !CommandDispatcher_ParseFloat(
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

            return;
        }

        (void)MotionControl_SetCommand(
            float_value,
            second_float_value
        );

        Serial_WriteLine(
            "OK: balance command updated."
        );

        return;
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

            return;
        }

        if (
            !CommandDispatcher_ParseInt32(
                arguments[3],
                0,
                5,
                &column
            ) ||
            !CommandDispatcher_ParseFloat(
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

            return;
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

            return;
        }

        Serial_WriteLine(
            "OK: balance gain updated."
        );

        return;
    }

    if (argument_count != 2)
    {
        Serial_WriteLine(
            "ERROR: usage: balance <start|stop|status> OR balance max-torque <mNm> OR balance gain-scale <percent> OR balance command <v> <yaw> OR balance gain <left|right> <0...5> <value>"
        );

        return;
    }

    if (strcmp(arguments[1], "start") == 0)
    {
        MotorTest_Stop();
        MotionControl_Enable();

        Serial_WriteLine(
            "OK: balance control started."
        );

        return;
    }

    if (strcmp(arguments[1], "stop") == 0)
    {
        MotionControl_Disable();

        Serial_WriteLine(
            "OK: balance control stopped."
        );

        return;
    }

    if (strcmp(arguments[1], "status") == 0)
    {
        CommandDispatcher_PrintBalanceStatus();
        return;
    }

    Serial_WriteLine(
        "ERROR: usage: balance <start|stop|status> OR balance max-torque <mNm> OR balance gain-scale <percent> OR balance command <v> <yaw> OR balance gain <left|right> <0...5> <value>"
    );
}

static void CommandDispatcher_HandleTelemetry(
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
            "ERROR: usage: telemetry read"
        );

        return;
    }

    TelemetryStream_WriteSnapshot();
}

static void CommandDispatcher_HandleSystem(
    int argument_count,
    char *arguments[]
)
{
    if (
        argument_count != 2 ||
        strcmp(arguments[1], "reset") != 0
    )
    {
        Serial_WriteLine(
            "ERROR: usage: system reset"
        );

        return;
    }

    Serial_WriteLine(
        "OK: resetting STM32."
    );

    HAL_Delay(20U);
    NVIC_SystemReset();
}

static bool CommandDispatcher_ParseSpeed(
    const char *text,
    int16_t *speed
)
{
    char *end;
    long value;

    if (
        text == NULL ||
        speed == NULL ||
        text[0] == '\0'
    )
    {
        return false;
    }

    errno = 0;

    value =
        strtol(
            text,
            &end,
            10
        );

    if (
        end == text ||
        *end != '\0' ||
        errno == ERANGE ||
        value < -100L ||
        value > 100L
    )
    {
        return false;
    }

    *speed =
        (int16_t)value;

    return true;
}

static bool CommandDispatcher_ParseInt32(
    const char *text,
    int32_t minimum,
    int32_t maximum,
    int32_t *value
)
{
    char *end;
    long parsed;

    if (
        text == NULL ||
        value == NULL ||
        text[0] == '\0'
    )
    {
        return false;
    }

    errno = 0;

    parsed =
        strtol(
            text,
            &end,
            10
        );

    if (
        end == text ||
        *end != '\0' ||
        errno == ERANGE ||
        parsed < minimum ||
        parsed > maximum
    )
    {
        return false;
    }

    *value =
        (int32_t)parsed;

    return true;
}

static bool CommandDispatcher_ParseFloat(
    const char *text,
    float minimum,
    float maximum,
    float *value
)
{
    char *end;
    float parsed;

    if (
        text == NULL ||
        value == NULL ||
        text[0] == '\0'
    )
    {
        return false;
    }

    errno = 0;

    parsed =
        strtof(
            text,
            &end
        );

    if (
        end == text ||
        *end != '\0' ||
        errno == ERANGE ||
        parsed < minimum ||
        parsed > maximum
    )
    {
        return false;
    }

    *value =
        parsed;

    return true;
}

static void CommandDispatcher_PrintActuatorConfig(void)
{
    ActuatorConfig config;

    Actuator_GetConfig(
        &config
    );

    Serial_WriteLine("Actuator config:");

    Serial_Write("  Period [ms]: ");
    Serial_WriteUInt32(config.control_period_ms);
    Serial_WriteLine("");

    Serial_Write("  Battery [mV]: ");
    Serial_WriteInt32(config.battery_voltage_mv);
    Serial_WriteLine("");

    Serial_Write("  Kp [mV/A]: ");
    Serial_WriteInt32(config.proportional_gain_mv_per_a);
    Serial_WriteLine("");

    Serial_Write("  Ki [mV/(A*s)]: ");
    Serial_WriteInt32(config.integral_gain_mv_per_a_s);
    Serial_WriteLine("");

    Serial_Write("  Integral limit [mV]: ");
    Serial_WriteInt32(config.integral_limit_mv);
    Serial_WriteLine("");

    Serial_Write("  Current limit [mA]: ");
    Serial_WriteInt32(config.max_current_reference_ma);
    Serial_WriteLine("");

    Serial_Write("  PWM limit [%]: ");
    Serial_WriteInt32(config.max_command_percent);
    Serial_WriteLine("");

    Serial_Write("  Torque constant [mN*m/A]: ");
    Serial_WriteInt32(config.wheel_torque_constant_mnm_per_a);
    Serial_WriteLine("");
}

static void CommandDispatcher_PrintActuatorStatus(void)
{
    ActuatorStatus status;

    Actuator_GetStatus(
        &status
    );

    Serial_Write("Actuator: enabled=");
    Serial_Write(status.enabled ? "yes" : "no");

    Serial_Write(" fault=");
    Serial_Write(status.fault_active ? "ACTIVE" : "normal");

    Serial_Write(" read_failed=");
    Serial_Write(status.read_failed ? "yes" : "no");

    Serial_Write(" left_ref=");
    Serial_WriteInt32(status.left_current_reference_ma);

    Serial_Write(" left_meas=");
    Serial_WriteInt32(status.left_current_measured_ma);

    Serial_Write(" left_err=");
    Serial_WriteInt32(status.left_error_ma);

    Serial_Write(" left_pwm=");
    Serial_WriteInt32(status.left_command_permille);

    Serial_Write(" right_ref=");
    Serial_WriteInt32(status.right_current_reference_ma);

    Serial_Write(" right_meas=");
    Serial_WriteInt32(status.right_current_measured_ma);

    Serial_Write(" right_err=");
    Serial_WriteInt32(status.right_error_ma);

    Serial_Write(" right_pwm=");
    Serial_WriteInt32(status.right_command_permille);
    Serial_WriteLine("");
}

static void CommandDispatcher_PrintBalanceStatus(void)
{
    MotionControlStatus status;

    MotionControl_GetStatus(
        &status
    );

    Serial_Write("Balance: enabled=");
    Serial_Write(status.enabled ? "yes" : "no");

    Serial_Write(" fault=");
    Serial_Write(status.fault_active ? "ACTIVE" : "normal");

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

static void CommandDispatcher_PrintState(void)
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

static void CommandDispatcher_PrintHelp(void)
{
    Serial_WriteLine("Commands:");

    Serial_WriteLine(
        "  motor left <-100...100>"
    );

    Serial_WriteLine(
        "  motor right <-100...100>"
    );

    Serial_WriteLine(
        "  motor stop"
    );

    Serial_WriteLine(
        "  encoder read"
    );

    Serial_WriteLine(
        "  encoder reset"
    );

    Serial_WriteLine(
        "  current read"
    );

    Serial_WriteLine(
        "  current scope <left|right> <samples>"
    );

    Serial_WriteLine(
        "  actuator left <mA>"
    );

    Serial_WriteLine(
        "  actuator right <mA>"
    );

    Serial_WriteLine(
        "  actuator both <left_mA> <right_mA>"
    );

    Serial_WriteLine(
        "  actuator torque <left_mNm> <right_mNm>"
    );

    Serial_WriteLine(
        "  actuator stop"
    );

    Serial_WriteLine(
        "  actuator status"
    );

    Serial_WriteLine(
        "  actuator config"
    );

    Serial_WriteLine(
        "  actuator kp <mV_per_A>"
    );

    Serial_WriteLine(
        "  actuator ki <mV_per_A_s>"
    );

    Serial_WriteLine(
        "  actuator battery <mV>"
    );

    Serial_WriteLine(
        "  actuator max-current <mA>"
    );

    Serial_WriteLine(
        "  actuator max-pwm <percent>"
    );

    Serial_WriteLine(
        "  actuator integral-limit <mV>"
    );

    Serial_WriteLine(
        "  actuator period <ms>"
    );

    Serial_WriteLine(
        "  actuator torque-constant <mNm_per_A>"
    );

    Serial_WriteLine(
        "  state read"
    );

    Serial_WriteLine(
        "  balance start"
    );

    Serial_WriteLine(
        "  balance stop"
    );

    Serial_WriteLine(
        "  balance status"
    );

    Serial_WriteLine(
        "  balance max-torque <mNm>"
    );

    Serial_WriteLine(
        "  balance gain-scale <percent>"
    );

    Serial_WriteLine(
        "  balance command <v_mps> <yaw_rate_rads>"
    );

    Serial_WriteLine(
        "  balance gain <left|right> <0...5> <value>"
    );

    Serial_WriteLine(
        "  telemetry read"
    );

    Serial_WriteLine(
        "  system reset"
    );

    Serial_WriteLine(
        "  imu read"
    );

    Serial_WriteLine(
        "  help"
    );

    Serial_WriteLine("");

    Serial_WriteLine(
        "Positive motor command = forward"
    );

    Serial_WriteLine(
        "Negative motor command = reverse"
    );

    Serial_WriteLine(
        "Zero = stop that motor"
    );
}
