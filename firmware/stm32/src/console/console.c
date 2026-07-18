#include "console.h"

#include "control/actuator/actuator.h"
#include "serial/serial.h"
#include "tests/current_sensor_test.h"
#include "tests/encoder_test.h"
#include "tests/imu_test.h"
#include "tests/motor_test.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define CONSOLE_LINE_LENGTH    64U
#define CONSOLE_MAX_ARGUMENTS  4

static char input_line[CONSOLE_LINE_LENGTH];
static uint32_t input_length;

static void Console_ProcessByte(uint8_t byte);
static void Console_ExecuteLine(char *line);

static int Console_Tokenize(
    char *line,
    char *arguments[],
    int maximum_arguments
);

static void Console_HandleMotor(
    int argument_count,
    char *arguments[]
);

static void Console_HandleEncoder(
    int argument_count,
    char *arguments[]
);

static void Console_HandleCurrent(
    int argument_count,
    char *arguments[]
);

static void Console_HandleActuator(
    int argument_count,
    char *arguments[]
);

static void Console_HandleIMU(
    int argument_count,
    char *arguments[]
);

static bool Console_ParseSpeed(
    const char *text,
    int16_t *speed
);

static bool Console_ParseInt32(
    const char *text,
    int32_t minimum,
    int32_t maximum,
    int32_t *value
);

static void Console_PrintActuatorConfig(void);
static void Console_PrintActuatorStatus(void);
static void Console_PrintHelp(void);
static void Console_PrintPrompt(void);

void Console_Init(void)
{
    input_length = 0U;
    input_line[0] = '\0';

    Serial_WriteLine("");
    Serial_WriteLine(
        "SEBA-ROBOT console ready."
    );

    Serial_WriteLine(
        "Type 'help' for commands."
    );

    Console_PrintPrompt();
}

void Console_Process(void)
{
    uint8_t byte;

    while (Serial_ReadByte(&byte))
    {
        Console_ProcessByte(byte);
    }
}

static void Console_ProcessByte(uint8_t byte)
{
    if (
        byte == '\r' ||
        byte == '\n'
    )
    {
        if (input_length == 0U)
        {
            return;
        }

        input_line[input_length] = '\0';

        Serial_WriteLine("");

        Console_ExecuteLine(input_line);

        input_length = 0U;
        input_line[0] = '\0';

        Console_PrintPrompt();

        return;
    }

    if (
        byte == '\b' ||
        byte == 127U
    )
    {
        if (input_length > 0U)
        {
            input_length--;

            input_line[input_length] = '\0';

            /*
             * Move back, erase the displayed character, then
             * move back again on a simple serial terminal.
             */
            Serial_Write("\b \b");
        }

        return;
    }

    if (
        byte < 32U ||
        byte > 126U
    )
    {
        return;
    }

    if (
        input_length >=
        CONSOLE_LINE_LENGTH - 1U
    )
    {
        /*
         * Discard the partial command so the next line starts
         * from a known empty buffer.
         */
        input_length = 0U;
        input_line[0] = '\0';

        Serial_WriteLine("");

        Serial_WriteLine(
            "ERROR: command is too long."
        );

        Console_PrintPrompt();

        return;
    }

    input_line[input_length] =
        (char)byte;

    input_length++;

    input_line[input_length] = '\0';

    {
        char echo[2];

        echo[0] = (char)byte;
        echo[1] = '\0';

        Serial_Write(echo);
    }
}

static void Console_ExecuteLine(char *line)
{
    char *arguments[CONSOLE_MAX_ARGUMENTS];

    const int argument_count =
        Console_Tokenize(
            line,
            arguments,
            CONSOLE_MAX_ARGUMENTS
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

        Console_PrintHelp();

        return;
    }

    if (strcmp(arguments[0], "motor") == 0)
    {
        Console_HandleMotor(
            argument_count,
            arguments
        );

        return;
    }

    if (strcmp(arguments[0], "encoder") == 0)
    {
        Console_HandleEncoder(
            argument_count,
            arguments
        );

        return;
    }

    if (strcmp(arguments[0], "current") == 0)
    {
        Console_HandleCurrent(
            argument_count,
            arguments
        );

        return;
    }

    if (strcmp(arguments[0], "actuator") == 0)
    {
        Console_HandleActuator(
            argument_count,
            arguments
        );

        return;
    }

    if (strcmp(arguments[0], "imu") == 0)
    {
        Console_HandleIMU(
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

static int Console_Tokenize(
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

static void Console_HandleMotor(
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
        Actuator_Disable();
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

    if (!Console_ParseSpeed(
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
        Actuator_Disable();

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
        Actuator_Disable();

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

static void Console_HandleEncoder(
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

        return;
    }

    Serial_WriteLine(
        "ERROR: usage: encoder <read|reset>"
    );
}

static void Console_HandleCurrent(
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

        if (!Console_ParseInt32(
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

static void Console_HandleActuator(
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
            Actuator_Disable();

            Serial_WriteLine(
                "OK: actuator current control stopped."
            );

            return;
        }

        if (strcmp(arguments[1], "status") == 0)
        {
            Console_PrintActuatorStatus();
            return;
        }

        if (strcmp(arguments[1], "config") == 0)
        {
            Console_PrintActuatorConfig();
            return;
        }
    }

    if (argument_count == 3)
    {
        if (!Console_ParseInt32(
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
            !Console_ParseInt32(
                arguments[2],
                -100000L,
                100000L,
                &value
            ) ||
            !Console_ParseInt32(
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

static void Console_HandleIMU(
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

static bool Console_ParseSpeed(
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

static bool Console_ParseInt32(
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

static void Console_PrintActuatorConfig(void)
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

static void Console_PrintActuatorStatus(void)
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

static void Console_PrintHelp(void)
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

static void Console_PrintPrompt(void)
{
    Serial_Write("> ");
}
