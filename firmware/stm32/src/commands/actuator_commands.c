#include "actuator_commands.h"
#include "command_parser.h"
#include "control/actuator/actuator.h"
#include "control/motion_control/motion_control.h"
#include "serial/serial.h"
#include "tests/motor_test.h"

#include <stdint.h>
#include <string.h>

static void ActuatorCommands_PrintConfig(void);
static void ActuatorCommands_PrintStatus(void);

void ActuatorCommands_Handle(
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
            ActuatorCommands_PrintStatus();
            return;
        }

        if (strcmp(arguments[1], "config") == 0)
        {
            ActuatorCommands_PrintConfig();
            return;
        }
    }

    if (argument_count == 3)
    {
        if (!CommandParser_ParseInt32(
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
            !CommandParser_ParseInt32(
                arguments[2],
                -100000L,
                100000L,
                &value
            ) ||
            !CommandParser_ParseInt32(
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

static void ActuatorCommands_PrintConfig(void)
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

static void ActuatorCommands_PrintStatus(void)
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
