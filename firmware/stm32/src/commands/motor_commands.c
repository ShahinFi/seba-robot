#include "motor_commands.h"
#include "command_parser.h"
#include "control/motion_control/motion_control.h"
#include "serial/serial.h"
#include "tests/motor_test.h"

#include <stdint.h>
#include <string.h>

void MotorCommands_Handle(
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

    if (!CommandParser_ParseSpeed(
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
