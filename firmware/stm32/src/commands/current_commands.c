#include "current_commands.h"
#include "command_parser.h"
#include "control/actuator/actuator.h"
#include "current_sensor/current_sensor.h"
#include "serial/serial.h"
#include "tests/current_sensor_test.h"

#include <stdint.h>
#include <string.h>

void CurrentCommands_Handle(
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

        if (!CommandParser_ParseInt32(
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
