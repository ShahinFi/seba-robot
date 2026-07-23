#include "imu_commands.h"
#include "serial/serial.h"
#include "tests/imu_test.h"

#include <string.h>

CommandResult IMUCommands_Handle(
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

        return COMMAND_RESULT_ERROR;
    }

    if (!IMUTest_PrintLatest())
    {
        Serial_WriteLine(
            "ERROR: IMU is not initialized."
        );

        return COMMAND_RESULT_ERROR;
    }

    return COMMAND_RESULT_OK;
}
