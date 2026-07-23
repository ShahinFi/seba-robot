#include "imu_commands.h"
#include "serial/serial.h"
#include "tests/imu_test.h"

#include <string.h>

void IMUCommands_Handle(
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
