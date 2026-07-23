#include "encoder_commands.h"
#include "control/state_estimation/state_estimation.h"
#include "serial/serial.h"
#include "tests/encoder_test.h"

#include <string.h>

void EncoderCommands_Handle(
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
