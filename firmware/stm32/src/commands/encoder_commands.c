#include "encoder_commands.h"
#include "control/state_estimation/state_estimation.h"
#include "serial/serial.h"
#include "tests/encoder_test.h"

#include <string.h>

CommandResult EncoderCommands_Handle(
    int argument_count,
    char *arguments[]
)
{
    if (argument_count != 2)
    {
        Serial_WriteLine(
            "ERROR: usage: encoder <read|reset>"
        );

        return COMMAND_RESULT_ERROR;
    }

    if (strcmp(arguments[1], "read") == 0)
    {
        EncoderTest_PrintPositions();

        return COMMAND_RESULT_OK;
    }

    if (strcmp(arguments[1], "reset") == 0)
    {
        EncoderTest_ResetPositions();

        /*
         * Encoder reset changes the wheel-position reference used
         * by state estimation, so the estimator is reset with it.
         */
        StateEstimation_Reset();

        return COMMAND_RESULT_OK;
    }

    Serial_WriteLine(
        "ERROR: usage: encoder <read|reset>"
    );

    return COMMAND_RESULT_ERROR;
}
