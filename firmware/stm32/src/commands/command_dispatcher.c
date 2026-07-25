#include "command_dispatcher.h"

#include "actuator_commands.h"
#include "balance_commands.h"
#include "command_parser.h"
#include "current_commands.h"
#include "encoder_commands.h"
#include "help_commands.h"
#include "imu_commands.h"
#include "motor_commands.h"
#include "serial/serial.h"
#include "state_commands.h"
#include "system_commands.h"
#include "telemetry_commands.h"

#include <string.h>

#define COMMAND_DISPATCHER_MAX_ARGUMENTS  18

/*
 * Top-level command words are stable public serial commands.
 * Handler modules own validation and command-specific output.
 */
CommandResult CommandDispatcher_ExecuteLine(
    char *line
)
{
    char *arguments[COMMAND_DISPATCHER_MAX_ARGUMENTS];

    const int argument_count =
        CommandParser_Tokenize(
            line,
            arguments,
            COMMAND_DISPATCHER_MAX_ARGUMENTS
        );

    if (argument_count == 0)
    {
        return COMMAND_RESULT_OK;
    }

    if (strcmp(arguments[0], "help") == 0)
    {
        if (argument_count != 1)
        {
            Serial_WriteLine(
                "ERROR: usage: help"
            );

            return COMMAND_RESULT_ERROR;
        }

        return HelpCommands_Print();
    }

    if (strcmp(arguments[0], "motor") == 0)
    {
        return MotorCommands_Handle(argument_count, arguments);
    }

    if (strcmp(arguments[0], "encoder") == 0)
    {
        return EncoderCommands_Handle(argument_count, arguments);
    }

    if (strcmp(arguments[0], "current") == 0)
    {
        return CurrentCommands_Handle(argument_count, arguments);
    }

    if (strcmp(arguments[0], "actuator") == 0)
    {
        return ActuatorCommands_Handle(argument_count, arguments);
    }

    if (strcmp(arguments[0], "state") == 0)
    {
        return StateCommands_Handle(argument_count, arguments);
    }

    if (strcmp(arguments[0], "balance") == 0)
    {
        return BalanceCommands_Handle(argument_count, arguments);
    }

    if (strcmp(arguments[0], "telemetry") == 0)
    {
        return TelemetryCommands_Handle(argument_count, arguments);
    }

    if (strcmp(arguments[0], "system") == 0)
    {
        return SystemCommands_Handle(argument_count, arguments);
    }

    if (strcmp(arguments[0], "imu") == 0)
    {
        return IMUCommands_Handle(argument_count, arguments);
    }

    Serial_WriteLine(
        "ERROR: unknown command."
    );

    Serial_WriteLine(
        "Type 'help' for available commands."
    );

    return COMMAND_RESULT_ERROR;
}
