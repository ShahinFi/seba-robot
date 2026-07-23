#ifndef TELEMETRY_COMMANDS_H
#define TELEMETRY_COMMANDS_H

#include "command_result.h"

/*
 * Handles the telemetry command group.
 *
 * arguments[0] must be "telemetry".
 */
CommandResult TelemetryCommands_Handle(
    int argument_count,
    char *arguments[]
);

#endif
