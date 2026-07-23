#ifndef TELEMETRY_COMMANDS_H
#define TELEMETRY_COMMANDS_H

/*
 * Handles the telemetry command group.
 *
 * arguments[0] must be "telemetry".
 */
void TelemetryCommands_Handle(
    int argument_count,
    char *arguments[]
);

#endif
