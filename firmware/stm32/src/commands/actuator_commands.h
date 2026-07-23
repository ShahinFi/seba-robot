#ifndef ACTUATOR_COMMANDS_H
#define ACTUATOR_COMMANDS_H

#include "command_result.h"

/*
 * Handles the actuator command group.
 *
 * arguments[0] must be "actuator".
 */
CommandResult ActuatorCommands_Handle(
    int argument_count,
    char *arguments[]
);

#endif
