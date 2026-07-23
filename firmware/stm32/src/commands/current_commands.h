#ifndef CURRENT_COMMANDS_H
#define CURRENT_COMMANDS_H

#include "command_result.h"

/*
 * Handles the current-sensor diagnostic command group.
 *
 * arguments[0] must be "current".
 */
CommandResult CurrentCommands_Handle(
    int argument_count,
    char *arguments[]
);

#endif
