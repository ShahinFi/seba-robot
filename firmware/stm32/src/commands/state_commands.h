#ifndef STATE_COMMANDS_H
#define STATE_COMMANDS_H

#include "command_result.h"

/*
 * Handles the state-estimation command group.
 *
 * arguments[0] must be "state".
 */
CommandResult StateCommands_Handle(
    int argument_count,
    char *arguments[]
);

#endif
