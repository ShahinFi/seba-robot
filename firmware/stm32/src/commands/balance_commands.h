#ifndef BALANCE_COMMANDS_H
#define BALANCE_COMMANDS_H

#include "command_result.h"

/*
 * Handles the balance command group.
 *
 * arguments[0] must be "balance".
 */
CommandResult BalanceCommands_Handle(
    int argument_count,
    char *arguments[]
);

#endif
