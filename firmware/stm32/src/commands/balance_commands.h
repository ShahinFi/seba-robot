#ifndef BALANCE_COMMANDS_H
#define BALANCE_COMMANDS_H

/*
 * Handles the balance command group.
 *
 * arguments[0] must be "balance".
 */
void BalanceCommands_Handle(
    int argument_count,
    char *arguments[]
);

#endif
