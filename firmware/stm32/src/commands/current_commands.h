#ifndef CURRENT_COMMANDS_H
#define CURRENT_COMMANDS_H

/*
 * Handles the current-sensor diagnostic command group.
 *
 * arguments[0] must be "current".
 */
void CurrentCommands_Handle(
    int argument_count,
    char *arguments[]
);

#endif
