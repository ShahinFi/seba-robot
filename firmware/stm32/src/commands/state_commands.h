#ifndef STATE_COMMANDS_H
#define STATE_COMMANDS_H

/*
 * Handles the state-estimation command group.
 *
 * arguments[0] must be "state".
 */
void StateCommands_Handle(
    int argument_count,
    char *arguments[]
);

#endif
