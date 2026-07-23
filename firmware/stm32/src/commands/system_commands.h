#ifndef SYSTEM_COMMANDS_H
#define SYSTEM_COMMANDS_H

/*
 * Handles system-level commands that affect the STM32 runtime.
 *
 * arguments[0] must be "system".
 */
void SystemCommands_Handle(
    int argument_count,
    char *arguments[]
);

#endif
