#ifndef SYSTEM_COMMANDS_H
#define SYSTEM_COMMANDS_H

#include "command_result.h"

/*
 * Handles system-level commands that affect the STM32 runtime.
 *
 * arguments[0] must be "system".
 */
CommandResult SystemCommands_Handle(
    int argument_count,
    char *arguments[]
);

/*
 * Performs a reset requested by the system command handler.
 * Call after command processing has had a chance to finish
 * writing its response.
 */
void SystemCommands_ProcessResetRequest(void);

#endif
