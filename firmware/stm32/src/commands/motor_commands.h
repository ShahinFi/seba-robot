#ifndef MOTOR_COMMANDS_H
#define MOTOR_COMMANDS_H

#include "command_result.h"

/*
 * Handles the motor bring-up command group.
 *
 * arguments[0] must be "motor".
 */
CommandResult MotorCommands_Handle(
    int argument_count,
    char *arguments[]
);

#endif
