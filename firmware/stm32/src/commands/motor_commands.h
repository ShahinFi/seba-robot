#ifndef MOTOR_COMMANDS_H
#define MOTOR_COMMANDS_H

/*
 * Handles the motor bring-up command group.
 *
 * arguments[0] must be "motor".
 */
void MotorCommands_Handle(
    int argument_count,
    char *arguments[]
);

#endif
