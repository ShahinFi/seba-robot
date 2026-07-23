#ifndef ACTUATOR_COMMANDS_H
#define ACTUATOR_COMMANDS_H

/*
 * Handles the actuator command group.
 *
 * arguments[0] must be "actuator".
 */
void ActuatorCommands_Handle(
    int argument_count,
    char *arguments[]
);

#endif
