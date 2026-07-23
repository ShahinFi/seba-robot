#ifndef IMU_COMMANDS_H
#define IMU_COMMANDS_H

#include "command_result.h"

/*
 * Handles the IMU diagnostic command group.
 *
 * arguments[0] must be "imu".
 */
CommandResult IMUCommands_Handle(
    int argument_count,
    char *arguments[]
);

#endif
