#ifndef IMU_COMMANDS_H
#define IMU_COMMANDS_H

/*
 * Handles the IMU diagnostic command group.
 *
 * arguments[0] must be "imu".
 */
void IMUCommands_Handle(
    int argument_count,
    char *arguments[]
);

#endif
