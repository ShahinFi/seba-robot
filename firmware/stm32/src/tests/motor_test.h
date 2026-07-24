#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Clears manual motor-test state and leaves both drivers
 * disabled.
 */
void MotorTest_Init(void);

/*
 * Command range is -100 to +100 percent. A zero command stops
 * that motor.
 */
bool MotorTest_SetLeft(int16_t command_percent);
bool MotorTest_SetRight(int16_t command_percent);

/*
 * Stops both motors and disables both drivers.
 */
void MotorTest_Stop(void);

/*
 * Returns the last accepted manual motor-test command in
 * percent.
 */
int16_t MotorTest_GetLeftCommand(void);
int16_t MotorTest_GetRightCommand(void);

#endif
