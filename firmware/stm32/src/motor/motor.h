#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void Motor_Init(void);

/*
 * Enable wakes the drivers with both PWM channels stopped.
 * Disable stops both PWM channels before sleeping the drivers.
 */
void Motor_Enable(void);
void Motor_Disable(void);

/*
 * Command range:
 *   -100 = full duty, reverse direction
 *      0 = stopped
 *   +100 = full duty, forward direction
 */
void Motor_SetLeft(int16_t command_percent);
void Motor_SetRight(int16_t command_percent);

/*
 * Per-mille commands provide finer PWM resolution for current
 * control:
 *   -1000 = full duty, reverse direction
 *       0 = stopped
 *   +1000 = full duty, forward direction
 */
void Motor_SetLeftPermille(int16_t command_permille);
void Motor_SetRightPermille(int16_t command_permille);

/*
 * Stop functions set PWM duty to zero without changing driver
 * sleep state.
 */
void Motor_StopLeft(void);
void Motor_StopRight(void);
void Motor_StopAll(void);

#endif
