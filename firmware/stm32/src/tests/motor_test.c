#include "motor_test.h"

#include "motor/motor.h"

/*
 * Manual motor bring-up state. This layer owns direct motor
 * commands while closed-loop control is stopped.
 */
static int16_t left_command;
static int16_t right_command;
static bool drivers_enabled;

static bool MotorTest_IsCommandValid(
    int16_t command_percent
);

static void MotorTest_EnableIfRequired(void);
static void MotorTest_DisableIfStopped(void);

void MotorTest_Init(void)
{
    left_command = 0;
    right_command = 0;
    drivers_enabled = false;

    Motor_StopAll();
    Motor_Disable();
}

bool MotorTest_SetLeft(int16_t command_percent)
{
    if (!MotorTest_IsCommandValid(command_percent))
    {
        return false;
    }

    if (command_percent == 0)
    {
        Motor_StopLeft();
        left_command = 0;

        MotorTest_DisableIfStopped();
        return true;
    }

    MotorTest_EnableIfRequired();

    Motor_SetLeft(command_percent);
    left_command = command_percent;

    return true;
}

bool MotorTest_SetRight(int16_t command_percent)
{
    if (!MotorTest_IsCommandValid(command_percent))
    {
        return false;
    }

    if (command_percent == 0)
    {
        Motor_StopRight();
        right_command = 0;

        MotorTest_DisableIfStopped();
        return true;
    }

    MotorTest_EnableIfRequired();

    Motor_SetRight(command_percent);
    right_command = command_percent;

    return true;
}

void MotorTest_Stop(void)
{
    Motor_StopAll();
    Motor_Disable();

    left_command = 0;
    right_command = 0;
    drivers_enabled = false;
}

int16_t MotorTest_GetLeftCommand(void)
{
    return left_command;
}

int16_t MotorTest_GetRightCommand(void)
{
    return right_command;
}

static bool MotorTest_IsCommandValid(
    int16_t command_percent
)
{
    return
        command_percent >= -100 &&
        command_percent <= 100;
}

static void MotorTest_EnableIfRequired(void)
{
    if (drivers_enabled)
    {
        return;
    }

    /*
     * Motor_Enable() zeros both PWM channels before waking
     * the drivers.
     */
    Motor_Enable();
    drivers_enabled = true;
}

static void MotorTest_DisableIfStopped(void)
{
    /*
     * Keep the drivers enabled while either motor command is
     * nonzero.
     */
    if (
        left_command == 0 &&
        right_command == 0
    )
    {
        Motor_Disable();
        drivers_enabled = false;
    }
}
