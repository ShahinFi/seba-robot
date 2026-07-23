#include "stm32g4xx_hal.h"

#include "commands/system_commands.h"
#include "communication/telemetry_stream.h"
#include "control/actuator/actuator.h"
#include "control/motion_control/motion_control.h"
#include "control/state_estimation/state_estimation.h"
#include "console/console.h"
#include "current_sensor/current_sensor.h"
#include "encoder/encoder.h"
#include "imu/imu.h"
#include "motor/motor.h"
#include "serial/serial.h"
#include "tests/motor_test.h"

static void SystemClock_Init(void);
static void StreamTelemetry(void);
static void Error_Handler(void);

#define TELEMETRY_STREAM_PERIOD_MS  500U

int main(void)
{
    HAL_Init();
    SystemClock_Init();

    Motor_Init();
    MotorTest_Init();
    Encoder_Init();

    if (!CurrentSensor_Init())
    {
        Error_Handler();
    }

    Actuator_Init();
    MotionControl_Init();

    /*
     * Serial must be available before initializing the IMU,
     * so initialization errors can be reported.
     */
    if (!Serial_Init())
    {
        Error_Handler();
    }

    const bool imu_initialized =
        IMU_Init();

    StateEstimation_Init();

    /*
     * Console input starts after IMU initialization so startup
     * messages cannot interleave with command processing. The
     * console still starts if IMU initialization fails.
     */
    Console_Init();

    while (1)
    {
        Console_Process();
        SystemCommands_ProcessResetRequest();

        if (imu_initialized)
        {
            IMU_Process();
        }

        Console_Process();
        SystemCommands_ProcessResetRequest();
        StreamTelemetry();
    }
}

static void StreamTelemetry(void)
{
    static uint32_t last_stream_ms = 0U;
    const uint32_t now_ms =
        HAL_GetTick();

    if (
        now_ms - last_stream_ms <
        TELEMETRY_STREAM_PERIOD_MS
    )
    {
        return;
    }

    last_stream_ms =
        now_ms;

    TelemetryStream_WriteSnapshot();
}

static void SystemClock_Init(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    RCC_ClkInitTypeDef clock = {0};

    /*
     * Peripheral timing constants are based on a 16 MHz HSI
     * system clock with no PLL.
     */
    oscillator.OscillatorType =
        RCC_OSCILLATORTYPE_HSI;

    oscillator.HSIState =
        RCC_HSI_ON;

    oscillator.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;

    oscillator.PLL.PLLState =
        RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(
            &oscillator
        ) != HAL_OK)
    {
        Error_Handler();
    }

    clock.ClockType =
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    clock.SYSCLKSource =
        RCC_SYSCLKSOURCE_HSI;

    clock.AHBCLKDivider =
        RCC_SYSCLK_DIV1;

    clock.APB1CLKDivider =
        RCC_HCLK_DIV1;

    clock.APB2CLKDivider =
        RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(
            &clock,
            FLASH_LATENCY_0
        ) != HAL_OK)
    {
        Error_Handler();
    }
}

static void Error_Handler(void)
{
    /*
     * Motor_Init() runs before any fallible peripheral setup,
     * so error handling can always force the drivers off.
     */
    Motor_StopAll();
    Motor_Disable();

    while (1)
    {
    }
}

/*
 * Required for HAL_Delay() and HAL timeout handling in this
 * PlatformIO STM32Cube project.
 */
void SysTick_Handler(void)
{
    HAL_IncTick();
}
