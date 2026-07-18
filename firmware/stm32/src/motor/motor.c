#include "motor.h"

#include "stm32g4xx_hal.h"

#include <stdbool.h>

#define MOTOR_PWM_FREQUENCY_HZ  20000U
#define MOTOR_TIMER_CLOCK_HZ    16000000U
#define MOTOR_PWM_PERIOD_COUNTS \
    (MOTOR_TIMER_CLOCK_HZ / MOTOR_PWM_FREQUENCY_HZ)
#define MOTOR_PWM_ARR_VALUE     (MOTOR_PWM_PERIOD_COUNTS - 1U)
#define MOTOR_COMMAND_PERCENT_SCALE  10

/*
 * Left motor:
 *   PWM: PC7 / TIM3_CH2
 *   DIR: PA8
 *   SLP: PA10
 *
 * Right motor:
 *   PWM: PB6 / TIM4_CH1
 *   DIR: PA9
 *   SLP: PB5
 */

#define LEFT_DIR_PORT      GPIOA
#define LEFT_DIR_PIN       GPIO_PIN_8
#define LEFT_SLEEP_PORT    GPIOA
#define LEFT_SLEEP_PIN     GPIO_PIN_10

#define RIGHT_DIR_PORT     GPIOA
#define RIGHT_DIR_PIN      GPIO_PIN_9
#define RIGHT_SLEEP_PORT   GPIOB
#define RIGHT_SLEEP_PIN    GPIO_PIN_5

#define LEFT_MOTOR_SIGN   -1
#define RIGHT_MOTOR_SIGN   1

static int16_t left_command_permille;
static int16_t right_command_permille;

static void Motor_GPIO_Init(void);
static void Motor_PWM_Init(void);

static int16_t Motor_ApplySign(
    int16_t command,
    int16_t motor_sign
);

static int16_t Motor_ClampCommand(int16_t command_percent);
static int16_t Motor_ClampPermille(int16_t command_permille);
static uint32_t Motor_PermilleToCompare(uint16_t duty_permille);
static bool Motor_DirectionChanges(
    int16_t previous_command,
    int16_t next_command
);

void Motor_Init(void)
{
    /*
     * PB6 shares the STM32G4 UCPD function. Disable the
     * dead-battery pull-down before using PB6 as TIM4_CH1.
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    SET_BIT(PWR->CR3, PWR_CR3_UCPD_DBDIS);

    Motor_GPIO_Init();
    Motor_PWM_Init();

    left_command_permille = 0;
    right_command_permille = 0;

    Motor_StopAll();
    Motor_Disable();
}

void Motor_Enable(void)
{
    /*
     * Wake the drivers only after both PWM commands are zero.
     */
    Motor_StopAll();

    /*
     * Pololu SLP is active low:
     * high = enabled
     * low  = asleep/disabled
     */
    HAL_GPIO_WritePin(
        LEFT_SLEEP_PORT,
        LEFT_SLEEP_PIN,
        GPIO_PIN_SET
    );

    HAL_GPIO_WritePin(
        RIGHT_SLEEP_PORT,
        RIGHT_SLEEP_PIN,
        GPIO_PIN_SET
    );

    /*
     * Pololu driver wake time before PWM may be applied.
     */
    HAL_Delay(2U);
}

void Motor_Disable(void)
{
    /*
     * Remove PWM before disabling the H-bridges.
     */
    Motor_StopAll();

    HAL_GPIO_WritePin(
        LEFT_SLEEP_PORT,
        LEFT_SLEEP_PIN,
        GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        RIGHT_SLEEP_PORT,
        RIGHT_SLEEP_PIN,
        GPIO_PIN_RESET
    );
}

void Motor_SetLeft(int16_t command_percent)
{
    Motor_SetLeftPermille(
        (int16_t)(
            Motor_ClampCommand(command_percent) *
            MOTOR_COMMAND_PERCENT_SCALE
        )
    );
}

void Motor_SetRight(int16_t command_percent)
{
    Motor_SetRightPermille(
        (int16_t)(
            Motor_ClampCommand(command_percent) *
            MOTOR_COMMAND_PERCENT_SCALE
        )
    );
}

void Motor_SetLeftPermille(int16_t command_permille)
{
    uint16_t duty_permille;

    command_permille =
        Motor_ApplySign(
            Motor_ClampPermille(command_permille),
            LEFT_MOTOR_SIGN
        );

    if (Motor_DirectionChanges(
            left_command_permille,
            command_permille
        ))
    {
        Motor_StopLeft();
    }

    if (command_permille < 0)
    {
        HAL_GPIO_WritePin(
            LEFT_DIR_PORT,
            LEFT_DIR_PIN,
            GPIO_PIN_SET
        );

        duty_permille = (uint16_t)(-command_permille);
    }
    else
    {
        HAL_GPIO_WritePin(
            LEFT_DIR_PORT,
            LEFT_DIR_PIN,
            GPIO_PIN_RESET
        );

        duty_permille = (uint16_t)command_permille;
    }

    TIM3->CCR2 = Motor_PermilleToCompare(duty_permille);
    left_command_permille = command_permille;
}

void Motor_SetRightPermille(int16_t command_permille)
{
    uint16_t duty_permille;

    command_permille =
        Motor_ApplySign(
            Motor_ClampPermille(command_permille),
            RIGHT_MOTOR_SIGN
        );

    if (Motor_DirectionChanges(
            right_command_permille,
            command_permille
        ))
    {
        Motor_StopRight();
    }

    if (command_permille < 0)
    {
        HAL_GPIO_WritePin(
            RIGHT_DIR_PORT,
            RIGHT_DIR_PIN,
            GPIO_PIN_SET
        );

        duty_permille = (uint16_t)(-command_permille);
    }
    else
    {
        HAL_GPIO_WritePin(
            RIGHT_DIR_PORT,
            RIGHT_DIR_PIN,
            GPIO_PIN_RESET
        );

        duty_permille = (uint16_t)command_permille;
    }

    TIM4->CCR1 = Motor_PermilleToCompare(duty_permille);
    right_command_permille = command_permille;
}

void Motor_StopLeft(void)
{
    TIM3->CCR2 = 0U;
    left_command_permille = 0;
}

void Motor_StopRight(void)
{
    TIM4->CCR1 = 0U;
    right_command_permille = 0;
}

void Motor_StopAll(void)
{
    Motor_StopLeft();
    Motor_StopRight();
}

static int16_t Motor_ApplySign(
    int16_t command,
    int16_t motor_sign
)
{
    return
        (int16_t)(
            command *
            motor_sign
        );
}

static int16_t Motor_ClampCommand(int16_t command_percent)
{
    if (command_percent > 100)
    {
        return 100;
    }

    if (command_percent < -100)
    {
        return -100;
    }

    return command_percent;
}

static int16_t Motor_ClampPermille(int16_t command_permille)
{
    if (command_permille > 1000)
    {
        return 1000;
    }

    if (command_permille < -1000)
    {
        return -1000;
    }

    return command_permille;
}

static uint32_t Motor_PermilleToCompare(uint16_t duty_permille)
{
    if (duty_permille > 1000U)
    {
        duty_permille = 1000U;
    }

    return
        (
            MOTOR_PWM_PERIOD_COUNTS *
            duty_permille +
            500U
        ) /
        1000U;
}

static bool Motor_DirectionChanges(
    int16_t previous_command,
    int16_t next_command
)
{
    return
        (
            previous_command < 0 &&
            next_command > 0
        ) ||
        (
            previous_command > 0 &&
            next_command < 0
        );
}

static void Motor_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /*
     * Establish safe output levels before the pins become
     * push-pull outputs.
     */
    HAL_GPIO_WritePin(
        GPIOA,
        LEFT_DIR_PIN |
        RIGHT_DIR_PIN |
        LEFT_SLEEP_PIN,
        GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        GPIOB,
        RIGHT_SLEEP_PIN,
        GPIO_PIN_RESET
    );

    gpio.Pin =
        LEFT_DIR_PIN |
        RIGHT_DIR_PIN |
        LEFT_SLEEP_PIN;

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = RIGHT_SLEEP_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOC, &gpio);

    gpio.Pin = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static void Motor_PWM_Init(void)
{
    /*
     * Reset both timers to known states.
     */
    __HAL_RCC_TIM3_FORCE_RESET();
    __HAL_RCC_TIM4_FORCE_RESET();

    __HAL_RCC_TIM3_RELEASE_RESET();
    __HAL_RCC_TIM4_RELEASE_RESET();

    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    /*
     * TIM3_CH2: left motor, 20 kHz PWM.
     */
    TIM3->CR1 = 0U;
    TIM3->CR2 = 0U;
    TIM3->SMCR = 0U;
    TIM3->DIER = 0U;
    TIM3->CCER = 0U;

    TIM3->PSC = 0U;
    TIM3->ARR = MOTOR_PWM_ARR_VALUE;
    TIM3->CCR2 = 0U;

    /*
     * OC2M = 110 selects PWM mode 1. OC2PE enables compare
     * preload so duty changes update on timer events.
     */
    TIM3->CCMR1 =
        (6UL << TIM_CCMR1_OC2M_Pos) |
        TIM_CCMR1_OC2PE;

    TIM3->CCER = TIM_CCER_CC2E;
    TIM3->CR1 = TIM_CR1_ARPE;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 |= TIM_CR1_CEN;

    /*
     * TIM4_CH1: right motor, 20 kHz PWM.
     */
    TIM4->CR1 = 0U;
    TIM4->CR2 = 0U;
    TIM4->SMCR = 0U;
    TIM4->DIER = 0U;
    TIM4->CCER = 0U;

    TIM4->PSC = 0U;
    TIM4->ARR = MOTOR_PWM_ARR_VALUE;
    TIM4->CCR1 = 0U;

    /*
     * OC1M = 110 selects PWM mode 1. OC1PE enables compare
     * preload so duty changes update on timer events.
     */
    TIM4->CCMR1 =
        (6UL << TIM_CCMR1_OC1M_Pos) |
        TIM_CCMR1_OC1PE;

    TIM4->CCER = TIM_CCER_CC1E;
    TIM4->CR1 = TIM_CR1_ARPE;
    TIM4->EGR = TIM_EGR_UG;
    TIM4->CR1 |= TIM_CR1_CEN;
}
