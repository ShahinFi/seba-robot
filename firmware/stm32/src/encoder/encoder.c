#include "encoder.h"

#include "stm32g4xx_hal.h"

#include <stdint.h>

/*
 * Encoder connections:
 *
 * Left encoder:
 *   A: PA15 = AF1 = TIM2_CH1
 *   B: PB3  = AF1 = TIM2_CH2
 *
 * Right encoder:
 *   A: PC6 = AF4  = TIM8_CH1
 *   B: PB8 = AF10 = TIM8_CH2
 */

/*
 * Sign constants define the robot-forward counting convention.
 */
#define LEFT_ENCODER_SIGN   1
#define RIGHT_ENCODER_SIGN  -1

#define ENCODER_COUNTER_MAX       0xFFFFU
#define ENCODER_HALF_RANGE        32768U
#define ENCODER_FULL_RANGE        65536L

static uint16_t previous_left_count;
static uint16_t previous_right_count;

static int64_t left_position;
static int64_t right_position;

static void Encoder_GPIO_Init(void);
static void Encoder_TIM2_Init(void);
static void Encoder_TIM8_Init(void);

static int32_t Encoder_CalculateDelta(
    uint16_t current_count,
    uint16_t previous_count
);

void Encoder_Init(void)
{
    Encoder_GPIO_Init();

    Encoder_TIM2_Init();
    Encoder_TIM8_Init();

    /*
     * Synchronize the software state with the hardware
     * counters before accumulation starts.
     */
    previous_left_count = (uint16_t)TIM2->CNT;
    previous_right_count = (uint16_t)TIM8->CNT;

    left_position = 0;
    right_position = 0;
}

void Encoder_Update(void)
{
    const uint16_t current_left_count =
        (uint16_t)TIM2->CNT;

    const uint16_t current_right_count =
        (uint16_t)TIM8->CNT;

    const int32_t left_delta =
        Encoder_CalculateDelta(
            current_left_count,
            previous_left_count
        );

    const int32_t right_delta =
        Encoder_CalculateDelta(
            current_right_count,
            previous_right_count
        );

    left_position +=
        (int64_t)(LEFT_ENCODER_SIGN * left_delta);

    right_position +=
        (int64_t)(RIGHT_ENCODER_SIGN * right_delta);

    previous_left_count = current_left_count;
    previous_right_count = current_right_count;
}

int64_t Encoder_GetLeftPosition(void)
{
    return left_position;
}

int64_t Encoder_GetRightPosition(void)
{
    return right_position;
}

void Encoder_ResetLeft(void)
{
    /*
     * Discard any movement that occurred since the most
     * recent update, then establish the current position
     * as the new zero.
     */
    previous_left_count = (uint16_t)TIM2->CNT;
    left_position = 0;
}

void Encoder_ResetRight(void)
{
    previous_right_count = (uint16_t)TIM8->CNT;
    right_position = 0;
}

void Encoder_ResetAll(void)
{
    Encoder_ResetLeft();
    Encoder_ResetRight();
}

static int32_t Encoder_CalculateDelta(
    uint16_t current_count,
    uint16_t previous_count
)
{
    /*
     * This subtraction deliberately uses modulo-65536
     * arithmetic.
     */
    const uint16_t wrapped_difference =
        (uint16_t)(current_count - previous_count);

    if (wrapped_difference < ENCODER_HALF_RANGE)
    {
        return (int32_t)wrapped_difference;
    }

    return
        (int32_t)wrapped_difference -
        ENCODER_FULL_RANGE;
}

static void Encoder_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &gpio);

    /*
     * PB3 has a reset-time debug/trace function, so this
     * configuration explicitly assigns it to TIM2_CH2.
     */
    gpio.Pin = GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF4_TIM8;
    HAL_GPIO_Init(GPIOC, &gpio);

    gpio.Pin = GPIO_PIN_8;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF10_TIM8;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static void Encoder_TIM2_Init(void)
{
    __HAL_RCC_TIM2_FORCE_RESET();
    __HAL_RCC_TIM2_RELEASE_RESET();
    __HAL_RCC_TIM2_CLK_ENABLE();

    TIM2->CR1 = 0U;
    TIM2->CR2 = 0U;
    TIM2->SMCR = 0U;
    TIM2->DIER = 0U;
    TIM2->CCER = 0U;

    TIM2->PSC = 0U;

    /*
     * TIM2 is capable of a wider counter, but we deliberately
     * use a 16-bit range so both encoders share the same wrap
     * handling.
     */
    TIM2->ARR = ENCODER_COUNTER_MAX;
    TIM2->CNT = 0U;

    /*
     * CC1S = 01: channel 1 reads timer input 1.
     * CC2S = 01: channel 2 reads timer input 2.
     *
     * Digital input filtering is disabled so raw encoder edges
     * are counted directly.
     */
    TIM2->CCMR1 =
        (1UL << TIM_CCMR1_CC1S_Pos) |
        (1UL << TIM_CCMR1_CC2S_Pos);

    /*
     * CC1P = 0 and CC2P = 0:
     * both timer inputs use their normal polarity.
     */
    TIM2->CCER = 0U;

    /*
     * SMS = 011: encoder mode 3.
     *
     * The counter responds to transitions on both encoder
     * inputs, giving x4 quadrature counting.
     */
    TIM2->SMCR =
        (3UL << TIM_SMCR_SMS_Pos);

    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 |= TIM_CR1_CEN;
}

static void Encoder_TIM8_Init(void)
{
    /*
     * TIM8 uses the same 16-bit x4 encoder configuration as
     * TIM2 for the right encoder.
     */
    __HAL_RCC_TIM8_FORCE_RESET();
    __HAL_RCC_TIM8_RELEASE_RESET();
    __HAL_RCC_TIM8_CLK_ENABLE();

    TIM8->CR1 = 0U;
    TIM8->CR2 = 0U;
    TIM8->SMCR = 0U;
    TIM8->DIER = 0U;
    TIM8->CCER = 0U;

    TIM8->PSC = 0U;
    TIM8->ARR = ENCODER_COUNTER_MAX;
    TIM8->CNT = 0U;

    TIM8->CCMR1 =
        (1UL << TIM_CCMR1_CC1S_Pos) |
        (1UL << TIM_CCMR1_CC2S_Pos);

    TIM8->CCER = 0U;

    TIM8->SMCR =
        (3UL << TIM_SMCR_SMS_Pos);

    TIM8->EGR = TIM_EGR_UG;
    TIM8->CR1 |= TIM_CR1_CEN;
}
