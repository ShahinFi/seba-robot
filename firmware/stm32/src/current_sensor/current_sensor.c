#include "current_sensor.h"
#include "current_sensor_diagnostic.h"

#include "stm32g4xx_hal.h"

#include <limits.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Left ACS711:
 *   VOUT  -> PA4 -> ADC2_IN17
 *   FAULT -> PC2
 *
 * Right ACS711:
 *   VOUT  -> PB0 -> ADC1_IN15
 *   FAULT -> PC3
 */

#define CURRENT_SENSOR_ADC_MAX           4095L
#define CURRENT_SENSOR_REFERENCE_MV      3300L
#define CURRENT_SENSOR_SENSITIVITY_MV_A  90L

#define CURRENT_SENSOR_PWM_HZ                 20000U
#define CURRENT_SENSOR_SAMPLES_PER_PWM_PERIOD     4U
#define CURRENT_SENSOR_TIMED_SAMPLE_HZ            \
    (CURRENT_SENSOR_PWM_HZ * CURRENT_SENSOR_SAMPLES_PER_PWM_PERIOD)
#define CURRENT_SENSOR_DMA_BUFFER_SAMPLES        80U
#define CURRENT_SENSOR_DMA_HALF_SAMPLES          \
    (CURRENT_SENSOR_DMA_BUFFER_SAMPLES / 2U)
#define CURRENT_SENSOR_ZERO_SAMPLES             256U
#define CURRENT_SENSOR_ADC_TIMEOUT_MS            10U
#define CURRENT_SENSOR_SETTLE_TIME_MS            20U

/*
 * Sign constants define the positive motor-current convention.
 */
#define LEFT_CURRENT_SIGN   -1L
#define RIGHT_CURRENT_SIGN  -1L

#define LEFT_FAULT_PORT   GPIOC
#define LEFT_FAULT_PIN    GPIO_PIN_2

#define RIGHT_FAULT_PORT  GPIOC
#define RIGHT_FAULT_PIN   GPIO_PIN_3

typedef struct
{
    volatile bool active;
    volatile bool done;
    volatile uint32_t target_count;
    volatile uint32_t sample_count;
    volatile uint64_t adc_total;
    volatile uint16_t minimum_adc;
    volatile uint16_t maximum_adc;
} CurrentSensorCaptureState;

static ADC_HandleTypeDef left_adc;
static ADC_HandleTypeDef right_adc;

static DMA_HandleTypeDef left_dma;
static DMA_HandleTypeDef right_dma;

static uint32_t left_dma_buffer[
    CURRENT_SENSOR_DMA_BUFFER_SAMPLES
];

static uint32_t right_dma_buffer[
    CURRENT_SENSOR_DMA_BUFFER_SAMPLES
];

static uint16_t left_zero_adc;
static uint16_t right_zero_adc;

static volatile uint16_t left_latest_adc;
static volatile uint16_t right_latest_adc;
static volatile int32_t left_latest_current_ma;
static volatile int32_t right_latest_current_ma;
static volatile uint32_t left_filtered_blocks;
static volatile uint32_t right_filtered_blocks;
static volatile bool left_latest_ready;
static volatile bool right_latest_ready;

static CurrentSensorCaptureState left_capture;
static CurrentSensorCaptureState right_capture;

static void CurrentSensor_GPIO_Init(void);
static bool CurrentSensor_DMA_Init(void);
static bool CurrentSensor_Timer_Init(void);

static bool CurrentSensor_ADC1_Init(
    bool timer_triggered,
    bool calibrate
);

static bool CurrentSensor_ADC2_Init(
    bool timer_triggered,
    bool calibrate
);

static bool CurrentSensor_StartTimedSampling(void);

static bool CurrentSensor_ReadAverage(
    ADC_HandleTypeDef *adc,
    uint32_t number_of_samples,
    uint16_t *average
);

static bool CurrentSensor_ReadRawSample(
    ADC_HandleTypeDef *adc,
    uint16_t *raw_adc
);

static bool CurrentSensor_ReadLatest(
    CurrentSensorChannel channel,
    CurrentSensorReading *reading
);

static void CurrentSensor_ProcessSamples(
    CurrentSensorChannel channel,
    const uint32_t *samples,
    uint32_t offset,
    uint32_t count
);

static void CurrentSensor_UpdateCapture(
    CurrentSensorCaptureState *state,
    uint16_t raw_adc
);

static int32_t CurrentSensor_CountsToMilliamps(
    int32_t adc_difference,
    int32_t direction_sign
);

static int32_t CurrentSensor_DivideSignedRounded(
    int64_t numerator,
    uint32_t denominator
);

bool CurrentSensor_Init(void)
{
    CurrentSensor_GPIO_Init();

    __HAL_RCC_ADC12_CLK_ENABLE();

    if (!CurrentSensor_ADC1_Init(
            false,
            true
        ))
    {
        return false;
    }

    if (!CurrentSensor_ADC2_Init(
            false,
            true
        ))
    {
        return false;
    }

    HAL_Delay(CURRENT_SENSOR_SETTLE_TIME_MS);

    if (!CurrentSensor_ReadAverage(
            &left_adc,
            CURRENT_SENSOR_ZERO_SAMPLES,
            &left_zero_adc
        ))
    {
        return false;
    }

    if (!CurrentSensor_ReadAverage(
            &right_adc,
            CURRENT_SENSOR_ZERO_SAMPLES,
            &right_zero_adc
        ))
    {
        return false;
    }

    if (!CurrentSensor_StartTimedSampling())
    {
        return false;
    }

    return true;
}

bool CurrentSensor_ReadLeft(
    CurrentSensorReading *reading
)
{
    return CurrentSensor_ReadLatest(
        CURRENT_SENSOR_LEFT,
        reading
    );
}

bool CurrentSensor_ReadRight(
    CurrentSensorReading *reading
)
{
    return CurrentSensor_ReadLatest(
        CURRENT_SENSOR_RIGHT,
        reading
    );
}

bool CurrentSensor_ReadBoth(
    CurrentSensorReading *left,
    CurrentSensorReading *right
)
{
    if (
        left == NULL ||
        right == NULL
    )
    {
        return false;
    }

    if (!CurrentSensor_ReadLeft(left))
    {
        return false;
    }

    if (!CurrentSensor_ReadRight(right))
    {
        return false;
    }

    return true;
}

bool CurrentSensor_DiagnosticCapture(
    CurrentSensorChannel channel,
    uint32_t sample_count,
    CurrentSensorDiagnosticCapture *capture
)
{
    CurrentSensorCaptureState *state;
    int32_t minimum_adc_current_ma;
    int32_t maximum_adc_current_ma;
    const uint32_t timeout_ms =
        sample_count /
        (CURRENT_SENSOR_TIMED_SAMPLE_HZ / 1000U) +
        100U;

    const uint32_t start_ms =
        HAL_GetTick();

    if (
        capture == NULL ||
        sample_count == 0U
    )
    {
        return false;
    }

    if (channel == CURRENT_SENSOR_LEFT)
    {
        state = &left_capture;
    }
    else if (channel == CURRENT_SENSOR_RIGHT)
    {
        state = &right_capture;
    }
    else
    {
        return false;
    }

    __disable_irq();

    state->active = true;
    state->done = false;
    state->target_count = sample_count;
    state->sample_count = 0U;
    state->adc_total = 0ULL;
    state->minimum_adc = UINT16_MAX;
    state->maximum_adc = 0U;

    __enable_irq();

    while (!state->done)
    {
        if (
            HAL_GetTick() -
            start_ms >
            timeout_ms
        )
        {
            __disable_irq();
            state->active = false;
            __enable_irq();
            return false;
        }
    }

    __disable_irq();

    capture->sample_count = state->sample_count;
    capture->zero_adc =
        channel == CURRENT_SENSOR_LEFT
            ? left_zero_adc
            : right_zero_adc;

    capture->minimum_adc = state->minimum_adc;
    capture->maximum_adc = state->maximum_adc;

    capture->average_adc =
        (uint16_t)(
            (
                state->adc_total +
                state->sample_count / 2U
            ) /
            state->sample_count
        );

    minimum_adc_current_ma =
        CurrentSensor_CountsToMilliamps(
            (int32_t)capture->minimum_adc -
            (int32_t)capture->zero_adc,
            channel == CURRENT_SENSOR_LEFT
                ? LEFT_CURRENT_SIGN
                : RIGHT_CURRENT_SIGN
        );

    maximum_adc_current_ma =
        CurrentSensor_CountsToMilliamps(
            (int32_t)capture->maximum_adc -
            (int32_t)capture->zero_adc,
            channel == CURRENT_SENSOR_LEFT
                ? LEFT_CURRENT_SIGN
                : RIGHT_CURRENT_SIGN
        );

    if (minimum_adc_current_ma < maximum_adc_current_ma)
    {
        capture->minimum_current_ma =
            minimum_adc_current_ma;

        capture->maximum_current_ma =
            maximum_adc_current_ma;
    }
    else
    {
        capture->minimum_current_ma =
            maximum_adc_current_ma;

        capture->maximum_current_ma =
            minimum_adc_current_ma;
    }

    capture->average_current_ma =
        CurrentSensor_CountsToMilliamps(
            (int32_t)capture->average_adc -
            (int32_t)capture->zero_adc,
            channel == CURRENT_SENSOR_LEFT
                ? LEFT_CURRENT_SIGN
                : RIGHT_CURRENT_SIGN
        );

    __enable_irq();

    if (channel == CURRENT_SENSOR_LEFT)
    {
        capture->fault_active =
            HAL_GPIO_ReadPin(
                LEFT_FAULT_PORT,
                LEFT_FAULT_PIN
            ) == GPIO_PIN_RESET;
    }
    else
    {
        capture->fault_active =
            HAL_GPIO_ReadPin(
                RIGHT_FAULT_PORT,
                RIGHT_FAULT_PIN
            ) == GPIO_PIN_RESET;
    }

    return true;
}

static void CurrentSensor_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_4;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gpio);

    /*
     * ACS711 FAULT outputs are active low.
     */
    gpio.Pin =
        LEFT_FAULT_PIN |
        RIGHT_FAULT_PIN;

    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);
}

static bool CurrentSensor_DMA_Init(void)
{
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    right_dma.Instance =
        DMA1_Channel1;

    right_dma.Init.Request =
        DMA_REQUEST_ADC1;

    right_dma.Init.Direction =
        DMA_PERIPH_TO_MEMORY;

    right_dma.Init.PeriphInc =
        DMA_PINC_DISABLE;

    right_dma.Init.MemInc =
        DMA_MINC_ENABLE;

    right_dma.Init.PeriphDataAlignment =
        DMA_PDATAALIGN_WORD;

    right_dma.Init.MemDataAlignment =
        DMA_MDATAALIGN_WORD;

    right_dma.Init.Mode =
        DMA_CIRCULAR;

    right_dma.Init.Priority =
        DMA_PRIORITY_HIGH;

    if (HAL_DMA_Init(&right_dma) != HAL_OK)
    {
        return false;
    }

    __HAL_LINKDMA(
        &right_adc,
        DMA_Handle,
        right_dma
    );

    left_dma.Instance =
        DMA1_Channel2;

    left_dma.Init.Request =
        DMA_REQUEST_ADC2;

    left_dma.Init.Direction =
        DMA_PERIPH_TO_MEMORY;

    left_dma.Init.PeriphInc =
        DMA_PINC_DISABLE;

    left_dma.Init.MemInc =
        DMA_MINC_ENABLE;

    left_dma.Init.PeriphDataAlignment =
        DMA_PDATAALIGN_WORD;

    left_dma.Init.MemDataAlignment =
        DMA_MDATAALIGN_WORD;

    left_dma.Init.Mode =
        DMA_CIRCULAR;

    left_dma.Init.Priority =
        DMA_PRIORITY_HIGH;

    if (HAL_DMA_Init(&left_dma) != HAL_OK)
    {
        return false;
    }

    __HAL_LINKDMA(
        &left_adc,
        DMA_Handle,
        left_dma
    );

    HAL_NVIC_SetPriority(
        DMA1_Channel1_IRQn,
        5U,
        0U
    );

    HAL_NVIC_EnableIRQ(
        DMA1_Channel1_IRQn
    );

    HAL_NVIC_SetPriority(
        DMA1_Channel2_IRQn,
        5U,
        0U
    );

    HAL_NVIC_EnableIRQ(
        DMA1_Channel2_IRQn
    );

    return true;
}

static bool CurrentSensor_Timer_Init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();

    TIM6->CR1 = 0U;
    TIM6->CR2 = 0U;
    TIM6->DIER = 0U;
    TIM6->PSC = 0U;
    TIM6->ARR =
        (
            SystemCoreClock /
            CURRENT_SENSOR_TIMED_SAMPLE_HZ
        ) -
        1U;

    /*
     * MMS = 010 makes TIM6 update events drive TRGO.
     */
    TIM6->CR2 =
        2UL << TIM_CR2_MMS_Pos;

    TIM6->EGR =
        TIM_EGR_UG;

    TIM6->CR1 =
        TIM_CR1_CEN;

    return true;
}

static bool CurrentSensor_ADC1_Init(
    bool timer_triggered,
    bool calibrate
)
{
    ADC_ChannelConfTypeDef channel = {0};

    right_adc.Instance = ADC1;

    right_adc.Init.ClockPrescaler =
        ADC_CLOCK_SYNC_PCLK_DIV1;

    right_adc.Init.Resolution =
        ADC_RESOLUTION_12B;

    right_adc.Init.DataAlign =
        ADC_DATAALIGN_RIGHT;

    right_adc.Init.GainCompensation = 0;

    right_adc.Init.ScanConvMode =
        ADC_SCAN_DISABLE;

    right_adc.Init.EOCSelection =
        ADC_EOC_SINGLE_CONV;

    right_adc.Init.LowPowerAutoWait =
        DISABLE;

    right_adc.Init.ContinuousConvMode =
        DISABLE;

    right_adc.Init.NbrOfConversion = 1;

    right_adc.Init.DiscontinuousConvMode =
        DISABLE;

    right_adc.Init.ExternalTrigConv =
        timer_triggered
            ? ADC_EXTERNALTRIG_T6_TRGO
            : ADC_SOFTWARE_START;

    right_adc.Init.ExternalTrigConvEdge =
        timer_triggered
            ? ADC_EXTERNALTRIGCONVEDGE_RISING
            : ADC_EXTERNALTRIGCONVEDGE_NONE;

    right_adc.Init.DMAContinuousRequests =
        timer_triggered
            ? ENABLE
            : DISABLE;

    right_adc.Init.Overrun =
        ADC_OVR_DATA_OVERWRITTEN;

    right_adc.Init.OversamplingMode =
        DISABLE;

    if (HAL_ADC_Init(&right_adc) != HAL_OK)
    {
        return false;
    }

    if (
        calibrate &&
        HAL_ADCEx_Calibration_Start(
            &right_adc,
            ADC_SINGLE_ENDED
        ) != HAL_OK
    )
    {
        return false;
    }

    channel.Channel =
        ADC_CHANNEL_15;

    channel.Rank =
        ADC_REGULAR_RANK_1;

    channel.SamplingTime =
        ADC_SAMPLETIME_47CYCLES_5;

    channel.SingleDiff =
        ADC_SINGLE_ENDED;

    channel.OffsetNumber =
        ADC_OFFSET_NONE;

    channel.Offset = 0;

    return HAL_ADC_ConfigChannel(
               &right_adc,
               &channel
           ) == HAL_OK;
}

static bool CurrentSensor_ADC2_Init(
    bool timer_triggered,
    bool calibrate
)
{
    ADC_ChannelConfTypeDef channel = {0};

    left_adc.Instance = ADC2;

    left_adc.Init.ClockPrescaler =
        ADC_CLOCK_SYNC_PCLK_DIV1;

    left_adc.Init.Resolution =
        ADC_RESOLUTION_12B;

    left_adc.Init.DataAlign =
        ADC_DATAALIGN_RIGHT;

    left_adc.Init.GainCompensation = 0;

    left_adc.Init.ScanConvMode =
        ADC_SCAN_DISABLE;

    left_adc.Init.EOCSelection =
        ADC_EOC_SINGLE_CONV;

    left_adc.Init.LowPowerAutoWait =
        DISABLE;

    left_adc.Init.ContinuousConvMode =
        DISABLE;

    left_adc.Init.NbrOfConversion = 1;

    left_adc.Init.DiscontinuousConvMode =
        DISABLE;

    left_adc.Init.ExternalTrigConv =
        timer_triggered
            ? ADC_EXTERNALTRIG_T6_TRGO
            : ADC_SOFTWARE_START;

    left_adc.Init.ExternalTrigConvEdge =
        timer_triggered
            ? ADC_EXTERNALTRIGCONVEDGE_RISING
            : ADC_EXTERNALTRIGCONVEDGE_NONE;

    left_adc.Init.DMAContinuousRequests =
        timer_triggered
            ? ENABLE
            : DISABLE;

    left_adc.Init.Overrun =
        ADC_OVR_DATA_OVERWRITTEN;

    left_adc.Init.OversamplingMode =
        DISABLE;

    if (HAL_ADC_Init(&left_adc) != HAL_OK)
    {
        return false;
    }

    if (
        calibrate &&
        HAL_ADCEx_Calibration_Start(
            &left_adc,
            ADC_SINGLE_ENDED
        ) != HAL_OK
    )
    {
        return false;
    }

    channel.Channel =
        ADC_CHANNEL_17;

    channel.Rank =
        ADC_REGULAR_RANK_1;

    channel.SamplingTime =
        ADC_SAMPLETIME_47CYCLES_5;

    channel.SingleDiff =
        ADC_SINGLE_ENDED;

    channel.OffsetNumber =
        ADC_OFFSET_NONE;

    channel.Offset = 0;

    return HAL_ADC_ConfigChannel(
               &left_adc,
               &channel
           ) == HAL_OK;
}

static bool CurrentSensor_StartTimedSampling(void)
{
    left_latest_ready = false;
    right_latest_ready = false;
    left_filtered_blocks = 0U;
    right_filtered_blocks = 0U;

    left_capture.active = false;
    left_capture.done = false;
    right_capture.active = false;
    right_capture.done = false;

    if (!CurrentSensor_DMA_Init())
    {
        return false;
    }

    if (!CurrentSensor_ADC1_Init(
            true,
            false
        ))
    {
        return false;
    }

    if (!CurrentSensor_ADC2_Init(
            true,
            false
        ))
    {
        return false;
    }

    if (HAL_ADC_Start_DMA(
            &right_adc,
            right_dma_buffer,
            CURRENT_SENSOR_DMA_BUFFER_SAMPLES
        ) != HAL_OK)
    {
        return false;
    }

    if (HAL_ADC_Start_DMA(
            &left_adc,
            left_dma_buffer,
            CURRENT_SENSOR_DMA_BUFFER_SAMPLES
        ) != HAL_OK)
    {
        return false;
    }

    if (!CurrentSensor_Timer_Init())
    {
        return false;
    }

    const uint32_t start_ms =
        HAL_GetTick();

    while (
        !left_latest_ready ||
        !right_latest_ready
    )
    {
        if (
            HAL_GetTick() -
            start_ms >
            CURRENT_SENSOR_ADC_TIMEOUT_MS
        )
        {
            return false;
        }
    }

    return true;
}

static bool CurrentSensor_ReadAverage(
    ADC_HandleTypeDef *adc,
    uint32_t number_of_samples,
    uint16_t *average
)
{
    uint32_t total = 0U;

    if (
        adc == NULL ||
        average == NULL ||
        number_of_samples == 0U
    )
    {
        return false;
    }

    for (
        uint32_t sample = 0U;
        sample < number_of_samples;
        sample++
    )
    {
        uint16_t raw_adc;

        if (!CurrentSensor_ReadRawSample(
                adc,
                &raw_adc
            ))
        {
            return false;
        }

        total += raw_adc;
    }

    *average =
        (uint16_t)(
            (
                total +
                number_of_samples / 2U
            ) /
            number_of_samples
        );

    return true;
}

static bool CurrentSensor_ReadRawSample(
    ADC_HandleTypeDef *adc,
    uint16_t *raw_adc
)
{
    if (
        adc == NULL ||
        raw_adc == NULL
    )
    {
        return false;
    }

    if (HAL_ADC_Start(adc) != HAL_OK)
    {
        return false;
    }

    if (HAL_ADC_PollForConversion(
            adc,
            CURRENT_SENSOR_ADC_TIMEOUT_MS
        ) != HAL_OK)
    {
        HAL_ADC_Stop(adc);
        return false;
    }

    *raw_adc =
        (uint16_t)HAL_ADC_GetValue(adc);

    return HAL_ADC_Stop(adc) == HAL_OK;
}

static bool CurrentSensor_ReadLatest(
    CurrentSensorChannel channel,
    CurrentSensorReading *reading
)
{
    if (reading == NULL)
    {
        return false;
    }

    __disable_irq();

    if (channel == CURRENT_SENSOR_LEFT)
    {
        if (!left_latest_ready)
        {
            __enable_irq();
            return false;
        }

        reading->raw_adc = left_latest_adc;
        reading->zero_adc = left_zero_adc;
        reading->current_ma = left_latest_current_ma;
    }
    else if (channel == CURRENT_SENSOR_RIGHT)
    {
        if (!right_latest_ready)
        {
            __enable_irq();
            return false;
        }

        reading->raw_adc = right_latest_adc;
        reading->zero_adc = right_zero_adc;
        reading->current_ma = right_latest_current_ma;
    }
    else
    {
        __enable_irq();
        return false;
    }

    __enable_irq();

    if (channel == CURRENT_SENSOR_LEFT)
    {
        reading->fault_active =
            HAL_GPIO_ReadPin(
                LEFT_FAULT_PORT,
                LEFT_FAULT_PIN
            ) == GPIO_PIN_RESET;
    }
    else
    {
        reading->fault_active =
            HAL_GPIO_ReadPin(
                RIGHT_FAULT_PORT,
                RIGHT_FAULT_PIN
            ) == GPIO_PIN_RESET;
    }

    return true;
}

static void CurrentSensor_ProcessSamples(
    CurrentSensorChannel channel,
    const uint32_t *samples,
    uint32_t offset,
    uint32_t count
)
{
    uint64_t adc_total = 0ULL;
    uint16_t average_adc;
    CurrentSensorCaptureState *capture;

    if (channel == CURRENT_SENSOR_LEFT)
    {
        capture = &left_capture;
    }
    else
    {
        capture = &right_capture;
    }

    for (
        uint32_t index = 0U;
        index < count;
        index++
    )
    {
        const uint16_t raw_adc =
            (uint16_t)(
                samples[offset + index] &
                0xFFFFU
            );

        adc_total += raw_adc;

        CurrentSensor_UpdateCapture(
            capture,
            raw_adc
        );
    }

    average_adc =
        (uint16_t)(
            (
                adc_total +
                count / 2U
            ) /
            count
        );

    if (channel == CURRENT_SENSOR_LEFT)
    {
        left_latest_adc = average_adc;
        left_latest_current_ma =
            CurrentSensor_CountsToMilliamps(
                (int32_t)average_adc -
                (int32_t)left_zero_adc,
                LEFT_CURRENT_SIGN
            );

        left_filtered_blocks++;
        left_latest_ready = true;
    }
    else
    {
        right_latest_adc = average_adc;
        right_latest_current_ma =
            CurrentSensor_CountsToMilliamps(
                (int32_t)average_adc -
                (int32_t)right_zero_adc,
                RIGHT_CURRENT_SIGN
            );

        right_filtered_blocks++;
        right_latest_ready = true;
    }
}

static void CurrentSensor_UpdateCapture(
    CurrentSensorCaptureState *state,
    uint16_t raw_adc
)
{
    if (
        !state->active ||
        state->done
    )
    {
        return;
    }

    if (raw_adc < state->minimum_adc)
    {
        state->minimum_adc = raw_adc;
    }

    if (raw_adc > state->maximum_adc)
    {
        state->maximum_adc = raw_adc;
    }

    state->adc_total += raw_adc;
    state->sample_count++;

    if (state->sample_count >= state->target_count)
    {
        state->active = false;
        state->done = true;
    }
}

static int32_t CurrentSensor_CountsToMilliamps(
    int32_t adc_difference,
    int32_t direction_sign
)
{
    const int64_t numerator =
        (int64_t)adc_difference *
        CURRENT_SENSOR_REFERENCE_MV *
        1000LL *
        direction_sign;

    const int64_t denominator =
        CURRENT_SENSOR_ADC_MAX *
        CURRENT_SENSOR_SENSITIVITY_MV_A;

    return CurrentSensor_DivideSignedRounded(
        numerator,
        (uint32_t)denominator
    );
}

static int32_t CurrentSensor_DivideSignedRounded(
    int64_t numerator,
    uint32_t denominator
)
{
    if (denominator == 0U)
    {
        return 0;
    }

    if (numerator >= 0)
    {
        numerator +=
            (int64_t)denominator / 2LL;
    }
    else
    {
        numerator -=
            (int64_t)denominator / 2LL;
    }

    return (int32_t)(
        numerator /
        (int64_t)denominator
    );
}

void DMA1_Channel1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(
        &right_dma
    );
}

void DMA1_Channel2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(
        &left_dma
    );
}

void HAL_ADC_ConvHalfCpltCallback(
    ADC_HandleTypeDef *adc
)
{
    if (
        adc != NULL &&
        adc->Instance == ADC1
    )
    {
        CurrentSensor_ProcessSamples(
            CURRENT_SENSOR_RIGHT,
            right_dma_buffer,
            0U,
            CURRENT_SENSOR_DMA_HALF_SAMPLES
        );
    }
    else if (
        adc != NULL &&
        adc->Instance == ADC2
    )
    {
        CurrentSensor_ProcessSamples(
            CURRENT_SENSOR_LEFT,
            left_dma_buffer,
            0U,
            CURRENT_SENSOR_DMA_HALF_SAMPLES
        );
    }
}

void HAL_ADC_ConvCpltCallback(
    ADC_HandleTypeDef *adc
)
{
    if (
        adc != NULL &&
        adc->Instance == ADC1
    )
    {
        CurrentSensor_ProcessSamples(
            CURRENT_SENSOR_RIGHT,
            right_dma_buffer,
            CURRENT_SENSOR_DMA_HALF_SAMPLES,
            CURRENT_SENSOR_DMA_HALF_SAMPLES
        );
    }
    else if (
        adc != NULL &&
        adc->Instance == ADC2
    )
    {
        CurrentSensor_ProcessSamples(
            CURRENT_SENSOR_LEFT,
            left_dma_buffer,
            CURRENT_SENSOR_DMA_HALF_SAMPLES,
            CURRENT_SENSOR_DMA_HALF_SAMPLES
        );
    }
}
