#include "current_sensor.h"

#include "stm32g4xx_hal.h"

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

#define CURRENT_SENSOR_READ_SAMPLES       64U
#define CURRENT_SENSOR_ZERO_SAMPLES      256U
#define CURRENT_SENSOR_ADC_TIMEOUT_MS     10U
#define CURRENT_SENSOR_SETTLE_TIME_MS     20U

/*
 * Sign constants define the positive motor-current convention.
 */
#define LEFT_CURRENT_SIGN   -1L
#define RIGHT_CURRENT_SIGN  -1L

#define LEFT_FAULT_PORT   GPIOC
#define LEFT_FAULT_PIN    GPIO_PIN_2

#define RIGHT_FAULT_PORT  GPIOC
#define RIGHT_FAULT_PIN   GPIO_PIN_3

static ADC_HandleTypeDef left_adc;
static ADC_HandleTypeDef right_adc;

static uint16_t left_zero_adc;
static uint16_t right_zero_adc;

static void CurrentSensor_GPIO_Init(void);

static bool CurrentSensor_ADC1_Init(void);
static bool CurrentSensor_ADC2_Init(void);

static bool CurrentSensor_ReadAverage(
    ADC_HandleTypeDef *adc,
    uint32_t number_of_samples,
    uint16_t *average
);

static int32_t CurrentSensor_CountsToMilliamps(
    int32_t adc_difference,
    int32_t direction_sign
);

bool CurrentSensor_Init(void)
{
    CurrentSensor_GPIO_Init();

    __HAL_RCC_ADC12_CLK_ENABLE();

    if (!CurrentSensor_ADC1_Init())
    {
        return false;
    }

    if (!CurrentSensor_ADC2_Init())
    {
        return false;
    }

    /*
     * Allow the sensors and ADC inputs to settle before
     * measuring their zero-current offsets.
     */
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

    return true;
}

bool CurrentSensor_ReadLeft(
    CurrentSensorReading *reading
)
{
    uint16_t raw_adc;

    if (reading == NULL)
    {
        return false;
    }

    if (!CurrentSensor_ReadAverage(
            &left_adc,
            CURRENT_SENSOR_READ_SAMPLES,
            &raw_adc
        ))
    {
        return false;
    }

    reading->raw_adc = raw_adc;
    reading->zero_adc = left_zero_adc;

    reading->current_ma =
        CurrentSensor_CountsToMilliamps(
            (int32_t)raw_adc -
            (int32_t)left_zero_adc,
            LEFT_CURRENT_SIGN
        );

    reading->fault_active =
        HAL_GPIO_ReadPin(
            LEFT_FAULT_PORT,
            LEFT_FAULT_PIN
        ) == GPIO_PIN_RESET;

    return true;
}

bool CurrentSensor_ReadRight(
    CurrentSensorReading *reading
)
{
    uint16_t raw_adc;

    if (reading == NULL)
    {
        return false;
    }

    if (!CurrentSensor_ReadAverage(
            &right_adc,
            CURRENT_SENSOR_READ_SAMPLES,
            &raw_adc
        ))
    {
        return false;
    }

    reading->raw_adc = raw_adc;
    reading->zero_adc = right_zero_adc;

    reading->current_ma =
        CurrentSensor_CountsToMilliamps(
            (int32_t)raw_adc -
            (int32_t)right_zero_adc,
            RIGHT_CURRENT_SIGN
        );

    reading->fault_active =
        HAL_GPIO_ReadPin(
            RIGHT_FAULT_PORT,
            RIGHT_FAULT_PIN
        ) == GPIO_PIN_RESET;

    return true;
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

static bool CurrentSensor_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef channel = {0};

    /*
     * ADC1 reads the right sensor on PB0 / ADC1_IN15.
     */
    right_adc.Instance = ADC1;

    right_adc.Init.ClockPrescaler =
        ADC_CLOCK_SYNC_PCLK_DIV4;

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
        ADC_SOFTWARE_START;

    right_adc.Init.ExternalTrigConvEdge =
        ADC_EXTERNALTRIGCONVEDGE_NONE;

    right_adc.Init.DMAContinuousRequests =
        DISABLE;

    right_adc.Init.Overrun =
        ADC_OVR_DATA_OVERWRITTEN;

    right_adc.Init.OversamplingMode =
        DISABLE;

    if (HAL_ADC_Init(&right_adc) != HAL_OK)
    {
        return false;
    }

    if (HAL_ADCEx_Calibration_Start(
            &right_adc,
            ADC_SINGLE_ENDED
        ) != HAL_OK)
    {
        return false;
    }

    channel.Channel =
        ADC_CHANNEL_15;

    channel.Rank =
        ADC_REGULAR_RANK_1;

    channel.SamplingTime =
        ADC_SAMPLETIME_247CYCLES_5;

    channel.SingleDiff =
        ADC_SINGLE_ENDED;

    channel.OffsetNumber =
        ADC_OFFSET_NONE;

    channel.Offset = 0;

    if (HAL_ADC_ConfigChannel(
            &right_adc,
            &channel
        ) != HAL_OK)
    {
        return false;
    }

    return true;
}

static bool CurrentSensor_ADC2_Init(void)
{
    ADC_ChannelConfTypeDef channel = {0};

    /*
     * ADC2 reads the left sensor on PA4 / ADC2_IN17.
     */
    left_adc.Instance = ADC2;

    left_adc.Init.ClockPrescaler =
        ADC_CLOCK_SYNC_PCLK_DIV4;

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
        ADC_SOFTWARE_START;

    left_adc.Init.ExternalTrigConvEdge =
        ADC_EXTERNALTRIGCONVEDGE_NONE;

    left_adc.Init.DMAContinuousRequests =
        DISABLE;

    left_adc.Init.Overrun =
        ADC_OVR_DATA_OVERWRITTEN;

    left_adc.Init.OversamplingMode =
        DISABLE;

    if (HAL_ADC_Init(&left_adc) != HAL_OK)
    {
        return false;
    }

    if (HAL_ADCEx_Calibration_Start(
            &left_adc,
            ADC_SINGLE_ENDED
        ) != HAL_OK)
    {
        return false;
    }

    channel.Channel =
        ADC_CHANNEL_17;

    channel.Rank =
        ADC_REGULAR_RANK_1;

    channel.SamplingTime =
        ADC_SAMPLETIME_247CYCLES_5;

    channel.SingleDiff =
        ADC_SINGLE_ENDED;

    channel.OffsetNumber =
        ADC_OFFSET_NONE;

    channel.Offset = 0;

    if (HAL_ADC_ConfigChannel(
            &left_adc,
            &channel
        ) != HAL_OK)
    {
        return false;
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

        total += HAL_ADC_GetValue(adc);

        if (HAL_ADC_Stop(adc) != HAL_OK)
        {
            return false;
        }
    }

    /*
     * Rounded integer average.
     */
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

static int32_t CurrentSensor_CountsToMilliamps(
    int32_t adc_difference,
    int32_t direction_sign
)
{
    /*
     * current_mA =
     *
     * ADC difference
     * * 3300 mV
     * * 1000 mA/A
     * / 4095 counts
     * / 90 mV/A
     */
    int64_t numerator =
        (int64_t)adc_difference *
        CURRENT_SENSOR_REFERENCE_MV *
        1000LL *
        direction_sign;

    const int64_t denominator =
        CURRENT_SENSOR_ADC_MAX *
        CURRENT_SENSOR_SENSITIVITY_MV_A;

    /*
     * Symmetric rounding for positive and negative values.
     */
    if (numerator >= 0)
    {
        numerator += denominator / 2LL;
    }
    else
    {
        numerator -= denominator / 2LL;
    }

    return (int32_t)(numerator / denominator);
}
