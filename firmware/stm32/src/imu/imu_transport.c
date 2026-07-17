#include "imu_transport.h"

#include "stm32g4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * BNO085 wiring:
 *
 * PC8  = I2C3_SCL
 * PC9  = I2C3_SDA
 * PB11 = active-low BNO085 INT
 *
 * BNO085 RST is not connected.
 */

#define BNO085_I2C_ADDRESS_7BIT    0x4AU
#define BNO085_I2C_ADDRESS_HAL     \
    (BNO085_I2C_ADDRESS_7BIT << 1U)

#define BNO085_I2C_TIMEOUT_MS      100U
#define BNO085_I2C_RETRY_COUNT       3U
#define BNO085_BOOT_TIMEOUT_MS    2000U
#define BNO085_READY_POLL_MS        10U

/*
 * I2C3 timing for:
 *
 * I2C kernel clock = 16 MHz
 * bus speed        = 400 kHz
 *
 * This value assumes the 16 MHz HSI system-clock configuration.
 */
#define BNO085_I2C_TIMING          0x0010061AU

#define BNO085_INT_PORT            GPIOB
#define BNO085_INT_PIN             GPIO_PIN_11

static I2C_HandleTypeDef imu_i2c;

static volatile bool imu_data_ready;
static volatile uint32_t imu_interrupt_timestamp_us;

static int IMU_Transport_Open(
    sh2_Hal_t *self
);

static void IMU_Transport_Close(
    sh2_Hal_t *self
);

static int IMU_Transport_Read(
    sh2_Hal_t *self,
    uint8_t *buffer,
    unsigned buffer_length,
    uint32_t *timestamp_us
);

static int IMU_Transport_Write(
    sh2_Hal_t *self,
    uint8_t *buffer,
    unsigned length
);

static uint32_t IMU_Transport_GetTimeUs(
    sh2_Hal_t *self
);

static void IMU_Transport_GPIO_Init(void);
static bool IMU_Transport_I2C_Init(void);
static void IMU_Transport_Timebase_Init(void);
static bool IMU_Transport_WaitForDeviceReady(void);

static bool IMU_Transport_I2CReceive(
    uint8_t *buffer,
    uint16_t length
);

static bool IMU_Transport_I2CTransmit(
    uint8_t *buffer,
    uint16_t length
);

static sh2_Hal_t imu_hal =
{
    .open = IMU_Transport_Open,
    .close = IMU_Transport_Close,
    .read = IMU_Transport_Read,
    .write = IMU_Transport_Write,
    .getTimeUs = IMU_Transport_GetTimeUs
};

sh2_Hal_t *IMU_Transport_GetHAL(void)
{
    return &imu_hal;
}

bool IMU_Transport_DataReady(void)
{
    /*
     * Also inspect the physical pin. This prevents a missed
     * interrupt edge from leaving unread data in the BNO085.
     */
    return
        imu_data_ready ||
        HAL_GPIO_ReadPin(
            BNO085_INT_PORT,
            BNO085_INT_PIN
        ) == GPIO_PIN_RESET;
}

void IMU_Transport_ClearDataReady(void)
{
    imu_data_ready = false;
}

static int IMU_Transport_Open(
    sh2_Hal_t *self
)
{
    (void)self;

    imu_data_ready = false;
    imu_interrupt_timestamp_us = 0U;

    IMU_Transport_Timebase_Init();
    IMU_Transport_GPIO_Init();

    if (!IMU_Transport_I2C_Init())
    {
        return -1;
    }

    if (!IMU_Transport_WaitForDeviceReady())
    {
        return -1;
    }

    return 0;
}

static void IMU_Transport_Close(
    sh2_Hal_t *self
)
{
    (void)self;

    HAL_NVIC_DisableIRQ(
        EXTI15_10_IRQn
    );

    HAL_I2C_DeInit(
        &imu_i2c
    );

    __HAL_RCC_I2C3_CLK_DISABLE();
}

static int IMU_Transport_Read(
    sh2_Hal_t *self,
    uint8_t *buffer,
    unsigned buffer_length,
    uint32_t *timestamp_us
)
{
    uint8_t header[4];
    uint16_t packet_length;

    (void)self;

    if (
        buffer == NULL ||
        timestamp_us == NULL ||
        buffer_length < 4U
    )
    {
        return 0;
    }

    if (!IMU_Transport_DataReady())
    {
        return 0;
    }

    /*
     * The CEVA reference implementation first reads the
     * complete four-byte SHTP header.
     */
    if (!IMU_Transport_I2CReceive(
            header,
            sizeof(header)
        ))
    {
        return 0;
    }

    packet_length =
        (uint16_t)header[0] |
        ((uint16_t)header[1] << 8U);

    /*
     * Bit 15 is the SHTP continuation flag, not part of
     * the actual packet length.
     */
    packet_length &= 0x7FFFU;

    if (
        packet_length < 4U ||
        packet_length > buffer_length
    )
    {
        return 0;
    }

    /*
     * Read the complete transfer, including its header.
     * This matches the CEVA Nucleo I2C transport behavior.
     */
    if (!IMU_Transport_I2CReceive(
            buffer,
            packet_length
        ))
    {
        return 0;
    }

    if (imu_interrupt_timestamp_us != 0U)
    {
        /*
         * Prefer the EXTI timestamp because it records when
         * the BNO085 signaled data ready.
         */
        *timestamp_us =
            imu_interrupt_timestamp_us;
    }
    else
    {
        *timestamp_us =
            IMU_Transport_GetTimeUs(self);
    }

    imu_data_ready = false;

    return (int)packet_length;
}

static int IMU_Transport_Write(
    sh2_Hal_t *self,
    uint8_t *buffer,
    unsigned length
)
{
    (void)self;

    if (
        buffer == NULL ||
        length == 0U ||
        length > UINT16_MAX
    )
    {
        return 0;
    }

    if (!IMU_Transport_I2CTransmit(
            buffer,
            (uint16_t)length
        ))
    {
        return 0;
    }

    return (int)length;
}

static uint32_t IMU_Transport_GetTimeUs(
    sh2_Hal_t *self
)
{
    (void)self;

    /*
     * TIM5 is configured as a free-running 1 MHz,
     * 32-bit timer.
     */
    return TIM5->CNT;
}

static void IMU_Transport_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    gpio.Pin =
        GPIO_PIN_8 |
        GPIO_PIN_9;

    gpio.Mode =
        GPIO_MODE_AF_OD;

    gpio.Pull =
        GPIO_NOPULL;

    gpio.Speed =
        GPIO_SPEED_FREQ_HIGH;

    gpio.Alternate =
        GPIO_AF8_I2C3;

    HAL_GPIO_Init(
        GPIOC,
        &gpio
    );

    /*
     * BNO085 INT is active low.
     *
     * Use a falling-edge EXTI interrupt. The breakout board
     * already provides the required signal conditioning.
     */
    gpio.Pin =
        BNO085_INT_PIN;

    gpio.Mode =
        GPIO_MODE_IT_FALLING;

    gpio.Pull =
        GPIO_PULLUP;

    gpio.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        BNO085_INT_PORT,
        &gpio
    );

    HAL_NVIC_SetPriority(
        EXTI15_10_IRQn,
        5U,
        0U
    );

    HAL_NVIC_EnableIRQ(
        EXTI15_10_IRQn
    );
}

static bool IMU_Transport_I2C_Init(void)
{
    RCC_PeriphCLKInitTypeDef peripheral_clock = {0};

    peripheral_clock.PeriphClockSelection =
        RCC_PERIPHCLK_I2C3;

    peripheral_clock.I2c3ClockSelection =
        RCC_I2C3CLKSOURCE_PCLK1;

    if (HAL_RCCEx_PeriphCLKConfig(
            &peripheral_clock
        ) != HAL_OK)
    {
        return false;
    }

    __HAL_RCC_I2C3_CLK_ENABLE();

    imu_i2c.Instance =
        I2C3;

    imu_i2c.Init.Timing =
        BNO085_I2C_TIMING;

    imu_i2c.Init.OwnAddress1 =
        0U;

    imu_i2c.Init.AddressingMode =
        I2C_ADDRESSINGMODE_7BIT;

    imu_i2c.Init.DualAddressMode =
        I2C_DUALADDRESS_DISABLE;

    imu_i2c.Init.OwnAddress2 =
        0U;

    imu_i2c.Init.OwnAddress2Masks =
        I2C_OA2_NOMASK;

    imu_i2c.Init.GeneralCallMode =
        I2C_GENERALCALL_DISABLE;

    imu_i2c.Init.NoStretchMode =
        I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(
            &imu_i2c
        ) != HAL_OK)
    {
        return false;
    }

    /*
     * The BNO085 uses clock stretching. It must remain
     * enabled, so NoStretchMode is disabled.
     */
    if (HAL_I2CEx_ConfigAnalogFilter(
            &imu_i2c,
            I2C_ANALOGFILTER_ENABLE
        ) != HAL_OK)
    {
        return false;
    }

    if (HAL_I2CEx_ConfigDigitalFilter(
            &imu_i2c,
            0U
        ) != HAL_OK)
    {
        return false;
    }

    return true;
}

static void IMU_Transport_Timebase_Init(void)
{
    /*
     * TIM5 is a 32-bit free-running microsecond counter.
     *
     * Existing timer clock: 16 MHz
     * Prescaler: 16 - 1
     * Result: 1 MHz
     */
    __HAL_RCC_TIM5_FORCE_RESET();
    __HAL_RCC_TIM5_RELEASE_RESET();
    __HAL_RCC_TIM5_CLK_ENABLE();

    TIM5->CR1 = 0U;
    TIM5->PSC = 15U;
    TIM5->ARR = 0xFFFFFFFFUL;
    TIM5->CNT = 0U;
    TIM5->EGR = TIM_EGR_UG;
    TIM5->CR1 = TIM_CR1_CEN;
}

static bool IMU_Transport_WaitForDeviceReady(void)
{
    const uint32_t start_ms =
        HAL_GetTick();

    /*
     * BNO085 power-up time can exceed the MCU startup time.
     * Poll for an I2C ACK and continue as soon as the device
     * responds.
     */
    while (
        HAL_GetTick() - start_ms <
        BNO085_BOOT_TIMEOUT_MS
    )
    {
        if (HAL_I2C_IsDeviceReady(
                &imu_i2c,
                BNO085_I2C_ADDRESS_HAL,
                1U,
                BNO085_I2C_TIMEOUT_MS
            ) == HAL_OK)
        {
            return true;
        }

        HAL_Delay(BNO085_READY_POLL_MS);
    }

    return false;
}

static bool IMU_Transport_I2CReceive(
    uint8_t *buffer,
    uint16_t length
)
{
    for (
        uint32_t attempt = 0U;
        attempt < BNO085_I2C_RETRY_COUNT;
        attempt++
    )
    {
        if (HAL_I2C_Master_Receive(
                &imu_i2c,
                BNO085_I2C_ADDRESS_HAL,
                buffer,
                length,
                BNO085_I2C_TIMEOUT_MS
            ) == HAL_OK)
        {
            return true;
        }

        /*
         * Clear recoverable peripheral error state before
         * retrying a NACK or transient bus failure.
         */
        __HAL_I2C_CLEAR_FLAG(
            &imu_i2c,
            I2C_FLAG_STOPF
        );

        HAL_Delay(1U);
    }

    return false;
}

static bool IMU_Transport_I2CTransmit(
    uint8_t *buffer,
    uint16_t length
)
{
    for (
        uint32_t attempt = 0U;
        attempt < BNO085_I2C_RETRY_COUNT;
        attempt++
    )
    {
        if (HAL_I2C_Master_Transmit(
                &imu_i2c,
                BNO085_I2C_ADDRESS_HAL,
                buffer,
                length,
                BNO085_I2C_TIMEOUT_MS
            ) == HAL_OK)
        {
            return true;
        }

        /*
         * Clear recoverable peripheral error state before
         * retrying a NACK or transient bus failure.
         */
        __HAL_I2C_CLEAR_FLAG(
            &imu_i2c,
            I2C_FLAG_STOPF
        );

        HAL_Delay(1U);
    }

    return false;
}

void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(
        BNO085_INT_PIN
    );
}

void HAL_GPIO_EXTI_Callback(
    uint16_t gpio_pin
)
{
    if (gpio_pin != BNO085_INT_PIN)
    {
        return;
    }

    imu_interrupt_timestamp_us =
        TIM5->CNT;

    imu_data_ready = true;
}
