#include "serial.h"

#include "stm32g4xx_hal.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/*
 * ST-LINK Virtual COM Port:
 *
 * PA2 = LPUART1_TX
 * PA3 = LPUART1_RX
 * Alternate function 12
 */
#define SERIAL_TX_PORT              GPIOA
#define SERIAL_TX_PIN               GPIO_PIN_2

#define SERIAL_RX_PORT              GPIOA
#define SERIAL_RX_PIN               GPIO_PIN_3

#define SERIAL_BAUD_RATE            115200U
#define SERIAL_TX_TIMEOUT_MS        100U

/*
 * Power-of-two buffer size allows uint8_t indexes to wrap
 * naturally.
 *
 * The buffer holds at most 255 unread bytes because one
 * position is reserved to distinguish full from empty.
 */
#define SERIAL_RX_BUFFER_SIZE       256U

static UART_HandleTypeDef serial_uart;

/*
 * HAL receives one byte at a time and stores it here before
 * invoking HAL_UART_RxCpltCallback().
 */
static uint8_t serial_received_byte;

/*
 * Ring buffer shared between:
 *
 * interrupt context:
 *     writes bytes and advances head
 *
 * main-loop context:
 *     reads bytes and advances tail
 */
static volatile uint8_t serial_rx_buffer[
    SERIAL_RX_BUFFER_SIZE
];

static volatile uint8_t serial_rx_head;
static volatile uint8_t serial_rx_tail;

static volatile bool serial_rx_overflow;

static bool Serial_GPIOInit(void);
static bool Serial_UARTInit(void);
static bool Serial_StartReception(void);

static void Serial_WriteUnsigned64(
    uint64_t value
);

bool Serial_Init(void)
{
    serial_rx_head = 0U;
    serial_rx_tail = 0U;
    serial_rx_overflow = false;
    serial_received_byte = 0U;

    if (!Serial_GPIOInit())
    {
        return false;
    }

    if (!Serial_UARTInit())
    {
        return false;
    }

    if (!Serial_StartReception())
    {
        return false;
    }

    return true;
}

bool Serial_ReadByte(
    uint8_t *byte
)
{
    uint8_t tail;

    if (byte == NULL)
    {
        return false;
    }

    tail = serial_rx_tail;

    if (tail == serial_rx_head)
    {
        return false;
    }

    *byte = serial_rx_buffer[tail];

    /*
     * uint8_t overflow wraps 255 back to 0, matching the
     * 256-byte ring-buffer size.
     */
    serial_rx_tail =
        (uint8_t)(tail + 1U);

    return true;
}

void Serial_Write(
    const char *text
)
{
    size_t length;

    if (text == NULL)
    {
        return;
    }

    length = strlen(text);

    if (length == 0U)
    {
        return;
    }

    /*
     * Transmit remains blocking because console output is
     * diagnostic and not part of the balancing control path.
     *
     * Reception is interrupt-driven, so incoming characters
     * are still buffered while this function is transmitting.
     */
    (void)HAL_UART_Transmit(
        &serial_uart,
        (const uint8_t *)text,
        (uint16_t)length,
        SERIAL_TX_TIMEOUT_MS
    );
}

void Serial_WriteLine(
    const char *text
)
{
    Serial_Write(text);
    Serial_Write("\r\n");
}

void Serial_WriteUInt32(
    uint32_t value
)
{
    Serial_WriteUnsigned64(
        (uint64_t)value
    );
}

void Serial_WriteInt32(
    int32_t value
)
{
    uint32_t magnitude;

    if (value < 0)
    {
        Serial_Write("-");

        /*
         * This form also works for INT32_MIN without signed
         * overflow.
         */
        magnitude =
            (uint32_t)(-(value + 1)) + 1U;
    }
    else
    {
        magnitude =
            (uint32_t)value;
    }

    Serial_WriteUInt32(magnitude);
}

void Serial_WriteInt64(
    int64_t value
)
{
    uint64_t magnitude;

    if (value < 0)
    {
        Serial_Write("-");

        /*
         * This form also works for INT64_MIN.
         */
        magnitude =
            (uint64_t)(-(value + 1)) + 1ULL;
    }
    else
    {
        magnitude =
            (uint64_t)value;
    }

    Serial_WriteUnsigned64(magnitude);
}

void Serial_WriteMilliamps(
    int32_t current_ma
)
{
    Serial_WriteInt32(current_ma);
    Serial_Write(" mA");
}

void Serial_WriteFloat3(
    float value
)
{
    int32_t scaled;
    uint32_t magnitude;
    uint32_t whole;
    uint32_t fraction;

    /*
     * Convert to a signed integer containing three decimal
     * places, with rounding.
     */
    if (value >= 0.0F)
    {
        scaled =
            (int32_t)(
                value * 1000.0F +
                0.5F
            );
    }
    else
    {
        scaled =
            (int32_t)(
                value * 1000.0F -
                0.5F
            );
    }

    if (scaled < 0)
    {
        Serial_Write("-");

        magnitude =
            (uint32_t)(-(scaled + 1)) + 1U;
    }
    else
    {
        magnitude =
            (uint32_t)scaled;
    }

    whole =
        magnitude / 1000U;

    fraction =
        magnitude % 1000U;

    Serial_WriteUInt32(whole);
    Serial_Write(".");

    if (fraction < 100U)
    {
        Serial_Write("0");
    }

    if (fraction < 10U)
    {
        Serial_Write("0");
    }

    Serial_WriteUInt32(fraction);
}

static bool Serial_GPIOInit(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin =
        SERIAL_TX_PIN |
        SERIAL_RX_PIN;

    gpio.Mode =
        GPIO_MODE_AF_PP;

    gpio.Pull =
        GPIO_NOPULL;

    gpio.Speed =
        GPIO_SPEED_FREQ_HIGH;

    gpio.Alternate =
        GPIO_AF12_LPUART1;

    HAL_GPIO_Init(
        GPIOA,
        &gpio
    );

    return true;
}

static bool Serial_UARTInit(void)
{
    RCC_PeriphCLKInitTypeDef peripheral_clock = {0};

    peripheral_clock.PeriphClockSelection =
        RCC_PERIPHCLK_LPUART1;

    peripheral_clock.Lpuart1ClockSelection =
        RCC_LPUART1CLKSOURCE_PCLK1;

    if (HAL_RCCEx_PeriphCLKConfig(
            &peripheral_clock
        ) != HAL_OK)
    {
        return false;
    }

    __HAL_RCC_LPUART1_CLK_ENABLE();

    serial_uart.Instance =
        LPUART1;

    serial_uart.Init.BaudRate =
        SERIAL_BAUD_RATE;

    serial_uart.Init.WordLength =
        UART_WORDLENGTH_8B;

    serial_uart.Init.StopBits =
        UART_STOPBITS_1;

    serial_uart.Init.Parity =
        UART_PARITY_NONE;

    serial_uart.Init.Mode =
        UART_MODE_TX_RX;

    serial_uart.Init.HwFlowCtl =
        UART_HWCONTROL_NONE;

    serial_uart.Init.OneBitSampling =
        UART_ONE_BIT_SAMPLE_DISABLE;

    serial_uart.Init.ClockPrescaler =
        UART_PRESCALER_DIV1;

    serial_uart.AdvancedInit.AdvFeatureInit =
        UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(
            &serial_uart
        ) != HAL_OK)
    {
        return false;
    }

    if (HAL_UARTEx_SetTxFifoThreshold(
            &serial_uart,
            UART_TXFIFO_THRESHOLD_1_8
        ) != HAL_OK)
    {
        return false;
    }

    if (HAL_UARTEx_SetRxFifoThreshold(
            &serial_uart,
            UART_RXFIFO_THRESHOLD_1_8
        ) != HAL_OK)
    {
        return false;
    }

    if (HAL_UARTEx_DisableFifoMode(
            &serial_uart
        ) != HAL_OK)
    {
        return false;
    }

    /*
     * Enable the LPUART1 interrupt after the peripheral has
     * been completely configured.
     */
    HAL_NVIC_SetPriority(
        LPUART1_IRQn,
        6U,
        0U
    );

    HAL_NVIC_EnableIRQ(
        LPUART1_IRQn
    );

    return true;
}

static bool Serial_StartReception(void)
{
    return HAL_UART_Receive_IT(
               &serial_uart,
               &serial_received_byte,
               1U
           ) == HAL_OK;
}

static void Serial_WriteUnsigned64(
    uint64_t value
)
{
    char buffer[21];
    uint32_t index = 0U;

    if (value == 0ULL)
    {
        Serial_Write("0");
        return;
    }

    while (
        value > 0ULL &&
        index < sizeof(buffer) - 1U
    )
    {
        buffer[index] =
            (char)(
                '0' +
                (value % 10ULL)
            );

        value /= 10ULL;
        index++;
    }

    /*
     * Digits were generated in reverse order.
     */
    for (
        uint32_t left = 0U,
                 right = index - 1U;
        left < right;
        left++,
        right--
    )
    {
        const char temporary =
            buffer[left];

        buffer[left] =
            buffer[right];

        buffer[right] =
            temporary;
    }

    buffer[index] = '\0';

    Serial_Write(buffer);
}

/*
 * STM32 interrupt entry point.
 */
void LPUART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(
        &serial_uart
    );
}

/*
 * Called by HAL after one byte has been received.
 */
void HAL_UART_RxCpltCallback(
    UART_HandleTypeDef *uart
)
{
    uint8_t head;
    uint8_t next_head;

    if (
        uart == NULL ||
        uart->Instance != LPUART1
    )
    {
        return;
    }

    head =
        serial_rx_head;

    next_head =
        (uint8_t)(head + 1U);

    if (next_head != serial_rx_tail)
    {
        serial_rx_buffer[head] =
            serial_received_byte;

        serial_rx_head =
            next_head;
    }
    else
    {
        /*
         * Preserve unread data when the software buffer is full.
         */
        serial_rx_overflow = true;
    }

    /*
     * Arm reception for the next byte before returning to HAL.
     */
    (void)HAL_UART_Receive_IT(
        &serial_uart,
        &serial_received_byte,
        1U
    );
}

/*
 * Recover from UART overrun, framing, or noise errors.
 *
 * Without this recovery, an overrun can leave interrupt-driven
 * reception stopped.
 */
void HAL_UART_ErrorCallback(
    UART_HandleTypeDef *uart
)
{
    if (
        uart == NULL ||
        uart->Instance != LPUART1
    )
    {
        return;
    }

    __HAL_UART_CLEAR_OREFLAG(uart);
    __HAL_UART_CLEAR_FEFLAG(uart);
    __HAL_UART_CLEAR_NEFLAG(uart);
    __HAL_UART_CLEAR_PEFLAG(uart);

    /*
     * Abort only the failed receive operation and then restart
     * one-byte interrupt reception.
     */
    (void)HAL_UART_AbortReceive(uart);

    (void)HAL_UART_Receive_IT(
        uart,
        &serial_received_byte,
        1U
    );
}
