#ifndef SERIAL_H
#define SERIAL_H

#include <stdbool.h>
#include <stdint.h>

bool Serial_Init(void);

/*
 * Nonblocking read from the interrupt-driven RX buffer.
 * Returns false when byte is NULL or no byte is available.
 */
bool Serial_ReadByte(
    uint8_t *byte
);

/*
 * Blocking diagnostic transmit.
 */
void Serial_Write(
    const char *text
);

void Serial_WriteLine(
    const char *text
);

void Serial_WriteUInt32(
    uint32_t value
);

void Serial_WriteInt32(
    int32_t value
);

void Serial_WriteInt64(
    int64_t value
);

void Serial_WriteMilliamps(
    int32_t current_ma
);

/*
 * Writes a signed decimal value with three fractional digits.
 */
void Serial_WriteFloat3(
    float value
);

#endif
