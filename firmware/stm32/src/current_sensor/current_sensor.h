#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    /*
     * raw_adc is the latest filtered ADC block average.
     * zero_adc is the measured zero-current offset.
     * current_ma is signed milliamps after offset correction.
     */
    uint16_t raw_adc;
    uint16_t zero_adc;
    int32_t current_ma;
    bool fault_active;
} CurrentSensorReading;

typedef enum
{
    CURRENT_SENSOR_LEFT,
    CURRENT_SENSOR_RIGHT
} CurrentSensorChannel;

/*
 * Initializes both ADC inputs, measures zero-current offsets,
 * and starts timer-triggered current sampling.
 *
 * The motors must already be stopped and disabled.
 */
bool CurrentSensor_Init(void);

bool CurrentSensor_ReadLeft(
    CurrentSensorReading *reading
);

bool CurrentSensor_ReadRight(
    CurrentSensorReading *reading
);

/*
 * Returns the latest filtered readings for both sensors. The
 * readings are produced by the timer-triggered DMA sample path.
 */
bool CurrentSensor_ReadBoth(
    CurrentSensorReading *left,
    CurrentSensorReading *right
);

#endif
