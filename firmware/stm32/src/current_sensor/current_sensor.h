#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    /*
     * raw_adc and zero_adc are 12-bit ADC counts. current_ma
     * is signed milliamps after zero-offset subtraction.
     */
    uint16_t raw_adc;
    uint16_t zero_adc;
    int32_t current_ma;
    bool fault_active;
} CurrentSensorReading;

/*
 * Initializes both ADC inputs and measures zero-current
 * offsets.
 *
 * The motors must already be stopped and disabled.
 */
bool CurrentSensor_Init(void);

/*
 * Returns false if reading is NULL or if the ADC conversion
 * sequence fails.
 */
bool CurrentSensor_ReadLeft(
    CurrentSensorReading *reading
);

/*
 * Returns false if reading is NULL or if the ADC conversion
 * sequence fails.
 */
bool CurrentSensor_ReadRight(
    CurrentSensorReading *reading
);

/*
 * Reads the left sensor first, then the right sensor.
 */
bool CurrentSensor_ReadBoth(
    CurrentSensorReading *left,
    CurrentSensorReading *right
);

#endif
