#ifndef CURRENT_SENSOR_TEST_H
#define CURRENT_SENSOR_TEST_H

#include "current_sensor/current_sensor_diagnostic.h"

#include <stdbool.h>

/*
 * Manual bring-up check for current sensor ADC readings,
 * zero offsets, converted current, and fault state.
 */
bool CurrentSensorTest_PrintReadings(void);

/*
 * Bring-up diagnostic for raw timed ADC samples while PWM is
 * active. Production control uses CurrentSensor_ReadBoth().
 */
bool CurrentSensorTest_PrintCapture(
    CurrentSensorChannel channel,
    uint32_t sample_count
);

#endif
