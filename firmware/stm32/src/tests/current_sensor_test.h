#ifndef CURRENT_SENSOR_TEST_H
#define CURRENT_SENSOR_TEST_H

#include <stdbool.h>

/*
 * Manual bring-up check for current sensor ADC readings,
 * zero offsets, converted current, and fault state.
 */
bool CurrentSensorTest_PrintReadings(void);

#endif
