#ifndef CURRENT_SENSOR_DIAGNOSTIC_H
#define CURRENT_SENSOR_DIAGNOSTIC_H

#include "current_sensor.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    /*
     * ADC values are raw 12-bit counts. Current values are
     * offset-corrected signed milliamps.
     */
    uint32_t sample_count;
    uint16_t zero_adc;
    uint16_t minimum_adc;
    uint16_t maximum_adc;
    uint16_t average_adc;
    int32_t minimum_current_ma;
    int32_t maximum_current_ma;
    int32_t average_current_ma;
    bool fault_active;
} CurrentSensorDiagnosticCapture;

/*
 * Captures raw timer-triggered samples for one channel. Intended
 * for bring-up diagnostics while production control is stopped.
 */
bool CurrentSensor_DiagnosticCapture(
    CurrentSensorChannel channel,
    uint32_t sample_count,
    CurrentSensorDiagnosticCapture *capture
);

#endif
