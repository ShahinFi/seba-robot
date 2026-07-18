#ifndef CURRENT_SENSOR_DIAGNOSTIC_H
#define CURRENT_SENSOR_DIAGNOSTIC_H

#include "current_sensor.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
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

bool CurrentSensor_DiagnosticCapture(
    CurrentSensorChannel channel,
    uint32_t sample_count,
    CurrentSensorDiagnosticCapture *capture
);

#endif
