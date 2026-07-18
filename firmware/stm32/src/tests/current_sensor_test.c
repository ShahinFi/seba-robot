#include "current_sensor_test.h"

#include "serial/serial.h"

static void CurrentSensorTest_PrintReading(
    const char *label,
    const CurrentSensorReading *reading
);

static void CurrentSensorTest_PrintCaptureResult(
    const char *label,
    const CurrentSensorDiagnosticCapture *capture
);

bool CurrentSensorTest_PrintReadings(void)
{
    CurrentSensorReading left;
    CurrentSensorReading right;

    if (!CurrentSensor_ReadBoth(
            &left,
            &right
        ))
    {
        return false;
    }

    CurrentSensorTest_PrintReading(
        "Left current sensor:",
        &left
    );

    Serial_WriteLine("");

    CurrentSensorTest_PrintReading(
        "Right current sensor:",
        &right
    );

    return true;
}

bool CurrentSensorTest_PrintCapture(
    CurrentSensorChannel channel,
    uint32_t sample_count
)
{
    CurrentSensorDiagnosticCapture capture;

    if (!CurrentSensor_DiagnosticCapture(
            channel,
            sample_count,
            &capture
        ))
    {
        return false;
    }

    CurrentSensorTest_PrintCaptureResult(
        channel == CURRENT_SENSOR_LEFT
            ? "Left current sensor capture:"
            : "Right current sensor capture:",
        &capture
    );

    return true;
}

static void CurrentSensorTest_PrintReading(
    const char *label,
    const CurrentSensorReading *reading
)
{
    Serial_WriteLine(label);

    Serial_Write("  Raw ADC: ");

    Serial_WriteUInt32(
        reading->raw_adc
    );

    Serial_WriteLine("");

    Serial_Write("  Zero ADC: ");

    Serial_WriteUInt32(
        reading->zero_adc
    );

    Serial_WriteLine("");

    Serial_Write("  Current: ");

    Serial_WriteMilliamps(
        reading->current_ma
    );

    Serial_WriteLine("");

    Serial_Write("  Fault: ");

    Serial_WriteLine(
        reading->fault_active
            ? "ACTIVE"
            : "normal"
    );
}

static void CurrentSensorTest_PrintCaptureResult(
    const char *label,
    const CurrentSensorDiagnosticCapture *capture
)
{
    Serial_WriteLine(label);

    Serial_Write("  Samples: ");
    Serial_WriteUInt32(capture->sample_count);
    Serial_WriteLine("");

    Serial_Write("  Zero ADC: ");
    Serial_WriteUInt32(capture->zero_adc);
    Serial_WriteLine("");

    Serial_Write("  Raw ADC min/avg/max: ");
    Serial_WriteUInt32(capture->minimum_adc);
    Serial_Write(" / ");
    Serial_WriteUInt32(capture->average_adc);
    Serial_Write(" / ");
    Serial_WriteUInt32(capture->maximum_adc);
    Serial_WriteLine("");

    Serial_Write("  Current min/avg/max: ");
    Serial_WriteMilliamps(capture->minimum_current_ma);
    Serial_Write(" / ");
    Serial_WriteMilliamps(capture->average_current_ma);
    Serial_Write(" / ");
    Serial_WriteMilliamps(capture->maximum_current_ma);
    Serial_WriteLine("");

    Serial_Write("  Fault: ");
    Serial_WriteLine(
        capture->fault_active
            ? "ACTIVE"
            : "normal"
    );
}
