#include "current_sensor_test.h"

#include "current_sensor/current_sensor.h"
#include "serial/serial.h"

static void CurrentSensorTest_PrintReading(
    const char *label,
    const CurrentSensorReading *reading
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
