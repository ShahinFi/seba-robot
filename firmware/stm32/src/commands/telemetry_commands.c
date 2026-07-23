#include "telemetry_commands.h"
#include "communication/telemetry_stream.h"
#include "serial/serial.h"

#include <string.h>

void TelemetryCommands_Handle(
    int argument_count,
    char *arguments[]
)
{
    if (
        argument_count != 2 ||
        strcmp(arguments[1], "read") != 0
    )
    {
        Serial_WriteLine(
            "ERROR: usage: telemetry read"
        );

        return;
    }

    TelemetryStream_WriteSnapshot();
}
