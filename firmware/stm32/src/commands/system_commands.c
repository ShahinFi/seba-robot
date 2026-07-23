#include "system_commands.h"
#include "serial/serial.h"

#include "stm32g4xx_hal.h"

#include <string.h>

void SystemCommands_Handle(
    int argument_count,
    char *arguments[]
)
{
    if (
        argument_count != 2 ||
        strcmp(arguments[1], "reset") != 0
    )
    {
        Serial_WriteLine(
            "ERROR: usage: system reset"
        );

        return;
    }

    Serial_WriteLine(
        "OK: resetting STM32."
    );

    HAL_Delay(20U);
    NVIC_SystemReset();
}
