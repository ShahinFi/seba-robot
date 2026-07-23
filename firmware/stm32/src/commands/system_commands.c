#include "system_commands.h"
#include "serial/serial.h"

#include "stm32g4xx_hal.h"

#include <stdbool.h>
#include <string.h>

static bool reset_requested;

CommandResult SystemCommands_Handle(
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

        return COMMAND_RESULT_ERROR;
    }

    Serial_WriteLine(
        "OK: resetting STM32."
    );

    reset_requested =
        true;

    return COMMAND_RESULT_OK;
}

void SystemCommands_ProcessResetRequest(void)
{
    if (!reset_requested)
    {
        return;
    }

    HAL_Delay(20U);
    NVIC_SystemReset();
}
