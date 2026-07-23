#include "console.h"

#include "communication/command_protocol.h"
#include "commands/command_dispatcher.h"
#include "serial/serial.h"

#include <stdint.h>

#define CONSOLE_LINE_LENGTH   160U

static char input_line[CONSOLE_LINE_LENGTH];
static uint32_t input_length;

static void Console_ProcessByte(uint8_t byte);
static void Console_ExecuteLine(
    char *line
);

void Console_Init(void)
{
    input_length = 0U;
    input_line[0] = '\0';
    CommandProtocol_Init();
}

void Console_Process(void)
{
    uint8_t byte;

    while (Serial_ReadByte(&byte))
    {
        Console_ProcessByte(byte);
    }
}

static void Console_ProcessByte(uint8_t byte)
{
    if (
        byte == '\r' ||
        byte == '\n'
    )
    {
        if (input_length == 0U)
        {
            return;
        }

        input_line[input_length] = '\0';

        Console_ExecuteLine(input_line);

        input_length = 0U;
        input_line[0] = '\0';

        return;
    }

    if (
        byte == '\b' ||
        byte == 127U
    )
    {
        if (input_length > 0U)
        {
            input_length--;

            input_line[input_length] = '\0';
        }

        return;
    }

    if (
        byte < 32U ||
        byte > 126U
    )
    {
        return;
    }

    if (
        input_length >=
        CONSOLE_LINE_LENGTH - 1U
    )
    {
        /*
         * Discard the partial command so the next line starts
         * from a known empty buffer.
         */
        input_length = 0U;
        input_line[0] = '\0';

        Serial_WriteLine(
            "ERROR: command is too long."
        );

        return;
    }

    input_line[input_length] =
        (char)byte;

    input_length++;

    input_line[input_length] = '\0';
}

static void Console_ExecuteLine(
    char *line
)
{
    if (CommandProtocol_TryExecuteLine(line))
    {
        return;
    }

    CommandDispatcher_ExecuteLine(line);
}
