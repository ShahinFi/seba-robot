#include "command_protocol.h"

#include "commands/command_dispatcher.h"
#include "commands/command_result.h"
#include "serial/serial.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define COMMAND_PROTOCOL_RECENT_IDS  16U

/*
 * Pi commands use:
 *
 * CMD <id> <command text>
 *
 * The recent-command table makes repeated command IDs
 * idempotent when the Pi retries after a serial timeout.
 */
typedef struct
{
    uint32_t command_id;
    CommandResult result;
} CommandProtocolRecord;

static CommandProtocolRecord recent_commands[COMMAND_PROTOCOL_RECENT_IDS];
static uint32_t recent_command_id_index;

static bool CommandProtocol_ParseId(
    const char *text,
    uint32_t *command_id
);

static bool CommandProtocol_FindResult(
    uint32_t command_id,
    CommandResult *result
);

static void CommandProtocol_RememberId(
    uint32_t command_id,
    CommandResult result
);

static void CommandProtocol_WriteAck(
    uint32_t command_id,
    CommandResult result
);

void CommandProtocol_Init(void)
{
    memset(
        recent_commands,
        0,
        sizeof(recent_commands)
    );

    recent_command_id_index = 0U;
}

bool CommandProtocol_TryExecuteLine(
    char *line
)
{
    char *id_text;
    char *command;
    uint32_t command_id;
    CommandResult result;

    if (strncmp(line, "CMD ", 4U) != 0)
    {
        return false;
    }

    id_text =
        line + 4U;

    command =
        strchr(id_text, ' ');

    if (command == NULL)
    {
        Serial_WriteLine(
            "ACK 0 ERROR"
        );

        return true;
    }

    *command = '\0';
    command++;

    if (
        !CommandProtocol_ParseId(
            id_text,
            &command_id
        ) ||
        command[0] == '\0'
    )
    {
        Serial_WriteLine(
            "ACK 0 ERROR"
        );

        return true;
    }

    if (CommandProtocol_FindResult(
            command_id,
            &result
        ))
    {
        /*
         * Return the original ACK without executing a repeated
         * command again.
         */
        CommandProtocol_WriteAck(
            command_id,
            result
        );

        return true;
    }

    result =
        CommandDispatcher_ExecuteLine(command);

    CommandProtocol_RememberId(
        command_id,
        result
    );

    CommandProtocol_WriteAck(
        command_id,
        result
    );

    return true;
}

static bool CommandProtocol_ParseId(
    const char *text,
    uint32_t *command_id
)
{
    char *end;
    unsigned long parsed;

    if (
        text == NULL ||
        command_id == NULL ||
        text[0] == '\0'
    )
    {
        return false;
    }

    parsed =
        strtoul(
            text,
            &end,
            10
        );

    if (
        end == text ||
        *end != '\0' ||
        parsed == 0UL ||
        parsed > 2147483647UL
    )
    {
        return false;
    }

    *command_id =
        (uint32_t)parsed;

    return true;
}

static bool CommandProtocol_FindResult(
    uint32_t command_id,
    CommandResult *result
)
{
    for (
        uint32_t index = 0U;
        index < COMMAND_PROTOCOL_RECENT_IDS;
        index++
    )
    {
        if (recent_commands[index].command_id == command_id)
        {
            if (result != NULL)
            {
                *result =
                    recent_commands[index].result;
            }

            return true;
        }
    }

    return false;
}

static void CommandProtocol_RememberId(
    uint32_t command_id,
    CommandResult result
)
{
    recent_commands[recent_command_id_index].command_id =
        command_id;

    recent_commands[recent_command_id_index].result =
        result;

    recent_command_id_index++;

    if (recent_command_id_index >= COMMAND_PROTOCOL_RECENT_IDS)
    {
        recent_command_id_index = 0U;
    }
}

static void CommandProtocol_WriteAck(
    uint32_t command_id,
    CommandResult result
)
{
    Serial_Write("ACK ");
    Serial_WriteUInt32(command_id);

    if (result == COMMAND_RESULT_OK)
    {
        Serial_WriteLine(" OK");
    }
    else
    {
        Serial_WriteLine(" ERROR");
    }
}
