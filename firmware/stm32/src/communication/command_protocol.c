#include "command_protocol.h"

#include "commands/command_dispatcher.h"
#include "serial/serial.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define COMMAND_PROTOCOL_RECENT_IDS  16U

static uint32_t recent_command_ids[COMMAND_PROTOCOL_RECENT_IDS];
static uint32_t recent_command_id_index;

static bool CommandProtocol_ParseId(
    const char *text,
    uint32_t *command_id
);

static bool CommandProtocol_IdWasSeen(
    uint32_t command_id
);

static void CommandProtocol_RememberId(
    uint32_t command_id
);

void CommandProtocol_Init(void)
{
    memset(
        recent_command_ids,
        0,
        sizeof(recent_command_ids)
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

    Serial_Write("ACK ");
    Serial_WriteUInt32(command_id);
    Serial_WriteLine(" OK");

    if (CommandProtocol_IdWasSeen(command_id))
    {
        return true;
    }

    CommandProtocol_RememberId(command_id);
    CommandDispatcher_ExecuteLine(command);

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

static bool CommandProtocol_IdWasSeen(
    uint32_t command_id
)
{
    for (
        uint32_t index = 0U;
        index < COMMAND_PROTOCOL_RECENT_IDS;
        index++
    )
    {
        if (recent_command_ids[index] == command_id)
        {
            return true;
        }
    }

    return false;
}

static void CommandProtocol_RememberId(
    uint32_t command_id
)
{
    recent_command_ids[recent_command_id_index] =
        command_id;

    recent_command_id_index++;

    if (recent_command_id_index >= COMMAND_PROTOCOL_RECENT_IDS)
    {
        recent_command_id_index = 0U;
    }
}
