#ifndef COMMAND_PROTOCOL_H
#define COMMAND_PROTOCOL_H

#include <stdbool.h>

void CommandProtocol_Init(void);

bool CommandProtocol_TryExecuteLine(
    char *line
);

#endif
