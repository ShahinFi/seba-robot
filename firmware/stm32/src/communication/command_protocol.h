#ifndef COMMAND_PROTOCOL_H
#define COMMAND_PROTOCOL_H

#include <stdbool.h>

/*
 * Clears the recent command ID history used by framed Pi
 * commands.
 */
void CommandProtocol_Init(void);

/*
 * Executes a framed command line when the line starts with
 * "CMD <id> ". Returns true when the line belongs to the
 * framed protocol, even if the frame is malformed.
 *
 * The input buffer is modified in place.
 */
bool CommandProtocol_TryExecuteLine(
    char *line
);

#endif
