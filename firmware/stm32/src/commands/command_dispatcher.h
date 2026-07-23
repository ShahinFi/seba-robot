#ifndef COMMAND_DISPATCHER_H
#define COMMAND_DISPATCHER_H

#include "command_result.h"

/*
 * Executes one complete command line.
 *
 * The input buffer is tokenized in place.
 */
CommandResult CommandDispatcher_ExecuteLine(
    char *line
);

#endif
