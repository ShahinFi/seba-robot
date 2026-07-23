#ifndef COMMAND_DISPATCHER_H
#define COMMAND_DISPATCHER_H

/*
 * Executes one complete command line.
 *
 * The input buffer is tokenized in place.
 */
void CommandDispatcher_ExecuteLine(
    char *line
);

#endif
