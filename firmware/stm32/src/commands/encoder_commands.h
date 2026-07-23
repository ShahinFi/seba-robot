#ifndef ENCODER_COMMANDS_H
#define ENCODER_COMMANDS_H

#include "command_result.h"

/*
 * Handles the encoder bring-up command group.
 *
 * arguments[0] must be "encoder".
 */
CommandResult EncoderCommands_Handle(
    int argument_count,
    char *arguments[]
);

#endif
