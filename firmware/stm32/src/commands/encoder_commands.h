#ifndef ENCODER_COMMANDS_H
#define ENCODER_COMMANDS_H

/*
 * Handles the encoder bring-up command group.
 *
 * arguments[0] must be "encoder".
 */
void EncoderCommands_Handle(
    int argument_count,
    char *arguments[]
);

#endif
