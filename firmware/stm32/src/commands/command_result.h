#ifndef COMMAND_RESULT_H
#define COMMAND_RESULT_H

/*
 * Command handlers return this result after writing their
 * existing human-readable serial response.
 */
typedef enum
{
    COMMAND_RESULT_OK = 0,
    COMMAND_RESULT_ERROR
} CommandResult;

#endif
