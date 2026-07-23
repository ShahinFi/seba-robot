#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Splits a mutable command line into whitespace-separated
 * arguments. The line buffer is modified in place.
 */
int CommandParser_Tokenize(
    char *line,
    char *arguments[],
    int maximum_arguments
);

/*
 * Parses a motor test command value in percent.
 *
 * Valid range:
 *   -100 = full reverse
 *      0 = stopped
 *   +100 = full forward
 */
bool CommandParser_ParseSpeed(
    const char *text,
    int16_t *speed
);

/*
 * Parses a signed decimal integer inside the caller-supplied
 * inclusive range.
 */
bool CommandParser_ParseInt32(
    const char *text,
    int32_t minimum,
    int32_t maximum,
    int32_t *value
);

/*
 * Parses a decimal floating-point value inside the
 * caller-supplied inclusive range.
 */
bool CommandParser_ParseFloat(
    const char *text,
    float minimum,
    float maximum,
    float *value
);

#endif
