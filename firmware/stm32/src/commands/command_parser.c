#include "command_parser.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>

/*
 * Numeric command parsers require complete-token conversion, so
 * inputs such as "10abc" are rejected instead of being truncated.
 */
int CommandParser_Tokenize(
    char *line,
    char *arguments[],
    int maximum_arguments
)
{
    int argument_count = 0;

    char *token =
        strtok(line, " \t");

    while (token != NULL)
    {
        if (argument_count < maximum_arguments)
        {
            arguments[argument_count] =
                token;
        }

        argument_count++;

        token =
            strtok(NULL, " \t");
    }

    return argument_count;
}

bool CommandParser_ParseSpeed(
    const char *text,
    int16_t *speed
)
{
    char *end;
    long value;

    if (
        text == NULL ||
        speed == NULL ||
        text[0] == '\0'
    )
    {
        return false;
    }

    errno = 0;

    value =
        strtol(
            text,
            &end,
            10
        );

    if (
        end == text ||
        *end != '\0' ||
        errno == ERANGE ||
        value < -100L ||
        value > 100L
    )
    {
        return false;
    }

    *speed =
        (int16_t)value;

    return true;
}

bool CommandParser_ParseInt32(
    const char *text,
    int32_t minimum,
    int32_t maximum,
    int32_t *value
)
{
    char *end;
    long parsed;

    if (
        text == NULL ||
        value == NULL ||
        text[0] == '\0'
    )
    {
        return false;
    }

    errno = 0;

    parsed =
        strtol(
            text,
            &end,
            10
        );

    if (
        end == text ||
        *end != '\0' ||
        errno == ERANGE ||
        parsed < minimum ||
        parsed > maximum
    )
    {
        return false;
    }

    *value =
        (int32_t)parsed;

    return true;
}

bool CommandParser_ParseFloat(
    const char *text,
    float minimum,
    float maximum,
    float *value
)
{
    char *end;
    float parsed;

    if (
        text == NULL ||
        value == NULL ||
        text[0] == '\0'
    )
    {
        return false;
    }

    errno = 0;

    parsed =
        strtof(
            text,
            &end
        );

    if (
        end == text ||
        *end != '\0' ||
        errno == ERANGE ||
        parsed < minimum ||
        parsed > maximum
    )
    {
        return false;
    }

    *value =
        parsed;

    return true;
}
