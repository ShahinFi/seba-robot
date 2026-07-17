#include "console.h"

#include "serial/serial.h"
#include "tests/current_sensor_test.h"
#include "tests/encoder_test.h"
#include "tests/imu_test.h"
#include "tests/motor_test.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define CONSOLE_LINE_LENGTH    64U
#define CONSOLE_MAX_ARGUMENTS  4

static char input_line[CONSOLE_LINE_LENGTH];
static uint32_t input_length;

static void Console_ProcessByte(uint8_t byte);
static void Console_ExecuteLine(char *line);

static int Console_Tokenize(
    char *line,
    char *arguments[],
    int maximum_arguments
);

static void Console_HandleMotor(
    int argument_count,
    char *arguments[]
);

static void Console_HandleEncoder(
    int argument_count,
    char *arguments[]
);

static void Console_HandleCurrent(
    int argument_count,
    char *arguments[]
);

static void Console_HandleIMU(
    int argument_count,
    char *arguments[]
);

static bool Console_ParseSpeed(
    const char *text,
    int16_t *speed
);

static void Console_PrintHelp(void);
static void Console_PrintPrompt(void);

void Console_Init(void)
{
    input_length = 0U;
    input_line[0] = '\0';

    Serial_WriteLine("");
    Serial_WriteLine(
        "SEBA-ROBOT console ready."
    );

    Serial_WriteLine(
        "Type 'help' for commands."
    );

    Console_PrintPrompt();
}

void Console_Process(void)
{
    uint8_t byte;

    while (Serial_ReadByte(&byte))
    {
        Console_ProcessByte(byte);
    }
}

static void Console_ProcessByte(uint8_t byte)
{
    if (
        byte == '\r' ||
        byte == '\n'
    )
    {
        if (input_length == 0U)
        {
            return;
        }

        input_line[input_length] = '\0';

        Serial_WriteLine("");

        Console_ExecuteLine(input_line);

        input_length = 0U;
        input_line[0] = '\0';

        Console_PrintPrompt();

        return;
    }

    if (
        byte == '\b' ||
        byte == 127U
    )
    {
        if (input_length > 0U)
        {
            input_length--;

            input_line[input_length] = '\0';

            /*
             * Move back, erase the displayed character, then
             * move back again on a simple serial terminal.
             */
            Serial_Write("\b \b");
        }

        return;
    }

    if (
        byte < 32U ||
        byte > 126U
    )
    {
        return;
    }

    if (
        input_length >=
        CONSOLE_LINE_LENGTH - 1U
    )
    {
        /*
         * Discard the partial command so the next line starts
         * from a known empty buffer.
         */
        input_length = 0U;
        input_line[0] = '\0';

        Serial_WriteLine("");

        Serial_WriteLine(
            "ERROR: command is too long."
        );

        Console_PrintPrompt();

        return;
    }

    input_line[input_length] =
        (char)byte;

    input_length++;

    input_line[input_length] = '\0';

    {
        char echo[2];

        echo[0] = (char)byte;
        echo[1] = '\0';

        Serial_Write(echo);
    }
}

static void Console_ExecuteLine(char *line)
{
    char *arguments[CONSOLE_MAX_ARGUMENTS];

    const int argument_count =
        Console_Tokenize(
            line,
            arguments,
            CONSOLE_MAX_ARGUMENTS
        );

    if (argument_count == 0)
    {
        return;
    }

    if (strcmp(arguments[0], "help") == 0)
    {
        if (argument_count != 1)
        {
            Serial_WriteLine(
                "ERROR: usage: help"
            );

            return;
        }

        Console_PrintHelp();

        return;
    }

    if (strcmp(arguments[0], "motor") == 0)
    {
        Console_HandleMotor(
            argument_count,
            arguments
        );

        return;
    }

    if (strcmp(arguments[0], "encoder") == 0)
    {
        Console_HandleEncoder(
            argument_count,
            arguments
        );

        return;
    }

    if (strcmp(arguments[0], "current") == 0)
    {
        Console_HandleCurrent(
            argument_count,
            arguments
        );

        return;
    }

    if (strcmp(arguments[0], "imu") == 0)
    {
        Console_HandleIMU(
            argument_count,
            arguments
        );

        return;
    }

    Serial_WriteLine(
        "ERROR: unknown command."
    );

    Serial_WriteLine(
        "Type 'help' for available commands."
    );
}

static int Console_Tokenize(
    char *line,
    char *arguments[],
    int maximum_arguments
)
{
    int argument_count = 0;

    char *token =
        strtok(line, " \t");

    while (
        token != NULL &&
        argument_count < maximum_arguments
    )
    {
        arguments[argument_count] =
            token;

        argument_count++;

        token =
            strtok(NULL, " \t");
    }

    return argument_count;
}

static void Console_HandleMotor(
    int argument_count,
    char *arguments[]
)
{
    int16_t speed;

    if (
        argument_count == 2 &&
        strcmp(arguments[1], "stop") == 0
    )
    {
        MotorTest_Stop();

        Serial_WriteLine(
            "OK: all motors stopped and disabled."
        );

        return;
    }

    if (argument_count != 3)
    {
        Serial_WriteLine(
            "ERROR: usage: motor <left|right> <-100...100> OR motor stop"
        );

        return;
    }

    if (!Console_ParseSpeed(
            arguments[2],
            &speed
        ))
    {
        Serial_WriteLine(
            "ERROR: speed must be an integer from -100 to 100."
        );

        return;
    }

    if (strcmp(arguments[1], "left") == 0)
    {
        if (!MotorTest_SetLeft(speed))
        {
            Serial_WriteLine(
                "ERROR: invalid left motor command."
            );

            return;
        }

        Serial_Write(
            "OK: left motor = "
        );

        Serial_Write(arguments[2]);

        Serial_WriteLine("%");

        return;
    }

    if (strcmp(arguments[1], "right") == 0)
    {
        if (!MotorTest_SetRight(speed))
        {
            Serial_WriteLine(
                "ERROR: invalid right motor command."
            );

            return;
        }

        Serial_Write(
            "OK: right motor = "
        );

        Serial_Write(arguments[2]);

        Serial_WriteLine("%");

        return;
    }

    Serial_WriteLine(
        "ERROR: motor must be 'left', 'right', or 'stop'."
    );
}

static void Console_HandleEncoder(
    int argument_count,
    char *arguments[]
)
{
    if (argument_count != 2)
    {
        Serial_WriteLine(
            "ERROR: usage: encoder <read|reset>"
        );

        return;
    }

    if (strcmp(arguments[1], "read") == 0)
    {
        EncoderTest_PrintPositions();

        return;
    }

    if (strcmp(arguments[1], "reset") == 0)
    {
        EncoderTest_ResetPositions();

        return;
    }

    Serial_WriteLine(
        "ERROR: usage: encoder <read|reset>"
    );
}

static void Console_HandleCurrent(
    int argument_count,
    char *arguments[]
)
{
    if (
        argument_count != 2 ||
        strcmp(arguments[1], "read") != 0
    )
    {
        Serial_WriteLine(
            "ERROR: usage: current read"
        );

        return;
    }

    if (!CurrentSensorTest_PrintReadings())
    {
        Serial_WriteLine(
            "ERROR: current sensor read failed."
        );

        return;
    }
}

static void Console_HandleIMU(
    int argument_count,
    char *arguments[]
)
{
    if (
        argument_count != 2 ||
        strcmp(arguments[1], "read") != 0
    )
    {
        Serial_WriteLine(
            "ERROR: usage: imu read"
        );

        return;
    }

    if (!IMUTest_PrintLatest())
    {
        Serial_WriteLine(
            "ERROR: IMU is not initialized."
        );

        return;
    }
}

static bool Console_ParseSpeed(
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

static void Console_PrintHelp(void)
{
    Serial_WriteLine("Commands:");

    Serial_WriteLine(
        "  motor left <-100...100>"
    );

    Serial_WriteLine(
        "  motor right <-100...100>"
    );

    Serial_WriteLine(
        "  motor stop"
    );

    Serial_WriteLine(
        "  encoder read"
    );

    Serial_WriteLine(
        "  encoder reset"
    );

    Serial_WriteLine(
        "  current read"
    );

    Serial_WriteLine(
        "  imu read"
    );

    Serial_WriteLine(
        "  help"
    );

    Serial_WriteLine("");

    Serial_WriteLine(
        "Positive motor command = forward"
    );

    Serial_WriteLine(
        "Negative motor command = reverse"
    );

    Serial_WriteLine(
        "Zero = stop that motor"
    );
}

static void Console_PrintPrompt(void)
{
    Serial_Write("> ");
}
