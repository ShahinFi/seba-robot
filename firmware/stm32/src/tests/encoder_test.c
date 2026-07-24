#include "encoder_test.h"

#include "encoder/encoder.h"
#include "serial/serial.h"

/*
 * Encoder diagnostics print the signed accumulated counts used
 * by the estimator.
 */
void EncoderTest_PrintPositions(void)
{
    Encoder_Update();

    Serial_Write("Left: ");

    Serial_WriteInt64(
        Encoder_GetLeftPosition()
    );

    Serial_WriteLine("");

    Serial_Write("Right: ");

    Serial_WriteInt64(
        Encoder_GetRightPosition()
    );

    Serial_WriteLine("");
}

void EncoderTest_ResetPositions(void)
{
    Encoder_ResetAll();

    Serial_WriteLine(
        "OK: encoder positions reset."
    );
}
