#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

void Encoder_Init(void);

/*
 * Transfers wrapping hardware counts into signed accumulated
 * positions. The state estimator calls this at its fixed rate.
 */
void Encoder_Update(void);

int64_t Encoder_GetLeftPosition(void);
int64_t Encoder_GetRightPosition(void);

/*
 * Resets the accumulated position to zero at the current
 * hardware counter value.
 */
void Encoder_ResetLeft(void);
void Encoder_ResetRight(void);
void Encoder_ResetAll(void);

#endif
