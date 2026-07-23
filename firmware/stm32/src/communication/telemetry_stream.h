#ifndef TELEMETRY_STREAM_H
#define TELEMETRY_STREAM_H

/*
 * Writes one complete TEL snapshot using the current robot
 * state, balance status, and actuator status.
 */
void TelemetryStream_WriteSnapshot(void);

#endif
