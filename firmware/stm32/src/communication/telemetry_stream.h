#ifndef TELEMETRY_STREAM_H
#define TELEMETRY_STREAM_H

/*
 * Writes one complete TEL snapshot using the current robot
 * state, balance status, and actuator status.
 *
 * Field names are the host protocol. Numeric units match the
 * state and status structure member names.
 */
void TelemetryStream_WriteSnapshot(void);

#endif
