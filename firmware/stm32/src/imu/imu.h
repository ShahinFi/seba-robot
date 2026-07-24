#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    /*
     * BNO085 native accelerometer axes.
     *
     * Physical mounting:
     * X = robot sideways
     * Y = robot forward
     * Z = robot upward
     */
    float acceleration_x_mps2;
    float acceleration_y_mps2;
    float acceleration_z_mps2;

    /*
     * BNO085 native gyroscope axes.
     */
    float angular_velocity_x_rads;
    float angular_velocity_y_rads;
    float angular_velocity_z_rads;

    /*
     * BNO085 game-rotation-vector quaternion.
     *
     * Quaternion order:
     * w, x, y, z
     */
    float quaternion_w;
    float quaternion_x;
    float quaternion_y;
    float quaternion_z;

    /*
     * Robot-facing orientation values.
     *
     * Because sensor X points sideways:
     * robot pitch is rotation around sensor X.
     *
     * Because sensor Z points upward:
     * robot yaw is rotation around sensor Z.
     */
    float robot_pitch_rad;
    float robot_yaw_rad;

    /*
     * Robot-facing angular rates.
     */
    float robot_pitch_rate_rads;
    float robot_yaw_rate_rads;

    uint8_t acceleration_accuracy;
    uint8_t gyroscope_accuracy;
    uint8_t orientation_accuracy;

    /*
     * Sensor timestamps are SH-2 timestamps in microseconds.
     */
    uint64_t acceleration_timestamp_us;
    uint64_t gyroscope_timestamp_us;
    uint64_t orientation_timestamp_us;

    uint32_t acceleration_update_count;
    uint32_t gyroscope_update_count;
    uint32_t orientation_update_count;

    uint32_t acceleration_age_ms;
    uint32_t gyroscope_age_ms;
    uint32_t orientation_age_ms;

    bool acceleration_valid;
    bool gyroscope_valid;
    bool orientation_valid;
} IMUData;

bool IMU_Init(void);

/*
 * Services pending SH-2 events. Call regularly from the main
 * loop after successful initialization.
 */
void IMU_Process(void);

/*
 * Copies the most recent decoded sensor values. Age fields are
 * computed at read time from the last update received for each
 * report type.
 */
bool IMU_GetLatest(
    IMUData *data
);

bool IMU_IsReady(void);

#endif
