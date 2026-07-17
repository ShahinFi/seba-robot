#include "imu.h"

#include "imu_transport.h"

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/*
 * BNO085 report periods.
 *
 * Accelerometer:
 * 5000 us = 200 Hz
 *
 * Calibrated gyroscope:
 * 5000 us = 200 Hz
 *
 * Game rotation vector:
 * 10000 us = 100 Hz
 */
#define IMU_ACCEL_INTERVAL_US          5000U
#define IMU_GYRO_INTERVAL_US           5000U
#define IMU_ORIENTATION_INTERVAL_US   10000U

/*
 * Physical mounting:
 *
 * Sensor X = robot sideways
 * Sensor Y = robot forward
 * Sensor Z = robot upward
 *
 * Therefore:
 *
 * Robot pitch angle/rate = rotation around sensor X.
 * Robot yaw angle/rate   = rotation around sensor Z.
 *
 * Sign constants define the robot-positive pitch and yaw
 * convention.
 */
#define ROBOT_PITCH_SIGN   1.0F
#define ROBOT_YAW_SIGN     1.0F

static IMUData latest_data;

static bool imu_ready;
static bool imu_reset_detected;

static void IMU_AsyncEventCallback(
    void *cookie,
    sh2_AsyncEvent_t *event
);

static void IMU_SensorCallback(
    void *cookie,
    sh2_SensorEvent_t *event
);

static bool IMU_EnableReport(
    sh2_SensorId_t sensor_id,
    uint32_t interval_us
);

static void IMU_UpdateRobotAngles(void);

static void IMU_ClearData(void);

bool IMU_Init(void)
{
    sh2_ProductIds_t product_ids;

    IMU_ClearData();

    imu_ready = false;
    imu_reset_detected = false;

    if (sh2_open(
            IMU_Transport_GetHAL(),
            IMU_AsyncEventCallback,
            NULL
        ) != SH2_OK)
    {
        return false;
    }

    if (sh2_setSensorCallback(
            IMU_SensorCallback,
            NULL
        ) != SH2_OK)
    {
        sh2_close();
        return false;
    }

    memset(
        &product_ids,
        0,
        sizeof(product_ids)
    );

    if (sh2_getProdIds(
            &product_ids
        ) != SH2_OK)
    {
        sh2_close();
        return false;
    }

    if (product_ids.numEntries == 0U)
    {
        sh2_close();
        return false;
    }

    if (!IMU_EnableReport(
            SH2_ACCELEROMETER,
            IMU_ACCEL_INTERVAL_US
        ))
    {
        sh2_close();
        return false;
    }

    if (!IMU_EnableReport(
            SH2_GYROSCOPE_CALIBRATED,
            IMU_GYRO_INTERVAL_US
        ))
    {
        sh2_close();
        return false;
    }

    if (!IMU_EnableReport(
            SH2_GAME_ROTATION_VECTOR,
            IMU_ORIENTATION_INTERVAL_US
        ))
    {
        sh2_close();
        return false;
    }

    imu_ready = true;

    return true;
}

void IMU_Process(void)
{
    if (!imu_ready)
    {
        return;
    }

    /*
     * The EXTI interrupt only records that data is ready.
     * Actual I2C and SH-2 processing occurs here in the
     * main loop.
     */
    if (
        IMU_Transport_DataReady() ||
        imu_reset_detected
    )
    {
        IMU_Transport_ClearDataReady();

        sh2_service();

        imu_reset_detected = false;
    }
}

bool IMU_GetLatest(
    IMUData *data
)
{
    if (data == NULL)
    {
        return false;
    }

    if (!imu_ready)
    {
        return false;
    }

    *data = latest_data;

    return true;
}

bool IMU_IsReady(void)
{
    return imu_ready;
}

static void IMU_AsyncEventCallback(
    void *cookie,
    sh2_AsyncEvent_t *event
)
{
    (void)cookie;

    if (event == NULL)
    {
        return;
    }

    if (event->eventId == SH2_RESET)
    {
        imu_reset_detected = true;

        latest_data.acceleration_valid = false;
        latest_data.gyroscope_valid = false;
        latest_data.orientation_valid = false;
    }
}

static void IMU_SensorCallback(
    void *cookie,
    sh2_SensorEvent_t *event
)
{
    sh2_SensorValue_t value;

    (void)cookie;

    if (event == NULL)
    {
        return;
    }

    if (sh2_decodeSensorEvent(
            &value,
            event
        ) != SH2_OK)
    {
        return;
    }

    switch (value.sensorId)
    {
        case SH2_ACCELEROMETER:
        {
            /*
             * Store accelerometer values in the sensor's
             * mounted physical axes.
             */
            latest_data.acceleration_x_mps2 =
                value.un.accelerometer.x;

            latest_data.acceleration_y_mps2 =
                value.un.accelerometer.y;

            latest_data.acceleration_z_mps2 =
                value.un.accelerometer.z;

            latest_data.acceleration_accuracy =
                value.status & 0x03U;

            latest_data.acceleration_timestamp_us =
                value.timestamp;

            latest_data.acceleration_valid = true;

            break;
        }

        case SH2_GYROSCOPE_CALIBRATED:
        {
            latest_data.angular_velocity_x_rads =
                value.un.gyroscope.x;

            latest_data.angular_velocity_y_rads =
                value.un.gyroscope.y;

            latest_data.angular_velocity_z_rads =
                value.un.gyroscope.z;

            /*
             * Sensor X points sideways, so X angular velocity
             * is robot pitch rate.
             */
            latest_data.robot_pitch_rate_rads =
                ROBOT_PITCH_SIGN *
                latest_data.angular_velocity_x_rads;

            /*
             * Sensor Z points upward, so Z angular velocity is
             * robot yaw rate.
             */
            latest_data.robot_yaw_rate_rads =
                ROBOT_YAW_SIGN *
                latest_data.angular_velocity_z_rads;

            latest_data.gyroscope_accuracy =
                value.status & 0x03U;

            latest_data.gyroscope_timestamp_us =
                value.timestamp;

            latest_data.gyroscope_valid = true;

            break;
        }

        case SH2_GAME_ROTATION_VECTOR:
        {
            /*
             * CEVA SH-2 quaternion field names:
             *
             * real = w
             * i    = x
             * j    = y
             * k    = z
             */
            latest_data.quaternion_w =
                value.un.gameRotationVector.real;

            latest_data.quaternion_x =
                value.un.gameRotationVector.i;

            latest_data.quaternion_y =
                value.un.gameRotationVector.j;

            latest_data.quaternion_z =
                value.un.gameRotationVector.k;

            latest_data.orientation_accuracy =
                value.status & 0x03U;

            latest_data.orientation_timestamp_us =
                value.timestamp;

            latest_data.orientation_valid = true;

            IMU_UpdateRobotAngles();

            break;
        }

        default:
        {
            break;
        }
    }
}

static bool IMU_EnableReport(
    sh2_SensorId_t sensor_id,
    uint32_t interval_us
)
{
    sh2_SensorConfig_t configuration;

    memset(
        &configuration,
        0,
        sizeof(configuration)
    );

    configuration.reportInterval_us =
        interval_us;

    configuration.batchInterval_us =
        0U;

    configuration.changeSensitivity =
        0U;

    configuration.changeSensitivityEnabled =
        false;

    configuration.changeSensitivityRelative =
        false;

    configuration.wakeupEnabled =
        false;

    configuration.alwaysOnEnabled =
        true;

    configuration.sniffEnabled =
        false;

    return sh2_setSensorConfig(
               sensor_id,
               &configuration
           ) == SH2_OK;
}

static void IMU_UpdateRobotAngles(void)
{
    const float w =
        latest_data.quaternion_w;

    const float x =
        latest_data.quaternion_x;

    const float y =
        latest_data.quaternion_y;

    const float z =
        latest_data.quaternion_z;

    /*
     * Quaternion roll corresponds to robot pitch because
     * sensor X points sideways.
     */
    latest_data.robot_pitch_rad =
        ROBOT_PITCH_SIGN *
        atan2f(
            2.0F * (w * x + y * z),
            1.0F - 2.0F * (x * x + y * y)
        );

    /*
     * Quaternion yaw corresponds to robot yaw because sensor Z
     * points upward.
     */
    latest_data.robot_yaw_rad =
        ROBOT_YAW_SIGN *
        atan2f(
            2.0F * (w * z + x * y),
            1.0F - 2.0F * (y * y + z * z)
        );
}

static void IMU_ClearData(void)
{
    memset(
        &latest_data,
        0,
        sizeof(latest_data)
    );
}
