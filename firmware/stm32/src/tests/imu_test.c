#include "imu_test.h"

#include "imu/imu.h"
#include "serial/serial.h"

/*
 * IMU diagnostics expose both robot-facing values and native
 * sensor axes so mounting/sign problems can be checked directly.
 */
bool IMUTest_PrintLatest(void)
{
    IMUData data;

    if (!IMU_GetLatest(&data))
    {
        return false;
    }

    Serial_WriteLine(
        "Robot orientation:"
    );

    Serial_Write(
        "  Pitch [rad]: "
    );

    Serial_WriteFloat3(
        data.robot_pitch_rad
    );

    Serial_WriteLine("");

    Serial_Write(
        "  Yaw [rad]: "
    );

    Serial_WriteFloat3(
        data.robot_yaw_rad
    );

    Serial_WriteLine("");

    Serial_Write(
        "  Pitch rate [rad/s]: "
    );

    Serial_WriteFloat3(
        data.robot_pitch_rate_rads
    );

    Serial_WriteLine("");

    Serial_Write(
        "  Yaw rate [rad/s]: "
    );

    Serial_WriteFloat3(
        data.robot_yaw_rate_rads
    );

    Serial_WriteLine("");

    Serial_Write(
        "  Orientation accuracy: "
    );

    Serial_WriteUInt32(
        data.orientation_accuracy
    );

    Serial_WriteLine("");

    Serial_Write(
        "  Orientation valid: "
    );

    Serial_WriteLine(
        data.orientation_valid
            ? "yes"
            : "no"
    );

    Serial_Write(
        "  Gyroscope valid: "
    );

    Serial_WriteLine(
        data.gyroscope_valid
            ? "yes"
            : "no"
    );

    Serial_WriteLine("");

    Serial_WriteLine(
        "Accelerometer [m/s^2]:"
    );

    Serial_Write("  X sideways: ");

    Serial_WriteFloat3(
        data.acceleration_x_mps2
    );

    Serial_WriteLine("");

    Serial_Write("  Y forward: ");

    Serial_WriteFloat3(
        data.acceleration_y_mps2
    );

    Serial_WriteLine("");

    Serial_Write("  Z upward: ");

    Serial_WriteFloat3(
        data.acceleration_z_mps2
    );

    Serial_WriteLine("");

    Serial_Write(
        "  Accuracy: "
    );

    Serial_WriteUInt32(
        data.acceleration_accuracy
    );

    Serial_WriteLine("");

    Serial_Write(
        "  Valid: "
    );

    Serial_WriteLine(
        data.acceleration_valid
            ? "yes"
            : "no"
    );

    Serial_WriteLine("");

    Serial_WriteLine(
        "Gyroscope native axes [rad/s]:"
    );

    Serial_Write("  X sideways axis: ");

    Serial_WriteFloat3(
        data.angular_velocity_x_rads
    );

    Serial_WriteLine("");

    Serial_Write("  Y forward axis: ");

    Serial_WriteFloat3(
        data.angular_velocity_y_rads
    );

    Serial_WriteLine("");

    Serial_Write("  Z upward axis: ");

    Serial_WriteFloat3(
        data.angular_velocity_z_rads
    );

    Serial_WriteLine("");

    Serial_Write(
        "  Accuracy: "
    );

    Serial_WriteUInt32(
        data.gyroscope_accuracy
    );

    Serial_WriteLine("");

    Serial_WriteLine("");

    Serial_WriteLine(
        "Game rotation quaternion:"
    );

    Serial_Write("  W: ");

    Serial_WriteFloat3(
        data.quaternion_w
    );

    Serial_WriteLine("");

    Serial_Write("  X: ");

    Serial_WriteFloat3(
        data.quaternion_x
    );

    Serial_WriteLine("");

    Serial_Write("  Y: ");

    Serial_WriteFloat3(
        data.quaternion_y
    );

    Serial_WriteLine("");

    Serial_Write("  Z: ");

    Serial_WriteFloat3(
        data.quaternion_z
    );

    Serial_WriteLine("");

    return true;
}
