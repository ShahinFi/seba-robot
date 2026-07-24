#ifndef IMU_TRANSPORT_H
#define IMU_TRANSPORT_H

#include <stdbool.h>

#include "sh2_hal.h"

/*
 * Returns the SH-2 HAL transport used by imu.c. The transport
 * owns BNO085 I2C, INT handling, and microsecond timestamps.
 */
sh2_Hal_t *IMU_Transport_GetHAL(void);

/*
 * Data-ready state combines the EXTI latch with the physical
 * active-low INT pin so a missed interrupt edge does not leave
 * unread BNO085 packets pending.
 */
bool IMU_Transport_DataReady(void);
void IMU_Transport_ClearDataReady(void);

#endif
