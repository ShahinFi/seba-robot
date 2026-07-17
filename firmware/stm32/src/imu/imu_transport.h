#ifndef IMU_TRANSPORT_H
#define IMU_TRANSPORT_H

#include <stdbool.h>

#include "sh2_hal.h"

sh2_Hal_t *IMU_Transport_GetHAL(void);

bool IMU_Transport_DataReady(void);
void IMU_Transport_ClearDataReady(void);

#endif