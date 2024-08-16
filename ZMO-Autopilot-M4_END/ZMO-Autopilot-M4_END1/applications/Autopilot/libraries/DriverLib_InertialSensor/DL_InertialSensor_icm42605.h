/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-28     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_INERTIALSENSOR_DL_INERTIALSENSOR_ICM42605_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_INERTIALSENSOR_DL_INERTIALSENSOR_ICM42605_H_

#include "CL_Vector.h"
#include "loco_config.h"
//USR0
#define ICM42605_DEVICE_CONFIG    0x11
#define ICM42605_TEMP_DATA1       0x1D
#define ICM42605_TEMP_DATA2       0x1E
#define ICM42605_ACCEL_XOUT_H     0x1F
#define ICM42605_ACCEL_XOUT_L     0x20
#define ICM42605_ACCEL_YOUT_H     0x21
#define ICM42605_ACCEL_YOUT_L     0x22
#define ICM42605_ACCEL_ZOUT_H     0x23
#define ICM42605_ACCEL_ZOUT_L     0x24
#define ICM42605_GYRO_XOUT_H      0x25
#define ICM42605_GYRO_XOUT_L      0x26
#define ICM42605_GYRO_YOUT_H      0x27
#define ICM42605_GYRO_YOUT_L      0x28
#define ICM42605_GYRO_ZOUT_H      0x29
#define ICM42605_GYRO_ZOUT_L      0x2A

#define ICM42605_PWR_MGMT0                0x4E
#define ICM42605_GYRO_CONFIG0             0x4F
#define ICM42605_ACCEL_CONFIG0            0x50
#define ICM42605_GYRO_CONFIG1             0x51
#define ICM42605_GYRO_ACCEL_CONFIG0       0x52
#define ICM42605_ACCEL_CONFIG1            0x53

#define ICM42605_WHO_AM_I                         0x75

#define ICM42605_REG_BANK_SEL                     0x76        /*  Select USER BANK  */


//USR1
#define ICM42605_SENSOR_CONFIG0           0x03
void icm42605_reg_read(rt_uint8_t addr,rt_uint8_t *rev_buf,rt_uint32_t len);
void icm42605_reg_write(rt_uint8_t addr,rt_uint8_t value);
int icm42605_get_gyro_raw(Vector3i16*raw_gyro,uint8_t instance);
int icm42605_get_acce_raw(Vector3i16*raw_acce,uint8_t instance);

#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_INERTIALSENSOR_DL_INERTIALSENSOR_ICM42605_H_ */
