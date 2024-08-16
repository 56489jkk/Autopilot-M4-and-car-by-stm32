/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-23     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_COMPASSSENSOR_DL_COMPASS_BMM150_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_COMPASSSENSOR_DL_COMPASS_BMM150_H_
#include "CL_Vector.h"
#include "loco_config.h"

#define BMM_CHIP_ID     0x40

#define BMM_XOUT_L      0x42
#define BMM_XOUT_H      0x43
#define BMM_YOUT_L      0x44
#define BMM_YOUT_H      0x45
#define BMM_ZOUT_L      0x46
#define BMM_ZOUT_H      0x47

#define BMM_PWR_CTRL    0x4B

int bmm150_get_magn_raw(Vector3i16*Raw_magn);


#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_COMPASSSENSOR_DL_COMPASS_BMM150_H_ */
