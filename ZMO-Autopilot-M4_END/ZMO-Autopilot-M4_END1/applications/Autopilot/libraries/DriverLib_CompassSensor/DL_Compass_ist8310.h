/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-10-01     Administrator       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_COMPASSSENSOR_DL_COMPASS_IST8310_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_COMPASSSENSOR_DL_COMPASS_IST8310_H_

#include "CL_Vector.h"
#include "loco_config.h"
int ist8310_get_magn_raw(Vector3i16*raw_magn);


#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_COMPASSSENSOR_DL_COMPASS_IST8310_H_ */
