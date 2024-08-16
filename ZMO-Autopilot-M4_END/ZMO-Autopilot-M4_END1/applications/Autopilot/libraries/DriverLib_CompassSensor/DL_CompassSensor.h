/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-19     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_COMPASSSENSOR_DL_COMPASSSENSOR_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_COMPASSSENSOR_DL_COMPASSSENSOR_H_

Vector3f get_imu_sensorsMagn(void);  //为原始数据经第一次低通滤波后的角速度数据 deg/s
void magn_data_update(Vector3f *magn,uint8_t instance);

#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_COMPASSSENSOR_DL_COMPASSSENSOR_H_ */
