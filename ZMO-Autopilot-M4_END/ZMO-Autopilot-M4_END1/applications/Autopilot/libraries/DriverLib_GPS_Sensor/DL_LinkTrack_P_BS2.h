/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-04-11     Administrator       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_GPS_SENSOR_DL_LINKTRACK_P_BS2_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_GPS_SENSOR_DL_LINKTRACK_P_BS2_H_

#include "DL_OpticalFlow.h"

typedef struct {
    uint8_t data[128];
    uint8_t dataLen;
    uint8_t id;
    uint8_t dataFre;
    uint8_t role;
    int32_t raw_px;
    int32_t raw_py;
    int32_t raw_pz;
    int32_t raw_vx;
    int32_t raw_vy;
    int32_t raw_vz;
    int32_t distance[8];
    uint16_t sonar_vol;
    uint32_t sonar_time;
    Vector3f eop;
    Vector3f pos;
    Vector3f vel;
}uwb_data_t;
extern uwb_data_t  UWB;
void uwb_data_receive(uint8_t data);

#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_GPS_SENSOR_DL_LINKTRACK_P_BS2_H_ */
