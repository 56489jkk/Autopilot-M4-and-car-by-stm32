/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-29     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_RANGEFINDERSENSOR_DL_RANGESENSOR_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_RANGEFINDERSENSOR_DL_RANGESENSOR_H_

#include "loco_config.h"

enum Range_Type {
    TF_Luna =          0,  //北醒激光模块
    TFmini_Plus =      1,  //北醒激光模块
};

typedef struct
{
    float distance;
    float strength;
    float temperature;
    float last_distance;
    float last_strength;
    float div;
    float acc;
    float last_div;
    uint8_t mode;
    uint8_t health;
    uint8_t dataLen;
    uint8_t data[30];
}range_data_t;

extern range_data_t range;

#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_RANGEFINDERSENSOR_DL_RANGESENSOR_H_ */
