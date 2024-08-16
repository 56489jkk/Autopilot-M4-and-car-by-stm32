/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-29     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_OPTICALFLOWSENSOR_DL_OPTICALFLOW_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_OPTICALFLOWSENSOR_DL_OPTICALFLOW_H_

#include "CL_Vector.h"
#include "board_interface.h"

enum Opt_Type {
    LC302 =          0,  // 优象光流模块
    LC306 =          1,  // 优象光流模块
    LC307 =          2,  // 优象光流模块
    T1_001_Plus =    3,  // 优象光流激光一体模块
    };

typedef struct {
    uint8_t data[20];
    int16_t dataLen;
    int16_t raw_x;
    int16_t raw_y;
    int16_t distance;
    uint8_t qual;
    int16_t opt_valid;
    int16_t tof_valid;
    int16_t sonar_timestamp;
    int32_t last_update_ms;
}opt_data; //光流模块共有的数据

extern opt_data  opt;

Vector2f get_opt_radians(void);
float get_opt_distance(void);   //
bool get_opt_health(void);

void update_optical_flow_data(enum Opt_Type new_opt_type,uint8_t data); //
Vector2f opticalflow_rotate_complementary_filter(float dt); //光流角速度与陀螺仪角速度融合
Vector2f t265_rotate_complementary_filter(float dt); //双目速度与陀螺仪角速度融合
void applySensorAlignment_2(int16_t * dest, int16_t * src, uint8_t rotation);
#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_OPTICALFLOWSENSOR_DL_OPTICALFLOW_H_ */
