/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-28     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_AHRS_CL_AHRS_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_AHRS_CL_AHRS_H_

#include "CL_Math.h"
#include "CL_Vector.h"
typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;

    float roll;
    float pitch;
    float yaw;
} ahrs_t ;

extern float rMat[3][3];    /*旋转矩阵*/
extern float imuAttitudeYaw;
extern ahrs_t ahrs;
extern ahrs_t Quaternion;
extern attitude_t att;

int get_ahrs_queue_entry();
attitude_t get_ahrs_eulerAngles(void);  //为四元数姿态解算得到的欧拉角  单位：度
void imu_transform_vector_body_to_earth(Vector3f *bf,Vector3f *ef);
void imu_transform_vector_earth_to_body(Vector3f *bf,Vector3f *ef);
void ahrs_update_attitude(const sensorData_t *sensorData, state_t *state, float dt);
/*从队列读取欧拉角数据*/
bool ahrsReadEulerAngles(attitude_t *attitude);  //为四元数姿态解算得到的欧拉角  单位：度
void EKF_AHRS_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float*q);

#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_AHRS_CL_AHRS_H_ */
