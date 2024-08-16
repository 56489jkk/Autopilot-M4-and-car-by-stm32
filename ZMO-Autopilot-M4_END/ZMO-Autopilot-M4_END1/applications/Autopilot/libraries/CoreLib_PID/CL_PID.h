/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-28     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_PID_CL_PID_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_PID_CL_PID_H_
#include "stdbool.h"
#include "CL_Vector.h"
#include "CL_ADRC.h"
typedef struct
{
    int16_t eLimit ;//偏差限幅  赋值为0时，偏差不限幅
    int16_t iLimit ;//积分限幅  赋值为0时，积分不限幅
    float target;    //期望
    float measurement;  //反馈值
    float error;//偏差
    float error_last;//上次偏差
    float integrate;//积分值
    float Kp;//控制参数Kp
    float Ki;//控制参数Ki
    float Kd;//控制参数Kd
    float output;//控制器总输出
    int16_t output_limit;//输出限幅
    float measurement_last;//上次反馈值
    float derivative;//微分量
    float dt;             //< delta-time dt
    bool enableDFilter;   //< filter for D term enable flag
    float alpha;
    TD TD_filter;
}pid_struct;

/*pid结构体初始化*/
void pidInit(pid_struct* pid, float kp, float ki, float kd, int16_t eLimit,int16_t iLimit, int16_t outputLimit, float dt, bool enableDFilter, float cutoffFreq);
float pid_rate_updata(pid_struct*pid,float target,float measurement);
float pid_pos_updata(pid_struct*pid,float target,float measurement);
float pid_yaw_updata(pid_struct*pid,float measurement);
float Attpid_Updata(pid_struct*pid,float FeedBack,float diff_FeedBack);
float pid_angle_updata(pid_struct*pid,float target,float measurement,float measurement_dis);
void pid_reset(pid_struct* pid);
void pid_reset_integral(pid_struct* pid);
void pid_set_integral(pid_struct* pid, int integrate);

extern pid_struct pid[Pid_Num];


#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_PID_CL_PID_H_ */
