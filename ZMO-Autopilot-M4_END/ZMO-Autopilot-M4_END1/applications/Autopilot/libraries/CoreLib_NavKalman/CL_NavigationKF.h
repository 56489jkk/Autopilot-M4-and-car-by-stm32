/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-01     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_NAVKALMAN_CL_NAVIGATIONKF_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_NAVKALMAN_CL_NAVIGATIONKF_H_

#include "loco_config.h"
#include "CL_Vector.h"

#define Num  10
typedef struct
{
    Vector3f Position;
    Vector3f Speed;

    Vector3f Ori_Position;
    Vector3f Ori_Speed;
    float Pos_History[3][Num];//历史惯导位置
    float Vel_History[3][Num];//历史惯导速度
    Vector3f Acce;
    Vector3f last_Acce;
    Vector3f Acce_Bias;
    float  Q[4];         //过程噪声方差
    float  R[2];         //观测噪声方差
    float  Pre_cov[4];

    Vector3f body_acce;
    Vector3f earth_acce;
}Kalman_Filter_t;

extern Kalman_Filter_t nav;
extern Kalman_Filter_t INS;
extern Kalman_Filter_t Flow_INS;
extern Kalman_Filter_t UWB_INS;
extern Kalman_Filter_t Radar_INS;

void kalman_filter_init(Kalman_Filter_t*Filter,float*R,float*Q,float*Pre_cov);
void  Kalman_Filter_updata_1(Kalman_Filter_t *Pilot,//惯性导航结构体
                             float Observation,     //位置观测量
                             float accel,           //系统原始驱动量，惯导加速度
                             float dt,              //系统积分步长
                             uint8_t *update_flag,  //数据更新标志
                             uint8_t Axis);          //坐标系，必须为（"X","Y","Z",）其中之一
void  Kalman_Filter_updata_2(Kalman_Filter_t *Pilot,//惯性导航结构体
                             float Vel_Observation, //速度观测量
                             float accel,           //系统原始驱动量，惯导加速度
                             float dt,              //系统积分步长
                             uint8_t *update_flag,  //数据更新标志
                             uint8_t Axis);          //坐标系，必须为（"X","Y","Z",）其中之一


void Strapdown_INS_Pos(Kalman_Filter_t *Pilot, //惯性导航结构体
                        float Observation,     //速度观测量
                        float accel,           //系统原始驱动量，惯导加速度
                        float dt,              //系统积分步长
                        uint16_t Pos_Delay_Cnt,//观测传感器延时量
                        uint8_t *update_flag,  //数据更新标志
                        uint8_t Axis);         //坐标系，必须为（"X","Y","Z",）其中之一


struct _1_ekf_filter
{
    float LastP;
    float Now_P;
    float out;
    float Kg;
    float Q;
    float R;
};

extern float kalman_1(struct _1_ekf_filter *ekf,float input);  //一维卡尔曼

void navigation_accel_update(Vector3f accBF);
Vector3f get_navigation_body_frame_acce(void);
Vector3f get_navigation_earth_frame_acce(void);
void position_estimation_update(float dt);



Vector3f get_position_neu_cm(void);  //返回当前位置
Vector3f get_velocity_neu_cms(void); //返回当前速度
float get_position_z_up_cm(void);    //返回当前高度
float get_velocity_z_up_cms(void);   //返回当前高度方向的速度
uint8_t get_pos_vel_type(void);

bool get_vision_health(void);
#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_NAVKALMAN_CL_NAVIGATIONKF_H_ */
