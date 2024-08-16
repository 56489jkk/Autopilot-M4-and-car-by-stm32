/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-28     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_MATH_CL_MATH_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_MATH_CL_MATH_H_
#include "float.h"
#include "math.h"
#include "stdint.h"
#include "stdbool.h"

#define LPF1_run_T 20       //低通滤波执行周期

#define QMF_SORT(type,a,b) { if ((a)>(b)) QMF_SWAP(type, (a),(b)); }
#define QMF_SWAP(type,a,b) { type temp=(a);(a)=(b);(b)=temp; }
int32_t quickMedianFilter3(int32_t * v);
int16_t quickMedianFilter3_16(int16_t * v);

/*-----------------------------------------------------------------*/

/* 低通滤波数据域 */
enum
{
    LPF_motor_hover ,
    LPF_altitude_speed,
    LPF_position_x_speed ,
    LPF_position_y_speed ,
    LPF_position_Z,
    LPF_all
};
typedef struct
{
    float input;
    float prev_out;
    float output;
    float radio;
}LPF_one;

extern LPF_one LPF_1[LPF_all];
/*-----------------------------------------------------------------*/

/* 数据监测数据域 */
typedef struct
{
    int8_t prev_data;
    int8_t now_data;
    int8_t capture_flag;
}Data_change_watch;

extern Data_change_watch task_watch;//监测任务是否是刚进入
extern Data_change_watch position_going_watch;//_——_
extern Data_change_watch height_going_watch;
int8_t data_watchdog(Data_change_watch *watch, bool input);


/*-----------------------------------------------------------------*/

/* 预减速加速数据域 */
typedef struct
{
    float speed;
    float different;
    float original_different;
    float speed_buff;
}Advance_speed;
extern Advance_speed advance_height;
extern Advance_speed advance_position_x;
extern Advance_speed advance_position_y;

void advance_speed_contral(float dt, Advance_speed *advance);
/*-----------------------------------------------------------------*/


typedef struct stdev_s
{
  float m_oldM, m_newM, m_oldS, m_newS;
  int m_n;
}stdev_t;

#define rate_do_excute(RATE_HZ, TICK) ((TICK % (1000 / RATE_HZ)) == 0)
#define Gravity_Acce  980.0f //当地重力加速度

#define PI              3.14159265358979323846f
#define RAD_PER_DEG     0.01745329251994f      //角度转化为弧度的系数   3.14/180
#define DEG_PER_RAD     57.29577951308232f     //弧度转化为角度的系数   180/3.14
// 2D vector length
#define sq(x) (float)((float)(x)*(float)(x))

float absolute(float x);
float max(float a, float b);
float min(float a, float b);
float limit(float data, float min, float max);
float square(float x);
float acos_approx(float x);

float atan2_approx(float y, float x);
float constrainf(float data, float low, float high);


void devClear(stdev_t *dev);
void devPush(stdev_t *dev, float x);
float devVariance(stdev_t *dev);
float devStandardDeviation(stdev_t *dev);
float data_to_deadzone(float x,float ref,float zoom);
float applyDeadband(float value, float deadband);
float LPF1(float dt, LPF_one *LPF_1);
void LPF1_Reset(LPF_one *LPF_1, int16_t input, float radio);
float sin_approx(float x);
float cos_approx(float x);
float sqrt_reciprocal(float number);//开平方的倒数
float my_sqrt(float number);
float fast_atan2(float y, float x);
float calc_lowpass_alpha(float dt, float cutoff_freq);

float pythagorous2(float a, float b);
/*
 * @brief: Check whether a float is zero
 */
static inline bool is_zero(const float x) {
    return fabsf(x) < FLT_EPSILON;
}
/*
 * @brief: Check whether a float is greater than zero
 */
static inline bool is_positive(const float x) {
    return x >= FLT_EPSILON;
}
/*
 * @brief: Check whether a float is less than zero
 */
static inline bool is_negative(const float x) {
    return x <= -1.0f*FLT_EPSILON;
}
float wrap_PI(const float radian);
float wrap_2PI(const float radian);
#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_MATH_CL_MATH_H_ */
