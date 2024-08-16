/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-29     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_MATH_CL_VECTOR_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_MATH_CL_VECTOR_H_

#include "rtthread.h"
#include "stdbool.h"
#include "math.h"
//三维 int16_t类型 数据
typedef union
{
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
    };
    int16_t axis[3];
}Vector3i16;

typedef union
{
    struct
    {
        int32_t x;
        int32_t y;
        int32_t z;
    };
    int16_t axis[3];
}Vector3i32;

typedef union
{
    struct
    {
        int64_t x;
        int64_t y;
        int64_t z;
    };
    int64_t axis[3];
}Vector3i64;

typedef union
{
    struct
    {
        float x;
        float y;
        float z;
    };
    float axis[3];
}Vector3f;

typedef struct
{
  float x;
  float y;
}Vector2f;

float length_v3(Vector3f *v);
float length_v2(Vector2f *v);


/*****************************************************************************/
typedef union
{
    struct
    {
        float E;//东
        float N;//北
        float Z;//天
    };
    float axis[3];
} Vector3f_Nav;

typedef struct
{
  int32_t x;
  int32_t y;
}Vector2i32;



//气压数据结构
typedef struct
{
    uint32_t timestamp;
    float pressure;
    float temperature;
    float altitude;
} baro_t;

//所有传感器数据集合
typedef struct
{
    Vector3f acce;                //加速度（G）
    Vector3f gyro;                //陀螺仪（deg/s）
    Vector3f magn;                //磁力计（gauss）
    baro_t baro;
} sensorData_t;

extern sensorData_t sensors;



/****************************************************************************/
//姿态角数据结构
typedef struct
{
    uint32_t timestamp;  /*时间戳*/
    float roll;
    float pitch;
    float yaw;
} attitude_t;

struct  vec3_s
{
    uint32_t timestamp;
    float x;
    float y;
    float z;
};

typedef struct vec3_s point_t;      //位置信息
typedef struct vec3_s velocity_t;   //速度信息
typedef struct vec3_s acce_t;        //加速度信息
//四轴姿态数据结构
typedef struct
{
    attitude_t  attitude;   //姿态角度（deg）
    point_t     position;   //估算的位置（cm）
    velocity_t  velocity;   //估算的速度（cm/s）
    acce_t      accel;       //估算的加速度（cm/ss）
} state_t;

typedef enum
{
    modeDisable = 0,
    modeAbs,
    modeVelocity
} mode_e;

typedef struct
{
    mode_e x;
    mode_e y;
    mode_e z;
    mode_e roll;
    mode_e pitch;
    mode_e yaw;
}mode_st;

//目标姿态数据结构
typedef struct
{
    attitude_t attitude;        //目标姿态角度（deg）
    attitude_t attitudeRate;    //目标角速度（deg/s）
    point_t position;           //目标位置（cm）
    velocity_t velocity;        //目标速度（cm/s）
    mode_st mode;
    float thrust;
} setpoint_t;

//控制数据结构
typedef struct
{
    float roll;
    float pitch;
    float yaw;
    float thrust;
} control_t;

/*******************************************************************/
enum
{
    rol_angle=0,
    pit_angle,
    yaw_angle,

    rol_rate,
    pit_rate,
    yaw_rate,

    x_position,
    y_position,
    z_position,
    x_velocity,
    y_velocity,
    z_velocity,

    x_posGps,
    x_velGps,
    y_posGps,
    y_velGps,

    x_posFlow,
    x_velFlow,
    y_posFlow,
    y_velFlow,

    Pid_Num
};



typedef enum{
    ArmPowerOFF  = 0x00,
    ArmPowerON  = 0x01,
}ArmPowerStatus_TypeDef;

typedef struct
{
    ArmPowerStatus_TypeDef  arm_power;
    int landFlag;
    bool LaunchFlag;
    bool ControlStart;
    bool Auto_WP_Flag;         //航点模式
    unsigned char poshold;    //设置目标位置标志位
    unsigned char setTargetPos;  //设置目标位置标志位
    int data_flag;
    int FlightMode;
}sensors_state_t;

extern sensors_state_t state;
#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_MATH_CL_VECTOR_H_ */
