/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-29     CGY       the first version
 */
#include "CL_DigitalFilter.h"
#include "DL_OpticalFlow_LC302.h"
#include "DL_OpticalFlow_LC306.h"
#include "DL_OpticalFlow_T1-001-Plus.h"

Vector2f _opt_radians;
float    _opt_distance;
uint32_t last_opt_update_ms;

bool set_init_optical_flow_type(enum Opt_Type opt_type)
{
    //初始化刚设置的光流
    switch(opt_type) {
        case LC302:
            //优象光流模块LC302
        break;
        case LC306:
            //优象光流模块LC306
        break;
        case LC307:
            //优象光流模块LC307
        break;
        case T1_001_Plus:
            //优象光流模块T1_001_Plus
        break;

        default:
            return false;
        break;
    }
    return true;
}

Vector2f get_opt_radians(void)   //
{
    return _opt_radians;
}

float get_opt_distance(void)   //
{
    return _opt_distance;
}

bool get_opt_health(void)
{
    if(rt_tick_get() - last_opt_update_ms > 500){
        return  false;
    }
    else  return  true;
}

void update_optical_flow_data(enum Opt_Type new_opt_type,uint8_t data) //
{
    opt_data  opt={0};
    //循环读取光流模块的数据
    switch(new_opt_type) {
        case LC302:
            opt = LC302_opt_receive(data);  ;//优象光流模块LC302
        break;

        case LC306:
            //优象光流模块LC306
        break;

        case LC307:
            //优象光流模块LC307
        break;

        case T1_001_Plus:
            opt = T1_001_Plus_opt_receive(data);  //优象光流模块T1_001_Plus
        break;

        default:
            return ;
        break;
    }

    last_opt_update_ms = opt.last_update_ms;
    _opt_radians.x = opt.raw_x/1.0f;
    _opt_radians.y = opt.raw_y/1.0f;
    _opt_distance  = opt.distance/1.0f;
}

Vector2f opt_pos;
Vector2f opt_filter;
Vector2f gyr_filter;
Vector2f optflow_gyro_filter;
static uint8_t isOptFilterInit=0;
Vector2f opticalflow_rotate_complementary_filter(float dt) //光流角速度与陀螺仪角速度融合
{
    if(isOptFilterInit == 0) {
        isOptFilterInit = 1;
        biquadFilterInitLPF(&flow_LPF[0],1/dt,20);
        biquadFilterInitLPF(&flow_LPF[1],1/dt,20);
        biquadFilterInitLPF(&gyro_LPF[0],1/dt,4);
        biquadFilterInitLPF(&gyro_LPF[1],1/dt,4);
    }
    float RotateScale = 200.0f;
    Vector3f _gyro_deg = get_imu_sensorsGyro();

    opt_filter.x = biquadFilterApply(&flow_LPF[0],_opt_radians.x); //光流角速度rad/s
    opt_filter.y = biquadFilterApply(&flow_LPF[1],_opt_radians.y); //光流角速度rad/s

    gyr_filter.x = biquadFilterApply(&gyro_LPF[0],_gyro_deg.x); //角速度rad/s
    gyr_filter.y = biquadFilterApply(&gyro_LPF[1],_gyro_deg.y); //角速度rad/s

    optflow_gyro_filter.x = opt_filter.x/RotateScale - limit( gyr_filter.y/57.3f,-3,3); //用陀螺仪数据抵消光流在原地晃动的数据
    optflow_gyro_filter.y = opt_filter.y/RotateScale - limit(-gyr_filter.x/57.3f,-3,3); //用陀螺仪数据抵消光流在原地晃动的数据


    opt_pos.x += optflow_gyro_filter.x*limit(get_position_z_up_cm(), 10, 250)*dt;
    opt_pos.y += optflow_gyro_filter.y*limit(get_position_z_up_cm(), 10, 250)*dt;

    return opt_pos;  //补偿后的光流数据 rad/s
}

Vector2f t265_filter;
Vector2f gyro_filter;
Vector2f t265_gyro_filter;
static uint8_t isT265FilterInit=0;
Vector2f t265_rotate_complementary_filter(float dt) //双目速度与陀螺仪角速度融合
{
    if(isT265FilterInit == 0) {
        isT265FilterInit = 1;
        biquadFilterInitLPF(&t265_LPF[0],1/dt,25);
        biquadFilterInitLPF(&t265_LPF[1],1/dt,25);
        biquadFilterInitLPF(&t265_gyro_LPF[0],1/dt,5);
        biquadFilterInitLPF(&t265_gyro_LPF[1],1/dt,5);
    }
    float RotateScale = 6.5f;
    Vector3f _gyro_deg = get_imu_sensorsGyro();

    t265_filter.x = biquadFilterApply(&t265_LPF[0],t265_estimate.vel_x); //
    t265_filter.y = biquadFilterApply(&t265_LPF[1],t265_estimate.vel_y); //

//  t265_filter.x = biquadFilterApply(&t265_LPF[0],-t265_speed_estimate.x*100); //
//  t265_filter.y = biquadFilterApply(&t265_LPF[1],t265_speed_estimate.y*100); //

    gyro_filter.x = biquadFilterApply(&t265_gyro_LPF[0],_gyro_deg.x);  //角速度rad/s
    gyro_filter.y = biquadFilterApply(&t265_gyro_LPF[1],_gyro_deg.y);  //角速度rad/s

    t265_gyro_filter.x = t265_filter.x + limit( gyro_filter.y/57.3f,-3,3)*RotateScale; //用陀螺仪数据抵消光流在原地晃动的数据
    t265_gyro_filter.y = t265_filter.y + limit(-gyro_filter.x/57.3f,-3,3)*RotateScale; //用陀螺仪数据抵消光流在原地晃动的数据

    return t265_gyro_filter;  //补偿后的光流数据 rad/s
}


//调整传感器坐标系 使传感器坐标和飞控坐标对齐
void applySensorAlignment_2(int16_t * dest, int16_t * src, uint8_t rotation)
{
    const int16_t x = src[0];
    const int16_t y = src[1];

    switch (rotation) {
    default:
    case 0://不旋转
        dest[0] = x;
        dest[1] = y;
        break;
    case 1://顺时针旋转90度
        dest[0] =  -y;
        dest[1] =  x;
        break;
    case 2://顺时针旋转180度
        dest[0] = -x;
        dest[1] = -y;
        break;
    case 3://顺时针旋转270度
        dest[0] =  y;
        dest[1] = -x;
        break;
    }
}

