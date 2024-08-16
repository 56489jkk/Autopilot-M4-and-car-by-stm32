/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-19     CGY       the first version
 */
#include "DL_Compass_ist8310.h"

Vector3i16 magn_raw_adc;    //陀螺仪原始ADC数据
Vector3f magn_adc;          //转换单位为角度每秒的数据
Vector3f _imu_magn;

/*从队列读取陀螺数据*/
Vector3f get_imu_sensorsMagn(void)  //为原始数据经第一次低通滤波后的角速度数据 deg/s
{
    return _imu_magn;
}

/*********************************************************
*@brief   传感器的地磁场数据读取及处理
*@param[in] instance==0时，就是主传感器。
*********************************************************/
void magn_data_update(Vector3f *magn,uint8_t instance)
{
    ist8310_get_magn_raw(&magn_raw_adc);

    magn_adc.x = magn_raw_adc.x * 1.0f;
    magn_adc.y = magn_raw_adc.y * 1.0f;
    magn_adc.z = magn_raw_adc.z * 1.0f;

    _imu_magn = magn_adc;

    *magn = magn_adc;//返回一个健康imu传感器的数据
}





