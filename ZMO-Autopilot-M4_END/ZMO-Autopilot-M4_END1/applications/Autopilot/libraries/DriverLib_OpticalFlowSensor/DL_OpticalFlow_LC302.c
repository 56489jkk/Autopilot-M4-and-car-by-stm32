/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-29     CGY       the first version
 */
#include "DL_OpticalFlow_LC302.h"

opt_data LC302_opt_receive(uint8_t data)
{
    static opt_data T1_opt={0};
    static int16_t raw_data[2],dest_data[2];
    static uint8_t data_len = 0;
    static uint8_t state = 0;

    if(state==0&&data==0xFE)    //
    {
        state=1;
    }
    else if(state==1&&data==0x0A)   //
    {
        state=2;
        data_len = 12;
        T1_opt.dataLen = 0;
    }
    else if(state==2&&data_len>0)
    {
        data_len--;
        T1_opt.data[T1_opt.dataLen++]=data;
        if(data_len==0 && T1_opt.data[11]==0x55){
            state=0;
            T1_opt.sonar_timestamp = T1_opt.data[5]<<8|T1_opt.data[4];    //  20800us
            T1_opt.distance =  (T1_opt.data[7]<<8|T1_opt.data[6])/10;     //cm
            T1_opt.qual = T1_opt.data[8];
            if (T1_opt.qual==0xF5) {
                raw_data[0] =  ((int16_t)(T1_opt.data[1]<<8)|T1_opt.data[0]);   //向前
                raw_data[1] =  ((int16_t)(T1_opt.data[3]<<8)|T1_opt.data[2]);   //向右
                //调整传感器坐标系 使传感器坐标和飞控坐标对齐
                applySensorAlignment_2(dest_data,raw_data,Flow_Sensor_Rotation);
                T1_opt.raw_x = dest_data[0];   //向前
                T1_opt.raw_y = dest_data[1];   //向右
                T1_opt.last_update_ms = rt_tick_get();
            } else {
                T1_opt.raw_x = 0;   //
                T1_opt.raw_y = 0;
            }
        }
    }
    else
    {
        state=0;
    }

    return T1_opt;
}









