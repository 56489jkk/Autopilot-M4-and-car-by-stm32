/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-29     CGY       the first version
 */
#include "DL_TFmini.h"
#include "loco_config.h"

tfmini tfdata;
int _tof_strength;
int32_t last_tof_update_ms;

int get_tof_strength()
{
    return _tof_strength;
}
bool get_tof_health()
{
    if(rt_tick_get() - last_tof_update_ms > 100){
        return  false;
    }
    else  return  true;
}


void TFmini_Statemachine(uint8_t data)
{
    static uint8_t data_len = 0;
    static uint8_t state = 0;

    if(state==0&&data==0x59)       //帧头
    {
        state=1;
    }
    else if(state==1&&data==0x59)  //帧头֡
    {
        state=2;
        data_len = 7;              //帧长    一个校验值（帧长的字节异或值)  + 一个包尾(0x55)
        tfdata.dataLen = 0;
    }
    else if(state==2&&data_len>0)
    {
        data_len--;
        tfdata.data[tfdata.dataLen++] = data;
        if(data_len==0)           //数据长度减到零
        {
            state=0;
            uint16_t Strength = (((uint16_t)tfdata.data[3]) << 8) + tfdata.data[2];
            _tof_strength = Strength;
            if(Strength>450 && Strength<60000) //光强范围正常
            {
                int16_t distance = ((int16_t)(tfdata.data[1]<<8)|tfdata.data[0]);//*rMat[2][2];   //单位 cm
                if (distance>=2) {
                    tfdata.distance = distance/1.0f;//*rMat[2][2];   //单位 cm
                    last_tof_update_ms = rt_tick_get();
                }
            }
        }
    }
    else
    {
        state=0;
    }
}



