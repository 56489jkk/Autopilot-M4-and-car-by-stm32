/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-12-30     CGY       the first version
 */
#include "loco_config.h"
#include "DL_ultrasonic.h"

Ultrason_data ultr;   //超声波数据结构体
/**************************KS103超声波初始化和数据解析**************************************/
void ultra_start_work(void)
{
    if(ultr.name == 0x01)  //超声波的类型 0x01为ks103超声波
    {
        ultr.cmd[0]=0xe8;
        ultr.cmd[1]=0x02;
        ultr.cmd[2]=0xb0;

        rt_device_t dev = serial_device_find(SerialProtocol_KS103);
        if(dev) rt_device_write(dev,0,ultr.cmd,3);
        ultr.cmd[3]=1;  //开始测距标志位
    }
}

void ultrasonic_receive(uint8_t com_data)
{
    static uint8_t ultra_tmp;

    if(ultr.cmd[3] == 1)
    {
        ultra_tmp = com_data;
        ultr.cmd[3] = 2;
    }
    else if(ultr.cmd[3] == 2)
    {
        float distance = ((ultra_tmp<<8) + com_data)/10.0f*1; //化成厘米为单位并补偿倾斜角;
        if (distance>1&&distance<500) {

            ultr.distance = distance;
        }
        ultr.cmd[3] = 0;
        ultr.health = 1;
    }
}





