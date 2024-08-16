/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-07-18     25984       the first version
 */

#include "loco_config.h"
#include "BlueTooth.h"
#include "stm32f4xx_hal_rng.h"
#include "drv_soft_i2c.h"

/**********************************************************************************************************************/

RNG_HandleTypeDef RNG_Handler;  //RNG句柄

uint8_t RNG_Init(void)                                   //初始化RNG
{
    uint16_t retry=0;

    RNG_Handler.Instance=RNG;
    HAL_RNG_Init(&RNG_Handler);//初始化RNG
    while(__HAL_RNG_GET_FLAG(&RNG_Handler,RNG_FLAG_DRDY)==RESET&&retry<1000)      //等待RNG准备就绪
    {
        retry++;
        //delay_us(10);
        rt_thread_delay(1);
    }
    if(retry>=1000) return 1;//随机数产生器工作不正常
    return 0;
}


void HAL_RNG_MspInit(RNG_HandleTypeDef *hrng)
{
     __HAL_RCC_RNG_CLK_ENABLE();//使能RNG时钟
}

uint32_t RNG_Get_RandomNum(void)                       //得到随机数
                                                      //返回值:获取到的随机数
{
    return HAL_RNG_GetRandomNumber(&RNG_Handler);
}


int RNG_Get_RandomRange(int min,int max)            //生成[min,max]范围的随机数
{
   return HAL_RNG_GetRandomNumber(&RNG_Handler)%(max-min+1) +min;
}

/*********************************************************************************************************************************/

void bluetooth_init(void)                           //蓝牙初始化
{
    uint8_t data1[32] = "AT+MASTER=04\r\n";
    uint8_t data2[32] = "AT+CONN=223450011112\r\n";
    rt_device_t dev = serial_device_find(BULETOOTH);
    if(dev )
    {
        rt_device_write(dev,0,data1,sizeof(data1)-1);
        rt_thread_delay(1000);
        rt_device_write(dev,0,data2,sizeof(data1)-1);
        rt_thread_delay(1000);
    }
}


void send_1(void)
{
    uint8_t dataToARM[] = "1";
    rt_device_t dev = serial_device_find(BULETOOTH);
    if(dev ) rt_device_write(dev,0,dataToARM,sizeof(dataToARM));
}


void send_2(void)
{
    uint8_t dataToARM[] = "2";
    rt_device_t dev = serial_device_find(BULETOOTH);
    if(dev ) rt_device_write(dev,0,dataToARM,sizeof(dataToARM));
}


void send_3(void)
{
    uint8_t dataToARM[] = "3";
    rt_device_t dev = serial_device_find(BULETOOTH);
    if(dev ) rt_device_write(dev,0,dataToARM,sizeof(dataToARM));
}

void send_0(void)                   //发送0，小车返回
{
    uint8_t dataToARM[] = "0";
    rt_device_t dev = serial_device_find(BULETOOTH);
    if(dev ) rt_device_write(dev,0,dataToARM,sizeof(dataToARM));
}
/******************************************************************************************************************/

