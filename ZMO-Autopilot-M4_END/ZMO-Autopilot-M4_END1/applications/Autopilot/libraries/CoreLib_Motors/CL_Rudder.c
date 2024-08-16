/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-04-26     Administrator       the first version
 */
#include "usb_transfer.h"
#include "CL_Rudder.h"
#include "loco_config.h"

#define P 2                              //时钟系数
#define hz_50        0.02*1000000000/P   //1/50=0.02s
#define pulse_1ms    1*1000000/P         //1ms
#define pulse_1_5ms  1.5*1000000/P       //1.5ms
#define pulse_2ms    2*1000000/P         //2ms

struct rt_device_pwm *rudder_dev;      /* PWM设备句柄 */
int rudder_pwm_output_init(void)
{
    /* 查找设备 */
    rudder_dev = (struct rt_device_pwm *)rt_device_find(PWM4_DEV_NAME);
    if (rudder_dev == RT_NULL)
    {
        rt_kprintf("pwm sample run failed! can't find %s device!\n", PWM4_DEV_NAME);
        return 0;
    }

    /* 设置PWM周期和脉冲宽度默认值 */
    rt_pwm_set(rudder_dev, 1, hz_50, pulse_1ms);
    rt_pwm_set(rudder_dev, 2, hz_50, pulse_1ms);
    rt_pwm_set(rudder_dev, 3, hz_50, pulse_1ms);
    rt_pwm_set(rudder_dev, 4, hz_50, pulse_1ms);

    /*使能设备*/
    rt_pwm_enable(rudder_dev, 1);
    rt_pwm_enable(rudder_dev, 2);
    rt_pwm_enable(rudder_dev, 3);
    rt_pwm_enable(rudder_dev, 4);

    return 0;
}
/* 导出到自动初始化 */
INIT_APP_EXPORT(rudder_pwm_output_init);

// 1000us<=m<=2500us
void ctrl_Rudder_PWM(int m1,int m2,int m3)
{
    if(rudder_dev == RT_NULL)  return ;       //没有设备时跳出

        rt_pwm_set(rudder_dev, 1, hz_50, m1*1000/P);
        rt_pwm_set(rudder_dev, 2, hz_50, m2*1000/P);
        rt_pwm_set(rudder_dev, 3, hz_50, m3*1000/P);
}

// off_on: 0(out low)     1(out high)
void ctrl_Laser_pointer_PWM(int off_on)
{
    if(rudder_dev == RT_NULL)  return ;       //没有设备时跳出
        rt_pwm_set(rudder_dev, 4, hz_50,off_on*hz_50);
}






