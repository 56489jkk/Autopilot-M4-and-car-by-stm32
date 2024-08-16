/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-03-08     CGY       the first version
 */
#ifndef APPLICATIONS_INCLUDE_PARAMETER_H_
#define APPLICATIONS_INCLUDE_PARAMETER_H_

#include "rtthread.h"
#include <stdio.h>
#include "CL_Vector.h"
#define VERSION  110  /*110 表示V11.0*/
#define END_DATA 65535  /*110 表示V11.0*/
struct Parameter_s
{
    float frist_init;   //飞控第一次初始化，flash内部并没有参数，需要把初始参数写入
    float kp_[Pid_Num]; //PID的比例系数参数
    float ki_[Pid_Num]; //PID的积分系数参数
    float kd_[Pid_Num]; //PID的微分系数参数
    float AcceZero[3];  //加速度计零偏
    float MangZero[3];  //磁力计零偏
    float mag_gain[3];  //磁力计校正比例
    float ESC_Calibration_Flag;//电调校准标志
    float final_init;   //飞控flash内部末尾一位的数据
};

union Parameter
{
    //这里使用联合体，长度是4KByte，联合体内部是一个结构体，该结构体内是需要保存的参数
    struct Parameter_s set;
    float  byte[512];
};

extern union Parameter Parame;

/*PID参数初始化*/
void All_PID_Init(void);
void Parame_Read_From_Flash(void);
void Data_Save_To_Flsh(void);
typedef struct
{
    rt_uint8_t save_en;
    rt_uint8_t save_trig;
    rt_uint16_t time_delay;
}parameter_state ;

extern parameter_state ParState;
extern rt_uint32_t semaphoreGiveTime;

void Parame_Write_Task(void);//
void parame_read_from_Flash(void);
void PID_Parameter_Rest(void);
/*PID初始化*/
void All_PID_Init(void);
void data_save_to_Flsh(void);
#endif /* APPLICATIONS_INCLUDE_PARAMETER_H_ */
