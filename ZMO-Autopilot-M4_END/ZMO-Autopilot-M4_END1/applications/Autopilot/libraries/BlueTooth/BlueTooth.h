/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-07-18     25984       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_BLUETOOTH_BLUETOOTH_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_BLUETOOTH_BLUETOOTH_H_
#include "stm32f4xx_hal_rng.h"
#define BULETOOTH  2


void bluetooth_init(void);                              //蓝牙初始化

void send_0(void);
void send_1(void);
void send_2(void);
void send_3(void);

uint8_t RNG_Init(void);                            //RNG初始化

uint32_t RNG_Get_RandomNum(void);                //得到随机数;返回值:获取到的随机数
int RNG_Get_RandomRange(int min,int max);        //生成[min,max]范围的随机数

#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_BLUETOOTH_BLUETOOTH_H_ */


