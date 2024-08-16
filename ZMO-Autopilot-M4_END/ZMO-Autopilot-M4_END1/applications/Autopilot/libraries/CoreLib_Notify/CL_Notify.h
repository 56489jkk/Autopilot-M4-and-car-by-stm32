/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-12     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_NOTIFY_CL_NOTIFY_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_NOTIFY_CL_NOTIFY_H_


#include <board.h>
#include "CL_Vector.h"
#include <rtdbg.h>

#define BIT_RLED 0x01       //红色
#define BIT_GLED 0x02       //绿色
#define BIT_BLED 0x04       //蓝色
#define BIT_WLED 0x07       //白色
#define BIT_PLED 0x05       //紫色
#define BIT_YLED 0x03       //黄色
#define BIT_CLED 0x06       //青色
#define safe_div(numerator,denominator,safe_value) ( (denominator == 0)? (safe_value) : ((numerator)/(denominator)) )

int led_init(void);
void LED_1ms_DRV(void); //0~20
void led_state_update(uint8_t dT_ms);

#define LED1_ON       rt_pin_write(LED_R_PIN, GPIO_LED_ON);
#define LED1_OFF      rt_pin_write(LED_R_PIN, !GPIO_LED_ON);
#define LED2_ON       rt_pin_write(LED_G_PIN, GPIO_LED_ON);
#define LED2_OFF      rt_pin_write(LED_G_PIN, !GPIO_LED_ON);
#define LED3_ON       rt_pin_write(LED_B_PIN, GPIO_LED_ON);
#define LED3_OFF      rt_pin_write(LED_B_PIN, !GPIO_LED_ON);


void cpu_usage_get(rt_uint8_t *major, rt_uint8_t *minor);
int cpu_usage_init(void);

#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_NOTIFY_CL_NOTIFY_H_ */
