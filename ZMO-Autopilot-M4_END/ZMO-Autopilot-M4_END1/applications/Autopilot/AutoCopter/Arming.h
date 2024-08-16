/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-02     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_AUTOCOPTER_ARMING_H_
#define APPLICATIONS_AUTOPILOT_AUTOCOPTER_ARMING_H_

void rc_unlock_arming(void);  //摇杆控制解锁上锁
void one_button_takeoff_check(void);
uint8_t is_one_button_takeoff(void);//
uint8_t cmd_unlock_arming(void);  //摇杆控制解锁上锁
#endif /* APPLICATIONS_AUTOPILOT_AUTOCOPTER_ARMING_H_ */
