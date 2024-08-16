/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-05     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_AUTOCOPTER_RC_COPTER_H_
#define APPLICATIONS_AUTOPILOT_AUTOCOPTER_RC_COPTER_H_
#include "usb_transfer.h"
#include <rtthread.h>
#include "loco_config.h"
#include "CL_AHRS.h"
#include "mode_all.h"
#include "CL_RC_Channel.h"


uint16_t get_flight_modes(uint8_t position);
void read_mode_switch(void);
#endif /* APPLICATIONS_AUTOPILOT_AUTOCOPTER_RC_COPTER_H_ */
