/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-12-18     Administrator       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_AUTOCOPTER_GCS_COPTER_H_
#define APPLICATIONS_AUTOPILOT_AUTOCOPTER_GCS_COPTER_H_
#include "mavlink.h"
#include "mode_all.h"

#define MAV_CMD_CAN_FORWARD 32000


MAV_MODE Copter_base_mode(void);

mavlink_set_position_target_local_ned_t  mavlink_set_target;

void Copter_handle_Message(mavlink_message_t msg);

#endif /* APPLICATIONS_AUTOPILOT_AUTOCOPTER_GCS_COPTER_H_ */
