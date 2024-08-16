/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-01     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_ATTITUDECONTROL_CL_ATTITUDECTRL_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_ATTITUDECONTROL_CL_ATTITUDECTRL_H_

int attitude_rate_pid_controller_run(void);  //无人机角速度内环控制。
void input_euler_angle_roll_pitch_euler_rate_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds);

void attitude_set_throttle_out(float throttle_in, bool apply_angle_boost, float filter_cutoff);

#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_ATTITUDECONTROL_CL_ATTITUDECTRL_H_ */
