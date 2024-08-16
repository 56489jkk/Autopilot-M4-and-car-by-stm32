/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-14     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_MATH_CL_MATHCTRL_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_MATH_CL_MATHCTRL_H_

#include "float.h"
#include "math.h"
#include "stdint.h"
#include "stdbool.h"
void update_vel_accel(float *vel, float accel, float dt, float limit, float vel_error);
void update_pos_vel_accel(float *pos, float *vel, float accel, float dt, float limit, float pos_error, float vel_error);
void shape_accel(float accel_input, float *accel,float jerk_max, float dt);
void shape_vel_accel(float vel_input, float accel_input,
                     float vel, float *accel,
                     float accel_min, float accel_max,
                     float jerk_max, float dt, bool limit_total_accel);


float sqrt_controller(float error, float p, float second_ord_lim, float dt);



#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_MATH_CL_MATHCTRL_H_ */
