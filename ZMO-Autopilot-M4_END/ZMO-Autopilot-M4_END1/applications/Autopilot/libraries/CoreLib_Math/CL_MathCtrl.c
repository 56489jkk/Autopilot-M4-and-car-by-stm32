/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-14     CGY       the first version
 */
#include "CL_Math.h"
#include "string.h"
#include "math.h"
#include "string.h"
#include "stdio.h"
#include <stdbool.h>
#include <rtthread.h>
#include <rtdbg.h>
#include <rtdef.h>
#include "CL_MathCtrl.h"


// control default definitions
#define CORNER_ACCELERATION_RATIO   1.0/sqrt(2.0)   // acceleration reduction to enable zero overshoot corners

// update_vel_accel - single axis projection of velocity, vel, forwards in time based on a time step of dt and acceleration of accel.
// the velocity is not moved in the direction of limit if limit is not set to zero.
// limit - specifies if the system is unable to continue to accelerate.
// vel_error - specifies the direction of the velocity error used in limit handling.
void update_vel_accel(float *vel, float accel, float dt, float limit, float vel_error)
{
    const float delta_vel = accel * dt;
    // do not add delta_vel if it will increase the velocity error in the direction of limit
    if (!(is_positive(delta_vel * limit) && is_positive(vel_error * limit))){
        *vel += delta_vel;
    }
}

void update_pos_vel_accel(float *pos, float *vel, float accel, float dt, float limit, float pos_error, float vel_error)
{
    // move position and velocity forward by dt if it does not increase error when limited.
    float delta_pos = *vel * dt + accel * 0.5f * sq(dt);
    // do not add delta_pos if it will increase the velocity error in the direction of limit
    if (!(is_positive(delta_pos * limit) && is_positive(pos_error * limit))){
        *pos += delta_pos;
    }

    update_vel_accel(vel,accel,dt,limit,vel_error);
}



void shape_accel(float accel_input, float *accel,float jerk_max, float dt)
{
    // jerk limit acceleration change
    float accel_delta = accel_input - *accel;
    if (is_positive(jerk_max)) {
        accel_delta = limit(accel_delta, -jerk_max * dt, jerk_max * dt);
    }
    *accel += accel_delta;
}

void shape_vel_accel(float vel_input, float accel_input,
                     float vel, float *accel,
                     float accel_min, float accel_max,
                     float jerk_max, float dt, bool limit_total_accel)
{
    // sanity check accel_max
    if (!(is_negative(accel_min) && is_positive(accel_max))) {
       // INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return;
    }

    // velocity error to be corrected
    float vel_error = vel_input - vel;

    // Calculate time constants and limits to ensure stable operation
    // The direction of acceleration limit is the same as the velocity error.
    // This is because the velocity error is negative when slowing down while
    // closing a positive position error.
    float KPa;
    if (is_positive(vel_error)) {
        KPa = jerk_max / accel_max;
    } else {
        KPa = jerk_max / (-accel_min);
    }

    // acceleration to correct velocity
    float accel_target = sqrt_controller(vel_error, KPa, jerk_max, dt);

    // constrain correction acceleration from accel_min to accel_max
    accel_target = limit(accel_target, accel_min, accel_max);

    // velocity correction with input velocity
    accel_target += accel_input;

    // constrain total acceleration from accel_min to accel_max
    if (limit_total_accel) {
        accel_target = limit(accel_target, accel_min, accel_max);
    }

    shape_accel(accel_target,accel,jerk_max,dt);
}



// sqrt_controller calculates the correction based on a proportional controller with piecewise sqrt sections to constrain second derivative.
float sqrt_controller(float error, float p, float second_ord_lim, float dt)
{
    float correction_rate;
    if (is_negative(second_ord_lim) || is_zero(second_ord_lim)) {
        // second order limit is zero or negative.
        correction_rate = error * p;
    } else if (is_zero(p)) {
        // P term is zero but we have a second order limit.
        if (is_positive(error)) {
            correction_rate = my_sqrt(2.0 * second_ord_lim * (error));
        } else if (is_negative(error)) {
            correction_rate = -my_sqrt(2.0 * second_ord_lim * (-error));
        } else {
            correction_rate = 0.0;
        }
    } else {
        // Both the P and second order limit have been defined.
        const float linear_dist = second_ord_lim / sq(p);
        if (error > linear_dist) {
            correction_rate = my_sqrt(2.0 * second_ord_lim * (error - (linear_dist / 2.0)));
        } else if (error < -linear_dist) {
            correction_rate = -my_sqrt(2.0 * second_ord_lim * (-error - (linear_dist / 2.0)));
        } else {
            correction_rate = error * p;
        }
    }
    if (!is_zero(dt)) {
        // this ensures we do not get small oscillations by over shooting the error correction in the last time step.
        return limit(correction_rate, -fabsf(error) / dt, fabsf(error) / dt);
    } else {
        return correction_rate;
    }
}

