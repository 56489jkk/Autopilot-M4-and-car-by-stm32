/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-03-02     Administrator       the first version
 */
#include "CL_Circle.h"
#include "CL_Math.h"
#include "CL_WayNavigation.h"
// degrees -> radians
float ToRad(float deg)
{
    return deg * RAD_PER_DEG;
}
float safe_sqrt(const float  v)
{
    float ret = sqrtf(v);
    if (isnan(ret)) {
        return 0;
    }
    return ret;
}
// return bearing in centi-degrees between two positions
float get_bearing_cd(Vector3f origin,Vector3f destination)
{
    float bearing = atan2f(destination.y-origin.y, destination.x-origin.x);
    if (bearing < 0) {
        bearing += 360.0f;
    }
    return bearing;
}
// calc_velocities - calculate angular velocity max and acceleration based on radius and rate
//      this should be called whenever the radius or rate are changed
//      initialises the yaw and current position around the circle
void circle_calc_velocities(bool init_velocity)
{
    // if we are doing a panorama set the circle_angle to the current heading
    if (_radius <= 0) {
        _angular_vel_max = ToRad(_rate);
        _angular_accel =  max(fabsf(_angular_vel_max),ToRad(AC_CIRCLE_ANGULAR_ACCEL_MIN));  // reach maximum yaw velocity in 1 second
    }else{
        // calculate max velocity based on waypoint speed ensuring we do not use more than half our max acceleration for accelerating towards the center of the circle
        float velocity_max = min(100, safe_sqrt(0.5f*50*_radius));

        // angular_velocity in radians per second
        _angular_vel_max = velocity_max/_radius;
        _angular_vel_max = constrainf(ToRad(_rate),-_angular_vel_max,_angular_vel_max);

        // angular_velocity in radians per second
        _angular_accel = max(50/_radius, ToRad(AC_CIRCLE_ANGULAR_ACCEL_MIN));
    }

    // initialise angular velocity
    if (init_velocity) {
        _angular_vel = 0;
    }
}
void init_start_angle(bool use_heading)
{
    // initialise angle total
    _angle_total = 0;

    // if the radius is zero we are doing panorama so init angle to the current heading
    if (_radius <= 0) {
        _angle = 0;
        return;
    }

//    // if use_heading is true
//    if (use_heading) {
//        _angle = wrap_PI(_ahrs.yaw-M_PI);
//    } else {
//        // if we are exactly at the center of the circle, init angle to directly behind vehicle (so vehicle will backup but not change heading)
//        const Vector3f &curr_pos = _inav.get_position_neu_cm();
//        if (is_equal(curr_pos.x,float(_center.x)) && is_equal(curr_pos.y,float(_center.y))) {
//            _angle = wrap_PI(_ahrs.yaw-M_PI);
//        } else {
//            // get bearing from circle center to vehicle in radians
//            float bearing_rad = atan2f(curr_pos.y-_center.y,curr_pos.x-_center.x);
//            _angle = wrap_PI(bearing_rad);
//        }
//    }
}
void circle_init()
{
    // initialize radius from params
    _radius = _radius_parm;
    _last_radius_param = _radius_parm;

    // initialise position controller (sets target roll angle, pitch angle and I terms based on vehicle current lean angles)
//    _pos_control.init_xy_controller_stopping_point();
//    _pos_control.init_z_controller_stopping_point();

    if (0) {
        _center.x += _radius * 1;
        _center.y += _radius * 1;
    }
    _terrain_alt = false;

    // calculate velocities
    circle_calc_velocities(true);

    // set starting angle from vehicle heading
    init_start_angle(true);
}

/// update - update circle controller
bool update_circle_controller(float climb_rate_cms)
{
    circle_calc_velocities(false);
    // calculate dt
    const float dt = 0.05f;

    // ramp angular velocity to maximum
    if (_angular_vel < _angular_vel_max) {
        _angular_vel += fabsf(_angular_accel) * dt;
        _angular_vel = min(_angular_vel, _angular_vel_max);
    }
    if (_angular_vel > _angular_vel_max) {
        _angular_vel -= fabsf(_angular_accel) * dt;
        _angular_vel = max(_angular_vel, _angular_vel_max);
    }

    // update the target angle and total angle traveled
    float angle_change = _angular_vel * dt;
    _angle += angle_change;
    _angle = wrap_PI(_angle);
    _angle_total += angle_change;

    // if the circle_radius is zero we are doing panorama so no need to update loiter target
    Vector3f target ;
    if (!is_zero(_radius)) {
        // calculate target position
        target.x += _radius * cosf(-_angle);
        target.y += - _radius * sinf(-_angle);

        // heading is from vehicle to center of circle
        _yaw = get_bearing_cd(get_position_neu_cm(), _center);

    } else {
        // heading is same as _angle but converted to centi-degrees
        _yaw = _angle * 5729.57795f;
    }
    Vector3f zero;
    //将新目标传递给位置控制器
    set_pos_vel_accel_for_pos_ctrl(target,zero,zero);
    set_pos_target_z_from_climb_rate_cm(climb_rate_cms);
    posittion_update_xy_controller(); //水平位置控制器

    _last_update_ms = rt_tick_get();
    return 0;
}

float get_circle_yaw()
{
    return _yaw;
}
