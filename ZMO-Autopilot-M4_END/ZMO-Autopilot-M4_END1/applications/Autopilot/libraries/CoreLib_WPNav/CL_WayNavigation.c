/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-11     CGY       the first version
 */

#include "loco_config.h"
#include "arming.h"
#include "CL_Vector.h"
#include "CL_WayNavigation.h"


bool set_waypoint_destination(const Vector3f* destination, bool terrain_alt) //设置航点目的地
{
    // re-initialise if previous destination has been interrupted
    if (!is_WpNav_active() || !wp_flags.reached_destination) {
       // wp_and_spline_init(_wp_desired_speed_xy_cms);
    }
    // use previous destination as origin
    _wp.origin = _wp.destination;

    _terrain_alt = terrain_alt;
    _wp.destination = * destination;


    wp_flags.fast_waypoint = false;   // default waypoint back to slow
    wp_flags.reached_destination = false;

    return  0;
}

float get_plus_or_minus(float data)
{
    if (data > 1.0e-7) {
        return   1.0f;
    }
    else if (data<-1.0e-7) {
        return   -1.0f;
    }
    else{
        return   0.0f;
    }
}

/// advance_wp_target_along_track - move target location along track from origin to destination
bool advance_waypoint_target_along_track(float dt)
{
    bool s_finished=1;
    //target position, velocity and acceleration from straight line or spline calculators
    Vector3f target_pos, target_vel, target_accel;

    const Vector3f current_pos = get_position_neu_cm();

    target_pos.x = _wp.destination.x;
    target_pos.y = _wp.destination.y;

    target_vel.x = target_vel.y = 0;  //期望速度 cm/s

    //将新目标传递给位置控制器
    set_pos_vel_accel_for_pos_ctrl(target_pos,target_vel,target_accel);

    //检查一下我们是否到达了目标航点
    if (!wp_flags.reached_destination) {
        if (s_finished) {
            //"fast" waypoints are complete once the intermediate point reaches the destination
            if (wp_flags.fast_waypoint) {
                wp_flags.reached_destination = true;
            } else {
                //regular waypoints also require the copter to be within the waypoint radius
                const float dist_to_dest_x = current_pos.x - _wp.destination.x;
                const float dist_to_dest_y = current_pos.y - _wp.destination.y;
                if (absolute(dist_to_dest_x) <= 8 && absolute(dist_to_dest_y) <= 8) {
                    wp_flags.reached_destination = true;
                }
            }
        }
    }

    //成功的沿目标航线前进
    return true;
}

bool update_waypoint_navigation()
{
    if (!wp_flags.reached_destination) {
        pid[x_position].Kp =  pid[y_position].Kp  = 0.4f*0.8f;  //走航线过程中适当调节速度
        pid[x_position].Ki =  pid[y_position].Ki  = 0.04f;  //走航线过程中适当调节速度
    }
    else
    {
        pid[x_position].Kp =  pid[y_position].Kp  = 0.4f*1.0f;
        pid[x_position].Ki =  pid[y_position].Ki  = 0.01f;  //走航线过程中适当调节速度
    }

    bool ret = true;
    // advance the target if necessary
    if (!advance_waypoint_target_along_track(0)) {
        // To-Do: handle inability to advance along track (probably because of missing terrain data)
        ret = false;
    }

    posittion_update_xy_controller(); //水平位置控制器

    _wp.last_update_ms = rt_tick_get();
    return  ret;
}




bool is_reached_waypoint_destination()  //是否到达航点
{
    return  wp_flags.reached_destination;
}
// returns true if update_wpnav has been run very recently
bool is_WpNav_active()
{
    return (rt_tick_get() - _wp.last_update_ms) < 200;
}

