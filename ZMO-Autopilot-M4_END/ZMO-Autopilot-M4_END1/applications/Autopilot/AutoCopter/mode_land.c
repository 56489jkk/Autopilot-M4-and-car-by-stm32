/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-13     CGY       the first version
 */
#include "loco_config.h"
#include "mode_all.h"
#include "CL_Math.h"
#include "CL_AttitudeCtrl.h"
void mode_land_initialization() //初始化飞行参数
{
    if (!is_active_z_ctrl()) {
        init_posittion_z_controller();
    }

    //if (!is_active_xy_ctrl())
    {
        init_posittion_xy_controller();
    }

    const Vector3f current_pos = get_position_neu_cm();

    set_waypoint_destination(&current_pos,false); //设置航点目的地
}


/**
  * @brief 降落模式   （有定位数据时首先定点降落，否则定高降落）
  */
void mode_land_run()
{
    static float down_vel = 0;
    float target_roll, target_pitch;
    float wp_roll1, wp_pitch1;

    float roll_in1 = get_channel_roll_control_in();  //用于判断遥控器摇杆位置
    float pitc_in1 = get_channel_pitch_control_in(); //用于判断遥控器摇杆位置

    //获取飞行员的摇杆期望,单位为角度.
    get_pilot_desired_lean_angles(&target_roll,&target_pitch, 0);

    //返回飞行员的偏航摇杆的期望,单位度每秒.
    float target_yaw_rate =  get_pilot_desired_yaw_rate(get_channel_yaw_control_in());
    //得到飞行员爬升的速度期望。 get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(get_channel_thr_control_in());

    float thr_in = get_channel_thr_control_in();
    if(is_zero(thr_in)) {
        down_vel = -25;
    }
    else down_vel = 0;

    target_climb_rate += down_vel;

    set_pos_target_z_from_climb_rate_cm(target_climb_rate);

    //run waypoint controller
    update_waypoint_navigation();

    if ((roll_in1 != 0) || (pitc_in1 != 0)) {
        wp_roll1  = 0;
        wp_pitch1 = 0;
    }
    else
    {
        wp_roll1  = get_wp_nav_roll();
        wp_pitch1 = get_wp_nav_pitch();
    }

    //摇杆量和导航控制量结合
    target_roll  += wp_roll1;
    target_pitch += wp_pitch1;

    //外环角度控制器,输入飞行员的摇杆期望(单位:角度,偏航为角速度)
    input_euler_angle_roll_pitch_euler_rate_yaw(target_roll,target_pitch,target_yaw_rate);
    posittion_update_z_controller(); //高度控制器
}
