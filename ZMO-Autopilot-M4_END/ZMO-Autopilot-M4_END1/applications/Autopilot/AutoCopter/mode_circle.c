/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-15     CGY       the first version
 */
#include "mode_all.h"
#include "loco_config.h"
#include "mode_all.h"
#include "stdbool.h"
#include "math.h"
// Circle
bool pilot_yaw_override = false; // true if pilot is overriding yaw
bool speed_changing = false;     // true when the roll stick is being held to facilitate stopping at 0 rate
void circle_mode_pid_par_init()
{
    pidInit(&pid[z_position],1.0f,0.0f,0.00f,100,100,100,0.005f,true,30);
    pidInit(&pid[z_velocity],3.1f,1.0f,0.01f,150,200,400,0.005f,true,50);
}

void mode_circle_initialization() //初始化飞行参数
{
    if (!is_active_z_ctrl()) {
        init_posittion_z_controller();
    }


    pilot_yaw_override = false; // true if pilot is overriding yaw
    speed_changing = false;

}

void mode_circle_run() //定高模式  （飞行员控制飞行器角度、油门中位时高度保持）//200hz
{
    float target_roll, target_pitch;
    //获取飞行员的摇杆期望,单位为角度.
    get_pilot_desired_lean_angles(&target_roll,&target_pitch,0);
    //返回飞行员的偏航摇杆的期望,单位度每秒.
    float target_yaw_rate =  get_pilot_desired_yaw_rate(get_channel_yaw_control_in());
    if(!is_zero(target_yaw_rate)) {
        pilot_yaw_override = true;
    }
    //得到飞行员爬升的速度期望。 get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(get_channel_thr_control_in());
    set_pos_target_z_from_climb_rate_cm(target_climb_rate);

    //call attitude controller
    if(pilot_yaw_override) {
        //外环角度控制器,输入飞行员的摇杆期望(单位:角度,偏航为角速度)
        input_euler_angle_roll_pitch_euler_rate_yaw(target_roll,target_pitch,target_yaw_rate);
    } else {
        //外环角度控制器,输入飞行员的摇杆期望(单位:角度,偏航为角速度)
        input_euler_angle_roll_pitch_euler_rate_yaw(target_roll,target_pitch,target_yaw_rate);
    }
    posittion_update_z_controller(); //高度控制器
}



