/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-01     CGY       the first version
 */

#include "mode_all.h"
#include "loco_config.h"
#include "mode_all.h"
#include "stdbool.h"
#include "math.h"


void position_hold_mode_pid_par_init()
{
    pidInit(&pid[x_velocity],3.3f,0.8f,0.0f,200,100,400,0.005f,true,20.0f);   //
    pidInit(&pid[y_velocity],3.3f,0.8f,0.0f,200,100,400,0.005f,true,20.0f);   //p 4.0     d 0.1

    pidInit(&pid[x_position],1.0f,0.00f,0.0f,150,80,200,0.005f,true,20.0f);   //
    pidInit(&pid[y_position],1.0f,0.00f,0.0f,150,80,200,0.005f,true,20.0f);   //p 4.0     d 0.1
}

static int lock=0;
void mode_position_hold_initialization() //初始化飞行参数
{
    lock = 0;
    pid_reset(&pid[x_position]);
    pid_reset(&pid[x_velocity]);
    pid_reset(&pid[y_position]);
    pid_reset(&pid[y_velocity]);
    if (!is_active_z_ctrl()) {
        init_posittion_z_controller();
    }
    if (!is_active_xy_ctrl()) {
        init_posittion_xy_controller();
    }
}

void mode_position_hold_run() //定点模式  (飞行员控制)
{
    float target_roll, target_pitch;
    float wp_roll, wp_pitch;
    float roll_in = get_channel_roll_control_in();  //用于判断遥控器摇杆位置
    float pitc_in = get_channel_pitch_control_in(); //用于判断遥控器摇杆位置
    //获取飞行员的摇杆期望,单位为角度.
    get_pilot_desired_lean_angles(&target_roll,&target_pitch, 0);
    //返回飞行员的偏航摇杆的期望,单位度每秒.
    float target_yaw_rate =  get_pilot_desired_yaw_rate(get_channel_yaw_control_in());

    //得到飞行员爬升的速度期望。 get pilot desired climb rate z轴方向上的控制
    float target_climb_rate = get_pilot_desired_climb_rate(get_channel_thr_control_in());
    set_pos_target_z_from_climb_rate_cm(target_climb_rate);

    static Vector3f target_pos={0}, target_vel={0}, target_accel={0};
    const Vector3f current_pos = get_position_neu_cm();
    if (lock==0) {
        lock = 1;
        target_pos = current_pos;
    }

    if ((roll_in != 0) || (pitc_in != 0)) {

        if(get_pos_vel_type() == 2)//机体坐标下
        {
            target_pos.x = target_pos.x-target_pitch*5*0.01;
            target_pos.y = target_pos.y+target_roll*5*0.01;
        }
        else {
            target_pos.x = target_pos.x-target_pitch*5*0.01;
            target_pos.y = target_pos.y-target_roll*5*0.01;
        }

//        target_vel.y = target_roll*6;
//        target_vel.x = target_pitch*6;  //期望速度 cm/s
    }
    else {
        target_vel.x = 0;
        target_vel.y = 0;  //期望速度 cm/s
    }

    //将新目标传递给位置控制器
    set_pos_vel_accel_for_pos_ctrl(target_pos,target_vel,target_accel);
    posittion_update_xy_controller(); //水平位置控制器

    wp_roll  = get_wp_nav_roll();
    wp_pitch = get_wp_nav_pitch();

    //外环角度控制器,输入飞行员的摇杆期望(单位:角度,偏航为角速度)
    input_euler_angle_roll_pitch_euler_rate_yaw(wp_roll,wp_pitch,target_yaw_rate);
    posittion_update_z_controller(); //高度控制器
}





