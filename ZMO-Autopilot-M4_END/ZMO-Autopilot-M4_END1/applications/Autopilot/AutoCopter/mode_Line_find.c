/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-27     CGY       the first version
 */
#include "mode_all.h"
#include "loco_config.h"



void line_find_mode_pid_par_init(void)
{

}

void mode_line_find_initialization(void)
{

}

void mode_line_find_run(void)
{
    float openmv_yaw;
    Vector2f bf_angles;
    //获取飞行员的摇杆期望,单位为角度.
    get_pilot_desired_lean_angles(&bf_angles.x,&bf_angles.y, 0);
    //返回飞行员的偏航摇杆的期望,单位度每秒。
    float target_yaw_rate =  get_pilot_desired_yaw_rate(get_channel_yaw_control_in());

    if (target_yaw_rate == 0) {
     openmv_yaw = 5*3.0f;
    }
    else openmv_yaw = target_yaw_rate;


    float roll_in = get_channel_roll_control_in();  //用于判断遥控器摇杆位置
    float pitc_in = get_channel_pitch_control_in(); //用于判断遥控器摇杆位置

    Vector2f openmv_angles; //光流定点控制的输出量
    if (roll_in==0 ||pitc_in==0) {
        openmv_angles.x = 5*0.2f;
        openmv_angles.x = limit(openmv_angles.x, -15, 15);

        if (1) {
            if(0)
            openmv_angles.y = 10-0 ;
        }
        else

        {
            openmv_angles.y = 0-0 ;


        }

    } else {
        openmv_angles.x = 0;
        openmv_angles.y = 0;
    }


    //摇杆量和光流定点控制量结合
    bf_angles.x += openmv_angles.x;
    bf_angles.y += openmv_angles.y;
    //外环角度控制器,输入飞行员的摇杆期望(单位:角度,偏航为角速度)
    input_euler_angle_roll_pitch_euler_rate_yaw(bf_angles.x,bf_angles.y,openmv_yaw);

    //得到飞行员爬升的速度期望。 get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(get_channel_thr_control_in());
    set_pos_target_z_from_climb_rate_cm(target_climb_rate);
    posittion_update_z_controller(); //高度控制器
}








