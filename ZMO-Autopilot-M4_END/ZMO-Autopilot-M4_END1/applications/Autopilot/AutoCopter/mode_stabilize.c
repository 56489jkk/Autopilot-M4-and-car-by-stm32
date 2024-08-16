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
#include "parameter.h"
/**
 * @brief  角速度环和角度环控制的pid参数初始化
 */
void stabilize_mode_pid_par_init()
{
    //角速度环pid参数初始化
    pidInit(&pid[rol_rate],Parame.set.kp_[rol_rate],Parame.set.ki_[rol_rate],Parame.set.kd_[rol_rate],150,100,500,0.002f,true,20);
    pidInit(&pid[pit_rate],Parame.set.kp_[pit_rate],Parame.set.ki_[pit_rate],Parame.set.kd_[pit_rate],150,100,500,0.002f,true,20);
    pidInit(&pid[yaw_rate],Parame.set.kp_[yaw_rate],Parame.set.ki_[yaw_rate],Parame.set.kd_[yaw_rate],200,100,500,0.002f,true,30);

    //角度环pid参数初始化
    pidInit(&pid[rol_angle],Parame.set.kp_[rol_angle],Parame.set.ki_[rol_angle],Parame.set.kd_[rol_angle],30,10,200,0.005f,true,20);
    pidInit(&pid[pit_angle],Parame.set.kp_[pit_angle],Parame.set.ki_[pit_angle],Parame.set.kd_[pit_angle],30,10,200,0.005f,true,20);
    pidInit(&pid[yaw_angle],Parame.set.kp_[yaw_angle],Parame.set.ki_[yaw_angle],Parame.set.kd_[yaw_angle],30,10,200,0.005f,true,20);
}

/**
 * @brief 切换到此模式时进行一次初始化
 */
void mode_stabilize_initialization() //初始化飞行参数
{

}

/**
 * @brief 自稳模式控制
 */
void mode_stabilize_run() //自稳模式  (飞行员控制飞行器角度,手动控制油门)
{
    float target_roll=0, target_pitch=0;
    //获取飞行员的摇杆期望,单位为角度.
    get_pilot_desired_lean_angles(&target_roll,&target_pitch,0);
    //返回飞行员的偏航摇杆的期望,单位度每秒.
    float target_yaw_rate =  get_pilot_desired_yaw_rate(get_channel_yaw_control_in());

    float throttle_in = get_pilot_desired_throttle()*1000; //获取摇杆的油门输入量

    //外环角度控制器,输入飞行员的摇杆期望(单位:角度,偏航为角速度)
    input_euler_angle_roll_pitch_euler_rate_yaw(target_roll,target_pitch,target_yaw_rate);
    attitude_set_throttle_out(throttle_in,0,0); //设置油门输出
}









