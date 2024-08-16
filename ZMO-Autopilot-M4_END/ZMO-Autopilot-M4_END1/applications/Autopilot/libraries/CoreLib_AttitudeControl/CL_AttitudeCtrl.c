/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-01     CGY       the first version
 */

#include "loco_config.h"
#include "arming.h"
#include "mode_all.h"

/**
 * @brief 飞行器角速度控制器，并把控制量送入电机控制
 */
int attitude_rate_pid_controller_run()  //无人机角速度内环pid控制。
{
//    if (rc_ppm_in[7]>1800) {
//        pid[rol_rate].Kp = (float)(rc_ppm_in[5]-1000)/1000.0f*2.0f+1.0f;
//        pid[pit_rate].Kp = (float)(rc_ppm_in[5]-1000)/1000.0f*2.0f+1.0f;
//    }

    rc_unlock_arming();                             //摇杆控制解锁上锁
    Vector3f angle_vel_body;                        //角速度期望
    Vector3f gyro_latest = get_imu_sensorsGyro();   //为原始数据经第一次低通滤波后的角速度数据 deg/s


    angle_vel_body.x = pid[rol_angle].output;    //绕x轴旋转的角速度期望
    angle_vel_body.y = pid[pit_angle].output;    //绕y轴旋转的角速度期望
    angle_vel_body.z = pid[yaw_angle].output;    //绕z轴旋转的角速度期望

    pid_rate_updata(&pid[rol_rate], angle_vel_body.x, gyro_latest.x);
    pid_rate_updata(&pid[pit_rate], angle_vel_body.y, gyro_latest.y);
    pid_rate_updata(&pid[yaw_rate], angle_vel_body.z, gyro_latest.z);

    if (motor.state != motor_active) {

        pid_reset(&pid[pit_rate]);
        pid_reset(&pid[rol_rate]);
        pid_reset(&pid[yaw_rate]);
        pid_reset(&pid[pit_angle]);
        pid_reset(&pid[rol_angle]);
        pid_reset(&pid[yaw_angle]);
    }

    ctrl_Attitude_MultiRotor_PWM(pid[rol_rate].output,pid[pit_rate].output,pid[yaw_rate].output);

    return 0;
}


/**
 * @brief 外环角度控制器,输入飞行员的摇杆期望(单位:角度,偏航为角速度)
 * @param 横滚角度期望     俯仰角度期望    偏航角速度期望
 * @return
 */
void input_euler_angle_roll_pitch_euler_rate_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds)
{
    attitude_t attitude = get_ahrs_eulerAngles();   //为四元数姿态解算得到的欧拉角  单位:度
    Vector3f gyro_latest = get_imu_sensorsGyro();   //为原始数据经第一次低通滤波后的角速度数据 deg/s

    float euler_roll_angle = (euler_roll_angle_cd);
    float euler_pitch_angle = (euler_pitch_angle_cd);
    float euler_yaw_rate = (euler_yaw_rate_cds);

    /***********************偏航角度控制****************************/

    if(is_zero(euler_yaw_rate_cds))                           //偏航杆置于中位
    {
        if(pid[yaw_angle].target == 0)                               //摇杆刚回中的一刻锁定锁定航向角
        {
            pid[yaw_angle].target = attitude.yaw;
        }
        if(get_one_way_throttle() > 0.3f)                             //油门大于一定值才启用偏航控制
        {
            pid_yaw_updata(&pid[yaw_angle],attitude.yaw);             //角度环控制
        }
    }
    else                                                         //波动偏航方向杆后,只进行内环角速度控制,其角速度期望来自摇杆
    {
        if(get_one_way_throttle() > 0.2f)                        //油门大于一定值才启用偏航控制
        {
            pid[yaw_angle].output = euler_yaw_rate;              //偏航角速度环期望,直接来源于遥控器打杆量
        }
        pid[yaw_angle].target = 0;                               //偏航角期望给0,不进行角度控制
    }

    pid_angle_updata(&pid[rol_angle],euler_roll_angle, attitude.roll, gyro_latest.x);
    pid_angle_updata(&pid[pit_angle],euler_pitch_angle,attitude.pitch,gyro_latest.y);
}

/**
 * @brief 设置油门输出
 * @param 期望油门    油门倾角补偿    油门的低通截止频率
 * @return
 */
void attitude_set_throttle_out(float throttle_in, bool apply_angle_boost, float filter_cutoff)
{
    static uint16_t arming_GUIDED_Mode = 0;
    float _throttle_in = throttle_in;

    if (apply_angle_boost) {
        // Apply angle boost
        float boost_factor = 1.0f / limit(rMat[2][2], 0.92f, 1.0f);
        _throttle_in = _throttle_in*boost_factor;
    } else {
        // Clear angle_boost for logging purposes
    }

    float get_channel_thr = get_channel_thr_control_in();
    if (get_new_mode() == STABILIZE_Mode) {
        if(motor.state == motor_idle && get_channel_thr>-0.7)  //电机怠速后油门大于250 才会pid控制电机
        {
            motor.state = motor_active;
        }
    }
    else if (get_new_mode() == GUIDED_Mode) {
        if(motor.state == motor_idle && get_channel_thr<0.001)  //电机怠速后油门大于250 才会pid控制电机
         {
            arming_GUIDED_Mode++;
            if (arming_GUIDED_Mode>=300) {
                motor.state = motor_active;
                arming_GUIDED_Mode = 0;
            }
         }
    }
    else {
        if(motor.state == motor_idle && get_channel_thr>0.001)  //电机怠速后油门大于250 才会pid控制电机
        {
            motor.state = motor_active;
        }
    }

    set_motors_throttle(_throttle_in);  //0到1000  把油门的控制量给电机
}





