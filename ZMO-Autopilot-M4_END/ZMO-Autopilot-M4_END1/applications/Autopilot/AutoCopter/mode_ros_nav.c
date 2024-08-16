/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-06-27     Administrator       the first version
 */
#include "loco_config.h"
#include "mode_all.h"

/***ros导航飞行模式***/
void ros_nav_mode_pid_par_init(void)
{


}


void mode_ros_nav_initialization(void)
{
    if (!is_active_z_ctrl()) {
        init_posittion_z_controller();
    }


//    if ( motor.state == motor_idle&&ros_command.command == 22) {
//        motor.state = motor_active;
//    }

}

//双目位置与速度信息用于位置控制
void ros_t265_hold_to_angle(Vector2f *bf_angles, bool stick_input)
{
    static bool braking = false;
    static uint32_t last_stick_input_ms;
    static Vector3f posFlow_target;
    uint32_t now_time = rt_tick_get();

    Vector3f body_acce = get_navigation_body_frame_acce();
    Vector3f current_pos = get_position_neu_cm();
    Vector3f current_vel = get_velocity_neu_cms();
    pid_rate_updata(&pid[x_velFlow],pid[x_posFlow].output, (current_vel.x + 0.00f*body_acce.x));
    pid_rate_updata(&pid[y_velFlow],pid[y_posFlow].output, (current_vel.y + 0.00f*body_acce.y));

    float roll_in = get_channel_roll_control_in();  //用于判断遥控器摇杆位置
    float pitc_in = get_channel_pitch_control_in(); //用于判断遥控器摇杆位置
    //电机不活跃或高度低,则清除光流控制量
    if(motor.state != motor_active || current_pos.z<10||((roll_in != 0) || (pitc_in != 0))) {
          pid_reset(&pid[x_velFlow]);
          pid_reset(&pid[y_velFlow]);
          pid_reset(&pid[x_posFlow]);
          pid_reset(&pid[y_posFlow]);
          bf_angles->x = 0;
          bf_angles->y = 0;
          posFlow_target.x = current_pos.x;
          posFlow_target.y = current_pos.y;

          braking = true;
          last_stick_input_ms = now_time;
    }
    else {
        if (stick_input == true) {
                pid_reset(&pid[x_posFlow]);
                pid_reset(&pid[y_posFlow]);
                posFlow_target.x = current_pos.x;
                posFlow_target.y = current_pos.y;

//                pid[x_posFlow].output=-ros_nav_target.vx*100;
//                pid[y_posFlow].output=ros_nav_target.vy*100;

                bf_angles->x = biquadFilterApply(&pilot_acce_LPF[0],limit(-pid[x_velFlow].output * 0.1f*1.5f ,-20,20));
                bf_angles->y = biquadFilterApply(&pilot_acce_LPF[1],limit( pid[y_velFlow].output * 0.1f*1.5f ,-20,20));

                braking = true;
                last_stick_input_ms = now_time;
            }

            if (!stick_input && braking) {
                // stop braking if either 3s has passed, or we have slowed below 0.3m/s
                if (now_time - last_stick_input_ms > 3000 ||pythagorous2(current_vel.x,current_vel.y)<=30) {
                    braking = false;
                    posFlow_target.x = current_pos.x;
                    posFlow_target.y = current_pos.y;
                }
            }
            if (!stick_input && !braking) {

                pid_rate_updata(&pid[x_posFlow],posFlow_target.x,current_pos.x);
                pid_rate_updata(&pid[y_posFlow],posFlow_target.y,current_pos.y);

                bf_angles->x = biquadFilterApply(&pilot_acce_LPF[0],limit(-pid[x_velFlow].output * 0.1f,-20,20));
                bf_angles->y = biquadFilterApply(&pilot_acce_LPF[1],limit( pid[y_velFlow].output * 0.1f,-20,20));
            }

            if (!stick_input && braking) {
                pid[x_posFlow].output=0;
                pid[y_posFlow].output=0;

                bf_angles->x = biquadFilterApply(&pilot_acce_LPF[0],limit(-pid[x_velFlow].output * 0.1f*1.5f ,-20,20));
                bf_angles->y = biquadFilterApply(&pilot_acce_LPF[1],limit( pid[y_velFlow].output * 0.1f*1.5f ,-20,20));
            }
    }

}

void mode_ros_nav_run(void)
{
    Vector2f bf_angles;
    //获取飞行员的摇杆期望,单位为角度.
    get_pilot_desired_lean_angles(&bf_angles.x,&bf_angles.y, 0);
    //返回飞行员的偏航摇杆的期望,单位度每秒.
    float target_yaw_rate =  get_pilot_desired_yaw_rate(get_channel_yaw_control_in());
    //得到飞行员爬升的速度期望。 get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(get_channel_thr_control_in());

    float roll_in = get_channel_roll_control_in();  //用于判断遥控器摇杆位置
    float pitc_in = get_channel_pitch_control_in(); //用于判断遥控器摇杆位置

    Vector2f ros_t265_angles; //光流定点控制的输出量

    //ros_t265_hold_to_angle(&ros_t265_angles,(absolute(ros_nav_target.vx*100)>1) || (absolute(ros_nav_target.vy*100)>1));

    if ((roll_in != 0) || (pitc_in != 0)) {
         ros_t265_angles.x=0;
         ros_t265_angles.y=0;

    } else {

    }

    //摇杆量和光流定点控制量结合
    bf_angles.x += ros_t265_angles.x;
    bf_angles.y += ros_t265_angles.y;

    if (is_zero(target_yaw_rate)) {

        //float ros_nav_yaw = ros_nav_target.yaw_rate*180/3.14;
        //外环角度控制器,输入飞行员的摇杆期望(单位:角度,偏航为角速度)
        //input_euler_angle_roll_pitch_euler_rate_yaw(bf_angles.x,bf_angles.y,ros_nav_yaw);
    }
    else {
        //外环角度控制器,输入飞行员的摇杆期望(单位:角度,偏航为角速度)
        input_euler_angle_roll_pitch_euler_rate_yaw(bf_angles.x,bf_angles.y,target_yaw_rate);
    }

    set_pos_target_z_from_climb_rate_cm(target_climb_rate);
    posittion_update_z_controller(); //高度控制器

//    if(ros_command.command == 21)
//    {
//       set_flight_mode(LAND_Mode,UNKNOWN);
//    }
}























