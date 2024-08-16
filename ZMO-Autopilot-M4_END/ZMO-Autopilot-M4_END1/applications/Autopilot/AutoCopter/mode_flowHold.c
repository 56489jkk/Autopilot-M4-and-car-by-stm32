/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-13     CGY       the first version
 */
#include "mode_all.h"
#include "loco_config.h"
#include "stdbool.h"
#include "math.h"
#include "GCS_Copter.h"
#include "GCS_Communication.h"
void flow_hold_mode_pid_par_init()
{
    pidInit(&pid[x_velFlow],2.2f,0.8f,0.0f,100,100,400,0.005f,true,20.0f);   //
    pidInit(&pid[y_velFlow],2.2f,0.8f,0.0f,100,100,400,0.005f,true,20.0f);   //p 4.0     d 0.1

    pidInit(&pid[x_posFlow],0.2f,0.0f,0.0f,50,50,100,0.005f,true,20.0f);     //
    pidInit(&pid[y_posFlow],0.2f,0.0f,0.0f,50,50,100,0.005f,true,20.0f);     //p 4.0     d 0.1

    biquadFilterInitLPF(&pilot_acce_LPF[0],200,50);
    biquadFilterInitLPF(&pilot_acce_LPF[1],200,50);
}

static Vector3f posFlow_target;
//光流位置与速度信息用于位置控制
void flowhold_flow_to_angle(Vector2f *bf_angles, bool stick_input)
{
    static bool braking = false;
    static uint32_t last_stick_input_ms;
    uint32_t now_time = rt_tick_get();

    Vector3f body_acce = get_navigation_body_frame_acce(); //获取机体加速度
    Vector3f current_pos = get_position_neu_cm();          //获取当前的位置
    Vector3f current_vel = get_velocity_neu_cms();         //获取当前的速度
    //mavlink_vision_position_estimate_t vision_estimate;
    //vision_estimate = get_vision_position_estimate();
//    /****************23年比赛把起飞改为光流定位*************************/
    current_pos.x = Flow_INS.Position.x;  //向前移动为正 （机头方向为前）
    current_pos.y = Flow_INS.Position.y;  //向右移动为正
    current_vel.x = Flow_INS.Speed.x;
    current_vel.y = Flow_INS.Speed.y;

    pid_rate_updata(&pid[x_velFlow],pid[x_posFlow].output, (current_vel.x + 0.00f*body_acce.x)); //前后的速度控制
    pid_rate_updata(&pid[y_velFlow],pid[y_posFlow].output, (current_vel.y + 0.00f*body_acce.y)); //左右的速度控制

    //电机不活跃或遥控器有输入,则清除光流控制量
    if (motor.state != motor_active || current_pos.z<10 || stick_input == true ||into_postion==1) {
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
          into_postion = 0;
    }
    if (!stick_input && braking) {
        // stop braking if either 3s has passed, or we have slowed below 0.3m/s
        if (now_time - last_stick_input_ms > 3000 ||pythagorous2(current_vel.x,current_vel.y)<=30) {
            braking = false;
            pid_reset(&pid[x_velFlow]);
            pid_reset(&pid[y_velFlow]);
            pid_reset(&pid[x_posFlow]);
            pid_reset(&pid[y_posFlow]);
            posFlow_target.x = current_pos.x;
            posFlow_target.y = current_pos.y;
        }
    }
    if (!stick_input && !braking) {

        pid_rate_updata(&pid[x_posFlow],posFlow_target.x,current_pos.x);
        pid_rate_updata(&pid[y_posFlow],posFlow_target.y,current_pos.y);

        bf_angles->x = -biquadFilterApply(&pilot_acce_LPF[0],limit( pid[x_velFlow].output * 0.1f,-20,20));
        bf_angles->y = +biquadFilterApply(&pilot_acce_LPF[1],limit( pid[y_velFlow].output * 0.1f,-20,20));

    }

    /**没有控制输入且正在刹车**/
    if (!stick_input && braking) {
        // calculate brake angle for each axis separately
        const float brake_gain = (15.0f * 1 + 95.0f) * 0.05f;
        float abs_vel_cms_x = fabsf(current_vel.x);
        float lean_angle_x = brake_gain * abs_vel_cms_x * (1.0f+300.0f/(abs_vel_cms_x+60.0f))*0.01;//计算刹车的倾角
        if (current_vel.x < 0) {
            lean_angle_x = -lean_angle_x;
        }

        float abs_vel_cms_y = fabsf(current_vel.y);
        float lean_angle_y = brake_gain * abs_vel_cms_y * (1.0f+300.0f/(abs_vel_cms_y+60.0f))*0.01;//计算刹车的倾角
        if (current_vel.y < 0) {
            lean_angle_y = -lean_angle_y;
        }

        bf_angles->x = +biquadFilterApply(&pilot_acce_LPF[0],limit(lean_angle_x,-20,20));
        bf_angles->y = -biquadFilterApply(&pilot_acce_LPF[1],limit(lean_angle_y,-20,20));
    }
}


void mode_flow_hold_initialization() //初始化飞行参数
{
    pid_reset(&pid[x_velFlow]);
    pid_reset(&pid[y_velFlow]);
    pid_reset(&pid[x_posFlow]);
    pid_reset(&pid[y_posFlow]);
    if (!is_active_z_ctrl()) {
        init_posittion_z_controller();
    }

    posFlow_target.x = Flow_INS.Position.x;
    posFlow_target.y = Flow_INS.Position.y;

//    send_cmd_restart_t265();
}

Vector3f _key_fly_pos_cm;
Vector3f get_key_fly_position_cm(void) //返回记录一键起飞的位置
{
    return _key_fly_pos_cm;
}


void mode_flow_hold_run() //定点模式  (飞行员控制)
{
    float target_roll, target_pitch;
    //获取飞行员的摇杆期望,单位为角度.
    get_pilot_desired_lean_angles(&target_roll,&target_pitch, 0);
    //返回飞行员的偏航摇杆的期望,单位度每秒。
    float target_yaw_rate =  get_pilot_desired_yaw_rate(get_channel_yaw_control_in());

    //get pilot desired climb rate 得到飞行员爬升的速度期望。
    float target_climb_rate = get_pilot_desired_climb_rate(get_channel_thr_control_in());

    float cur_pos_z = get_position_z_up_cm(); //返回当前高度
    static uint16_t one_key_take_off=0,speed=0,init=0;
    if (is_one_button_takeoff()) {
        if (one_key_take_off++>400) {
          if(one_key_take_off==402){
              motor.state = motor_active;
              _key_fly_pos_cm = get_position_neu_cm(); //获取一键起飞时的当前位置
          }
        }
        else motor.state = motor_idle;

        if (absolute(cur_pos_z-100)<25) {
            speed = 25;
            if (absolute(cur_pos_z-100)<8||init==1) {
                speed = 0;
                init = 1;
                set_flight_mode(GUIDED_Mode,UNKNOWN);
                //set_flight_mode(ROS_Nav_Mode,UNKNOWN);
            }
        }
        else  speed = 35;

    }
    else{
        one_key_take_off=0,speed=0,init=0;
    }
    target_climb_rate += speed;


    set_pos_target_z_from_climb_rate_cm(target_climb_rate); //发布升降期望速度

    float roll_in = get_channel_roll_control_in();  //用于判断遥控器摇杆位置
    float pitc_in = get_channel_pitch_control_in(); //用于判断遥控器摇杆位置

    Vector2f flow_angles; //光流定点控制的输出量
    flowhold_flow_to_angle(&flow_angles,(roll_in != 0) || (pitc_in != 0));

    //摇杆量和光流定点控制量结合
    target_roll  += flow_angles.y;
    target_pitch += flow_angles.x;

    //外环角度控制器,输入飞行员的摇杆期望(单位:角度,偏航为角速度)
    input_euler_angle_roll_pitch_euler_rate_yaw(target_roll,target_pitch,target_yaw_rate);
    posittion_update_z_controller(); //高度控制器
}










