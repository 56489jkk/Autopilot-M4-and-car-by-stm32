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
#include "GCS_Copter.h"
#include "GCS_Communication.h"
#include "BlueTooth.h"
#include <stdlib.h>
#include <time.h>
Vector3f path[500];
uint8_t path_num=0;

uint8_t set_num=0;
#define  USE_External_Route     1     //为1时：使用树莓派发送过来的航点期望， 为0时：使用飞控内部设置的航点期望

void mode_guided_initialization() //初始化飞行参数
{
    pid_reset(&pid[x_position]);
    pid_reset(&pid[x_velocity]);
    pid_reset(&pid[y_position]);
    pid_reset(&pid[y_velocity]);
    if (!is_active_z_ctrl()) {
        init_posittion_z_controller();
    }
    path[0] = get_key_fly_position_cm();  //获取一键起飞时记录的位置
    mavlink_set_target.x=0;
    mavlink_set_target.y=0;
//    Vector3f current_pos = get_position_neu_cm();
//    path[0].x =current_pos.x;
//    path[0].y =current_pos.y;

    if(get_pos_vel_type()==2) //飞控内部设定航线
    {
        //机头方向是X轴正方向，右手边是Y轴正方向
        path[1].x = path[0].x+175;       //x轴 向前
        path[1].y = path[0].y+0;         //y轴

        path[2].x = path[0].x+175;
        path[2].y = path[0].y+150;

        path[3].x = path[0].x;
        path[3].y = path[0].y+150
                ;
        //回到原点
        path[4].x = path[0].x;
        path[4].y = path[0].y;

        //这里以走以一个长方形为路线，走完航点自动降落。
        set_num = 4;  //设定的航点总数
    }
    else
        send_take_off_flag_to_cv();           //给树莓派发送启动航线任务命令


    path_num=0;                               //航点计数复位
    set_waypoint_destination(&path[0],false); //设置航点目的地
}


Vector3f t265_path[10];


//用户任务
/*********************************************************************************************************************/

int RNG_data=0;



void DOWN(void)            //自动投放
{

            if (get_channel_value(10)>1100) { //遥控器通道10控制（1100 - 2000）
                                if (get_channel_value(10)<1500)
                                {
                                    ctrl_Rudder_PWM(1100,1100,1100); //飞控的aux1 2 3 接口
                                }
                                if (get_channel_value(10)>1500)
                                {
                                    ctrl_Rudder_PWM(1500,1500,1500); //飞控的aux1 2 3 接口
                                }
                            }else { //mavlink 消息控制
                                ctrl_Rudder_PWM(get_set_servo_pwm(9-1),get_set_servo_pwm(10-1),get_set_servo_pwm(11-1)); //飞控的aux接口
                            }

}
/******************************/

/*****************************/
void GO_point()                            //蓝牙发送航点
{


    //RNG_data = 1;                       //投放点1
    //RNG_data = 2;                      //投放点2
    //RNG_data = 3;                      //投放点3
    /************************/           //随机点

    RNG_Init();                                        //初始化RNG
    rt_thread_delay(200);

    //while(RNG_data == 0||RNG_data == 2)            //获取1或3的随机航点   2为固定点
    if(RNG_data == 0)
    {
    RNG_data = RNG_Get_RandomRange(1,3);          //获取1-3的随机航点
    }
    /*****************************/

   if(RNG_data == 1)
   {
       if(path_num == 1)
       {
           DOWN();                           //自动投放
           rt_thread_delay(10);
           send_1();                          //到达航点1给小车发送指令
       }
   }
   if(RNG_data == 2)
   {
       if(path_num == 2)
       {
           DOWN();                                //自动投放
           rt_thread_delay(10);
           send_2();                          //到达航点2给小车发送指令
       }
   }
   if(RNG_data == 3)
     {
       if(path_num == 3)
       {
           DOWN();                                //自动投放
           rt_thread_delay(10);
           send_3();                         //到达航点3给小车发送指令
       }
     }
/**************************************************************************************************************************/
//自动投放

   /*if(Down == 1)
      {
         if (get_channel_value(10)>1100) { //遥控器通道10控制（1100 - 2000）
                             if (get_channel_value(10)<1500)
                             {
                                 ctrl_Rudder_PWM(1100,1100,1100); //飞控的aux1 2 3 接口
                             }
                             if (get_channel_value(10)>1500)
                             {
                                 ctrl_Rudder_PWM(1500,1500,1500); //飞控的aux1 2 3 接口
                             }
                         }else { //mavlink 消息控制
                             ctrl_Rudder_PWM(get_set_servo_pwm(9-1),get_set_servo_pwm(10-1),get_set_servo_pwm(11-1)); //飞控的aux接口
                         }
      }*/
}

/***************************************************************************************************/
//用户任务结束


void mode_guided_run()//指导飞行模式  (飞行员控制)
{
    if(get_pos_vel_type()==2) //飞控内部设定航线
    {
        if (path_num < set_num) {  //航线尚未走完

            if (is_reached_waypoint_destination()) {     //到达某个端点

                GO_point();                             //执行航点任务

                path_num++;
                set_waypoint_destination(&path[path_num],false);   //将下一个航点位置设置为导航控制模块的目标位置
            }
        } else if ((path_num == set_num) && is_reached_waypoint_destination()) {  //航线运行完成,自动进入降落模式

            set_flight_mode(LAND_Mode,UNKNOWN);

            RNG_data = 0;                               //随机数归0
            send_0();                                  //降落时给小车发送指令
        }
    }

    else //树莓派发送过来的航点
    {
        t265_path[0].x = mavlink_set_target.x*100;
        t265_path[0].y = mavlink_set_target.y*100;
        if (!is_t265_landFlag()) {                              //航线尚未走完
            if (is_reached_waypoint_destination()) {            //到达某个端点
                set_waypoint_destination(&t265_path[0],false);  //将下一个航点位置设置为导航控制模块的目标位置
                copter_sys.landed_state=0;
            }
            else {
                copter_sys.landed_state=2;
            }

        } else if (is_t265_landFlag()&& is_reached_waypoint_destination()) {
            set_flight_mode(LAND_Mode,UNKNOWN);
        }

        if (!get_vision_health()) {
            set_flight_mode(FLOWHOLD_Mode,UNKNOWN);
        }

        if (postion_flag==1) {
            set_flight_mode(POSHOLD_Mode,UNKNOWN);
            into_postion = 1;
        }
    }
    float cur_pos_z = get_position_z_up_cm(); //返回当前高度
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
    static float  z_speed=0;
    if(abs(mavlink_set_target.z*100-cur_pos_z)<=0||mavlink_set_target.z*100==0){
        z_speed=0;
    }
    else {
        z_speed = (mavlink_set_target.z*100-cur_pos_z)*0.3f;
    }

    if (target_climb_rate == 0) {
        set_pos_target_z_from_climb_rate_cm(z_speed);
    }
    else {
        set_pos_target_z_from_climb_rate_cm(target_climb_rate);
    }


    //run waypoint controller    x,y方向上的控制
    update_waypoint_navigation();
    if ((roll_in != 0) || (pitc_in != 0)) {
        wp_roll  = 0;
        wp_pitch = 0;
    }
    else {
        wp_roll  = get_wp_nav_roll();
        wp_pitch = get_wp_nav_pitch();
    }

    //摇杆量和导航控制量结合
    target_roll  += wp_roll;
    target_pitch += wp_pitch;

    //外环角度控制器,输入飞行员的摇杆期望(单位:角度,偏航为角速度)
    input_euler_angle_roll_pitch_euler_rate_yaw(target_roll,target_pitch,target_yaw_rate);
    posittion_update_z_controller(); //高度控制器
}








