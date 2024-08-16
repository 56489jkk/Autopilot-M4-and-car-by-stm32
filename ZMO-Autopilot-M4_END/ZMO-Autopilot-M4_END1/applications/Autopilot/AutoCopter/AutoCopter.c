/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-27     CGY       the first version
 */
#include "AutoCopter.h"
#include "GCS_Communication.h"
#include "BlueTooth.h"
/*
 *   Note：本文件中不涉及传感器原始的读取、处理等，主要为获取处理好的传感器数据进行无人机的相关控制。
 */

Copter_t copter={RT_NULL};

/***********************************************************
  *@brief  初始化无人机控制相关线程任务。
*************************************************************/
void AutoCopter_thread_task_init(void) //无人机相关任务初始化
{

    //创建姿态角速度控制任务,以无人机的角速度控制为核心。
    copter.thread[0] = rt_thread_create("attitude_rate_control_task",
                                        attitude_rate_control_task,
                                        RT_NULL,
                                        1024,
                                        2,    //优先级2
                                        20);
    if(copter.thread[0] != RT_NULL)
        rt_thread_startup(copter.thread[0]);//启动线程0

    //创建飞行模式控制任务,以无人机的角度\位置\高度等控制为核心。
    copter.thread[1] = rt_thread_create("flight_mode_task",
                                        flight_mode_task,
                                        RT_NULL,
                                        1024,
                                        3,
                                        20);//优先级12
    if(copter.thread[1] != RT_NULL)
        rt_thread_startup(copter.thread[1]);//启动线程1

    //创建电池和罗盘任务,以电池容量测量及罗盘数据补偿为核心。
    copter.thread[2] = rt_thread_create("update_batt_compass_task",
                                        update_batt_compass_task,
                                        RT_NULL,
                                        1024,
                                        15,
                                        20);//优先级15
    if(copter.thread[2] != RT_NULL)
        rt_thread_startup(copter.thread[2]); //启动线程2

    //创建飞行数据记录任务,以记录无人机飞行模式\姿态\位置等数据为核心。
    copter.thread[3] = rt_thread_create("logging_task",
                                        logging_task,
                                        RT_NULL,
                                        1024,
                                        16,
                                        20);//优先级16
    if(copter.thread[3] != RT_NULL)
        rt_thread_startup(copter.thread[3]); //启动线程3

    //创建参数储存任务,以储存pid、陀螺仪和磁力计校准等参数为核心。
    copter.thread[4] = rt_thread_create("parameter_storage_task",
                                        parameter_storage_task,
                                        RT_NULL,
                                        1024,
                                        17,
                                        20);//优先级50
    if(copter.thread[4] != RT_NULL)
        rt_thread_startup(copter.thread[4]);//启动线程4

    //创建用户任务,以用户需求为核心。
    copter.thread[5] = rt_thread_create("userhook_task1",
                                        userhook_task1,
                                        RT_NULL,
                                        1024,
                                        18,
                                        20);//优先级60
    if(copter.thread[5] != RT_NULL)
        rt_thread_startup(copter.thread[5]); //启动线程5
}

/***********************************************************
  *@brief  姿态角速度控制任务，以无人机的角速度控制为核心。
*************************************************************/
void attitude_rate_control_task(void* parameter)  //控制姿态
{
    while(1)
    {
        rt_thread_mdelay(2); //500Hz
        attitude_rate_pid_controller_run(); //无人机角速度内环pid控制。
        int d = get_loop_run_interval_ms();
        if (d!=2) ;
//            rt_kprintf("d:%d",(rt_uint16_t)d);
    }
}

/***********************************************************
  *@brief  飞行模式控制任务，以无人机的角度、位置、高度等控制为核心。
*************************************************************/
void flight_mode_task(void* parameter)//线程
{
    while(1)
    {
        rt_thread_mdelay(5); //200Hz

        read_mode_switch();

        update_flight_mode(get_new_mode());
    }
}

/***********************************************************
  *@brief 电池和罗盘任务，以电池容量测量及罗盘数据补偿为核心。
*************************************************************/
void update_batt_compass_task(void* parameter)//线程2
{

    while(1)
    {
        rt_thread_mdelay(1);
    }
}

/***********************************************************
  *@brief  飞行数据记录任务，以记录无人机飞行模式、姿态、位置等数据为核心。
*************************************************************/
void logging_task(void* parameter)//线程2
{
    while(1)
    {
        rt_thread_mdelay(10);
    }
}

/***********************************************************
  *@brief  参数储存任务，以储存pid、陀螺仪和磁力计校准等参数为核心。
*************************************************************/
void parameter_storage_task(void* parameter)//线程
{
    Parame_Write_Task();
}

/***********************************************************
  *@brief 创建用户任务，以用户需求为核心。
*************************************************************/

void userhook_task1(void* parameter)//线程2
{
    bluetooth_init();//蓝牙配置连接
    RNG_Init();//RNG随机数
    /******************************************************************************************************************************/
        /* rt_thread_delay(10000);
          send_3();
          rt_thread_delay(20000);
          send_0();*/
     /********************************************************************************************************************/


    while(1)
    {
                //控制舵机

           /*     if (get_channel_value(10)>1100) { //遥控器通道6控制（1100 - 2000）
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
                }*/


                  //控制舵机
             /*   if (get_channel_value(6)>1100) { //遥控器通道6控制（1100 - 2000）
                    ctrl_Rudder_PWM(get_channel_value(6),get_channel_value(6),get_channel_value(6)); //飞控的aux1 2 3 接口
                }else { //mavlink 消息控制
                    ctrl_Rudder_PWM(get_set_servo_pwm(9-1),get_set_servo_pwm(10-1),get_set_servo_pwm(11-1)); //飞控的aux接口

            }*/

            }
}


/*        if(rudder_flag==0)
        ctrl_Rudder_PWM(0,0,0);//    飞控的A1接口
        if(rudder_flag==1)
        rt_thread_mdelay(1000);
        ctrl_Rudder_PWM(1100,0,0);// 飞控的A1接口

        if(postion_flag==0)
        ctrl_Laser_pointer_PWM(0);// 飞控的A4接口
        if(postion_flag==1)
        ctrl_Laser_pointer_PWM(2500000);// 飞控的A4接口
*/

uint8_t get_my_gcs_sysid()
{
    return 1;
}
