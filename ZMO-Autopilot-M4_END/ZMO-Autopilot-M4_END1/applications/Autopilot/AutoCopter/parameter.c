/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-03-08     CGY       the first version
 */
#include "parameter.h"
#include "DL_W25Qxx.h"
#include "DL_FM25Vxx.h"
#include "CL_Math.h"
#include "mode_all.h"
parameter_state ParState;
union Parameter Parame;

static rt_uint16_t lenth = 0;
static struct rt_semaphore write_sem;    /*用于接收消息的信号量*/
rt_uint32_t semaphoreGiveTime=0;

void Parame_Write_Task(void)// 传感器任务
{
    parame_read_from_Flash(); //按照flash中数据初始化参数
    All_PID_Init();
    /* 初始化信号量 */
    rt_sem_init(&write_sem, "write_sem", 0, RT_IPC_FLAG_FIFO);

    while(1)
    {
        if(semaphoreGiveTime == 0)
        {
            rt_thread_mdelay(1);
        }
        if(semaphoreGiveTime == 1)
        {
            rt_thread_mdelay(2000);

            rt_enter_critical();
            Parame.set.frist_init = VERSION;
            w25qxx_Write((rt_uint8_t*)Parame.byte,0,lenth);
            rt_exit_critical();
            All_PID_Init();//存储PID参数后，重新初始化PID
            //send_string("Set save OK!");
            semaphoreGiveTime =0;
        }
    }
}

/****************************************************************************************
*@brief   开机后读取w25q64里存的数据；如果里面没有数据，则把原始数据存下
*@param[in]
*****************************************************************************************/
void parame_read_from_Flash(void)
{
    lenth=sizeof(Parame.byte);
    lenth=lenth/4+(lenth%4 ? 1:0);
    //w25qxx_Read((rt_uint8_t*)Parame.byte,0,lenth);   //读取参数
    fm25vxx_Read((rt_uint8_t*)Parame.byte,0,lenth);    //读取参数
    if(Parame.set.frist_init!=VERSION||Parame.set.final_init!=END_DATA)//如果读到的Parame.set.frist_init不等于VERSION，说明Flash内没有存下参数
    {
        //flash内部没有参数时，按照原始参数初始化并储存
        PID_Parameter_Rest();
        Parame.set.frist_init = VERSION;
        Parame.set.final_init = END_DATA;
        //w25qxx_Write((rt_uint8_t*)Parame.byte,0,lenth);
        fm25vxx_write((rt_uint8_t*)Parame.byte,0,lenth);
    }
}

/****************************************************************************************
*@brief     此函数用来发起数据保存命令，采用信号量实现。
*@param[in]
*****************************************************************************************/
void data_save_to_Flsh()
{
    /* 串口接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    //rt_sem_release(&write_sem);
    //  semaphoreGiveTime = rt_tick_get();
    semaphoreGiveTime = 1;
}


/*PID初始化*/
void All_PID_Init(void)
{
    stabilize_mode_pid_par_init();
    altitude_hold_mode_pid_par_init();  //高度控制的pid参数初始化
    position_hold_mode_pid_par_init();  //定点位置控制的pid参数初始化
    flow_hold_mode_pid_par_init();
}

/****************************************************************************************
*@brief     pid的原始比例积分微分参数
*@param[in]
*****************************************************************************************/
void PID_Parameter_Rest()
{
    Parame.set.kp_[rol_rate]=1.35f;
    Parame.set.ki_[rol_rate]=0.35f;
    Parame.set.kd_[rol_rate]=0.04f;

    Parame.set.kp_[pit_rate]=1.35f;
    Parame.set.ki_[pit_rate]=0.35f;
    Parame.set.kd_[pit_rate]=0.04f;

    Parame.set.kp_[yaw_rate]=1.8f;
    Parame.set.ki_[yaw_rate]=0.7f;
    Parame.set.kd_[yaw_rate]=0.01f;

    Parame.set.kp_[rol_angle]=5.0f;
    Parame.set.ki_[rol_angle]=0.0f;
    Parame.set.kd_[rol_angle]=0.0f;

    Parame.set.kp_[pit_angle]=5.0f;
    Parame.set.ki_[pit_angle]=0.0f;
    Parame.set.kd_[pit_angle]=0.0f;

    Parame.set.kp_[yaw_angle]=5.2f;
    Parame.set.ki_[yaw_angle]=0.0f;
    Parame.set.kd_[yaw_angle]=0.0f;

    Parame.set.kp_[z_position]=1.0f;
    Parame.set.ki_[z_position]=0.0f;
    Parame.set.kd_[z_position]=0.0f;

    Parame.set.kp_[z_velocity]=3.0f;
    Parame.set.ki_[z_velocity]=1.3f;
    Parame.set.kd_[z_velocity]=0.01f;

    Parame.set.kp_[x_posFlow]=0.2;
    Parame.set.ki_[x_posFlow]=0;
    Parame.set.kd_[x_posFlow]=0;
    Parame.set.kp_[y_posFlow]=0.2;
    Parame.set.ki_[y_posFlow]=0;
    Parame.set.kd_[y_posFlow]=0;

    Parame.set.kp_[x_velFlow]=2.2f;
    Parame.set.ki_[x_velFlow]=0.5f;
    Parame.set.kd_[x_velFlow]=0;
    Parame.set.kp_[y_velFlow]=2.2f;
    Parame.set.ki_[y_velFlow]=0.5f;
    Parame.set.kd_[y_velFlow]=0;

    Parame.set.kp_[x_posGps]=1.0f;
    Parame.set.ki_[x_posGps]=0.0f;
    Parame.set.kd_[x_posGps]=0.0f;
    Parame.set.kp_[y_posGps]=1.0f;
    Parame.set.ki_[y_posGps]=0.0f;
    Parame.set.kd_[y_posGps]=0.0f;

    Parame.set.kp_[x_velGps]=1.2f;
    Parame.set.ki_[x_velGps]=0.0f;
    Parame.set.kd_[x_velGps]=0.0f;
    Parame.set.kp_[y_velGps]=1.2f;
    Parame.set.ki_[y_velGps]=0.0f;
    Parame.set.kd_[y_velGps]=0.0f;

    //send_string("PID_Parameter reset!");
}

