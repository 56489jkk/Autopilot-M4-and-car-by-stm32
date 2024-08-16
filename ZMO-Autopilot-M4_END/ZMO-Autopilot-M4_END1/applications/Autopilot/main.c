/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-01-14     RT-Thread    first version
 */

#include <rtthread.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include <board.h>
#include <rtdevice.h>
#include "loco_config.h"
#include <drv_common.h>
#include "CL_AHRS.h"
#include "CL_RC_Channel.h"
#include "CL_NavigationKF.h"
#include "DL_T265.h"
#include "CL_Notify.h"
#include "GCS_Copter.h"
#include "DL_CompassSensor.h"
#include "DL_LinkTrack_P_BS2.h"
#include "BlueTooth.h"
/*
 *   Note：本文件中的线程专门用于传感器原始数据读取、处理、姿态解算、EKF位置估计等相关算法的实现，为核心线程。
 *   Note：目的是为AutoCopter.c中的无人机控制任务提供姿态、位置、速度、遥控器等处理过的实时数据。
 *   Note：是无人机或无人车或无人船或固定翼等相关控制的实时数处理，为控制提供可靠的反馈数据。
 */
static rt_thread_t optical_flow_thread = RT_NULL;       //线程句柄
static rt_thread_t imu_sensors_thread = RT_NULL;        //线程句柄
static rt_thread_t rc_data_process_thread = RT_NULL;    //线程句柄
static rt_thread_t tof_ranging_thread = RT_NULL;        //线程句柄
static rt_thread_t ros_thread = RT_NULL;                //线程句柄
static rt_thread_t usb_Tx_thread = RT_NULL;             //线程句柄
static rt_thread_t usb_Rx_thread = RT_NULL;             //线程句柄
static rt_thread_t mavlink_Rx_thread = RT_NULL;         //线程句柄
static rt_thread_t rgb_state_thread = RT_NULL;          //线程句柄

void imu_sensors_thread_entry(void* parameter);     //线程
void rc_data_process_thread_entry(void* parameter); //线程
void opt_uart_read_thread_entry(void* parameter);   //线程
void tof_uart_read_thread_entry(void* parameter);   //线程
void ros_uart_read_thread_entry(void* parameter);   //线程
void rgb_state_thread_entry(void* parameter);       //线程
void usb_dataTx_thread_entry(void* parameter);      //线程
void usb_dataRx_thread_entry(void* parameter);      //线程
void mavlink_dataRx_thread_entry(void* parameter);  //线程

int main(void)
{
    rt_thread_mdelay(1000);  //启动延时
    /*使能电源控制引脚*/
    rt_pin_mode(GET_PIN(E,0), PIN_MODE_OUTPUT); //
    rt_pin_mode(GET_PIN(E,1), PIN_MODE_OUTPUT); //
    rt_pin_mode(GET_PIN(E,6), PIN_MODE_OUTPUT); //
    rt_pin_write(GET_PIN(E,0), PIN_HIGH); //SENSORS_EN
    rt_pin_write(GET_PIN(E,1), PIN_HIGH); //SD_CARD_EN
    rt_pin_write(GET_PIN(E,6), PIN_HIGH); //5V_OUT_EN

    /*Ten-axis imu sensor data processing task and attitude settlement task*/
    imu_sensors_thread = rt_thread_create("imu_sensors_task",        //the name of thread, which shall be unique
                                           imu_sensors_thread_entry, //the entry function of thread
                                           RT_NULL, //the parameter of thread enter function
                                           1024,    //the size of thread stack
                                           1,       //the priority of thread
                                           20);     //time slice if there are same priority thread
    if(imu_sensors_thread != RT_NULL)
        rt_thread_startup(imu_sensors_thread);  //start a thread and put it to system ready queue

    /*Remote control original data reading and processing tasks*/
    rc_data_process_thread = rt_thread_create("rc_data_process_task",
                                               rc_data_process_thread_entry,
                                               RT_NULL,
                                               1024,
                                               5,   //the priority of thread
                                               20);
    if(rc_data_process_thread != RT_NULL)
        rt_thread_startup(rc_data_process_thread);

    /*Original reading and processing tasks of optical flow sensors*/
    optical_flow_thread = rt_thread_create("opt_read_task",
                                            opt_uart_read_thread_entry,
                                            RT_NULL,
                                            1024,
                                            8,  //the priority of thread
                                            20);
    if(optical_flow_thread != RT_NULL)
        rt_thread_startup(optical_flow_thread);

    /*激光类传感器原始读取及处理任务*/
    tof_ranging_thread = rt_thread_create("tof_read_task",
                                           tof_uart_read_thread_entry,
                                           RT_NULL,
                                           1024,
                                           7,   //the priority of thread
                                           20);
    if(tof_ranging_thread != RT_NULL)
        rt_thread_startup(tof_ranging_thread);

    /*usb发送传感器、参数等数据的任务*/
    usb_Tx_thread = rt_thread_create("usb_Tx_thread",
                                      usb_dataTx_thread_entry,
                                      RT_NULL,
                                      2048,
                                      9,  //优先级9
                                      20);
    if(usb_Tx_thread != RT_NULL)
        rt_thread_startup(usb_Tx_thread);

    /*usb接收数据的任务*/
    usb_Rx_thread = rt_thread_create("usb_Rx_thread",
                                      usb_dataRx_thread_entry,
                                      RT_NULL,
                                      2048,
                                      22,  //优先级22
                                      20);
    if(usb_Rx_thread != RT_NULL)
        rt_thread_startup(usb_Rx_thread);

    /*mavlink接收数据的任务*/
    mavlink_Rx_thread = rt_thread_create("mavlink_Rx_thread",
                                          mavlink_dataRx_thread_entry,
                                          RT_NULL,
                                          2048,
                                          22,  //优先级22
                                          20);
    if(mavlink_Rx_thread != RT_NULL)
        rt_thread_startup(mavlink_Rx_thread);

    /*读取机载电脑的数据，使用mavlink协议*/
    ros_thread = rt_thread_create("ROS_read_thread",
                                   ros_uart_read_thread_entry,
                                   RT_NULL,
                                   2048,
                                   10,  //优先级10
                                   20);
    if(ros_thread != RT_NULL)
        rt_thread_startup(ros_thread);

    /*初始化无人机控制相关线程*/
    AutoCopter_thread_task_init();

    /*  rgb   */
    rgb_state_thread = rt_thread_create("rgb_state_task",
                                         rgb_state_thread_entry,
                                         RT_NULL,
                                         1024,
                                         30,  //the priority of thread
                                         20);
    if(rgb_state_thread != RT_NULL)
        rt_thread_startup(rgb_state_thread);


    while(1)
    {

        rt_thread_mdelay(1000);
        /*调试输出cpu占用率*/
//        uint8_t major,minor;
//        cpu_usage_get(&major,&minor);
//        rt_kprintf("CPU USAGE %u.%u %%\n",major,minor);
    }

    return RT_EOK;
}

/***********************************************************
  *@brief imu传感器数据处理任务
*************************************************************/
void imu_sensors_thread_entry(void* parameter)
{
    state_t state;
    baro_t  baro;
    Vector3f gyro,acce,magn;
    Vector3f nav_acce;
    sensorData_t sensorData ;
    static uint32_t tick = 0;
    biquadFilterInitLPF(&acceFilterLPF[0],500,15);
    biquadFilterInitLPF(&acceFilterLPF[1],500,15);
    biquadFilterInitLPF(&acceFilterLPF[2],500,15);



      while(1)
    {




        rt_thread_mdelay(1);

        if(rate_do_excute(500,tick))  //500Hz
        {
            for (uint8_t i = 0; i < (imu.instance); ++i) {
                gyro_data_update(&gyro,i); //角速度数据更新
                acce_data_update(&acce,i); //加速度数据更新
            }
            sensorData.gyro = gyro;
            sensorData.acce = acce;
            sensorData.magn = magn;
            ahrs_update_attitude(&sensorData,&state,0.002f); //姿态解算

            /*导航加速度再次滤波*/
            nav_acce.x =  biquadFilterApply(&acceFilterLPF[0],acce.x);
            nav_acce.y =  biquadFilterApply(&acceFilterLPF[1],acce.y);
            nav_acce.z =  biquadFilterApply(&acceFilterLPF[2],acce.z);
            navigation_accel_update(nav_acce);  //导航加速度更新
        }

        if(rate_do_excute(25,tick))  //25Hz
        {
            Baro_Data_Update(&baro);    //气压计数据更新
        }

        if(rate_do_excute(10,tick))  //10Hz
        {
            magn_data_update(&magn,0);
        }

        tick++;
    }
}

/***********************************************************
  *@brief 处理遥控器的原始数据，采用ppm、sbus等信号
*************************************************************/
void rc_data_process_thread_entry(void* parameter)  //遥控器数据处理线程
{
    static uint32_t tick = 0;
    while(1)
    {
        rt_thread_mdelay(1);                        //调度
        raw_ppm_data_update();                      //获取原始的遥控器通道值
        if(rate_do_excute(200,tick))                //200Hz
        {
            main_channel_value_update();            //通道值更新
            position_estimation_update(0.005f);     //位置估计更新
        }
        tick++;
    }
}

/***********************************************************
  *@brief  用于和上位机通信，将相关数据通过usb发送到上位机
*************************************************************/
void usb_dataTx_thread_entry(void* parameter)//线程
{
    static uint32_t tick = 0;
    while(1)
    {
        rt_thread_mdelay(1);
        ZMO_PC_data_exchange();         //将数据发送至上位机,用于飞控调试

        if(rate_do_excute(15,tick))     //15Hz
        {
            ultra_start_work();
        }
        tick++;
    }
}

/**************************************USB串口*********************************************/
static struct rt_semaphore usb_uart_rx_sem;    /* 用于接收消息的信号量 */
/* 接收数据回调函数 */
static rt_err_t usb_uart_input(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断,调用此回调函数,然后发送接收信号量 */
    rt_sem_release(&usb_uart_rx_sem);
    return RT_EOK;
}
void usb_dataRx_thread_entry(void* parameter)//线程2
{
    uint8_t ret;
    static mavlink_message_t msg;
    static mavlink_status_t  status;
    char com_data;
    rt_device_t dev = rt_device_find("vcom");
    if (dev) {
        /* 初始化信号量 */
        rt_sem_init(&usb_uart_rx_sem, "usb_uart_rx_sem", 1, RT_IPC_FLAG_FIFO);
        /* 设置接收回调函数 */
        rt_device_set_rx_indicate(dev, usb_uart_input);
        set_serial_device_baud_rate(dev,115200);
        rt_device_open(dev, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    }
    while (1)
    {

        if(dev)
        {
            /* 从串口读取一个字节的数据,没有读取到则等待接收信号量 */
            while (rt_device_read(dev,-1,&com_data,1) != 1)
            {
                /* 阻塞等待接收信号量,等到信号量后再次读取数据 */
                rt_sem_take(&usb_uart_rx_sem,RT_WAITING_FOREVER);
            }
            /* 读取到的数据通过串口错位输出 */
            ret = mavlink_parse_char(0, com_data, &msg, &status);

            if(MAVLINK_FRAMING_OK == ret)
            {
                Copter_handle_Message(msg);
            }
        }
        else {
            rt_thread_mdelay(10);
        }
    }
}

/**************************************mavlink*********************************************/
static struct rt_semaphore mav_uart_rx_sem;    /* 用于接收消息的信号量 */
/* 接收数据回调函数 */
static rt_err_t mav_uart_input(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断,调用此回调函数,然后发送接收信号量 */
    rt_sem_release(&mav_uart_rx_sem);
    return RT_EOK;
}
void mavlink_dataRx_thread_entry(void* parameter)//线程2
{
    uint8_t ret;
    static mavlink_message_t msg;
    static mavlink_status_t  status;
    char com_data;
    rt_device_t dev = serial_device_find(SerialProtocol_Console);
    if (dev) {
        /* 初始化信号量 */
        rt_sem_init(&mav_uart_rx_sem, "mav_uart_rx_sem", 1, RT_IPC_FLAG_FIFO);
        /* 设置接收回调函数 */
        rt_device_set_rx_indicate(dev, mav_uart_input);
        set_serial_device_baud_rate(dev,115200);
        rt_device_open(dev, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    }
    while (1)
    {
        if(dev)
        {
            /* 从串口读取一个字节的数据,没有读取到则等待接收信号量 */
            while (rt_device_read(dev,-1,&com_data,1) != 1)
            {
                /* 阻塞等待接收信号量,等到信号量后再次读取数据 */
                rt_sem_take(&mav_uart_rx_sem,RT_WAITING_FOREVER);
            }
            /* 读取到的数据通过串口错位输出 */
            ret = mavlink_parse_char(0, com_data, &msg, &status);

            if(MAVLINK_FRAMING_OK == ret)
            {
                Copter_handle_Message(msg);
            }
        }
        else {
            rt_thread_mdelay(10);
        }
    }
}
/**************************************tof类串口*********************************************/
static struct rt_semaphore tof_uart_rx_sem;    /* 用于接收消息的信号量 */
/* 接收数据回调函数 */
static rt_err_t tof_uart_input(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断,调用此回调函数,然后发送接收信号量 */
    rt_sem_release(&tof_uart_rx_sem);
    return RT_EOK;
}
/***********************************************************
  *@brief  激光数据接收
*************************************************************/
void tof_uart_read_thread_entry(void* parameter)//线程2
{
    char com_data;
    rt_device_t dev = serial_device_find(SerialProtocol_Tof);
    if (dev) {
        /* 初始化信号量 */
        rt_sem_init(&tof_uart_rx_sem, "tof_uart_rx_sem", 1, RT_IPC_FLAG_FIFO);
        /* 设置接收回调函数 */
        rt_device_set_rx_indicate(dev, tof_uart_input);

        set_serial_device_baud_rate(dev,115200);
        rt_device_open(dev, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);

        //初始化
        //rt_uint8_t config[5]={0X5A, 0X05,0X05, 0X01,0X65};
        //rt_uint8_t config2[4]={0X5A ,0X04, 0X11, 0X6F};
        //rt_thread_mdelay(1000);
        //if(dev) rt_device_write(dev, 0, config,5);
        //rt_thread_mdelay(1000);
        //if(dev) rt_device_write(dev, 0, config2,4);
    }
    while (1)
    {
        if(dev)
        {
            /* 从串口读取一个字节的数据,没有读取到则等待接收信号量 */
            while (rt_device_read(dev,-1, &com_data, 1) != 1)
            {
                /* 阻塞等待接收信号量,等到信号量后再次读取数据 */
                rt_sem_take(&tof_uart_rx_sem, RT_WAITING_FOREVER);
            }
            //TFmini_Statemachine(com_data); //读取数据及解析数据
            uwb_data_receive(com_data);
        } else {
            rt_thread_mdelay(10);
        }
    }
}

/**************************************光流类串口*********************************************/
static struct rt_semaphore opt_uart_rx_sem;    /* 用于接收消息的信号量 */
/* 接收数据回调函数 */
static rt_err_t opt_uart_input(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断,调用此回调函数,然后发送接收信号量 */
    rt_sem_release(&opt_uart_rx_sem);
    return RT_EOK;
}
/***********************************************************
  *@brief  光流数据接收
*************************************************************/
void opt_uart_read_thread_entry(void* parameter)//线程2
{
    char com_data;
    rt_device_t dev = serial_device_find(SerialProtocol_Opt);
    if (dev) {
        /* 初始化信号量 */
        rt_sem_init(&opt_uart_rx_sem, "opt_uart_rx_sem", 1, RT_IPC_FLAG_FIFO);
        /* 设置接收回调函数 */
        rt_device_set_rx_indicate(dev, opt_uart_input);
        set_serial_device_baud_rate(dev,115200);
        rt_device_open(dev, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    }
    while (1)
    {
        if(dev)
        {
            /* 从串口读取一个字节的数据,没有读取到则等待接收信号量 */
            while (rt_device_read(dev,-1,&com_data,1) != 1)
            {
                /* 阻塞等待接收信号量,等到信号量后再次读取数据 */
                rt_sem_take(&opt_uart_rx_sem,RT_WAITING_FOREVER);
            }
            update_optical_flow_data(T1_001_Plus,com_data);
        } else {
            rt_thread_mdelay(10);
        }
    }
}



/********************************************************************************串口*****************************************************************************************/
static struct rt_semaphore ros_uart_rx_sem;    /* 用于接收消息的信号量 */
/* 接收数据回调函数 */
static rt_err_t ros_uart_input(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断,调用此回调函数,然后发送接收信号量 */
    rt_sem_release(&ros_uart_rx_sem);
    return RT_EOK;
}
#include "mavlink.h"
#include "GCS_Communication.h"
/***********************************************************
  *@brief  光流数据接收
*************************************************************/
void ros_uart_read_thread_entry(void* parameter)//线程2
{
    char com_data;
    rt_device_t dev = serial_device_find(SerialProtocol_ROS);
    if (dev) {
        /* 初始化信号量 */
        rt_sem_init(&ros_uart_rx_sem, "ros_uart_rx_sem", 1, RT_IPC_FLAG_FIFO);
        /* 设置接收回调函数 */
        rt_device_set_rx_indicate(dev,ros_uart_input);
        set_serial_device_baud_rate(dev,230400);
        rt_device_open(dev, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    }

    uint8_t ret;
    static mavlink_message_t msg;
    static mavlink_status_t  status;
    while (1)
    {
        if(dev)
        {
            /* 从串口读取一个字节的数据,没有读取到则等待接收信号量 */
            while (rt_device_read(dev,-1, &com_data, 1) != 1)
            {
                /* 阻塞等待接收信号量,等到信号量后再次读取数据 */
                rt_sem_take(&ros_uart_rx_sem, RT_WAITING_FOREVER);
            }
            /* 读取到的数据通过串口错位输出 */
            ret = mavlink_parse_char(0, com_data, &msg, &status);

            if(MAVLINK_FRAMING_OK == ret)
            {
                Copter_handle_Message(msg);
            }

        } else {
            rt_thread_mdelay(10);
        }
    }
}
/*******************************************************************************************************************************************************************/



/**************************************串口*********************************************/
static struct rt_semaphore SBUS_uart_rx_sem;    /* 用于接收消息的信号量 */
/* 接收数据回调函数 */
static rt_err_t SBUS_uart_input(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断,调用此回调函数,然后发送接收信号量 */
    rt_sem_release(&SBUS_uart_rx_sem);
    return RT_EOK;
}
/***********************************************************
  *@brief  激光数据接收
*************************************************************/
void SBUS_uart_dataRx_Task(void* parameter)//线程2
{
    char com_data;
    rt_device_t dev = serial_device_find(SerialProtocol_Sbus);
    if (dev) {
        /* 初始化信号量 */
        rt_sem_init(&SBUS_uart_rx_sem, "SBUS_uart_rx_sem", 1, RT_IPC_FLAG_FIFO);
        /* 设置接收回调函数 */
        rt_device_set_rx_indicate(dev, SBUS_uart_input);

        struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */

        config.baud_rate = 100000;                //修改波特率为 buad_rate
        config.data_bits = DATA_BITS_9;           //数据位 8
        config.stop_bits = STOP_BITS_2;           //停止位 2
        config.bufsz     = 512;                   //修改缓冲区 buff size 为 512
        config.parity    = PARITY_EVEN;           //偶校验位

        /*控制串口设备。通过控制接口传入命令控制字，与控制参数 */
        rt_device_control(dev, RT_DEVICE_CTRL_CONFIG, &config);
        rt_device_open(dev,RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    }

    while (1)
    {
        if(dev)
        {
            /* 从串口读取一个字节的数据,没有读取到则等待接收信号量 */
            while (rt_device_read(dev,-1, &com_data, 1) != 1)
            {
                /* 阻塞等待接收信号量,等到信号量后再次读取数据 */
                rt_sem_take(&SBUS_uart_rx_sem,RT_WAITING_FOREVER);
            }
            rc_sbus_data_receive(com_data);
        }
        else
        {
            rt_thread_mdelay(10);
        }
    }
}


/***********************************************************
  *@brief  RGB状态指示任务。
*************************************************************/
void rgb_state_thread_entry(void* parameter)  //线程
{
    static uint32_t tick = 0;
    static float scale[3] = {11.0f,23.0f,2.0f};
    while(1)
    {
        rt_thread_mdelay(1);
        LED_1ms_DRV(); //0~20

        if(rate_do_excute(100,tick))  //100Hz
        {
            led_state_update(10);
            adc_measurement_update(scale); //单位：V
        }

        tick++;
    }
}
