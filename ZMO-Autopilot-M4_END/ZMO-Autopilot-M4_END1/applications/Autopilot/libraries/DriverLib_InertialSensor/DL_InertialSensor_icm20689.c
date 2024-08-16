/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-01     CGY       the first version
 */
#include "DL_InertialSensor_icm20689.h"
#include "drv_spi.h"
#include "board_interface.h"
uint8_t icm20689_instance = 0xff;  //必须初始化为0xff

struct rt_spi_device *spi_dev_icm20689 = RT_NULL;     /*总线设备句柄 */
/***********************************************************
  *@brief ：spi通信配置函数初始化
*************************************************************/
int icm20689_spi_device_init(void)
{
    struct rt_spi_configuration spi_cfg;

    spi_dev_icm20689 = (struct rt_spi_device *)rt_device_find(ICM20689_SPI_DEVICE_NAME);/* 查找 spi2 设备获取设备句柄 */
    if (spi_dev_icm20689) {
        imu.instance++;                   //查找设备成功时，imu例化的个数加一，
        icm20689_instance = imu.instance-1; //当前的例化数值赋给此传感器。
    }
    else return RT_ERROR;

    spi_cfg.data_width = 8;
    spi_cfg.mode       = RT_SPI_MASTER | RT_SPI_MODE_3 | RT_SPI_MSB;
    spi_cfg.max_hz     = 10*1000*1000;  /*10M*/
    rt_spi_configure(spi_dev_icm20689,&spi_cfg);

    return RT_EOK;
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(icm20689_spi_device_init);

/***********************************************************
  *@brief  连续读取icm20602的n个字节数据
  *@param  寄存器的首地址
  *@param  储存读取到的数据
  *@param  要读数据的长度
*************************************************************/
void icm20689_reg_read(rt_uint8_t addr,rt_uint8_t *rev_buf,rt_uint32_t len)
{
    struct rt_spi_message msg1,msg2;

    rt_uint8_t reg =  addr|0x80;
    msg1.send_buf    = &reg;
    msg1.recv_buf    = RT_NULL;
    msg1.length      = 1;
    msg1.cs_take     = 1;
    msg1.cs_release  = 0;
    msg1.next        = &msg2;

    msg2.send_buf    = RT_NULL;
    msg2.recv_buf    = rev_buf;
    msg2.length      = len;
    msg2.cs_take     = 0;
    msg2.cs_release  = 1;
    msg2.next        = RT_NULL;

    /*给icm20602设备读取和发送消息*/
    rt_spi_transfer_message(spi_dev_icm20689, &msg1);//发送消息

}

/***********************************************************
  *@brief   给icm20602写一个字节数据
  *@param   寄存器地址
  *@param   要写入的数据
*************************************************************/
void icm20689_reg_write(rt_uint8_t addr,rt_uint8_t value)
{
    struct rt_spi_message msg1;
    rt_uint8_t send_buf[2];
    send_buf[0] = addr;
    send_buf[1] = value;

    msg1.send_buf   = send_buf;
    msg1.recv_buf   = RT_NULL;
    msg1.length     = 2;
    msg1.cs_take    = 1;
    msg1.cs_release = 1;
    msg1.next       = RT_NULL;

    rt_spi_transfer_message(spi_dev_icm20689, &msg1);//发送消息
}

/***********************************************************
  *@brief   icm20602初始化，配置参数
  *@retval  RT_EOK
*************************************************************/
int icm20689_init(void)
{
    if(spi_dev_icm20689)//查找设备成功时
    {
        rt_uint8_t icm_id;
        icm20689_reg_read(ICM_RA_WHO_AM_I,&icm_id,1);   //读取icm20602的id
        rt_thread_mdelay(10);
        icm20689_reg_write(ICM_RA_PWR_MGMT_1,0x80);
        rt_thread_mdelay(10);
        icm20689_reg_write(ICM_RA_PWR_MGMT_1,0x01);
        rt_thread_mdelay(10);   //延时,等待传感器稳定

        if(icm_id == 0x98)//id读取成功
        {
            /*复位reg*/
            icm20689_reg_write(ICM_RA_SIGNAL_PATH_RESET,0x03);
            rt_thread_mdelay(10);
            /*复位reg*/
            icm20689_reg_write(ICM_RA_USER_CTRL,0x01);
            rt_thread_mdelay(10);

            icm20689_reg_write(ICM_RA_DMP_CFG_1,0x40);//dmp
            rt_thread_mdelay(10);
            icm20689_reg_write(ICM_RA_PWR_MGMT_2,0x00);
            rt_thread_mdelay(10);
            icm20689_reg_write(ICM_RA_SMPLRT_DIV,0);
            rt_thread_mdelay(10);
            icm20689_reg_write(ICM_RA_CONFIG,ICM20602_LPF_20HZ);
            rt_thread_mdelay(10);
            icm20689_reg_write(ICM_RA_GYRO_CONFIG,(3 << 3));
            rt_thread_mdelay(10);
            icm20689_reg_write(ICM_RA_ACCEL_CONFIG,(2 << 3));
            rt_thread_mdelay(10);
            /*加速度计LPF 10HZ*/
            icm20689_reg_write(0X1D,0x05);
            rt_thread_mdelay(10);
            /*关闭低功耗*/
            icm20689_reg_write(0X1E,0x00);
            rt_thread_mdelay(10);
            /*关闭FIFO*/
            icm20689_reg_write(0X23,0x00);
            rt_thread_mdelay(10);
            imu.gyro_healthy[icm20689_instance] = true;
        }
        else
        {
            return RT_ERROR;
        }
    }
    return RT_EOK;
}

INIT_APP_EXPORT(icm20689_init);//自动加入初始化


/****************************************************************************************
  *@brief   读取角速度数据
*****************************************************************************************/
int icm20689_get_gyro_raw(Vector3i16*raw_gyro,uint8_t instance)
{
    rt_uint8_t buffer[6];
    Vector3i16 gyro_buf;
    if(spi_dev_icm20689 && instance == icm20689_instance&&imu.gyro_healthy[icm20689_instance]==true)//查找设备成功并且instance等于初始化被赋值的icm20602_instance。
    {
        icm20689_reg_read(ICM_RA_GYRO_XOUT_H, buffer, 6);
    }
    else return RT_ERROR;

    gyro_buf.x =  ((((int16_t) buffer[0]) << 8) | buffer[1]);
    gyro_buf.y =  ((((int16_t) buffer[2]) << 8) | buffer[3]);
    gyro_buf.z =  ((((int16_t) buffer[4]) << 8) | buffer[5]);

    applySensorAlignment(raw_gyro->axis,gyro_buf.axis,Inertial_Sensor_Rotation);//调整传感器方向，和机体坐标一致

    return RT_EOK;
}

/****************************************************************************************
  *@brief   读取加速度数据
*****************************************************************************************/
int icm20689_get_acce_raw(Vector3i16*raw_acce,uint8_t instance)
{
    rt_uint8_t buffer[6];
    Vector3i16 acce_buf;
    if(spi_dev_icm20689 && instance == icm20689_instance&&imu.gyro_healthy[icm20689_instance]==true)//查找设备成功并且instance等于初始化被赋值的icm20602_instance。
    {
        icm20689_reg_read(ICM_RA_ACCEL_XOUT_H, buffer, 6);
    }
    else return RT_ERROR;

    acce_buf.x =  ((((int16_t) buffer[0]) << 8) | buffer[1]);
    acce_buf.y =  ((((int16_t) buffer[2]) << 8) | buffer[3]);
    acce_buf.z =  ((((int16_t) buffer[4]) << 8) | buffer[5]);


    applySensorAlignment(raw_acce->axis,acce_buf.axis,Inertial_Sensor_Rotation);//调整传感器方向，和机体坐标一致


    return RT_EOK;
}
