/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-28     CGY       the first version
 */
#include "DL_InertialSensor_icm42605.h"
#include "drv_spi.h"
uint8_t icm42605_instance = 0xff;  //必须初始化为0xff
struct rt_spi_device *spi_dev_icm42605 = RT_NULL;     /*总线设备句柄 */

/************************************************************
  *@brief ：spi通信配置函数初始化
*************************************************************/
int icm42605_spi_device_init(void)
{
    struct rt_spi_configuration spi_cfg;
    spi_dev_icm42605 = (struct rt_spi_device *)rt_device_find(ICM42605_SPI_DEVICE_NAME);/* 查找 spi2 设备获取设备句柄 */
    if (spi_dev_icm42605) {
        imu.instance++;
        icm42605_instance = imu.instance-1;
    }
    else return RT_ERROR;
    spi_cfg.data_width = 8;
    spi_cfg.mode       = RT_SPI_MASTER | RT_SPI_MODE_3 | RT_SPI_MSB;
    spi_cfg.max_hz     = 10*1000*1000;  /*10M*/

    //spi_dev_icm42605->bus->owner=spi_dev_icm42605;
    rt_spi_configure(spi_dev_icm42605,&spi_cfg);

    return RT_EOK;
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(icm42605_spi_device_init);

/***********************************************************
  *@brief  连续读取icm42605的n个字节数据
  *@param  寄存器的首地址
  *@param  储存读取到的数据
  *@param  要读数据的长度
*************************************************************/
void icm42605_reg_read(rt_uint8_t addr,rt_uint8_t *rev_buf,rt_uint32_t len)
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

    /*给icm42605设备读取和发送消息*/
    rt_spi_transfer_message(spi_dev_icm42605, &msg1);//发送消息
}

/***********************************************************
  *@brief   给icm42605写一个字节数据
  *@param   寄存器地址
  *@param   要写入的数据
*************************************************************/
void icm42605_reg_write(rt_uint8_t addr,rt_uint8_t value)
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

    rt_spi_transfer_message(spi_dev_icm42605, &msg1);//发送消息
}


/***********************************************************
  *@brief   bank change
*************************************************************/
void icm42605_SwitchUserBank(uint8_t bank)    //使能某类型寄存器的读写
{
    icm42605_reg_write(ICM42605_REG_BANK_SEL,bank);
}

/***********************************************************
  *@brief   icm42605初始化，配置参数
  *@retval  RT_EOK
*************************************************************/
int icm42605_init(void)//icm42605初始化
{
    if (spi_dev_icm42605) {
    rt_uint8_t icm_id;
    //struct stm32_spi *spi_drv =  rt_container_of(spi_dev_icm42605->bus, struct stm32_spi, spi_bus); //查找状态
    //abc= spi_drv->handle.State;
    icm42605_reg_read(ICM42605_WHO_AM_I,&icm_id,1);//读取icm42605的id   0x42  = 66   先读id
    //abc=icm_id;
    if(icm_id == 0x42)
    {
        icm42605_reg_write(ICM42605_DEVICE_CONFIG, 0x01);     //USR0,软件复位
        rt_thread_mdelay(10); //延时,等待传感器稳定
        icm42605_reg_write(ICM42605_PWR_MGMT0, 0x0F);         //USR0,陀螺仪,加速度 置于低噪声(LN)模式下(使能)
        rt_thread_mdelay(5);
        icm42605_reg_write(ICM42605_GYRO_CONFIG0,0x06);       //USR0,  +-2000dps   ODR:1000hz
        rt_thread_mdelay(5);
        icm42605_reg_write(ICM42605_ACCEL_CONFIG0,0x26);      //USR0,  +-8G        ODR:1000hz
        rt_thread_mdelay(5);
        icm42605_reg_write(ICM42605_GYRO_CONFIG1,0x26);       //USR0,  temp_1ms    GYRO 2_Order
        rt_thread_mdelay(5);
        //icm42605_reg_write(ICM42605_GYRO_ACCEL_CONFIG0,0x00); //USR0,  BW=ODR/2
        icm42605_reg_write(ICM42605_GYRO_ACCEL_CONFIG0,0x76); //USR0,  A_BW=ODR/40   G_BW=ODR/20
        rt_thread_mdelay(5);
        icm42605_reg_write(ICM42605_ACCEL_CONFIG1,0x0C);      //USR0,   ACCEL 2_Order
        rt_thread_mdelay(5);

        //icm42605_SwitchUserBank(1);    //使能某类型寄存器的读写
        //icm42605_reg_write(ICM42605_SENSOR_CONFIG0, 0x00);     //USR1,  开启陀螺仪每个轴，默认开启

        //Set_Gyro_Sum_Cnt(Calibration_Gyro_Cycle);
        imu.gyro_healthy[icm42605_instance] = true;
    }
    else
    {
        return RT_ERROR;
    }
    }
    return RT_EOK;
}

INIT_APP_EXPORT(icm42605_init);//自动加入初始化

/****************************************************************************************
  *@brief   读取角速度数据
*****************************************************************************************/
int icm42605_get_gyro_raw(Vector3i16*raw_gyro,uint8_t instance)
{
    rt_uint8_t buffer[6];
    Vector3i16 gyro_buf;
    if (spi_dev_icm42605 && instance == icm42605_instance&&imu.gyro_healthy[icm42605_instance]==true) {
        icm42605_reg_read(ICM42605_GYRO_XOUT_H, buffer, 6);
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
int icm42605_get_acce_raw(Vector3i16*raw_acce,uint8_t instance)
{
    rt_uint8_t buffer[6];
    Vector3i16 acce_buf;
    if (spi_dev_icm42605 && instance == icm42605_instance&&imu.gyro_healthy[icm42605_instance]==true) {
        icm42605_reg_read(ICM42605_ACCEL_XOUT_H, buffer, 6);
    }
    else return RT_ERROR;

    acce_buf.x =  ((((int16_t) buffer[0]) << 8) | buffer[1]);
    acce_buf.y =  ((((int16_t) buffer[2]) << 8) | buffer[3]);
    acce_buf.z =  ((((int16_t) buffer[4]) << 8) | buffer[5]);

    applySensorAlignment(raw_acce->axis,acce_buf.axis,Inertial_Sensor_Rotation);//调整传感器方向，和机体坐标一致

    return RT_EOK;
}
