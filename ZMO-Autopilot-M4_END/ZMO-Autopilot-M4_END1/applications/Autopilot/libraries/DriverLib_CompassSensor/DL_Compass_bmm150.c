/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-23     CGY       the first version
 */
#include "rtdevice.h"
#include "drv_spi.h"
#include "loco_config.h"
#include "DL_Compass_bmm150.h"

struct rt_spi_device *spi_dev_bmm150 = RT_NULL;     /*总线设备句柄 */

/*挂载 bmm150 到 SPI 总线：*/
int bmm150_spi_device_init(void)
{
    struct rt_spi_configuration spi_cfg;

    spi_dev_bmm150 = (struct rt_spi_device *)rt_device_find(BMM150_SPI_DEVICE_NAME);/* 查找 spi2 设备获取设备句柄 */

    spi_cfg.data_width = 8;
    spi_cfg.mode       = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;
    spi_cfg.max_hz     = 10*1000*1000;  /*10M*/

    if(spi_dev_bmm150)
    rt_spi_configure(spi_dev_bmm150,&spi_cfg);

    return RT_EOK;
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(bmm150_spi_device_init);

/****************************************************************************************
*@brief   读取数据
*@param[in]
*****************************************************************************************/
void bmm150_reg_read(rt_uint8_t addr,rt_uint8_t *rev_buf,rt_uint32_t len)
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

    /*给bmm150设备读取和发送消息*/
    rt_spi_transfer_message(spi_dev_bmm150, &msg1);//发送消息
}

/****************************************************************************************
*@brief   写数据
*@param[in]
*****************************************************************************************/
void bmm150_reg_write(rt_uint8_t addr,rt_uint8_t value)
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

    rt_spi_transfer_message(spi_dev_bmm150, &msg1);//发送消息
}

int bmm150_init(void)//bmm150初始化
{
    if(spi_dev_bmm150)//查找设备成功时
    {
        rt_uint8_t bmm_id;
        bmm150_reg_read(BMM_CHIP_ID,&bmm_id,1);//读取bmm150的id
        rt_thread_mdelay(10);
        bmm150_reg_write(BMM_PWR_CTRL,0x83);
        rt_thread_mdelay(10);
        bmm150_reg_read(BMM_CHIP_ID,&bmm_id,1);//读取bmm150的id
        bmm150_reg_write(0x51,23);
        bmm150_reg_write(0x52,82);
        bmm150_reg_write(0x4C,0x28);
        rt_thread_mdelay(10);
    }
    else
    {
        return RT_ERROR;
    }

     return RT_EOK;
}

INIT_APP_EXPORT(bmm150_init);//自动加入初始化

int bmm150_get_magn_raw(Vector3i16*Raw_magn)
{
    rt_uint8_t buffer[10];
    if(spi_dev_bmm150) //查找设备成功
    {
        bmm150_reg_read(0x40,buffer,10);//读取bmm150
    }
    else  return RT_ERROR;

    //Raw_magn->x = ((buffer[1]<<8)+(buffer[0]&0xf8))>>3;
    Raw_magn->x = buffer[0];
    Raw_magn->y =  buffer[6];

    //Raw_magn->x = (int16_t) buffer[1] << 5 | buffer[0] >> 3;
    //Raw_magn->y = (int16_t) buffer[3] << 5 | buffer[2] >> 5;
    //Raw_magn->z = (int16_t) buffer[5] << 6 | buffer[4] >> 6;

    return RT_EOK;
}



