/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-07-19     李梦辉Q       the first version
 */
#include "loco_config.h"
#include "DL_FM25Vxx.h"

struct rt_spi_device *spi_dev_fm25v;  /*设备总线句柄*/
/***********************************************************
  *@brief ：FM25Vxx通信配置函数初始化
*************************************************************/
int fm25vxx_spi_device_init(void)
{
    struct rt_spi_configuration spi_cfg;

    spi_dev_fm25v = (struct rt_spi_device *)rt_device_find(FM25V_SPI_DEVICE_NAME); /* 查找 spi2 设备获取设备句柄 */
    if(spi_dev_fm25v!=RT_NULL){
        rt_kprintf("fm25vxx spi_device ok\r\n");
    }
    else{
        rt_kprintf("fm25vxx spi_device NO\r\n");
        return RT_ERROR;
    }

    spi_cfg.data_width = 8;
    spi_cfg.mode       = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;
    spi_cfg.max_hz     = 30*1000*1000;  /*30M*/
    rt_spi_configure(spi_dev_fm25v,&spi_cfg);

    return RT_EOK;
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(fm25vxx_spi_device_init);


int fm25vxx_init()
{
    rt_uint16_t  ID;
    ID = fm25vxx_read_id();         //读取FLASH ID.
    if(ID==FM25V02||ID==FM25V05)    //读取id成功
    {
        rt_kprintf("fm25vxx is OK!!!\r\n");
    }
    else
        rt_kprintf("fm25vxx is bad!!!\r\n");
    return RT_EOK;

}
INIT_APP_EXPORT(fm25vxx_init);


rt_uint8_t FM25V0XX_ReadSR(void)
{
    rt_uint8_t byte=0;
    rt_uint8_t send_buf = FM25_RDSR;  //发送读取状态寄存器命令
    struct rt_spi_message msg1,msg2;

    msg1.send_buf    = &send_buf;
    msg1.recv_buf    = RT_NULL;
    msg1.length      = 1;
    msg1.cs_take     = 1;
    msg1.cs_release  = 0;
    msg1.next        = &msg2;

    msg2.send_buf    = RT_NULL;
    msg2.recv_buf    = &byte;
    msg2.length      = 1;
    msg2.cs_take     = 0;
    msg2.cs_release  = 1;
    msg2.next        = RT_NULL;

    if(spi_dev_fm25v!=RT_NULL)
    {
        /*给w25qxx设备读取和发送消息*/
        rt_spi_transfer_message(spi_dev_fm25v, &msg1);//发送消息
    }
    return byte;
}

static void FM25v_write_enable(void)
{
    rt_uint8_t sr = FM25_WREN;
    if(spi_dev_fm25v!=RT_NULL)
    {
        rt_spi_send(spi_dev_fm25v, &sr, 1);
    }
}

static void FM25v_write_disable(void)
{
    rt_uint8_t sr = FM25_WRDI;
    if(spi_dev_fm25v!=RT_NULL)
    {
        rt_spi_send(spi_dev_fm25v, &sr, 1);
    }
}
//等待空闲
void FM25V0XX_Wait_Busy(void)
{
   while((FM25V0XX_ReadSR()&0x01)==0x01);   // 等待BUSY位清空
}


int fm25vxx_read_id(void)
{
    rt_uint8_t id[9] = {0};
    rt_uint16_t temp;
    rt_uint8_t fm25x_read_id = FM25_RDID;
    rt_thread_mdelay(10);//延时，等待传感器稳定

    if(spi_dev_fm25v!=RT_NULL)
    {
        /* 方式1：使用 rt_spi_send_then_recv()发送命令读取ID */
        rt_spi_send_then_recv(spi_dev_fm25v, &fm25x_read_id, 1, id,9);
    }
    temp = (((int16_t) id[7]) << 8) | id[8];

    return temp;
}

void fm25vxx_write(rt_uint8_t* pBuffer,rt_uint32_t WriteAddr,rt_uint16_t NumByteToWrite)
{
    rt_uint8_t addr[4];
    addr[0]= FM25_WRITE;                       //发送写命令
    addr[1]= ((rt_uint8_t)((WriteAddr)>>16));
    addr[2]= ((rt_uint8_t)((WriteAddr)>>8));
    addr[3]= ((rt_uint8_t)(WriteAddr));

    struct rt_spi_message msg1,msg2;

    FM25v_write_enable();;        //SET WEL

    msg1.send_buf    = addr; //发送读取命令
    msg1.recv_buf    = RT_NULL;
    msg1.length      = 4;
    msg1.cs_take     = 1;
    msg1.cs_release  = 0;
    msg1.next        = &msg2;

    msg2.send_buf    = pBuffer;
    msg2.recv_buf    = RT_NULL;
    msg2.length      = NumByteToWrite;
    msg2.cs_take     = 0;
    msg2.cs_release  = 1;
    msg2.next        = RT_NULL;

    if(spi_dev_fm25v!=RT_NULL)
    {
        /*给FM25Vxx设备读取和发送消息*/
        rt_spi_transfer_message(spi_dev_fm25v, &msg1);//发送消息
    }

    FM25V0XX_Wait_Busy();                    //等待写入结束
    FM25v_write_disable();
}

void fm25vxx_Read(rt_uint8_t* pBuffer,rt_uint32_t ReadAddr,rt_uint16_t NumByteToRead)
{
    rt_uint8_t addr[4];
    addr[0]= FM25_READ;
    addr[1]= ((rt_uint8_t)((ReadAddr)>>16));
    addr[2]= ((rt_uint8_t)((ReadAddr)>>8));
    addr[3]= ((rt_uint8_t)(ReadAddr));

    struct rt_spi_message msg1,msg2;

    msg1.send_buf    = addr;      //发送读取命令
    msg1.recv_buf    = RT_NULL;
    msg1.length      = 4;
    msg1.cs_take     = 1;
    msg1.cs_release  = 0;
    msg1.next        = &msg2;

    msg2.send_buf    = RT_NULL;
    msg2.recv_buf    = pBuffer;
    msg2.length      = NumByteToRead;
    msg2.cs_take     = 0;
    msg2.cs_release  = 1;
    msg2.next        = RT_NULL;

    if(spi_dev_fm25v!=RT_NULL)
    {
        /*给FM25Vxx设备读取和发送消息*/
        rt_spi_transfer_message(spi_dev_fm25v, &msg1);//发送消息
    }

}
