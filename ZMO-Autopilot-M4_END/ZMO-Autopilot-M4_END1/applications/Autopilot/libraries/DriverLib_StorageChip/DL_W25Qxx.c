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
#include "DL_W25Qxx.h"

struct rt_spi_device *spi_dev_w25q;  /*设备总线句柄*/
/***********************************************************
  *@brief ：w25qxx通信配置函数初始化
*************************************************************/
int w25qxx_spi_device_init(void)
{
    struct rt_spi_configuration spi_cfg;

    spi_dev_w25q = (struct rt_spi_device *)rt_device_find(W25QXX_SPI_DEVICE_NAME);/* 查找 spi2 设备获取设备句柄 */
    if(spi_dev_w25q!=RT_NULL)
    {
        rt_kprintf("wq25qxx spi_device ok\r\n");
    }
    else
        return RT_ERROR;

    spi_cfg.data_width = 8;
    spi_cfg.mode       = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;
    spi_cfg.max_hz     = 30*1000*1000;  /*30M*/
    rt_spi_configure(spi_dev_w25q,&spi_cfg);

    return RT_EOK;
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(w25qxx_spi_device_init);

int w25qxx_init()
{
    rt_uint16_t  ID;

    ID = w25qxx_read_id();        //读取FLASH ID.
    if(ID==W25Q32||ID==W25Q64)    //读取id成功
    {
        rt_kprintf("W25Qxx is OK!!!\r\n");
    }
    else
        rt_kprintf("W25Qxx is bad!!!\r\n");
    return RT_EOK;

}

INIT_APP_EXPORT(w25qxx_init);

int w25qxx_read_id(void)
{
    rt_uint8_t id[5] = {0};
    rt_uint16_t temp;
    rt_uint8_t w25x_read_id = 0x90;
    rt_thread_mdelay(10);//延时，等待传感器稳定

    if(spi_dev_w25q!=RT_NULL)
    {
        /* 方式1：使用 rt_spi_send_then_recv()发送命令读取ID */
        rt_spi_send_then_recv(spi_dev_w25q, &w25x_read_id, 1, id, 5);
    }
    temp = (((int16_t) id[4]) << 8) | id[3];

    return temp;
}



//读取W25QXX的状态寄存器
//BIT7  6   5   4   3   2   1   0
//SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:默认0,状态寄存器保护位,配合WP使用
//TB,BP2,BP1,BP0:FLASH区域写保护设置
//WEL:写使能锁定
//BUSY:忙标记位(1,忙;0,空闲)
//默认:0x00
rt_uint8_t W25QXX_ReadSR(void)
{
    rt_uint8_t byte=0;
    rt_uint8_t send_buf = W25X_ReadStatusReg;  //发送读取状态寄存器命令
    struct rt_spi_message msg1,msg2;

    msg1.send_buf    = &send_buf; //发送读取命令
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

    if(spi_dev_w25q!=RT_NULL)
    {
        /*给w25qxx设备读取和发送消息*/
        rt_spi_transfer_message(spi_dev_w25q, &msg1);//发送消息
    }
    return byte;
}

//写W25QXX状态寄存器
//只有SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)可以写!!!
void W25QXX_Write_SR(rt_uint8_t sr)
{
    if(spi_dev_w25q!=RT_NULL)
    {
        rt_spi_send(spi_dev_w25q, &sr, 1);
    }
}


//W25QXX写使能
//将WEL置位
void W25QXX_Write_Enable(void)
{
    rt_uint8_t sr = W25X_WriteEnable;
    if(spi_dev_w25q!=RT_NULL)
    {
        rt_spi_send(spi_dev_w25q, &sr, 1);
    }
}


//读取SPI FLASH
//在指定地址开始读取指定长度的数据
//pBuffer:数据存储区
//ReadAddr:开始读取的地址(24bit)
//NumByteToRead:要读取的字节数(最大65535)
void w25qxx_Read(rt_uint8_t* pBuffer,rt_uint32_t ReadAddr,rt_uint16_t NumByteToRead)
{
    rt_uint8_t addr[4];
    addr[0]= W25X_ReadData;
    addr[1]= ((rt_uint8_t)((ReadAddr)>>16));
    addr[2]= ((rt_uint8_t)((ReadAddr)>>8));
    addr[3]= ((rt_uint8_t)(ReadAddr));

    struct rt_spi_message msg1,msg2;

    msg1.send_buf    = addr; //发送读取命令
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

    if(spi_dev_w25q!=RT_NULL)
    {
        /*给w25qxx设备读取和发送消息*/
        rt_spi_transfer_message(spi_dev_w25q, &msg1);//发送消息
    }

}

//SPI在一页(0~65535)内写入少于256个字节的数据
//在指定地址开始写入最大256字节的数据
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大256),该数不应该超过该页的剩余字节数!!!
void w25qxx_write_page(rt_uint8_t* pBuffer,rt_uint32_t WriteAddr,rt_uint16_t NumByteToWrite)
{
    rt_uint8_t addr[4];
    addr[0]= W25X_PageProgram;                 //发送写页命令
    addr[1]= ((rt_uint8_t)((WriteAddr)>>16));
    addr[2]= ((rt_uint8_t)((WriteAddr)>>8));
    addr[3]= ((rt_uint8_t)(WriteAddr));

    struct rt_spi_message msg1,msg2;

    W25QXX_Write_Enable();        //SET WEL

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

    if(spi_dev_w25q!=RT_NULL)
    {
        /*给w25qxx设备读取和发送消息*/
        rt_spi_transfer_message(spi_dev_w25q, &msg1);//发送消息
    }

    W25QXX_Wait_Busy();                    //等待写入结束
}

//无检验写SPI FLASH
//必须确保所写的地址范围内的数据全部为0XFF,否则在非0XFF处写入的数据将失败!
//具有自动换页功能
//在指定地址开始写入指定长度的数据,但是要确保地址不越界!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大65535)
//CHECK OK
void w25qxx_write_nocheck(rt_uint8_t* pBuffer,rt_uint32_t WriteAddr,rt_uint16_t NumByteToWrite)
{
    rt_uint16_t pageremain;
    pageremain=256-WriteAddr%256; //单页剩余的字节数
    if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//不大于256个字节
    while(1)
    {
        w25qxx_write_page(pBuffer,WriteAddr,pageremain);
        if(NumByteToWrite==pageremain)break;//写入结束了
        else //NumByteToWrite>pageremain
        {
            pBuffer+=pageremain;
            WriteAddr+=pageremain;

            NumByteToWrite-=pageremain;           //减去已经写入了的字节数
            if(NumByteToWrite>256)pageremain=256; //一次可以写入256个字节
            else pageremain=NumByteToWrite;       //不够256个字节了
        }
    };
}

//写SPI FLASH
//在指定地址开始写入指定长度的数据
//该函数带擦除操作!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大65535)
rt_uint8_t W25QXX_BUFFER[4096];
void w25qxx_Write(rt_uint8_t* pBuffer,rt_uint32_t WriteAddr,rt_uint16_t NumByteToWrite)
{
    rt_uint32_t secpos;
    rt_uint16_t secoff;
    rt_uint16_t secremain;

    rt_uint16_t i;
    rt_uint8_t * W25QXX_BUF;
    W25QXX_BUF=W25QXX_BUFFER;
    secpos=WriteAddr/4096;//扇区地址
    secoff=WriteAddr%4096;//在扇区内的偏移
    secremain=4096-secoff;//扇区剩余空间大小

    if(NumByteToWrite<=secremain)secremain=NumByteToWrite;//不大于4096个字节
    while(1)
    {
        w25qxx_Read(W25QXX_BUF,secpos*4096,4096);//读出整个扇区的内容
        for(i=0;i<secremain;i++)//校验数据
        {
            if(W25QXX_BUF[secoff+i]!=0XFF)break;//需要擦除
        }
        if(i<secremain)//需要擦除
        {
            W25QXX_Erase_Sector(secpos);//擦除这个扇区
            for(i=0;i<secremain;i++)       //复制
            {
                W25QXX_BUF[i+secoff]=pBuffer[i];
            }
            w25qxx_write_nocheck(W25QXX_BUF,secpos*4096,4096);//写入整个扇区

        }
        else
            w25qxx_write_nocheck(pBuffer,WriteAddr,secremain);//写已经擦除了的,直接写入扇区剩余区间.

        if(NumByteToWrite==secremain)break;//写入结束了
        else//写入未结束
        {
            secpos++;//扇区地址增1
            secoff=0;//偏移位置为0
            pBuffer+=secremain;   //指针偏移
            WriteAddr+=secremain; //写地址偏移
            NumByteToWrite-=secremain;               //字节数递减
            if(NumByteToWrite>4096) secremain=4096;  //下一个扇区还是写不完
            else secremain=NumByteToWrite;           //下一个扇区可以写完了
        }
    };
}


//擦除整个芯片
//等待时间超长...
void W25QXX_Erase_Chip(void)
{
    rt_uint8_t sr=(W25X_ChipErase);        //发送片擦除命令

    W25QXX_Write_Enable();                  //SET WEL
    W25QXX_Wait_Busy();
    if(spi_dev_w25q!=RT_NULL)
    {
        rt_spi_send(spi_dev_w25q, &sr, 1);
    }
    W25QXX_Wait_Busy();                    //等待芯片擦除结束
}
//擦除一个扇区
//Dst_Addr:扇区地址 根据实际容量设置
//擦除一个山区的最少时间:150ms
void W25QXX_Erase_Sector(rt_uint32_t Dst_Addr)
{
    rt_uint8_t sr[4];
    rt_uint32_t Addr=Dst_Addr*4096;
    sr[0] = W25X_SectorErase;                 //发送扇区擦除指令
    sr[1] = ((rt_uint8_t)((Addr)>>16));
    sr[2] = ((rt_uint8_t)((Addr)>>8));
    sr[3] = ((rt_uint8_t)(Addr));

    W25QXX_Write_Enable();                  //SET WEL
    W25QXX_Wait_Busy();
    if(spi_dev_w25q!=RT_NULL)
    {
        rt_spi_send(spi_dev_w25q,sr, 4);
    }
    W25QXX_Wait_Busy();                   //等待擦除完成
}
//等待空闲
void W25QXX_Wait_Busy(void)
{
   while((W25QXX_ReadSR()&0x01)==0x01);   // 等待BUSY位清空
}

