/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-29     CGY       the first version
 */
#include "loco_config.h"
#include "DL_Baro_bmp388.h"

static struct bmp388_t bmp388;
static struct bmp388_t *p_bmp388;

struct rt_spi_device *spi_dev_bmp388 = RT_NULL;     /*总线设备句柄 */
/***********************************************************
  *@brief ：spi通信配置函数初始化
*************************************************************/
int bmp388_spi_device_init(void)
{
    struct rt_spi_configuration spi_cfg;

    spi_dev_bmp388 = (struct rt_spi_device *)rt_device_find(BMP388_SPI_DEVICE_NAME);/* 查找 spi2 设备获取设备句柄 */
    if(!spi_dev_bmp388) return RT_ERROR;
    spi_cfg.data_width = 8;
    spi_cfg.mode       = RT_SPI_MASTER |RT_SPI_MODE_3| RT_SPI_MSB;
    spi_cfg.max_hz     = 10*1000*1000;                                                     /*10M*/
    rt_spi_configure(spi_dev_bmp388,&spi_cfg);

    return RT_EOK;
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(bmp388_spi_device_init);

/***********************************************************
  *@brief   读取bmp388的一个字节数据
  *@param  寄存器地址
  *@retval 该寄存器下的数据
*************************************************************/
uint8_t bmp388_reg_read(rt_uint8_t addr)//读一个字节；需要接收两个字节，第一个字节恒为0，第二个字节才是数据，见数据手册SPI读
{
    struct rt_spi_message msg1,msg2;
    uint8_t reg_data[2];
    rt_uint8_t reg =  addr|0x80;
    msg1.send_buf    = &reg;
    msg1.recv_buf    = RT_NULL;
    msg1.length      = 1;
    msg1.cs_take     = 1;
    msg1.cs_release  = 0;
    msg1.next        = &msg2;

    msg2.send_buf    = RT_NULL;
    msg2.recv_buf    = reg_data;
    msg2.length      = 2;
    msg2.cs_take     = 0;
    msg2.cs_release  = 1;
    msg2.next        = RT_NULL;

    /*给icm20602设备读取和发送消息*/
    rt_spi_transfer_message(spi_dev_bmp388, &msg1);//发送消息

    return reg_data[1];
}

/***********************************************************
  *@brief  连续读取bmp388的n个字节数据
  *@param  寄存器的首地址
  *@param  储存读取到的数据
  *@param  要读数据的长度
  *@note   BMP388以sip方式读取数据，首先会返回一个字节为0，下一个字节才是需要的数据，见数据手册SPI读
*************************************************************/
void bmp388_reg_read_nbyte(rt_uint8_t addr,rt_uint8_t *rev_buf,rt_uint32_t len)//读取一个字节必须长度+1，得到的数据取第二个字节；
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

    /*给bmp388设备读取和发送消息*/
    rt_spi_transfer_message(spi_dev_bmp388, &msg1);//发送消息
}

/***********************************************************
  *@brief   给bmp388写一个字节数据
  *@param   寄存器地址
  *@param   要写入的数据
*************************************************************/
void bmp388_reg_write(rt_uint8_t addr,rt_uint8_t value)
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

    rt_spi_transfer_message(spi_dev_bmp388, &msg1);//发送消息
}

/***********************************************************
  *@brief   获取bmp388的校准参数
*************************************************************/
void bmp388_get_calib_param( void )
{
    uint8_t cal_buff[22];
    bmp388_reg_read_nbyte(BMP_ADR,cal_buff,22);  //接收到的第一个字节cal_buff[0]=0，这里直接舍弃，后面的才是真实数据。

    p_bmp388->calib_param.t1 = (int16_t) cal_buff[2] << 8 | cal_buff[1];
    p_bmp388->calib_param.t2 = (int16_t) cal_buff[4] << 8 | cal_buff[3];
    p_bmp388->calib_param.t3 = (int8_t) cal_buff[5];
    p_bmp388->calib_param.p1 = (int16_t) cal_buff[7] << 8 | cal_buff[6];
    p_bmp388->calib_param.p2 = (int16_t) cal_buff[9] << 8 | cal_buff[8];
    p_bmp388->calib_param.p3 = (int8_t) cal_buff[10];
    p_bmp388->calib_param.p4 = (int8_t) cal_buff[11];
    p_bmp388->calib_param.p5 = (int16_t) cal_buff[13] << 8 | cal_buff[12];
    p_bmp388->calib_param.p6 = (int16_t) cal_buff[15] << 8 | cal_buff[14];
    p_bmp388->calib_param.p7 = (int8_t) cal_buff[16];
    p_bmp388->calib_param.p8 = (int8_t) cal_buff[17];
    p_bmp388->calib_param.p9 = (int16_t) cal_buff[19] << 8 | cal_buff[18];
    p_bmp388->calib_param.p10 = (int8_t) cal_buff[20];
    p_bmp388->calib_param.p11 = (int8_t) cal_buff[21];
}

/***********************************************************
  *@brief   获取bmp388的温度的原始值，并转换成32Bits整数
*************************************************************/
void bmp388_get_raw_temp ( void )
{
    uint8_t buffer[4] = {0};
    bmp388_reg_read_nbyte(BMP_DATA_3,buffer,4);  //接收到的第一个字节buffer[0]=0，这里直接舍弃，后面的才是真实数据。
    p_bmp388->i32rawTemperature = ( int32_t ) buffer[3] << 16 | ( int32_t ) buffer[2] << 8 | ( int32_t ) buffer[1];
}

/***********************************************************
  *@brief   获取bmp388的压力原始值，并转换成32bits整数
*************************************************************/
void bmp388_get_raw_pressure ( void )
{
    uint8_t buffer[4];
    bmp388_reg_read_nbyte(BMP_DATA_0,buffer,4);  //接收到的第一个字节h[0]=0，这里直接舍弃，后面的才是真实数据。
    p_bmp388->i32rawPressure = ( int32_t ) buffer[3] << 16 | ( int32_t ) buffer[2] << 8 | ( int32_t ) buffer[1];
}

/***********************************************************
  *@brief   bmp388初始化，配置参数
  *@retval  RT_EOK
*************************************************************/
int bmp388_init(void)
{
    if(!spi_dev_bmp388) return RT_ERROR;
    p_bmp388 = &bmp388; /* read Chip Id */
    p_bmp388->i32rawPressure = 0;
    p_bmp388->i32rawTemperature = 0;
    rt_thread_mdelay(10);
    p_bmp388->chip_id = bmp388_reg_read(BMP_CHIP_ID);  //读取id
    if(p_bmp388->chip_id == BMP_WHO_AM_I)
    {
        bmp388_reg_write(BMP_CMD,0xB6);      //软件复位
        rt_thread_mdelay(10);
        bmp388_get_calib_param( );           //读取bmp388的校准参数
        bmp388_reg_write(BMP_OSR,0x03);      //“OSR”寄存器控制压力和温度测量的过采样设置。P*8 T*1
        bmp388_reg_write(BMP_ODR,0x02);      //“ODR”寄存器通过设置细分/子采样来设置输出数据速率的配置50Hz
        bmp388_reg_write(BMP_CONFIG,0x04);   //“配置”寄存器控制IIR滤波器系数。滤波系数为3
        bmp388_reg_write(BMP_PWR_CTRL,0x33); //寄存器可启用或禁用压力和温度测量 0x31压强  0x32温度  0x33压强和温度。
    }
    else
    {
        return RT_ERROR;
    }

    return RT_EOK;
}

INIT_APP_EXPORT(bmp388_init);//自动加入初始化

int64_t  calib_datalin;
/***********************************************************
  *@brief   在获取原始值的基础上，返回浮点校准后的温度值
  *@retval  以摄氏度为单位的温度值
*************************************************************/
float bmp388_get_temperature(void)
{
    if(!spi_dev_bmp388) return RT_ERROR;
    bmp388_get_raw_temp();//读取原始温度数据

    float fTCompensate;
    uint64_t partial_data1;
    uint64_t partial_data2;
    uint64_t partial_data3;
    int64_t  partial_data4;
    int64_t  partial_data5;
    int64_t  partial_data6;
    int64_t  comp_temp;

    partial_data1 =  p_bmp388->i32rawTemperature - (256 * p_bmp388->calib_param.t1);
    partial_data2 =  p_bmp388->calib_param.t2 * partial_data1;
    partial_data3 = partial_data1 * partial_data1;
    partial_data4 = (int64_t)partial_data3 *  p_bmp388->calib_param.t3;
    partial_data5 = ((int64_t)(partial_data2 * 262144) + partial_data4);
    partial_data6 = partial_data5 / 4294967296;
    calib_datalin = partial_data6;
    comp_temp = (int64_t)((partial_data6 * 25)  / 16384);

    fTCompensate = (float)comp_temp/100.0f;   //修正结果保存在comp结构体内
    return fTCompensate;
}

/***********************************************************
  *@brief   在获取原始值的基础上，返回浮点校准后的压强值
  *@retval  以帕为单位的压强值
*************************************************************/
float bmp388_get_pressure(void)
{
    if(!spi_dev_bmp388) return RT_ERROR;
    float fPCompensate;
    bmp388_get_raw_pressure();//读取原始压强数据
    int64_t partial_data1;
    int64_t partial_data2;
    int64_t partial_data3;
    int64_t partial_data4;
    int64_t partial_data5;
    int64_t partial_data6;
    int64_t offset;
    int64_t sensitivity;
    uint64_t comp_press;

    partial_data1 = calib_datalin * calib_datalin;
    partial_data2 = partial_data1 / 64;
    partial_data3 = (partial_data2 * calib_datalin) / 256;
    partial_data4 = (p_bmp388->calib_param.p8 * partial_data3) / 32;
    partial_data5 = (p_bmp388->calib_param.p7 * partial_data1) * 16;
    partial_data6 = (p_bmp388->calib_param.p6 * calib_datalin) * 4194304;
    offset = (p_bmp388->calib_param.p5 * 140737488355328) + partial_data4 + partial_data5 + partial_data6;

    partial_data2 = (p_bmp388->calib_param.p4 * partial_data3) / 32;
    partial_data4 = (p_bmp388->calib_param.p3 * partial_data1) * 4;
    partial_data5 = (p_bmp388->calib_param.p2 - 16384) * calib_datalin * 2097152;
    sensitivity = ((p_bmp388->calib_param.p1 - 16384) * 70368744177664) + partial_data2 + partial_data4 + partial_data5;

    partial_data1 = (sensitivity / 16777216) * p_bmp388->i32rawPressure;
    partial_data2 = p_bmp388->calib_param.p10 * calib_datalin;
    partial_data3 = partial_data2 + (65536 * p_bmp388->calib_param.p9);
    partial_data4 = (partial_data3 * p_bmp388->i32rawPressure) / 8192;
    partial_data5 = (partial_data4 * p_bmp388->i32rawPressure) / 512;
    partial_data6 = (int64_t)((uint64_t)p_bmp388->i32rawPressure * (uint64_t)p_bmp388->i32rawPressure);
    partial_data2 = (p_bmp388->calib_param.p11 * partial_data6) / 65536;
    partial_data3 = (partial_data2 * p_bmp388->i32rawPressure) / 128;
    partial_data4 = (offset / 4) + partial_data1 + partial_data5 + partial_data3;
    comp_press = (((uint64_t)partial_data4 * 25) / (uint64_t)1099511627776);
    fPCompensate = (float)comp_press/100.0f;

    return fPCompensate;
}


