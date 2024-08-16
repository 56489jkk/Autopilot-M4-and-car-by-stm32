/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-28     CGY       the first version
 */

#include "DL_Baro_spl06.h"
#include "drv_spi.h"
struct rt_spi_device *spi_dev_spl06 = RT_NULL;     /*总线设备句柄 */
/*挂载 spl06 到 SPI 总线：*/
int spl06_spi_device_init(void)
{
    struct rt_spi_configuration spi_cfg;

    spi_dev_spl06 = (struct rt_spi_device *)rt_device_find(SPL06_SPI_DEVICE_NAME);/* 查找 spi2 设备获取设备句柄 */
    if(!spi_dev_spl06)   return RT_ERROR;
    spi_cfg.data_width = 8;
    spi_cfg.mode       = RT_SPI_MASTER | RT_SPI_MODE_3 | RT_SPI_MSB;
    spi_cfg.max_hz     = 10*1000*1000;                                                     /*10M*/
    rt_spi_configure(spi_dev_spl06,&spi_cfg);

    return RT_EOK;
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(spl06_spi_device_init);

uint8_t spl06_reg_read(rt_uint8_t addr)
{
    struct rt_spi_message msg1,msg2;
    uint8_t reg_data[1];
    rt_uint8_t reg =  addr|0x80;
    msg1.send_buf    = &reg;
    msg1.recv_buf    = RT_NULL;
    msg1.length      = 1;
    msg1.cs_take     = 1;
    msg1.cs_release  = 0;
    msg1.next        = &msg2;

    msg2.send_buf    = RT_NULL;
    msg2.recv_buf    = reg_data;
    msg2.length      = 1;
    msg2.cs_take     = 0;
    msg2.cs_release  = 1;
    msg2.next        = RT_NULL;

    /*给spl06设备读取和发送消息*/
    rt_spi_transfer_message(spi_dev_spl06, &msg1);//发送消息

    return reg_data[0];
}

/***********************************************************
  *@brief  连续读取spl06的n个字节数据
  *@param  寄存器的首地址
  *@param  储存读取到的数据
  *@param  要读数据的长度
*************************************************************/
void spl06_reg_read_nbyte(rt_uint8_t addr,rt_uint8_t *rev_buf,rt_uint32_t len)
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

    /*给spl06设备读取和发送消息*/
    rt_spi_transfer_message(spi_dev_spl06, &msg1);//发送消息
}

void spl06_reg_write(rt_uint8_t addr,rt_uint8_t value)
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

    rt_spi_transfer_message(spi_dev_spl06, &msg1);//发送消息
}
static struct spl0601_t spl0601;
static struct spl0601_t *p_spl0601;
/*****************************************************************************
 函 数 名  : spl0601_rateset
 功能描述  :  设置温度传感器的每秒采样次数以及过采样率
 输入参数  : uint8 u8OverSmpl  过采样率         Maximal = 128
         uint8 u8SmplRate  每秒采样次数(Hz) Maximal = 128
         uint8 iSensor     0: Pressure; 1: Temperature
*****************************************************************************/
void spl0601_rateset ( uint8_t iSensor, uint8_t u8SmplRate, uint8_t u8OverSmpl )
{
    uint8_t reg = 0;
    int32_t i32kPkT = 0;
    switch ( u8SmplRate )
    {
        case 2:
            reg |= ( 1 << 4 );
            break;
        case 4:
            reg |= ( 2 << 4 );
            break;
        case 8:
            reg |= ( 3 << 4 );
            break;
        case 16:
            reg |= ( 4 << 4 );
            break;
        case 32:
            reg |= ( 5 << 4 );
            break;
        case 64:
            reg |= ( 6 << 4 );
            break;
        case 128:
            reg |= ( 7 << 4 );
            break;
        case 1:
        default:
            break;
    }
    switch ( u8OverSmpl )
    {
        case 2:
            reg |= 1;
            i32kPkT = 1572864;
            break;
        case 4:
            reg |= 2;
            i32kPkT = 3670016;
            break;
        case 8:
            reg |= 3;
            i32kPkT = 7864320;
            break;
        case 16:
            i32kPkT = 253952;
            reg |= 4;
            break;
        case 32:
            i32kPkT = 516096;
            reg |= 5;
            break;
        case 64:
            i32kPkT = 1040384;
            reg |= 6;
            break;
        case 128:
            i32kPkT = 2088960;
            reg |= 7;
            break;
        case 1:
        default:
            i32kPkT = 524288;
            break;
    }

    if ( iSensor == 0 )
    {
        p_spl0601->i32kP = i32kPkT;
        spl06_reg_write ( 0x06, reg );
        if ( u8OverSmpl > 8 )
        {
            reg = spl06_reg_read ( 0x09 );
            spl06_reg_write ( 0x09, reg | 0x04 );
        }
    }
    if ( iSensor == 1 )
    {
        p_spl0601->i32kT = i32kPkT;
        spl06_reg_write ( 0x07, reg | 0x80 ); //Using mems temperature
        if ( u8OverSmpl > 8 )
        {
            reg = spl06_reg_read ( 0x09 );
            spl06_reg_write ( 0x09, reg | 0x08 );
        }
    }
}

/*****************************************************************************
 函 数 名  : spl0601_get_calib_param
 功能描述  : 获取校准参数
 输入参数  : void
 输出参数  : 无
*****************************************************************************/
void spl0601_get_calib_param ( void )
{
    uint32_t h;
    uint32_t m;
    uint32_t l;
    h =  spl06_reg_read( 0x10 );
    l =  spl06_reg_read( 0x11 );
    p_spl0601->calib_param.c0 = ( int16_t ) h << 4 | l >> 4;
    p_spl0601->calib_param.c0 = ( p_spl0601->calib_param.c0 & 0x0800 ) ? ( 0xF000 | p_spl0601->calib_param.c0 ) : p_spl0601->calib_param.c0;
    h =  spl06_reg_read( 0x11 );
    l =  spl06_reg_read( 0x12 );
    p_spl0601->calib_param.c1 = ( int16_t ) ( h & 0x0F ) << 8 | l;
    p_spl0601->calib_param.c1 = ( p_spl0601->calib_param.c1 & 0x0800 ) ? ( 0xF000 | p_spl0601->calib_param.c1 ) : p_spl0601->calib_param.c1;
    h =  spl06_reg_read( 0x13 );
    m =  spl06_reg_read( 0x14 );
    l =  spl06_reg_read( 0x15 );
    p_spl0601->calib_param.c00 = ( int32_t ) h << 12 | ( int32_t ) m << 4 | ( int32_t ) l >> 4;
    p_spl0601->calib_param.c00 = ( p_spl0601->calib_param.c00 & 0x080000 ) ? ( 0xFFF00000 | p_spl0601->calib_param.c00 ) : p_spl0601->calib_param.c00;
    h =  spl06_reg_read( 0x15 );
    m =  spl06_reg_read( 0x16 );
    l =  spl06_reg_read( 0x17 );
    p_spl0601->calib_param.c10 = ( int32_t ) h << 16 | ( int32_t ) m << 8 | l;
    p_spl0601->calib_param.c10 = ( p_spl0601->calib_param.c10 & 0x080000 ) ? ( 0xFFF00000 | p_spl0601->calib_param.c10 ) : p_spl0601->calib_param.c10;
    h =  spl06_reg_read( 0x18 );
    l =  spl06_reg_read( 0x19 );
    p_spl0601->calib_param.c01 = ( int16_t ) h << 8 | l;
    h =  spl06_reg_read( 0x1A );
    l =  spl06_reg_read(0x1B );
    p_spl0601->calib_param.c11 = ( int16_t ) h << 8 | l;
    h =  spl06_reg_read( 0x1C );
    l =  spl06_reg_read( 0x1D );
    p_spl0601->calib_param.c20 = ( int16_t ) h << 8 | l;
    h =  spl06_reg_read( 0x1E );
    l =  spl06_reg_read( 0x1F );
    p_spl0601->calib_param.c21 = ( int16_t ) h << 8 | l;
    h =  spl06_reg_read( 0x20 );
    l =  spl06_reg_read( 0x21 );
    p_spl0601->calib_param.c30 = ( int16_t ) h << 8 | l;
}

void spl0601_get_calib_param1( void )
{
    uint8_t buffer[18];
    spl06_reg_read_nbyte(0x10,buffer,18);

    p_spl0601->calib_param.c0 = ( int16_t ) buffer[0] << 4 | buffer[1] >> 4;
    p_spl0601->calib_param.c0 = ( p_spl0601->calib_param.c0 & 0x0800 ) ? ( 0xF000 | p_spl0601->calib_param.c0 ) : p_spl0601->calib_param.c0;

    p_spl0601->calib_param.c1 = ( int16_t ) ( buffer[1] & 0x0F ) << 8 | buffer[2];
    p_spl0601->calib_param.c1 = ( p_spl0601->calib_param.c1 & 0x0800 ) ? ( 0xF000 | p_spl0601->calib_param.c1 ) : p_spl0601->calib_param.c1;

    p_spl0601->calib_param.c00 = ( int32_t ) buffer[3] << 12 | ( int32_t ) buffer[4] << 4 | ( int32_t ) buffer[5] >> 4;
    p_spl0601->calib_param.c00 = ( p_spl0601->calib_param.c00 & 0x080000 ) ? ( 0xFFF00000 | p_spl0601->calib_param.c00 ) : p_spl0601->calib_param.c00;

    p_spl0601->calib_param.c10 = ( int32_t ) buffer[5] << 16 | ( int32_t ) buffer[6] << 8 | buffer[7];
    p_spl0601->calib_param.c10 = ( p_spl0601->calib_param.c10 & 0x080000 ) ? ( 0xFFF00000 | p_spl0601->calib_param.c10 ) : p_spl0601->calib_param.c10;

    p_spl0601->calib_param.c01 = ( int16_t ) buffer[8] << 8 | buffer[9];
    p_spl0601->calib_param.c11 = ( int16_t ) buffer[10] << 8 | buffer[11];
    p_spl0601->calib_param.c20 = ( int16_t ) buffer[12] << 8 | buffer[13];
    p_spl0601->calib_param.c21 = ( int16_t ) buffer[14] << 8 | buffer[15];
    p_spl0601->calib_param.c30 = ( int16_t ) buffer[16] << 8 | buffer[17];
}




/*****************************************************************************
 函 数 名  : spl0601_start_temperature
 功能描述  : 发起一次温度测量
 输入参数  : void
 输出参数  : 无
*****************************************************************************/
void spl0601_start_temperature ( void )
{
    spl06_reg_write( 0x08, 0x02 );
}
/*****************************************************************************
 函 数 名  : spl0601_start_pressure
 功能描述  : 发起一次压力值测量
 输入参数  : void
 输出参数  : 无
*****************************************************************************/
void spl0601_start_pressure ( void )
{
    spl06_reg_write( 0x08, 0x01 );
}
/*****************************************************************************
 函 数 名  : spl0601_start_continuous
 功能描述  : Select node for the continuously measurement
 输入参数  : uint8 mode  1: pressure; 2: temperature; 3: pressure and temperature
*****************************************************************************/
void spl0601_start_continuous ( uint8_t mode )
{
    spl06_reg_write( 0x08, mode + 4 );
}

/*****************************************************************************
 函 数 名  : spl0601_get_raw_temp
 功能描述  : 获取温度的原始值，并转换成32Bits整数
 输入参数  : void
 输出参数  : 无
*****************************************************************************/
void spl0601_get_raw_temp ( void )
{
    uint8_t h[3] = {0};
    spl06_reg_read_nbyte(0x03,h,3);
    p_spl0601->i32rawTemperature = ( int32_t ) h[0] << 16 | ( int32_t ) h[1] << 8 | ( int32_t ) h[2];
    p_spl0601->i32rawTemperature = ( p_spl0601->i32rawTemperature & 0x800000 ) ? ( 0xFF000000 | p_spl0601->i32rawTemperature ) : p_spl0601->i32rawTemperature;
}

/*****************************************************************************
 函 数 名  : spl0601_get_raw_pressure
 功能描述  : 获取压力原始值，并转换成32bits整数
 输入参数  : void
 输出参数  : 无
*****************************************************************************/
void spl0601_get_raw_pressure ( void )
{
    uint8_t h[3];
    spl06_reg_read_nbyte(0x00,h,3);
    p_spl0601->i32rawPressure = ( int32_t ) h[0] << 16 | ( int32_t ) h[1] << 8 | ( int32_t ) h[2];
    p_spl0601->i32rawPressure = ( p_spl0601->i32rawPressure & 0x800000 ) ? ( 0xFF000000 | p_spl0601->i32rawPressure ) : p_spl0601->i32rawPressure;
}

int spl06_init(void)//icm20602初始化
{
    if(!spi_dev_spl06)   return RT_ERROR;
    p_spl0601 = &spl0601; /* read Chip Id */
    p_spl0601->i32rawPressure = 0;
    p_spl0601->i32rawTemperature = 0;
    p_spl0601->chip_id = spl06_reg_read(0x0D);//  0x10
    rt_thread_mdelay(10);
    spl0601_get_calib_param();
    spl0601_rateset(PRESSURE_SENSOR, 128, 16 );

    spl0601_rateset(TEMPERATURE_SENSOR, 8, 8 );

    spl0601_start_continuous ( CONTINUOUS_P_AND_T );

    if(p_spl0601->chip_id == 0x10)
    {
        alt.baro_healthy[0] = true;
        return RT_EOK;
    }
    else
    {
        return RT_ERROR;
    }
}

INIT_APP_EXPORT(spl06_init);//自动加入初始化

/*****************************************************************************
 函 数 名  : spl0601_get_temperature
 功能描述  : 在获取原始值的基础上，返回浮点校准后的温度值
 输入参数  : void
 输出参数  : 无
*****************************************************************************/
float spl0601_get_temperature(void)
{
    if(!spi_dev_spl06)   return RT_ERROR;
    spl0601_get_raw_temp();//读取原始温度数据

    float fTCompensate;
    float fTsc;
    fTsc = p_spl0601->i32rawTemperature / ( float ) p_spl0601->i32kT;
    fTCompensate =  p_spl0601->calib_param.c0 * 0.5 + p_spl0601->calib_param.c1 * fTsc;
    return fTCompensate;
}

/*****************************************************************************
 函 数 名  : spl0601_get_pressure
 功能描述  : 在获取原始值的基础上，返回浮点校准后的压力值
 输入参数  : void
*****************************************************************************/
float spl0601_get_pressure ( void )
{
    if(!spi_dev_spl06)   return RT_ERROR;
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;
    spl0601_get_raw_pressure();//读取原始压强数据

    fTsc = p_spl0601->i32rawTemperature / ( float ) p_spl0601->i32kT;
    fPsc = p_spl0601->i32rawPressure / ( float ) p_spl0601->i32kP;
    qua2 = p_spl0601->calib_param.c10 + fPsc * ( p_spl0601->calib_param.c20 + fPsc * p_spl0601->calib_param.c30 );
    qua3 = fTsc * fPsc * ( p_spl0601->calib_param.c11 + fPsc * p_spl0601->calib_param.c21 );
    fPCompensate = p_spl0601->calib_param.c00 + fPsc * qua2 + fTsc * p_spl0601->calib_param.c01 + qua3;

    return fPCompensate;
}
