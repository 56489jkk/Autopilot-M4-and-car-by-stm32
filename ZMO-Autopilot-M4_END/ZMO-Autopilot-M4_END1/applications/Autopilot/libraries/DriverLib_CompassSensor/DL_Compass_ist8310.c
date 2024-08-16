/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-10-01     Administrator       the first version
 */

#include "DL_Compass_ist8310.h"
#include "drv_spi.h"
#define ist_I2C_BUS_NAME      "i2c2"        /* 传感器连接的I2C总线设备名称 */
struct rt_i2c_bus_device *hmc_i2c_bus;      /* I2C总线设备句柄 */

/* 读传感器寄存器数据 */
static rt_err_t read_regs( rt_uint8_t addr, rt_uint8_t len, rt_uint8_t *buf)
{
    struct rt_i2c_msg msgs;

    msgs.addr = addr;     /* 从机地址 */
    msgs.flags = RT_I2C_RD;     /* 读标志 */
    msgs.buf = buf;             /* 读写数据缓冲区指针　*/
    msgs.len = len;             /* 读写数据字节数 */

    /* 调用I2C设备接口传输数据 */
    if (rt_i2c_transfer(hmc_i2c_bus, &msgs, 1) == 1)
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}
/* 写传感器寄存器 */
static rt_err_t write_reg(rt_uint8_t addr, rt_uint8_t reg, rt_uint8_t data)
{
    rt_uint8_t buf[3];
    struct rt_i2c_msg msgs;

    buf[0] = reg;    //cmd
    buf[1] = data;

    msgs.addr = addr;
    msgs.flags = RT_I2C_WR;
    msgs.buf = buf;
    msgs.len = 2;

    /* 调用I2C设备接口传输数据 */
    if (rt_i2c_transfer(hmc_i2c_bus, &msgs, 1) == 1)
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}
static rt_err_t write_reg1(rt_uint8_t addr, rt_uint8_t reg, rt_uint8_t data)
{
    rt_uint8_t buf[3];
    struct rt_i2c_msg msgs;

    buf[0] = reg;    //cmd

    msgs.addr = addr;
    msgs.flags = RT_I2C_WR;
    msgs.buf = buf;
    msgs.len = 1;

    /* 调用I2C设备接口传输数据 */
    if (rt_i2c_transfer(hmc_i2c_bus, &msgs, 1) == 1)
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}
#define IST8310_SLAVE_ADDRESS     0x0C
#define IST8310_REG_STB           0x0C  //Self-Test response
#define IST8310_REG_INFO          0x01  //More Info
#define IST8310_REG_WIA           0x00  //Who I am
#define IST8310_REG_DATAX         0x03  //Output Value x
#define IST8310_REG_DATAY         0x05  //Output Value y
#define IST8310_REG_DATAZ         0x07  //Output Value z
#define IST8310_REG_STAT1         0x02  //Status register
#define IST8310_REG_STAT2         0x09  //Status register
#define IST8310_REG_CNTRL1        0x0A  //Control setting register 1
#define IST8310_REG_CNTRL2        0x0B  //Control setting register 2
#define IST8310_REG_CNTRL3        0x0D  //Control setting register 3
#define IST8310_REG_OFFSET_START  0xDC  //Offset
#define IST8310_REG_SELECTION_REG 0x42   //Sensor Selection register
#define IST8310_REG_TEST_REG      0x40   //Chip Test register
#define IST8310_REG_TUNING_REG    0x47    //Bandgap Tuning register
/*---IST8310 cross-axis matrix Address-----------------danny-----*/
#define IST8310_REG_XX_CROSS_L    0x9C  //cross axis xx low byte
#define IST8310_REG_XX_CROSS_H    0x9D  //cross axis xx high byte
#define IST8310_REG_XY_CROSS_L    0x9E  //cross axis xy low byte
#define IST8310_REG_XY_CROSS_H    0x9F  //cross axis xy high byte
#define IST8310_REG_XZ_CROSS_L    0xA0  //cross axis xz low byte
#define IST8310_REG_XZ_CROSS_H    0xA1  //cross axis xz high byte                          =       ;
#define IST8310_REG_YX_CROSS_L    0xA2  //cross axis yx low byte
#define IST8310_REG_YX_CROSS_H    0xA3  //cross axis yx high byte
#define IST8310_REG_YY_CROSS_L    0xA4  //cross axis yy low byte
#define IST8310_REG_YY_CROSS_H    0xA5  //cross axis yy high byte
#define IST8310_REG_YZ_CROSS_L    0xA6  //cross axis yz low byte
#define IST8310_REG_YZ_CROSS_H    0xA7  //cross axis yz high byte                    =       ;
#define IST8310_REG_ZX_CROSS_L    0xA8  //cross axis zx low byte
#define IST8310_REG_ZX_CROSS_H    0xA9  //cross axis zx high byte
#define IST8310_REG_ZY_CROSS_L    0xAA  //cross axis zy low byte
#define IST8310_REG_ZY_CROSS_H    0xAB  //cross axis zy high byte
#define IST8310_REG_ZZ_CROSS_L    0xAC  //cross axis zz low byte
#define IST8310_REG_ZZ_CROSS_H    0xAD  //cross axis zz high byte
#define IST8310_AXES_NUM          3
uint8_t Mag_Health;

int ist8310Init(void)//GPS内置磁力计初始化
{
    /* 查找I2C总线设备，获取I2C总线设备句柄 */
    hmc_i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(ist_I2C_BUS_NAME);
    if (!hmc_i2c_bus) {
        return RT_ERROR;
    }

    rt_thread_mdelay(50);
    write_reg(IST8310_SLAVE_ADDRESS, 0x41, 0x24);// Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 75Hz ; normal measurement mode
    write_reg(IST8310_SLAVE_ADDRESS, 0x42, 0xC0);// Configuration Register B  -- 001 00000    configuration gain 1.3Ga

    return RT_EOK;
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(ist8310Init);


int ist8310_get_magn_raw(Vector3i16*raw_magn)
{
    if (!hmc_i2c_bus) {
        return RT_ERROR;
    }

    static uint16_t ist8310_sampling_cnt=0;
    uint8_t buf[6]={0};
    ist8310_sampling_cnt++;
    if(ist8310_sampling_cnt==1)
    {
        write_reg(IST8310_SLAVE_ADDRESS,IST8310_REG_CNTRL1,0x01);//Single Measurement Mode
    }
    else if(ist8310_sampling_cnt==(2))
    {
        write_reg1(IST8310_SLAVE_ADDRESS,0x03,0);
        read_regs(IST8310_SLAVE_ADDRESS,1,&buf[0]);

        write_reg1(IST8310_SLAVE_ADDRESS,0x04,0);
        read_regs(IST8310_SLAVE_ADDRESS,1,&buf[1]);

        write_reg1(IST8310_SLAVE_ADDRESS,0x05,0);
        read_regs(IST8310_SLAVE_ADDRESS,1,&buf[2]);

        write_reg1(IST8310_SLAVE_ADDRESS,0x06,0);
        read_regs(IST8310_SLAVE_ADDRESS,1,&buf[3]);

        write_reg1(IST8310_SLAVE_ADDRESS,0x07,0);
        read_regs(IST8310_SLAVE_ADDRESS,1,&buf[4]);

        write_reg1(IST8310_SLAVE_ADDRESS,0x08,0);
        read_regs(IST8310_SLAVE_ADDRESS,1,&buf[5]);

        raw_magn->x= (int16_t)((buf[1]<<8)|buf[0]);
        raw_magn->y=-(int16_t)((buf[3]<<8)|buf[2]);
        raw_magn->z= (int16_t)((buf[5]<<8)|buf[4]);
        ist8310_sampling_cnt=0;
        return 1;
    }
    return 0;
}













