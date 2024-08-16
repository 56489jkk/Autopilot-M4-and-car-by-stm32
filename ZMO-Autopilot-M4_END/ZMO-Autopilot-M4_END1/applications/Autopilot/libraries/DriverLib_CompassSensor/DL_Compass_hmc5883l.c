/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-10     CGY       the first version
 */
#include "DL_Compass_hmc5883l.h"
#include "drv_spi.h"

#define HMC_ADDRESS             0x1E
#define HMC_DATA_REGISTER       0x03
#define HMC_DATA_REGISTER_SPI   (0x03 | 0x40)
#define HMC58X3_R_CONFA         0x00
#define HMC58X3_R_CONFB         0x01
#define HMC58X3_R_MODE          0x02
#define HMC58X3_R_IDA           0x0A
#define HMC58X3_X_SELF_TEST_GAUSS (+1.16f)       // X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (+1.16f)       // Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08f)       // Z axis level when bias current is applied.
#define SELF_TEST_LOW_LIMIT  (243.0f / 390.0f)    // Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT (575.0f / 390.0f)    // High limit when gain is 5.
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2
uint8_t Mag_Health;
//static bool isInit = false;

//*****B¼Ä´æÆ÷ÅäÖÃÔöÒæÁ¿³Ì***********/
#define MAG_GAIN_SCALE0 1370//0x00   0.88Ga
#define MAG_GAIN_SCALE1 1090//0x20   1.30Ga
#define MAG_GAIN_SCALE2 820//0x40    1.90Ga
#define MAG_GAIN_SCALE3 660//0x60    2.50Ga
#define MAG_GAIN_SCALE4 440//0x80    4.00Ga
#define MAG_GAIN_SCALE5 390//0xA0    4.70Ga
#define MAG_GAIN_SCALE6 330//0xC0    5.66Ga
#define MAG_GAIN_SCALE7 230//0xE0    8.10Ga

#define hmc_I2C_BUS_NAME      "i2c4"        /* 传感器连接的I2C总线设备名称 */
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
static rt_err_t write_reg(rt_uint8_t addr, rt_uint8_t reg, rt_uint8_t *data)
{
    rt_uint8_t buf[3];
    struct rt_i2c_msg msgs;

    buf[0] = reg;    //cmd
    buf[1] = data[0];
    buf[2] = data[1];

    msgs.addr = addr;
    msgs.flags = RT_I2C_WR;
    msgs.buf = buf;
    msgs.len = 3;

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


int hmc5883lInit(void)//GPS内置磁力计初始化
{
    rt_uint8_t temp[2] = {0, 0};
    uint8_t hmc_id = 0x00;
    /* 查找I2C总线设备，获取I2C总线设备句柄 */
    hmc_i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(hmc_I2C_BUS_NAME);
    if (!hmc_i2c_bus) {
        return RT_ERROR;
    }
    write_reg(HMC_ADDRESS, HMC58X3_R_IDA, 0);         /* 发送命令 */ //读取ID
    rt_thread_mdelay(50);
    read_regs(HMC_ADDRESS,1, &hmc_id);                /* 获取传感器数据 */
    temp[0] = 0x78;
    write_reg(HMC_ADDRESS, HMC58X3_R_CONFA, temp);// Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 75Hz ; normal measurement mode
    temp[0] = 0x20;
    write_reg(HMC_ADDRESS, HMC58X3_R_CONFB, temp);// Configuration Register B  -- 001 00000    configuration gain 1.3Ga
    temp[0] = 0x00;
    write_reg(HMC_ADDRESS, HMC58X3_R_MODE, temp);// Mode register             -- 000000 00    continuous Conversion Mode

    return RT_EOK;
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(hmc5883lInit);


int hmc5883l_get_magn_raw(Vector3i16*raw_magn)
{
    if (!hmc_i2c_bus) {
        return RT_ERROR;
    }
    rt_uint8_t buffer[6];
    write_reg(HMC_ADDRESS, HMC_DATA_REGISTER, 0);      /* 发送命令 */
    read_regs(HMC_ADDRESS,6,buffer);                   /* 获取传感器数据 */

    raw_magn->x = (((int16_t) buffer[0]) << 8) | buffer[1];
    raw_magn->z = ((((int16_t) buffer[2]) << 8) | buffer[3]);
    raw_magn->y = ((((int16_t) buffer[4]) << 8) | buffer[5]);

    return RT_EOK;
}





