/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-25     CGY       the first version
 */
#include "loco_config.h"
#include "CL_lcd.h"

//管理LCD重要参数
//默认为竖屏
_lcd_dev lcddev;

struct rt_spi_device *spi_dev_lcd = RT_NULL;     /*总线设备句柄 */

/*挂载 icm20602 到 SPI 总线：*/
int lcd_spi_device_init(void)
{
    struct rt_spi_configuration spi_cfg;

    spi_dev_lcd = (struct rt_spi_device *)rt_device_find(DISPLAY_SPI_DEVICE_NAME);/* 查找 spi2 设备获取设备句柄 */

    spi_cfg.data_width = 8;
    spi_cfg.mode       = RT_SPI_MASTER | RT_SPI_MODE_3 | RT_SPI_MSB;
    spi_cfg.max_hz     = 50*1000*1000;  /*10M*/

    if(spi_dev_lcd)
    rt_spi_configure(spi_dev_lcd,&spi_cfg);

    return RT_EOK;
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(lcd_spi_device_init);

void lcd_wr_reg(rt_uint16_t regval)
{
    struct rt_spi_message msg1;
    rt_uint8_t send_buf[1];
    send_buf[0] = regval&0x00FF;

    LCD_RS_CLR;  //LCD_RS=0  //设置命令状态
    msg1.send_buf   = send_buf;
    msg1.recv_buf   = RT_NULL;
    msg1.length     = 1;
    msg1.cs_take    = 1;
    msg1.cs_release = 1;
    msg1.next       = RT_NULL;

    if(spi_dev_lcd)
    rt_spi_transfer_message(spi_dev_lcd, &msg1);//发送消息
}

void lcd_wr_data8(rt_uint8_t data)
{
    struct rt_spi_message msg1;
    rt_uint8_t send_buf[1];
    send_buf[0] = data;

    LCD_RS_SET;  //LCD_RS=0  //设置命令状态
    msg1.send_buf   = send_buf;
    msg1.recv_buf   = RT_NULL;
    msg1.length     = 1;
    msg1.cs_take    = 1;
    msg1.cs_release = 1;
    msg1.next       = RT_NULL;

    if(spi_dev_lcd)
    rt_spi_transfer_message(spi_dev_lcd, &msg1);//发送消息
}

void lcd_wr_data16(rt_uint16_t data)
{
    struct rt_spi_message msg1;
    rt_uint8_t send_buf[2];
    send_buf[0] = data>>8;
    send_buf[1] = data;
    LCD_RS_SET;                 //LCD_RS=0  //设置命令状态
    msg1.send_buf   = send_buf;
    msg1.recv_buf   = RT_NULL;
    msg1.length     = 2;
    msg1.cs_take    = 1;
    msg1.cs_release = 1;
    msg1.next       = RT_NULL;

    if(spi_dev_lcd)
    rt_spi_transfer_message(spi_dev_lcd, &msg1);//发送消息
}

//*******************************************************************/
//LCD_Reg:寄存器地址
//LCD_RegValue:要写入的数据
//*******************************************************************/
void lcd_write_reg(uint16_t LCD_Reg, uint16_t LCD_RegValue)
{
    lcd_wr_reg(LCD_Reg);           //写入要写的寄存器序号
    lcd_wr_data8(LCD_RegValue);    //写入数据
}

void lcd_soft_rest(void)
{
    lcd_wr_reg(0x01);          //发送软复位命令
    rt_thread_mdelay(50);      // delay 50 ms
}

void lcd_set_window(uint16_t sx,uint16_t sy,uint16_t width,uint16_t height)
{
    width=sx+width-1;
    height=sy+height-1;

    if(LCD_DIR_Mode==0)
        {
            lcd_wr_reg(lcddev.setxcmd);
            lcd_wr_data16(sx);      //设置 X方向起点
            lcd_wr_data16(width);   //设置 X方向终点

            lcd_wr_reg(lcddev.setycmd);
            lcd_wr_data16(sy);      //设置 Y方向起点
            lcd_wr_data16(height);  //设置 Y方向终点
        }
    else if(LCD_DIR_Mode==1)
        {
            lcd_wr_reg(lcddev.setxcmd);
            lcd_wr_data16(sx);      //设置 X方向起点
            lcd_wr_data16(width);   //设置 X方向终点

            lcd_wr_reg(lcddev.setycmd);
            lcd_wr_data16(sy);      //设置 Y方向起点
            lcd_wr_data16(height);  //设置 Y方向终点
        }
        else if(LCD_DIR_Mode==2)
        {
            lcd_wr_reg(lcddev.setxcmd);
            lcd_wr_data16(sx);      //设置 X方向起点
            lcd_wr_data16(width);   //设置 X方向终点

            lcd_wr_reg(lcddev.setycmd);
            lcd_wr_data16(sy+80);      //设置 Y方向起点
            lcd_wr_data16(height+80);  //设置 Y方向终点
        }
     else if(LCD_DIR_Mode==3)
        {
            lcd_wr_reg(lcddev.setxcmd);
            lcd_wr_data16(sx+80);      //设置 X方向起点
            lcd_wr_data16(width+80);   //设置 X方向终点

            lcd_wr_reg(lcddev.setycmd);
            lcd_wr_data16(sy);      //设置 Y方向起点
            lcd_wr_data16(height);  //设置 Y方向终点
        }
}

//*******************************************************************/
//函数功能：设置LCD的自动扫描方向
//默认设置为L2R_U2D,如果设置为其他扫描方式,可能导致显示不正常.
//dir:0~7,代表8个方向
/*******************************************************************/
void lcd_scan_dir(uint8_t dir)
{
    uint8_t regval=0;
    //扫描方向定义--扫描方式有不同规格，可能定义不左右和上下的参照方向不同，总结方式，只有一下八种
    switch(dir)
        {
            case 0:
                regval|=(0<<7)|(0<<6)|(0<<5);
                break;
            case 1:
                regval|=(0<<7)|(1<<6)|(1<<5);
                break;
            case 2:
                regval|=(1<<7)|(1<<6)|(0<<5);
                break;
            case 3:
                regval|=(1<<7)|(0<<6)|(1<<5);
                break;
        }


        lcd_write_reg(0x36,regval);//改变扫描方向命令  ---此处需要查看数据手册，确定RGB颜色交换位的配置
}


/**************************************************************************/
//函数：void LCD_Display_Dir(u8 dir)
//函数功能：设置LCD的显示方向及像素参数
//  dir:   0,竖屏  正
//         1,竖屏  反
//         2,横屏  左
//         3,横屏  右
//*************************************************************************/
void lcd_display_dir(uint8_t dir)
{

    uint8_t SCAN_DIR;

    if(dir==0)               //竖屏  正
    {
        lcddev.dir=0;        //竖屏
        lcddev.width=240;
        lcddev.height=240;

        lcddev.wramcmd=0X2C;
        lcddev.setxcmd=0X2A;
        lcddev.setycmd=0X2B;

        SCAN_DIR=0; //选择扫描方向
    }

   else if (dir==1)             //横屏
    {
        lcddev.dir=0;        //横屏
        lcddev.width=240;
        lcddev.height=240;

        lcddev.wramcmd=0X2C;
        lcddev.setxcmd=0X2A;
        lcddev.setycmd=0X2B;

        SCAN_DIR=1; //选择扫描方向
    }
    else if (dir==2)            //竖屏
    {
        lcddev.dir=1;        //竖屏
        lcddev.width=240;
        lcddev.height=240;

        lcddev.wramcmd=0X2C;
        lcddev.setxcmd=0X2A;
        lcddev.setycmd=0X2B;

        SCAN_DIR=2; //选择扫描方向
    }
 else if (dir==3)                 //横屏
    {
        lcddev.dir=1;           //横屏
        lcddev.width=240;
        lcddev.height=240;

        lcddev.wramcmd=0X2C;
        lcddev.setxcmd=0X2A;
        lcddev.setycmd=0X2B;

         SCAN_DIR=3; //选择扫描方向
    }
 else //设置默认为竖屏--正
 {
        lcddev.dir=0;      //竖屏
        lcddev.width=240;
        lcddev.height=240;

        lcddev.wramcmd=0X2C;
        lcddev.setxcmd=0X2A;
        lcddev.setycmd=0X2B;

        SCAN_DIR=0; //选择扫描方向
 }
 //////以下设置，为窗口参数设置，设置了全屏的显示范围
    lcd_set_window(0,0,lcddev.width,lcddev.height);//设置全屏窗口
 /////设置屏幕显示--扫描方向
    lcd_scan_dir(SCAN_DIR);  //设置屏幕显示--扫描方向

}


//*******************************************************************/
//函数：void LCD_SetCursor(u16 Xpos, u16 Ypos)
//函数功能：设置光标位置
//输入参数：
//Xpos:横坐标
//Ypos:纵坐标

//DevEBox  大越创新
//淘宝店铺：mcudev.taobao.com
//淘宝店铺：shop389957290.taobao.com
//*******************************************************************/
void lcd_set_cursor(uint16_t Xpos, uint16_t Ypos)
{
    if(LCD_DIR_Mode==2)Ypos=Ypos+80;
    if(LCD_DIR_Mode==3)Xpos=Xpos+80;

    lcd_wr_reg(lcddev.setxcmd);
    lcd_wr_data16(Xpos);

    lcd_wr_reg(lcddev.setycmd);
    lcd_wr_data16(Ypos);
}


void lcd_write_ram_prepare(void)
{
    lcd_wr_reg(lcddev.wramcmd);
}

void lcd_clear(uint16_t color)
{
    uint32_t index=0;
    uint32_t totalpoint;

    lcd_set_window(0,0,lcddev.width,lcddev.height);//设置全屏窗口

    totalpoint=lcddev.width * lcddev.height;            //得到总点数

    lcd_set_cursor(0x00,0x00);   //设置光标位置

    lcd_write_ram_prepare();             //开始写入GRAM

    for(index=0;index<totalpoint;index++)
    {
        lcd_wr_data16(color);
    }
}

int lcd_init(void)//icm20602初始化
{
    if(spi_dev_lcd)
    {
        lcd_soft_rest();        //软复位

        rt_thread_mdelay(50);      // delay 50 ms

        //************* Start Initial Sequence **********//

        lcd_wr_reg(0x36);//内存数据访问控制
        lcd_wr_data8(0x00);

        lcd_wr_reg(0x3A);//接口像素格式
        lcd_wr_data8(0x05);

        lcd_wr_reg(0xB2);     //门廊设置
        lcd_wr_data8(0x0C);
        lcd_wr_data8(0x0C);
        lcd_wr_data8(0x00);
        lcd_wr_data8(0x33);
        lcd_wr_data8(0x33);

        lcd_wr_reg(0xB7);    //门控制
        lcd_wr_data8(0x35);

        lcd_wr_reg(0xBB);    //VCOM Setting
        lcd_wr_data8(0x19);

        lcd_wr_reg(0xC0);    //LCM Control
        lcd_wr_data8(0x2C);

        lcd_wr_reg(0xC2);   //VDV and VRH Command Enable
        lcd_wr_data8(0x01);

        lcd_wr_reg(0xC3);  //VRH Set
        lcd_wr_data8(0x12);

        lcd_wr_reg(0xC4);  //VDV Set
        lcd_wr_data8(0x20);

        lcd_wr_reg(0xC6);   //正常模式下的帧率控制
        lcd_wr_data8(0x0F); //帧频率60帧

        lcd_wr_reg(0xD0);   //Power Control
        lcd_wr_data8(0xA4);
        lcd_wr_data8(0xA1);

        lcd_wr_reg(0xE0);    //正电压伽玛控制
        lcd_wr_data8(0xD0);
        lcd_wr_data8(0x04);
        lcd_wr_data8(0x0D);
        lcd_wr_data8(0x11);
        lcd_wr_data8(0x13);
        lcd_wr_data8(0x2B);
        lcd_wr_data8(0x3F);
        lcd_wr_data8(0x54);
        lcd_wr_data8(0x4C);
        lcd_wr_data8(0x18);
        lcd_wr_data8(0x0D);
        lcd_wr_data8(0x0B);
        lcd_wr_data8(0x1F);
        lcd_wr_data8(0x23);

        lcd_wr_reg(0xE1);   //负电压伽玛控制
        lcd_wr_data8(0xD0);
        lcd_wr_data8(0x04);
        lcd_wr_data8(0x0C);
        lcd_wr_data8(0x11);
        lcd_wr_data8(0x13);
        lcd_wr_data8(0x2C);
        lcd_wr_data8(0x3F);
        lcd_wr_data8(0x44);
        lcd_wr_data8(0x51);
        lcd_wr_data8(0x2F);
        lcd_wr_data8(0x1F);
        lcd_wr_data8(0x1F);
        lcd_wr_data8(0x20);
        lcd_wr_data8(0x23);

        lcd_wr_reg(0x21); //显示翻转开

        lcd_wr_reg(0x11); //退出休眠

        rt_thread_mdelay(120);

        lcd_wr_reg(0x29); //开显示

        rt_thread_mdelay(200);

        lcd_display_dir(LCD_DIR_Mode);    //选择--屏幕显示方式

        LCD_BLK_SET;                //点亮背光

        lcd_clear(GRED);
    }
    return RT_EOK;
}

INIT_APP_EXPORT(lcd_init);//自动加入初始化



