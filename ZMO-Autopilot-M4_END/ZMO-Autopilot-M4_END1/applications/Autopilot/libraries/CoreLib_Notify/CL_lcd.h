/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-25     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_NOTIFY_CL_LCD_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_NOTIFY_CL_LCD_H_


#include "board.h"

///液晶控制口置1操作语句宏定义
#define LCD_RS_SET      HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET)  //PB1置1     LCD_RS： PB1   //命令/数据切换

#define LCD_BLK_SET     HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET)  //PB0置1     LCD_BLK ：PB0   //背光控制


//液晶控制口置0操作语句宏定义

#define LCD_RS_CLR      HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET)          //PB1置0 //DC    LCD_RS： PB1   //命令/数据切换

#define LCD_BLK_CLR     HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET)        //PB0置0 //DIN   LCD_BLK ：PB0   //背光控制



//LCD重要参数集
typedef struct
{
    uint16_t width;          //LCD 宽度
    uint16_t height;         //LCD 高度
    uint16_t id;               //LCD ID
    uint8_t  dir;              //横屏还是竖屏控制：竖屏和横屏。
    uint16_t wramcmd;        //开始写gram指令
    uint16_t  setxcmd;       //设置x坐标指令
    uint16_t  setycmd;       //设置y坐标指令
}_lcd_dev;



/////////////////////////////////////用户配置区///////////////////////////////////

//支持横竖屏快速定义切换


#define LCD_DIR_Mode      0     //4种工作模式，0和2是竖屏模式，1和3是横屏模式

#define USE_HORIZONTAL      0     //方向设置：       0,竖屏模式   1,横屏模式.



//////////////////////////////////////////////////////////////////////////////////

//LCD参数
extern _lcd_dev lcddev; //管理LCD重要参数

//LCD的画笔颜色和背景色

extern uint16_t  POINT_COLOR;//默认红色
extern uint16_t  BACK_COLOR; //背景颜色.默认为白色



///////////////////////////  颜色值  ///////////////////////////////////////////////////////

//画笔颜色
#define WHITE            0xFFFF
#define BLACK            0x0000
#define BLUE               0x001F
#define BRED             0xF81F
#define GRED                   0xFFE0
#define GBLUE                  0x07FF
#define RED              0xF800
#define MAGENTA          0xF81F
#define GREEN            0x07E0
#define CYAN             0x7FFF
#define YELLOW           0xFFE0
#define BROWN                0xBC40 //棕色
#define BRRED                0xFC07 //棕红色
#define GRAY                 0x8430 //灰色


//GUI颜色

#define DARKBLUE         0x01CF //深蓝色
#define LIGHTBLUE        0x7D7C //浅蓝色
#define GRAYBLUE         0x5458 //灰蓝色
//以上三色为PANEL的颜色


#define LIGHTGREEN           0x841F //浅绿色
#define LGRAY                0xC618 //浅灰色(PANNEL),窗体背景色

#define GRAY0   0xEF7D          //灰色0
#define GRAY1   0x8410          //灰色1
#define GRAY2   0x4208          //灰色2

#define LGRAYBLUE        0xA651 //浅灰蓝色(中间层颜色)
#define LBBLUE           0x2B12 //浅棕蓝色(选择条目的反色)







#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_NOTIFY_CL_LCD_H_ */
