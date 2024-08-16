#include "bsp_Hcsro4.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "bsp_User_Debug.h"

//激活超声波定时器
void HCSR_04_Init()
{
    uint32_t i;
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
    for (i = 0; i < 72 * 40; i++)
        __NOP();
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
}
//关闭超声波定时器
void Stop_HCSR_04()
{
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
}

//计算超声波检测的距离,返回float值；

//uint8_t TIM3CH2_CAPTURE_STA;  // 输入捕获状态
//uint16_t TIM3CH2_CAPTURE_VAL; //输入捕获值
uint8_t unit_change;
float GetSR04Distance()
{
    float len = 0;
    uint32_t time = 0;
    if (TIM3CH2_CAPTURE_STA & 0X80) // 输入捕获 触发
    {
        time = TIM3CH2_CAPTURE_STA & 0X3f; // 获得溢出次数
        time *= 65535;                     // 一次溢出为65536 得到溢出的时间
        time += TIM3CH2_CAPTURE_VAL;       // 溢出的时间+现在定时器的值 得到总的时间
        if (unit_change == 0)
        {
            len = (time * 342.62 * 100 / 2000000); // 计算得到距离 cm
        }
//        else if (unit_change == 1)
//        {
//            len = time * 342.62 * 100 / 200000000; // 计算得到距离 m
//            usart_printf("m\r\n");
//        }
 
        TIM3CH2_CAPTURE_STA = 0; // 清除溢出
    }
    return len;
}

//    超声波模块主代码
//    char  dispBuff[100];
//	  float Count =0;
  
//    HAL_Delay(100);                     //延时,不做延时的话,会超过采样频率
//    HCSR_04_Init();                     //激活超声波模块
//    Count = GetSR04Distance();
                                                       //使用c标准库把变量Count转化成字符串
//    sprintf(dispBuff,"Distance : %.2fcm ",Count);   //将Count的值格式化为字符串,并存储在dispBuff数组中。
//    LCD_ClearLine(LINE(1));	  
//    ILI9341_DispStringLine_EN(LINE(1),dispBuff);   //显示dispBuff数组

