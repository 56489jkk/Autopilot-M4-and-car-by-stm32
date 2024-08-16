#include "bsp_servo.h"

/* 舵机转向函数*/
    
/*  PSC:20000-1     ARR:72-1
    角度值(value)    CCR的值
		0            500
       180           2500     
    */

void SG90_GetAngle(float value)
{
   value=value/180*2000+500;
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,value);
    
}

void SG90_Init()            //舵机初始化（摆正）
{
   SG90_GetAngle(90); 
}
void SG90_TurnL()         //舵机左转
{
   SG90_GetAngle(165); 
}
void SG90_TurnR()       //舵机右转
{
   SG90_GetAngle(15); 
}

