#include "bsp_servo.h"

/* ���ת����*/
    
/*  PSC:20000-1     ARR:72-1
    �Ƕ�ֵ(value)    CCR��ֵ
		0            500
       180           2500     
    */

void SG90_GetAngle(float value)
{
   value=value/180*2000+500;
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,value);
    
}

void SG90_Init()            //�����ʼ����������
{
   SG90_GetAngle(90); 
}
void SG90_TurnL()         //�����ת
{
   SG90_GetAngle(165); 
}
void SG90_TurnR()       //�����ת
{
   SG90_GetAngle(15); 
}

