#ifndef __Servo_H_
#define __Servo_H_
 
#include "main.h"
#include "tim.h"

void SG90_GetAngle(float value);
void SG90_Init(void);            //舵机头初始化，摆正
void SG90_TurnL(void);          //舵机头左转
void SG90_TurnR(void);         //舵机头右转

#endif

