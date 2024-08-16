#ifndef __Track_H_
#define __Track_H_
 
#include "main.h"
#include "tim.h"
//extern void Track(uint16_t value,uint16_t pos);                //寻迹功能
void Change_speed(uint16_t speed);                              //变速功能
void ultrasonic_waves(void);                                   //超声波+红外避障
#define  L1 HAL_GPIO_ReadPin(sensor4_GPIO_Port,sensor4_Pin)   //左 IN4 B4
#define  L2 HAL_GPIO_ReadPin(sensor3_GPIO_Port,sensor3_Pin)  //左 IN3 B3
#define  R1 HAL_GPIO_ReadPin(sensor2_GPIO_Port,sensor2_Pin) //右 IN2 B2
#define  R2 HAL_GPIO_ReadPin(sensor1_GPIO_Port,sensor1_Pin)//右 IN1 B1

//红外寻迹避障模块       前方有物体遮挡则输出0到单片机   （光敏）被强光光照时候也输出0
#define  LR1 HAL_GPIO_ReadPin(infrared_L_GPIO_Port,infrared_L_Pin)     //左  PB12
#define  LR2 HAL_GPIO_ReadPin(infrared_R_GPIO_Port,infrared_R_Pin)    //右  PB13

#endif


