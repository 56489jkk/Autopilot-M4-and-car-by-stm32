#ifndef __Track_H_
#define __Track_H_
 
#include "main.h"
#include "tim.h"
//extern void Track(uint16_t value,uint16_t pos);                //Ѱ������
void Change_speed(uint16_t speed);                              //���ٹ���
void ultrasonic_waves(void);                                   //������+�������
#define  L1 HAL_GPIO_ReadPin(sensor4_GPIO_Port,sensor4_Pin)   //�� IN4 B4
#define  L2 HAL_GPIO_ReadPin(sensor3_GPIO_Port,sensor3_Pin)  //�� IN3 B3
#define  R1 HAL_GPIO_ReadPin(sensor2_GPIO_Port,sensor2_Pin) //�� IN2 B2
#define  R2 HAL_GPIO_ReadPin(sensor1_GPIO_Port,sensor1_Pin)//�� IN1 B1

//����Ѱ������ģ��       ǰ���������ڵ������0����Ƭ��   ����������ǿ�����ʱ��Ҳ���0
#define  LR1 HAL_GPIO_ReadPin(infrared_L_GPIO_Port,infrared_L_Pin)     //��  PB12
#define  LR2 HAL_GPIO_ReadPin(infrared_R_GPIO_Port,infrared_R_Pin)    //��  PB13

#endif


