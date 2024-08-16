#include "bsp_track.h"
#include "bsp_motor.h"
#include "bsp_Hcsro4.h"
#include "bsp_servo.h"
#include "bsp_User_Debug.h"
#include "bsp_ili9341_lcd.h"
#include "stdio.h"
#include "string.h"


float  distance =0;
float  distance_res[5];              //������Ų����
int    i=1000;

char  dispBuff[100];	 
//float Count =0;

void Change_speed(uint16_t speed)
{/* ���� */
    // ���õ���ٶȷ�Χ��60-100����Խ��Խ��
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, speed);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, speed);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, speed);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, speed);
}

//������+�������
void ultrasonic_waves()
{
                                    //���ָ��90�� 
    SG90_Init();
    Change_speed(90);

    HAL_Delay(700);               //��ʱ,������ʱ�Ļ����ᳬ������Ƶ��
    HCSR_04_Init();               //�������ģ��
    distance_res[0] = GetSR04Distance();
  

//    Count = GetSR04Distance();
                                                       //ʹ��c��׼��ѱ���Countת�����ַ���
    sprintf(dispBuff,"Distance : %.2fcm ",distance_res[0]);   //��Count��ֵ��ʽ��Ϊ�ַ���,���洢��dispBuff�����С�
    LCD_ClearLine(LINE(1));	  
    ILI9341_DispStringLine_EN(LINE(1),dispBuff);   //��ʾdispBuff����




	HAL_Delay(10);
    
    
    if(LR1==0 || LR2==0)
    {
       HAL_Delay(10);
       if(LR1==0)
       {
           Backward();
           HAL_Delay(300);
           Turn_Right();
           HAL_Delay(200);
       }
        if(LR2==0)
       {
           Backward();
           HAL_Delay(300);
           Turn_Left();
           HAL_Delay(200);
       }
    }
    else if(distance_res[0]>30.00)
    {
      Forward();  
    }
//    else if(LR2==1&&LR2==1)
//    {
//        Forward();
//    }
     
	  else if(distance_res[0]<30.00) 
	//else if(LR2==1&&LR2==1)                  //���ǰ������С��30����  ͣ�������Ҿ���
   {
       
       Stopward();
       SG90_TurnL();                           //�����ת���
       HAL_Delay(1200);
       HCSR_04_Init();                        //�������ģ��
       distance_res[1]=GetSR04Distance();
       HAL_Delay(10);
        
       SG90_TurnR();                        //�����ת���
       HAL_Delay(1200);
       HCSR_04_Init();                     //�������ģ��
       distance_res[2]=GetSR04Distance();

     HAL_Delay(10);
     SG90_Init(); 
              
            if(distance_res[1]<30.00&&distance_res[2]<30.00&&distance_res[1]>distance_res[2])
        {
            SG90_Init(); //�������
            HAL_Delay(700);
            HCSR_04_Init();      //�������ģ��
            distance_res[0]=GetSR04Distance();	//�ظ���ǰ���ľ���ͬʱ
            HAL_Delay(10);						
            Backward();
            HAL_Delay(700);
            Turn_Left();
        }
        if(distance_res[1]<35.00&&distance_res[2]<35.00&&distance_res[1]<distance_res[2])
        {
            SG90_Init(); //�������
            HAL_Delay(700);
            HCSR_04_Init();      //�������ģ��
            distance_res[0] =GetSR04Distance();	//�ظ���ǰ���ľ���ͬʱ
            HAL_Delay(100);						
            Backward();
            HAL_Delay(700);
            Turn_Right();
        }

       
		if(distance_res[1]>distance_res[2])              //�����ߵľ�������ұߵľ���
        {
			
			SG90_Init();                               //�������
            HAL_Delay(700);
            HCSR_04_Init();                           //�������ģ��
            distance_res[0] =GetSR04Distance();	     //�ظ���ǰ���ľ���ͬʱ��ת
            HAL_Delay(100);					
			Turn_Left();
			
			
			if(distance_res[0]>35.0000)
            {
                 Forward(); 
            }

        if(distance_res[1]<distance_res[2])       //����ұߵľ��������ߵľ���
        {
			SG90_Init();
            HAL_Delay(700);
            HCSR_04_Init();      //�������ģ��
            distance_res[0] =GetSR04Distance();  //�ظ���ǰ���ľ���ͬʱ��ת
            HAL_Delay(10);		           
			Turn_Right();
			
				if(distance_res[0]>35.0000)
            {
                 Forward(); 
            }
            }
        }
    }
   } 





