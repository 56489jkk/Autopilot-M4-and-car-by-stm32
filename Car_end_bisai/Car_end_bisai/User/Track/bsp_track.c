#include "bsp_track.h"
#include "bsp_motor.h"
#include "bsp_Hcsro4.h"
#include "bsp_servo.h"
#include "bsp_User_Debug.h"
#include "bsp_ili9341_lcd.h"
#include "stdio.h"
#include "string.h"


float  distance =0;
float  distance_res[5];              //用来存放测距结果
int    i=1000;

char  dispBuff[100];	 
//float Count =0;

void Change_speed(uint16_t speed)
{/* 变速 */
    // 设置电机速度范围（60-100），越大越快
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, speed);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, speed);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, speed);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, speed);
}

//超声波+红外避障
void ultrasonic_waves()
{
                                    //舵机指向90° 
    SG90_Init();
    Change_speed(90);

    HAL_Delay(700);               //延时,不做延时的话，会超过采样频率
    HCSR_04_Init();               //激活超声波模块
    distance_res[0] = GetSR04Distance();
  

//    Count = GetSR04Distance();
                                                       //使用c标准库把变量Count转化成字符串
    sprintf(dispBuff,"Distance : %.2fcm ",distance_res[0]);   //将Count的值格式化为字符串,并存储在dispBuff数组中。
    LCD_ClearLine(LINE(1));	  
    ILI9341_DispStringLine_EN(LINE(1),dispBuff);   //显示dispBuff数组




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
	//else if(LR2==1&&LR2==1)                  //如果前方距离小于30厘米  停车测左右距离
   {
       
       Stopward();
       SG90_TurnL();                           //舵机左转测距
       HAL_Delay(1200);
       HCSR_04_Init();                        //激活超声波模块
       distance_res[1]=GetSR04Distance();
       HAL_Delay(10);
        
       SG90_TurnR();                        //舵机右转测距
       HAL_Delay(1200);
       HCSR_04_Init();                     //激活超声波模块
       distance_res[2]=GetSR04Distance();

     HAL_Delay(10);
     SG90_Init(); 
              
            if(distance_res[1]<30.00&&distance_res[2]<30.00&&distance_res[1]>distance_res[2])
        {
            SG90_Init(); //舵机摆正
            HAL_Delay(700);
            HCSR_04_Init();      //激活超声波模块
            distance_res[0]=GetSR04Distance();	//重复测前方的距离同时
            HAL_Delay(10);						
            Backward();
            HAL_Delay(700);
            Turn_Left();
        }
        if(distance_res[1]<35.00&&distance_res[2]<35.00&&distance_res[1]<distance_res[2])
        {
            SG90_Init(); //舵机摆正
            HAL_Delay(700);
            HCSR_04_Init();      //激活超声波模块
            distance_res[0] =GetSR04Distance();	//重复测前方的距离同时
            HAL_Delay(100);						
            Backward();
            HAL_Delay(700);
            Turn_Right();
        }

       
		if(distance_res[1]>distance_res[2])              //如果左边的距离大于右边的距离
        {
			
			SG90_Init();                               //舵机摆正
            HAL_Delay(700);
            HCSR_04_Init();                           //激活超声波模块
            distance_res[0] =GetSR04Distance();	     //重复测前方的距离同时左转
            HAL_Delay(100);					
			Turn_Left();
			
			
			if(distance_res[0]>35.0000)
            {
                 Forward(); 
            }

        if(distance_res[1]<distance_res[2])       //如果右边的距离大于左边的距离
        {
			SG90_Init();
            HAL_Delay(700);
            HCSR_04_Init();      //激活超声波模块
            distance_res[0] =GetSR04Distance();  //重复测前方的距离同时右转
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





