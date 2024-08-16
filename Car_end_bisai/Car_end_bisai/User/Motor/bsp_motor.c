#include "bsp_motor.h"
#include "bsp_ili9341_lcd.h"
#include "Systick.h"
#include "../../Core/src/i2c.h"
#include "usart.h"

int8_t stop[4]={0,0,0,0};      							//停止
int8_t forward[4]={-20,20,20,-20};      						//前进
int8_t back_toward[4]={15,-15,-15,15};      						//后退
int8_t right[4]={15,-15,15,-15};      						//后退
int8_t left[4]={-15,15,-15,15};      						//后退
/*  小车轮子转动驱动代码*/

void Forward(void)
{
	/* 直行 */
	#if  mode_car == 0
    HAL_GPIO_WritePin(motorF1_GPIO_Port,motorF1_Pin,GPIO_PIN_SET);//前左轮-IN1  1
    HAL_GPIO_WritePin(motorF2_GPIO_Port,motorF2_Pin,GPIO_PIN_RESET);//前左轮-IN2   0   正转
    
    HAL_GPIO_WritePin(motorF3_GPIO_Port,motorF3_Pin,GPIO_PIN_RESET);//前右轮-IN3   1
    HAL_GPIO_WritePin(motorF4_GPIO_Port,motorF4_Pin,GPIO_PIN_SET);//前右轮-IN4  0   正转
    /*            因为L298N驱动模块是对相安装   */
    HAL_GPIO_WritePin(motorR1_GPIO_Port,motorR1_Pin,GPIO_PIN_RESET);   //后左轮-IN3   0
    HAL_GPIO_WritePin(motorR2_GPIO_Port,motorR2_Pin,GPIO_PIN_SET);    //后左轮-IN4    1    反转
     
    HAL_GPIO_WritePin(motorR3_GPIO_Port,motorR3_Pin,GPIO_PIN_SET); //后右轮-IN1     0
    HAL_GPIO_WritePin(motorR4_GPIO_Port,motorR4_Pin,GPIO_PIN_RESET);  //后右轮-IN2      1    反转
    #endif
	
	#if  mode_car == 1
	//I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,stop,4);
	I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,forward,4);
	#endif 
}
 
void Backward()
{/* 后行 */
	#if  mode_car == 0
    HAL_GPIO_WritePin(motorF1_GPIO_Port,motorF1_Pin,GPIO_PIN_RESET);//前左轮-IN1   0
    HAL_GPIO_WritePin(motorF2_GPIO_Port,motorF2_Pin,GPIO_PIN_SET);//前左轮-IN2     1
    
    HAL_GPIO_WritePin(motorF3_GPIO_Port,motorF3_Pin,GPIO_PIN_SET);//前右轮-IN3   0
    HAL_GPIO_WritePin(motorF4_GPIO_Port,motorF4_Pin,GPIO_PIN_RESET);//前右轮-IN4     1
    /*            因为L298N驱动模块是对相安装   */
    HAL_GPIO_WritePin(motorR1_GPIO_Port,motorR1_Pin,GPIO_PIN_SET);//后左轮-IN3     1
    HAL_GPIO_WritePin(motorR2_GPIO_Port,motorR2_Pin,GPIO_PIN_RESET);//后左轮-IN4   0
     
    HAL_GPIO_WritePin(motorR3_GPIO_Port,motorR3_Pin,GPIO_PIN_RESET);//后右轮-IN1     1
    HAL_GPIO_WritePin(motorR4_GPIO_Port,motorR4_Pin,GPIO_PIN_SET);//后右轮-IN2   0 
	#endif
	
	#if  mode_car == 1
	I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,back_toward,4);
	#endif 
}
void Turn_Left()
{/* 左转 */
	#if  mode_car == 0
    HAL_GPIO_WritePin(motorF1_GPIO_Port,motorF1_Pin,GPIO_PIN_SET);//前左轮-IN1   0
    HAL_GPIO_WritePin(motorF2_GPIO_Port,motorF2_Pin,GPIO_PIN_RESET);//前左轮-IN2     1  反转
    
    HAL_GPIO_WritePin(motorF3_GPIO_Port,motorF3_Pin,GPIO_PIN_SET);//前右轮-IN3     1
    HAL_GPIO_WritePin(motorF4_GPIO_Port,motorF4_Pin,GPIO_PIN_RESET);//前右轮-IN4   0  正转
    /*            因为L298N驱动模块是对相安装   */
    HAL_GPIO_WritePin(motorR1_GPIO_Port,motorR1_Pin,GPIO_PIN_SET);//后左轮-IN3   0
    HAL_GPIO_WritePin(motorR2_GPIO_Port,motorR2_Pin,GPIO_PIN_RESET);//后左轮-IN4     1  反转 
     
    HAL_GPIO_WritePin(motorR3_GPIO_Port,motorR3_Pin,GPIO_PIN_SET);//后右轮-IN1     1
    HAL_GPIO_WritePin(motorR4_GPIO_Port,motorR4_Pin,GPIO_PIN_RESET);//后右轮-IN2   0  正转
    #endif
	
	#if  mode_car == 1
	I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,left,4);
	#endif 	
}
void Turn_Right()
{/* 右转 */
	#if  mode_car == 0
    HAL_GPIO_WritePin(motorF1_GPIO_Port,motorF1_Pin,GPIO_PIN_RESET);//前左轮-IN1 1
    HAL_GPIO_WritePin(motorF2_GPIO_Port,motorF2_Pin,GPIO_PIN_SET);//前左轮-IN2  0 正转
    
    HAL_GPIO_WritePin(motorF3_GPIO_Port,motorF3_Pin,GPIO_PIN_RESET);//前右轮-IN3  0
    HAL_GPIO_WritePin(motorF4_GPIO_Port,motorF4_Pin,GPIO_PIN_SET);//前右轮-IN4   1反转
    /*            因为L298N驱动模块是对相安装   */
    HAL_GPIO_WritePin(motorR1_GPIO_Port,motorR1_Pin,GPIO_PIN_RESET);//后左轮-IN3  1
    HAL_GPIO_WritePin(motorR2_GPIO_Port,motorR2_Pin,GPIO_PIN_SET);//后左轮-IN4   0 正转 
     
    HAL_GPIO_WritePin(motorR3_GPIO_Port,motorR3_Pin,GPIO_PIN_RESET);//后右轮-IN1  0
    HAL_GPIO_WritePin(motorR4_GPIO_Port,motorR4_Pin,GPIO_PIN_SET);//后右轮-IN2   1 反转 
    #endif
	
	#if  mode_car == 1
	I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,right,4);
	#endif 	
}
void Stopward()
{/* 停止 */
	#if  mode_car == 0
    HAL_GPIO_WritePin(motorF1_GPIO_Port,motorF1_Pin,GPIO_PIN_SET);//前左轮-IN1 
    HAL_GPIO_WritePin(motorF2_GPIO_Port,motorF2_Pin,GPIO_PIN_SET);//前左轮-IN2 
    
    HAL_GPIO_WritePin(motorF3_GPIO_Port,motorF3_Pin,GPIO_PIN_SET);//前右轮-IN3   
    HAL_GPIO_WritePin(motorF4_GPIO_Port,motorF4_Pin,GPIO_PIN_SET);//前右轮-IN4 
    /*            因为L298N驱动模块是对相安装   */
    HAL_GPIO_WritePin(motorR1_GPIO_Port,motorR1_Pin,GPIO_PIN_SET);//后左轮-IN3  
    HAL_GPIO_WritePin(motorR2_GPIO_Port,motorR2_Pin,GPIO_PIN_SET);//后左轮-IN4    
     
    HAL_GPIO_WritePin(motorR3_GPIO_Port,motorR3_Pin,GPIO_PIN_SET);//后右轮-IN1  
    HAL_GPIO_WritePin(motorR4_GPIO_Port,motorR4_Pin,GPIO_PIN_SET);//后右轮-IN2   
	#endif
	
	#if  mode_car == 1
	//I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,stop,4);
	#endif 
}


void Car_Forward(uint16_t value,int delay_ms)     //车直行（速度，持续时间）
{
	HAL_UART_Transmit(&huart3, "1b\r\n", sizeof("1b\r\n"),10000);
    #if  mode_car == 0
	Change_speed(value);
	#endif
	Forward();
	#if  mode_car == 0
	Delay1_ms(delay_ms);
	#endif
}

void Car_backward(uint16_t value,int delay_ms)    //车右转
{
    #if  mode_car == 1
	I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,back_toward,4);
	
	#endif
	#if  mode_car == 0
	Delay1_ms(delay_ms);
	#endif
}

void Car_Turn_Right(uint16_t value,int delay_ms)    //车右转
{
	HAL_UART_Transmit(&huart3, "2b\r\n", sizeof("2b\r\n"),10000);
    #if  mode_car == 0
	Change_speed(value);
	#endif
	Turn_Right();
	#if  mode_car == 0
	Delay1_ms(delay_ms);
	#endif
}

void Car_Turn_Left(uint16_t value,int delay_ms)    //车左转
{
	
	HAL_UART_Transmit(&huart3, "3b\r\n", sizeof("3b\r\n"),10000);
    #if  mode_car == 0
	Change_speed(value);
	#endif
	Turn_Left();
    #if  mode_car == 0
	Delay1_ms(delay_ms);
	#endif
}



#if mode_car == 0
void GO_POINT2(void)     //去航点2
{
      Car_Forward(72,8300);
      
	  Car_Turn_Right(68,1250);
      
	  Car_Forward(72,1700);
}

void GO_POINT1(void)     //去航点1
{
      Car_Forward(72,8300); 
      
	  Car_Turn_Left(67,1300);
      
	  Car_Forward(72,1700);
}


void GO_POINT3(void)     //去航点3
{
      Car_Forward(72,1500);
	  
	  Car_Turn_Right(67,1190);
	  
	  Car_Forward(72,1500);
}
void back(int point)     //从某航点返回
{
	switch(point){
		case 1:
	  Car_Turn_Left(67,2380);                 //从航点一返回左转九十度*2
	  Car_Forward(72,1700);                  //直行
	  Car_Turn_Right(67,1190);              //右转九十度
	  Car_Forward(72,8000);                //直行

			break;
		case 2 :
	  Car_Turn_Right(67,2380);            //返回右转九十度*2
	  Car_Forward(72,1700);              //直行
	  Car_Turn_Left(67,1190);           //左转九十度
	  Car_Forward(72,8000);            //直行

			break;
		case 3:
	  Car_Turn_Right(67,2380);      //返回右转九十度*2
	  Car_Forward(72,1300);        //直行
	  Car_Turn_Left(67,1190);     //左转九十度
	  Car_Forward(72,1300);      //直行

			break;
		default:
		ILI9341_DispStringLine_EN(LINE(7),"back_error");
			break;
	}
}
#endif


void navigate_to_ball(int x, int y) {
//ILI9341_DispStringLine_EN(LINE(12),rx_byte);
    if(coordinates_updated != -1)
	{
    // 根据X坐标调整小车方向
		if (coordinates_updated == 1) {
			// 如果小球在图像中心的左侧，向左转
			coordinates_updated=-1;
			ILI9341_DispStringLine_EN(LINE(9),"left");
			Car_Turn_Left(1,500);
		} else if (coordinates_updated == 2) {
			// 如果小球在图像中心的右侧，向右转
			ILI9341_DispStringLine_EN(LINE(9),"righ");
			coordinates_updated=-1;
			Car_Turn_Right(1,500);
		} 
		else if (coordinates_updated == 3) {
			ILI9341_DispStringLine_EN(LINE(9),"fow");
			coordinates_updated=-1;
		    Car_Forward(1,500);
			
		}else if (coordinates_updated == 0) {
			ILI9341_DispStringLine_EN(LINE(9),"stop");
			coordinates_updated=-1;
		    I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,stop,4);
		}

	}
}


void ms_Delay(uint16_t t_ms)    //机械延时
{
	uint32_t t=t_ms*3127;
	while(t--);
}

