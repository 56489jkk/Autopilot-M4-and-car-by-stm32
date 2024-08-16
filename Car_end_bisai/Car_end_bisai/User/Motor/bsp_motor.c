#include "bsp_motor.h"
#include "bsp_ili9341_lcd.h"
#include "Systick.h"
#include "../../Core/src/i2c.h"
#include "usart.h"

int8_t stop[4]={0,0,0,0};      							//ֹͣ
int8_t forward[4]={-20,20,20,-20};      						//ǰ��
int8_t back_toward[4]={15,-15,-15,15};      						//����
int8_t right[4]={15,-15,15,-15};      						//����
int8_t left[4]={-15,15,-15,15};      						//����
/*  С������ת����������*/

void Forward(void)
{
	/* ֱ�� */
	#if  mode_car == 0
    HAL_GPIO_WritePin(motorF1_GPIO_Port,motorF1_Pin,GPIO_PIN_SET);//ǰ����-IN1  1
    HAL_GPIO_WritePin(motorF2_GPIO_Port,motorF2_Pin,GPIO_PIN_RESET);//ǰ����-IN2   0   ��ת
    
    HAL_GPIO_WritePin(motorF3_GPIO_Port,motorF3_Pin,GPIO_PIN_RESET);//ǰ����-IN3   1
    HAL_GPIO_WritePin(motorF4_GPIO_Port,motorF4_Pin,GPIO_PIN_SET);//ǰ����-IN4  0   ��ת
    /*            ��ΪL298N����ģ���Ƕ��లװ   */
    HAL_GPIO_WritePin(motorR1_GPIO_Port,motorR1_Pin,GPIO_PIN_RESET);   //������-IN3   0
    HAL_GPIO_WritePin(motorR2_GPIO_Port,motorR2_Pin,GPIO_PIN_SET);    //������-IN4    1    ��ת
     
    HAL_GPIO_WritePin(motorR3_GPIO_Port,motorR3_Pin,GPIO_PIN_SET); //������-IN1     0
    HAL_GPIO_WritePin(motorR4_GPIO_Port,motorR4_Pin,GPIO_PIN_RESET);  //������-IN2      1    ��ת
    #endif
	
	#if  mode_car == 1
	//I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,stop,4);
	I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,forward,4);
	#endif 
}
 
void Backward()
{/* ���� */
	#if  mode_car == 0
    HAL_GPIO_WritePin(motorF1_GPIO_Port,motorF1_Pin,GPIO_PIN_RESET);//ǰ����-IN1   0
    HAL_GPIO_WritePin(motorF2_GPIO_Port,motorF2_Pin,GPIO_PIN_SET);//ǰ����-IN2     1
    
    HAL_GPIO_WritePin(motorF3_GPIO_Port,motorF3_Pin,GPIO_PIN_SET);//ǰ����-IN3   0
    HAL_GPIO_WritePin(motorF4_GPIO_Port,motorF4_Pin,GPIO_PIN_RESET);//ǰ����-IN4     1
    /*            ��ΪL298N����ģ���Ƕ��లװ   */
    HAL_GPIO_WritePin(motorR1_GPIO_Port,motorR1_Pin,GPIO_PIN_SET);//������-IN3     1
    HAL_GPIO_WritePin(motorR2_GPIO_Port,motorR2_Pin,GPIO_PIN_RESET);//������-IN4   0
     
    HAL_GPIO_WritePin(motorR3_GPIO_Port,motorR3_Pin,GPIO_PIN_RESET);//������-IN1     1
    HAL_GPIO_WritePin(motorR4_GPIO_Port,motorR4_Pin,GPIO_PIN_SET);//������-IN2   0 
	#endif
	
	#if  mode_car == 1
	I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,back_toward,4);
	#endif 
}
void Turn_Left()
{/* ��ת */
	#if  mode_car == 0
    HAL_GPIO_WritePin(motorF1_GPIO_Port,motorF1_Pin,GPIO_PIN_SET);//ǰ����-IN1   0
    HAL_GPIO_WritePin(motorF2_GPIO_Port,motorF2_Pin,GPIO_PIN_RESET);//ǰ����-IN2     1  ��ת
    
    HAL_GPIO_WritePin(motorF3_GPIO_Port,motorF3_Pin,GPIO_PIN_SET);//ǰ����-IN3     1
    HAL_GPIO_WritePin(motorF4_GPIO_Port,motorF4_Pin,GPIO_PIN_RESET);//ǰ����-IN4   0  ��ת
    /*            ��ΪL298N����ģ���Ƕ��లװ   */
    HAL_GPIO_WritePin(motorR1_GPIO_Port,motorR1_Pin,GPIO_PIN_SET);//������-IN3   0
    HAL_GPIO_WritePin(motorR2_GPIO_Port,motorR2_Pin,GPIO_PIN_RESET);//������-IN4     1  ��ת 
     
    HAL_GPIO_WritePin(motorR3_GPIO_Port,motorR3_Pin,GPIO_PIN_SET);//������-IN1     1
    HAL_GPIO_WritePin(motorR4_GPIO_Port,motorR4_Pin,GPIO_PIN_RESET);//������-IN2   0  ��ת
    #endif
	
	#if  mode_car == 1
	I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,left,4);
	#endif 	
}
void Turn_Right()
{/* ��ת */
	#if  mode_car == 0
    HAL_GPIO_WritePin(motorF1_GPIO_Port,motorF1_Pin,GPIO_PIN_RESET);//ǰ����-IN1 1
    HAL_GPIO_WritePin(motorF2_GPIO_Port,motorF2_Pin,GPIO_PIN_SET);//ǰ����-IN2  0 ��ת
    
    HAL_GPIO_WritePin(motorF3_GPIO_Port,motorF3_Pin,GPIO_PIN_RESET);//ǰ����-IN3  0
    HAL_GPIO_WritePin(motorF4_GPIO_Port,motorF4_Pin,GPIO_PIN_SET);//ǰ����-IN4   1��ת
    /*            ��ΪL298N����ģ���Ƕ��లװ   */
    HAL_GPIO_WritePin(motorR1_GPIO_Port,motorR1_Pin,GPIO_PIN_RESET);//������-IN3  1
    HAL_GPIO_WritePin(motorR2_GPIO_Port,motorR2_Pin,GPIO_PIN_SET);//������-IN4   0 ��ת 
     
    HAL_GPIO_WritePin(motorR3_GPIO_Port,motorR3_Pin,GPIO_PIN_RESET);//������-IN1  0
    HAL_GPIO_WritePin(motorR4_GPIO_Port,motorR4_Pin,GPIO_PIN_SET);//������-IN2   1 ��ת 
    #endif
	
	#if  mode_car == 1
	I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,right,4);
	#endif 	
}
void Stopward()
{/* ֹͣ */
	#if  mode_car == 0
    HAL_GPIO_WritePin(motorF1_GPIO_Port,motorF1_Pin,GPIO_PIN_SET);//ǰ����-IN1 
    HAL_GPIO_WritePin(motorF2_GPIO_Port,motorF2_Pin,GPIO_PIN_SET);//ǰ����-IN2 
    
    HAL_GPIO_WritePin(motorF3_GPIO_Port,motorF3_Pin,GPIO_PIN_SET);//ǰ����-IN3   
    HAL_GPIO_WritePin(motorF4_GPIO_Port,motorF4_Pin,GPIO_PIN_SET);//ǰ����-IN4 
    /*            ��ΪL298N����ģ���Ƕ��లװ   */
    HAL_GPIO_WritePin(motorR1_GPIO_Port,motorR1_Pin,GPIO_PIN_SET);//������-IN3  
    HAL_GPIO_WritePin(motorR2_GPIO_Port,motorR2_Pin,GPIO_PIN_SET);//������-IN4    
     
    HAL_GPIO_WritePin(motorR3_GPIO_Port,motorR3_Pin,GPIO_PIN_SET);//������-IN1  
    HAL_GPIO_WritePin(motorR4_GPIO_Port,motorR4_Pin,GPIO_PIN_SET);//������-IN2   
	#endif
	
	#if  mode_car == 1
	//I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,stop,4);
	#endif 
}


void Car_Forward(uint16_t value,int delay_ms)     //��ֱ�У��ٶȣ�����ʱ�䣩
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

void Car_backward(uint16_t value,int delay_ms)    //����ת
{
    #if  mode_car == 1
	I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,back_toward,4);
	
	#endif
	#if  mode_car == 0
	Delay1_ms(delay_ms);
	#endif
}

void Car_Turn_Right(uint16_t value,int delay_ms)    //����ת
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

void Car_Turn_Left(uint16_t value,int delay_ms)    //����ת
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
void GO_POINT2(void)     //ȥ����2
{
      Car_Forward(72,8300);
      
	  Car_Turn_Right(68,1250);
      
	  Car_Forward(72,1700);
}

void GO_POINT1(void)     //ȥ����1
{
      Car_Forward(72,8300); 
      
	  Car_Turn_Left(67,1300);
      
	  Car_Forward(72,1700);
}


void GO_POINT3(void)     //ȥ����3
{
      Car_Forward(72,1500);
	  
	  Car_Turn_Right(67,1190);
	  
	  Car_Forward(72,1500);
}
void back(int point)     //��ĳ���㷵��
{
	switch(point){
		case 1:
	  Car_Turn_Left(67,2380);                 //�Ӻ���һ������ת��ʮ��*2
	  Car_Forward(72,1700);                  //ֱ��
	  Car_Turn_Right(67,1190);              //��ת��ʮ��
	  Car_Forward(72,8000);                //ֱ��

			break;
		case 2 :
	  Car_Turn_Right(67,2380);            //������ת��ʮ��*2
	  Car_Forward(72,1700);              //ֱ��
	  Car_Turn_Left(67,1190);           //��ת��ʮ��
	  Car_Forward(72,8000);            //ֱ��

			break;
		case 3:
	  Car_Turn_Right(67,2380);      //������ת��ʮ��*2
	  Car_Forward(72,1300);        //ֱ��
	  Car_Turn_Left(67,1190);     //��ת��ʮ��
	  Car_Forward(72,1300);      //ֱ��

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
    // ����X�������С������
		if (coordinates_updated == 1) {
			// ���С����ͼ�����ĵ���࣬����ת
			coordinates_updated=-1;
			ILI9341_DispStringLine_EN(LINE(9),"left");
			Car_Turn_Left(1,500);
		} else if (coordinates_updated == 2) {
			// ���С����ͼ�����ĵ��Ҳ࣬����ת
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


void ms_Delay(uint16_t t_ms)    //��е��ʱ
{
	uint32_t t=t_ms*3127;
	while(t--);
}

