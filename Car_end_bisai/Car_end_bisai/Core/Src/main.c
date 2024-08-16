/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "bsp_Hcsro4.h"
#include "bsp_track.h"
#include "bsp_User_Debug.h"
#include "bsp_servo.h"
#include "bsp_ili9341_lcd.h"
#include "bsp_motor.h"
#include "Bluetooth.h"
#include "../../User/gps/gps.h"
#include "i2c.h"
#include "Systick.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART3_UART_Init();
	MX_USART4_UART_Init();
    MX_TIM4_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
	IIC_init();
   // usart_printf("各部分初始化完成\r\n");
  /* USER CODE BEGIN 2 */

    #if mode_car == 0
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);            //打开TIM1(舵机驱动)的第四输出通道
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);            //打开TIM4(电机驱动)的四个输出通道
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
    #endif
    HAL_TIM_Base_Start(&htim3);                      //开启TIM3(超声波)
    HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);      //开启TIM3的捕获通道2，并开启捕获
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);      //使能更新中断         
  
    //Change_speed(60);//电机初始速度:速度区间0-100;
                              
   #if mode_car == 1
    int8_t MotorEncoderPolarity = 1;			//电机极性控制变量
    int32_t EncodeTotal[4];						//用于暂存电机累积转动量的值，正转递增，反转递减
    int8_t MotorType = MOTOR_TYPE_JGB; 		//设置电机类型
	ms_delay(20);
	I2C_Write_Len(MOTOR_TYPE_ADDR,&MotorType,4);//在电机类型地址中写入电机类型编号
	ms_delay(5);
	I2C_Write_Len(MOTOR_ENCODER_POLARITY_ADDR,&MotorEncoderPolarity,1);	//设置电机极性设置
	ms_delay(5);   
   #endif
   
    ILI9341_Init();                              //LCD屏初始化
    //usart_printf("LCD OK\r\n");
    ILI9341_GramScan(6);    
    
	LCD_SetFont(&Font8x16);                           //设置字体(不变)
	LCD_SetColors(RED,BLUE);                         //设置字体与背景颜色    
    ILI9341_Clear(0,0,LCD_X_LENGTH,LCD_Y_LENGTH);	//清屏显示全黑	

/******************************************************************************************/   
//小车距离、转向测试   
	
	//Car_Turn_Left(67,2380);   //从航点一返回左转九十度*2
    
/******************************************************************************************/   
	Stopward();
	
	//ILI9341_DispStringLine_EN(LINE(0),"Init OK");
    HAL_UART_Receive_IT(&huart3, rx_data, 1);
    //ILI9341_DispStringLine_EN(LINE(5),"Init OK5");
	/******************************************************************************************/   
	 MX_USART1_UART_Init();
	 HAL_UART_Receive_IT(&huart1, rx_data1, 1);//串口1
	//ILI9341_DispStringLine_EN(LINE(8),"Init OK2");
	 HAL_UART_Receive_IT(&huart4, rx_data4, 1);//串口1
	//ILI9341_DispStringLine_EN(LINE(2),"uart4");
  /******************************************************************************************/   
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //Forward();
  while (1)
  {  
	  
	  
	  
	  //蓝牙发送 
	  /*
	  接受情况
	  "1a\r\n"-收到航点一
	  "2a\r\n"-收到航点二
	  "3a\r\n"-收到航点三
	  "0a\r\n"-收到返回指令
	  运动状态
	  "0b\r\n"-停止
	  "1b\r\n"-直行
	  "2b\r\n"-右转
	  "3b\r\n"-左转
	  */

	//  navigate_to_ball(x,y);
//ILI9341_DispStringLine_EN(LINE(0),"Init OK888");
	  if(newcar_point==1||newcar_point==2||newcar_point==3)
	  {
		 if(newcar_point==3&&stage == 0)
		 {
		 int8_t right90[4]={10,-10,10,-10}; 
		 I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,right90,4);
		 HAL_UART_Transmit(&huart3, "2b\r\n", sizeof("2b\r\n"),10000);
		 Delay1_ms(1800);
		 int8_t stop[4]={0,0,0,0};
		 HAL_UART_Transmit(&huart3, "0b\r\n", sizeof("0b\r\n"),10000);
		 I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,stop,4);
		 stage++;
		 }
		  if(newcar_point==1&&stage == 0)
		 {
		 int8_t left30[4]={-10,10,-10,10}; 
		 int8_t forward[4]={-20,20,20,-20}; 
		 I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,forward,4);
		 HAL_UART_Transmit(&huart3, "1b\r\n", sizeof("1b\r\n"),10000);
		 Delay1_ms(2500);
		 int8_t stop[4]={0,0,0,0};
		 I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,stop,4);
		 HAL_UART_Transmit(&huart3, "0b\r\n", sizeof("0b\r\n"),10000);
		 I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,left30,4);
		 HAL_UART_Transmit(&huart3, "3b\r\n", sizeof("3b\r\n"),10000);
		 Delay1_ms(400);
		 I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,stop,4);
		 HAL_UART_Transmit(&huart3, "0b\r\n", sizeof("0b\r\n"),10000);
		 stage++;
		 }
		 if(newcar_point==2&&stage == 0)
		 {
		 int8_t right30[4]={10,-10,10,-10}; 
		 int8_t forward[4]={-20,20,20,-20}; 
		 I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,forward,4);
		 HAL_UART_Transmit(&huart3, "1b\r\n", sizeof("1b\r\n"),10000);
		 Delay1_ms(2500);
		 int8_t stop[4]={0,0,0,0};
		 I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,stop,4);
		 HAL_UART_Transmit(&huart3, "0b\r\n", sizeof("0b\r\n"),10000);
		 I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,right30,4);
		 HAL_UART_Transmit(&huart3, "2b\r\n", sizeof("2b\r\n"),10000);
		 Delay1_ms(400);
		 I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,stop,4);
		 HAL_UART_Transmit(&huart3, "0b\r\n", sizeof("0b\r\n"),10000);
		 stage++;
		 }
		 //if(newcar_point==1&&stage == 1)
		 //{}
         navigate_to_ball(x,y);
	  }
	  else if(newcar_point==0)
	  {
		  
		  if(stage == 0)
		  {
			int8_t left180[4]={-10,10,-10,10}; 
			int8_t forward[4]={-20,20,20,-20}; 
			int8_t stop[4]={0,0,0,0};
			I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,left180,4);
			//HAL_UART_Transmit(&huart3, "3b\r\n", sizeof("3b\r\n"),10000);
			Delay1_ms(3400);
			I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,stop,4);
			
		  	I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,forward,4);
			Delay1_ms(1500);
			
			//HAL_UART_Transmit(&huart3, "0b\r\n", sizeof("0b\r\n"),10000);
			
			stage = 2;
	      }
		  navigate_to_ball(x,y);
	  }
		  
//ILI9341_DispStringLine_EN(LINE(0),"Init OK");
//	  ILI9341_DispStringLine_EN(LINE(0),"Init OK2");
//	  ILI9341_DispStringLine_EN(LINE(0),"Init OK3");
//	  ILI9341_DispStringLine_EN(LINE(0),"Init OK4");
 /* USER CODE END WHILE */	
	  
//	  GO_POINT();       //蓝牙传输去往航点     
     //parseGpsBuffer();
//	if (Save_Data.isGetData)
//	{
//	    ILI9341_DispStringLine_EN(LINE(14),Save_Data.latitude);
//		ILI9341_DispStringLine_EN(LINE(15),Save_Data.N_S);
//		ILI9341_DispStringLine_EN(LINE(16),Save_Data.longitude);
//		ILI9341_DispStringLine_EN(LINE(17),Save_Data.E_W);
//	}
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

