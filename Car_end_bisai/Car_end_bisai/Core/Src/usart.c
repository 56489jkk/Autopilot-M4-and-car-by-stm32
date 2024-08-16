/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"
#include "../../User/gps/gps.h"
#include "bsp_ili9341_lcd.h"
#include "string.h"
#include "bsp_motor.h"
#include <stdio.h>
#include <string.h>
/* USER CODE BEGIN 0 */
char tx_data[] = "Hello World!\r\n";              //���Ͳ�������
char tx_data2[] = "receive 0 LED OFF!\r\n"; 
char tx_data3[] = "receive 1 LED ON!\r\n"; 
uint8_t rx_data[2]; 
uint8_t rx_data4[2]; //���ղ�������
uint8_t rx_data1[2]; 
uint8_t rx_byte[32];
#define USART_REC_LEN  			200  	//�����������ֽ��� 200
char USART_RX_BUF[USART_REC_LEN]; 
unsigned int point1 = 0;
             // ����1�����ֽ�

//static uint8_t rx_byte;               // ����1�����ֽ�
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;

int point = 4;//Ĭ��4��
int point_back = 4;//Ĭ��4��
int newcar_point = 4;//��С����
int stage = 0;//�׶�
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitTypeDef GPIO_InitS = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */
	
  /* USER CODE END USART1_MspInit 1 */
  }
  if(uartHandle->Instance==USART3)
  {
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitS.Pin = GPIO_PIN_10;
    GPIO_InitS.Mode = GPIO_MODE_AF_PP;
    GPIO_InitS.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitS);

    GPIO_InitS.Pin = GPIO_PIN_11;
    GPIO_InitS.Mode = GPIO_MODE_INPUT;
    GPIO_InitS.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitS);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn); 
  }
  
  if(uartHandle->Instance==UART4)
  {
    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitS.Pin = GPIO_PIN_10;
    GPIO_InitS.Mode = GPIO_MODE_AF_PP;
    GPIO_InitS.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitS);

    GPIO_InitS.Pin = GPIO_PIN_11;
    GPIO_InitS.Mode = GPIO_MODE_INPUT;
    GPIO_InitS.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitS);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn); 
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);
	HAL_NVIC_DisableIRQ(USART1_IRQn);
  }
  if(uartHandle->Instance==USART3)
  {
    __HAL_RCC_USART3_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  }
  if(uartHandle->Instance==UART4)
  {
    __HAL_RCC_UART4_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11);
    HAL_NVIC_DisableIRQ(UART4_IRQn);
  }
}

void MX_USART4_UART_Init(void)
{
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USART1 init function */

void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}


//void back_point(void)            //�жϴ��ĸ��㷵�����
//{
//   switch(point_back)
//   {
//	   case 1:
//		   back(1);
//		   break;
//	   case 2:
//		    back(2);
//		   break;
//	   case 3:
//		    back(3);
//		   break;
//	   case 4:
//		   
//		break;
//	   default:
//		ILI9341_DispStringLine_EN(LINE(6),"Back_Error");  
//		   break;
//   }
//}



//void GO_POINT(void)                 //�жϺ�������
//{
//   switch(point)
//   {
//		   
//	case 1:
//      ILI9341_Clear(10, 100, 50, 30 );
//      ILI9341_DispString_EN(6, 80, "POINT: 1");
//	                                                          //�յ�ָ���ʼִ������
//	  GO_POINT1();                                           //ֱ�м���ת
//      ILI9341_DispStringLine_EN(LINE(2),"GO_POINT1:"); 
//	  point = 4;
//	  point_back = 1;
//    break;
//	
//	case 2:
//      ILI9341_Clear(10, 100, 50, 30 );
//      ILI9341_DispString_EN(3, 30, "POINT: 2"); 
//                                                  //�յ�ָ���ʼִ������
//	  GO_POINT2();                               //ֱ�м���ת
//      ILI9341_DispStringLine_EN(LINE(3),"GO_POINT2:"); 
//      point = 4;
//	  point_back = 2;
//    break;
//	
//	case 3:
//      ILI9341_Clear(10, 100, 50, 30 );
//      ILI9341_DispString_EN(6, 60, "POINT: 3"); 
//    
//	                                               //�յ�ָ���ʼִ������
//	  GO_POINT3();                                //ֱ�м���ת
//	  ILI9341_DispStringLine_EN(LINE(6),"GO_POINT3:"); 
//	  point = 4;
//	  point_back = 3;
//    break;
//	
//	case 0://����
//	ILI9341_DispString_EN(10, 100, "POINT: 0_back");
//		back_point();
//		point = 4;
//	    point_back = 4;
//		break;
//	
//	case 4:
//		break;
//	default:
//	break;
//   }
//}

/*
void HAL_UART3_RxCpltCallback(UART_HandleTypeDef *huart)       //�жϴ�����
{

  if (huart->Instance == USART3) 
  {

	  if (rx_data[0] == '1')      //1����
	  {
		  point=1;
	  }
	  if (rx_data[0] == '2')    //2����
	  {
		  point=2;
	  }
	  if (rx_data[0] == '3')   //3����
	  {
		  point=3;
	  }
	  if (rx_data[0] == '0')   //����Զ��
	  {
		  point=0;
	  }
      HAL_UART_Receive_IT(&huart3, rx_data, 1);
  }
}
*/



/*********************************************************************************************************/



int fputc(int ch, FILE *f)
{
	/* ����һ���ֽ����ݵ�����DEBUG_USART */
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 1000);	
	
	return (ch);
}//�ض���printf
int x, y;                            // ���ڴ洢�����������
int coordinates_updated = -1;
int uart_data_length = 0;//uart1�������ݳ���
// UART������ɻص�����
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // �����USART1���ж�
	if (huart->Instance == USART1)
    {
		coordinates_updated = rx_data1[0]-48;
	    HAL_UART_Receive_IT(&huart1, rx_data1, 1);//�ڴ������ٽ���
	}
	
	if (huart->Instance == USART3) //uart3
    {

	  if (rx_data[0] == '1') //1����
	  {
		  point=1;
		  newcar_point = 1;
		  stage = 0;
		 HAL_UART_Transmit(&huart3, "1a\r\n", sizeof("1a\r\n"),10000);
	  }
	  if (rx_data[0] == '2')    //2����
	  {
		  point=2;
		  newcar_point = 2;
		  stage = 0;
		  HAL_UART_Transmit(&huart3, "2a\r\n", sizeof("2a\r\n"),10000);
	  }
	  if (rx_data[0] == '3')   //3����
	  {
		  point=3;
		  newcar_point = 3;
		  stage = 0;
		  HAL_UART_Transmit(&huart3, "3a\r\n", sizeof("3a\r\n"),10000);
	  }
	  if (rx_data[0] == '0')   //����Զ��
	  {
		  point=0;
		  newcar_point= 0;
		  stage = 0;
		  HAL_UART_Transmit(&huart3, "0a\r\n", sizeof("0a\r\n"),10000);
	  }
      HAL_UART_Receive_IT(&huart3, rx_data, 1);
     }

	 if(huart->Instance == UART4)
	 {
		if(rx_data4[0] == '$')
		{
			point1 = 0;	
		}
		
		USART_RX_BUF[point1++] = rx_data4[0];

		if(USART_RX_BUF[0] == '$' && USART_RX_BUF[4] == 'M' && USART_RX_BUF[5] == 'C')			//ȷ���Ƿ��յ�"GPRMC/GNRMC"��һ֡����
		{
			if(rx_data4[0] == '\n')									   
			{
				memset(Save_Data.GPS_Buffer, 0, GPS_Buffer_Length);      //���
				memcpy(Save_Data.GPS_Buffer, USART_RX_BUF, point1); 	//��������
				Save_Data.isGetData = true;
				point1 = 0;
				memset(USART_RX_BUF, 0, USART_REC_LEN);      //���				
			}	
				
		}
	
		if(point1 >= USART_REC_LEN)
		{
			point1 = USART_REC_LEN;
		}
		HAL_UART_Receive_IT(&huart4, rx_data4, 1);
	 }
}



// USART1���жϴ������
//void USART1_IRQHandler(void)
//{
    // ����HAL����жϴ�����
 //   HAL_UART_IRQHandler(&huart1);
//}

/*********************************************************************************************************/



