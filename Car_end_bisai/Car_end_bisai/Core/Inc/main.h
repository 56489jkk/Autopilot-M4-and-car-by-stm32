/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_track.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define motorF3_Pin GPIO_PIN_2
#define motorF3_GPIO_Port GPIOA
#define motorF4_Pin GPIO_PIN_3
#define motorF4_GPIO_Port GPIOA
#define motorR1_Pin GPIO_PIN_4
#define motorR1_GPIO_Port GPIOA
#define motorR2_Pin GPIO_PIN_5
#define motorR2_GPIO_Port GPIOA
#define motorR3_Pin GPIO_PIN_6
#define motorR3_GPIO_Port GPIOA
#define motorR4_Pin GPIO_PIN_7
#define motorR4_GPIO_Port GPIOA
#define infrared_L_Pin GPIO_PIN_12
#define infrared_L_GPIO_Port GPIOB
#define infrared_R_Pin GPIO_PIN_13
#define infrared_R_GPIO_Port GPIOB
#define motorF1_Pin GPIO_PIN_8
#define motorF1_GPIO_Port GPIOC
#define motorF2_Pin GPIO_PIN_9
#define motorF2_GPIO_Port GPIOC
#define TRIG_Pin GPIO_PIN_12
#define TRIG_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
extern uint8_t  TIM3CH2_CAPTURE_STA;           // 输入捕获状态
extern uint16_t TIM3CH2_CAPTURE_VAL;         //输入捕获值
extern uint8_t unit_change;                 //单位变换
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
