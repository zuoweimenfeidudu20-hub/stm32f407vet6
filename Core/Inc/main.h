/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define KEY3_Pin GPIO_PIN_2
#define KEY3_GPIO_Port GPIOE
#define KEY2_Pin GPIO_PIN_3
#define KEY2_GPIO_Port GPIOE
#define KEY1_Pin GPIO_PIN_4
#define KEY1_GPIO_Port GPIOE
#define US_TRIG_Pin GPIO_PIN_3
#define US_TRIG_GPIO_Port GPIOC
#define TIM9_US_ECHO_Pin GPIO_PIN_2
#define TIM9_US_ECHO_GPIO_Port GPIOA
#define LCD_NSS_Pin GPIO_PIN_4
#define LCD_NSS_GPIO_Port GPIOA
#define LCD_SCL_Pin GPIO_PIN_5
#define LCD_SCL_GPIO_Port GPIOA
#define LCD_SDA_Pin GPIO_PIN_7
#define LCD_SDA_GPIO_Port GPIOA
#define LCD_NDC_Pin GPIO_PIN_4
#define LCD_NDC_GPIO_Port GPIOC
#define LCD_RES_Pin GPIO_PIN_5
#define LCD_RES_GPIO_Port GPIOC
#define LED_1_Pin GPIO_PIN_2
#define LED_1_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_8
#define LED_2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
