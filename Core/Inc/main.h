/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define LORA_RST_Pin GPIO_PIN_14
#define LORA_RST_GPIO_Port GPIOC
#define GPS_RST_Pin GPIO_PIN_15
#define GPS_RST_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_4
#define BUZZER_GPIO_Port GPIOA
#define SERVO_Pin GPIO_PIN_2
#define SERVO_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_13
#define LED_G_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_14
#define LED_B_GPIO_Port GPIOB
#define LED_Y_Pin GPIO_PIN_15
#define LED_Y_GPIO_Port GPIOB
#define CS_LORA_Pin GPIO_PIN_8
#define CS_LORA_GPIO_Port GPIOA
#define PYRO_2_Pin GPIO_PIN_11
#define PYRO_2_GPIO_Port GPIOA
#define PYRO_1_Pin GPIO_PIN_12
#define PYRO_1_GPIO_Port GPIOA
#define INT_M_Pin GPIO_PIN_5
#define INT_M_GPIO_Port GPIOB
#define INT_G_Pin GPIO_PIN_8
#define INT_G_GPIO_Port GPIOB
#define INT_A_Pin GPIO_PIN_9
#define INT_A_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
