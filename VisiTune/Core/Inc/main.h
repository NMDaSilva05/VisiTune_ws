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
#include "stm32l4xx_hal.h"

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
#define Col_4_Pin GPIO_PIN_0
#define Col_4_GPIO_Port GPIOF
#define Col_3_Pin GPIO_PIN_1
#define Col_3_GPIO_Port GPIOF
#define Col_2_Pin GPIO_PIN_2
#define Col_2_GPIO_Port GPIOF
#define Row_1_Pin GPIO_PIN_1
#define Row_1_GPIO_Port GPIOC
#define LOW_FILTER_Pin GPIO_PIN_0
#define LOW_FILTER_GPIO_Port GPIOA
#define MID_FILTER_Pin GPIO_PIN_1
#define MID_FILTER_GPIO_Port GPIOA
#define HIGH_FILTER_Pin GPIO_PIN_2
#define HIGH_FILTER_GPIO_Port GPIOA
#define DYNAMIC_FILTER_Pin GPIO_PIN_3
#define DYNAMIC_FILTER_GPIO_Port GPIOA
#define Row_2_Pin GPIO_PIN_4
#define Row_2_GPIO_Port GPIOC
#define Row_3_Pin GPIO_PIN_5
#define Row_3_GPIO_Port GPIOC
#define Row_4_Pin GPIO_PIN_2
#define Row_4_GPIO_Port GPIOB
#define PAUSE_Pin GPIO_PIN_0
#define PAUSE_GPIO_Port GPIOG
#define PAUSE_EXTI_IRQn EXTI0_IRQn
#define FORWARD_Pin GPIO_PIN_1
#define FORWARD_GPIO_Port GPIOG
#define FORWARD_EXTI_IRQn EXTI1_IRQn
#define REWIND_Pin GPIO_PIN_10
#define REWIND_GPIO_Port GPIOB
#define REWIND_EXTI_IRQn EXTI15_10_IRQn
#define SD_CS_Pin GPIO_PIN_12
#define SD_CS_GPIO_Port GPIOB
#define Col_1_Pin GPIO_PIN_6
#define Col_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define SD_SPI_HANDLE hspi2
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
