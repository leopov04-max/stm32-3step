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
#define LED_on_board_Pin GPIO_PIN_13
#define LED_on_board_GPIO_Port GPIOC
#define CS_X_Pin GPIO_PIN_4
#define CS_X_GPIO_Port GPIOA
#define CS_Y_Pin GPIO_PIN_0
#define CS_Y_GPIO_Port GPIOB
#define CS_Z_Pin GPIO_PIN_1
#define CS_Z_GPIO_Port GPIOB
#define Y_EN_Pin GPIO_PIN_10
#define Y_EN_GPIO_Port GPIOB
#define X_LIMIT_Pin GPIO_PIN_12
#define X_LIMIT_GPIO_Port GPIOB
#define X_LIMIT_EXTI_IRQn EXTI15_10_IRQn
#define Y_LIMIT_Pin GPIO_PIN_13
#define Y_LIMIT_GPIO_Port GPIOB
#define Y_LIMIT_EXTI_IRQn EXTI15_10_IRQn
#define Z_LIMIT_Pin GPIO_PIN_14
#define Z_LIMIT_GPIO_Port GPIOB
#define Z_LIMIT_EXTI_IRQn EXTI15_10_IRQn
#define Z_EN_Pin GPIO_PIN_8
#define Z_EN_GPIO_Port GPIOA
#define X_STEP_Pin GPIO_PIN_3
#define X_STEP_GPIO_Port GPIOB
#define Y_STEP_Pin GPIO_PIN_4
#define Y_STEP_GPIO_Port GPIOB
#define Z_STEP_Pin GPIO_PIN_5
#define Z_STEP_GPIO_Port GPIOB
#define X_DIR_Pin GPIO_PIN_6
#define X_DIR_GPIO_Port GPIOB
#define Y_DIR_Pin GPIO_PIN_7
#define Y_DIR_GPIO_Port GPIOB
#define Z_DIR_Pin GPIO_PIN_8
#define Z_DIR_GPIO_Port GPIOB
#define X_EN_Pin GPIO_PIN_9
#define X_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
