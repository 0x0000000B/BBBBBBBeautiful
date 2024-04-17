/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define C1_Pin GPIO_PIN_1
#define C1_GPIO_Port GPIOA
#define C2_Pin GPIO_PIN_2
#define C2_GPIO_Port GPIOA
#define C3_Pin GPIO_PIN_3
#define C3_GPIO_Port GPIOA
#define C4_Pin GPIO_PIN_4
#define C4_GPIO_Port GPIOA
#define C5_Pin GPIO_PIN_5
#define C5_GPIO_Port GPIOA
#define C6_Pin GPIO_PIN_6
#define C6_GPIO_Port GPIOA
#define C7_Pin GPIO_PIN_7
#define C7_GPIO_Port GPIOA
#define RV_Pin GPIO_PIN_1
#define RV_GPIO_Port GPIOB
#define YV_Pin GPIO_PIN_2
#define YV_GPIO_Port GPIOB
#define SEG2_Pin GPIO_PIN_10
#define SEG2_GPIO_Port GPIOB
#define SEG3_Pin GPIO_PIN_11
#define SEG3_GPIO_Port GPIOB
#define SEG4_Pin GPIO_PIN_12
#define SEG4_GPIO_Port GPIOB
#define SEG5_Pin GPIO_PIN_13
#define SEG5_GPIO_Port GPIOB
#define SEG6_Pin GPIO_PIN_14
#define SEG6_GPIO_Port GPIOB
#define SEG7_Pin GPIO_PIN_15
#define SEG7_GPIO_Port GPIOB
#define C8_Pin GPIO_PIN_8
#define C8_GPIO_Port GPIOA
#define C9_Pin GPIO_PIN_9
#define C9_GPIO_Port GPIOA
#define C10_Pin GPIO_PIN_10
#define C10_GPIO_Port GPIOA
#define C11_Pin GPIO_PIN_11
#define C11_GPIO_Port GPIOA
#define C12_Pin GPIO_PIN_12
#define C12_GPIO_Port GPIOA
#define GV_Pin GPIO_PIN_3
#define GV_GPIO_Port GPIOB
#define SEG1_Pin GPIO_PIN_9
#define SEG1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
