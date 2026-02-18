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
#ifdef halerror
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
#endif
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define T1_Pin GPIO_PIN_2
#define T1_GPIO_Port GPIOE
#define T2_Pin GPIO_PIN_3
#define T2_GPIO_Port GPIOE
#define T3_Pin GPIO_PIN_4
#define T3_GPIO_Port GPIOE
#define T4_Pin GPIO_PIN_5
#define T4_GPIO_Port GPIOE
#define T5_Pin GPIO_PIN_6
#define T5_GPIO_Port GPIOE
#define T6_Pin GPIO_PIN_13
#define T6_GPIO_Port GPIOC
#define L1_Pin GPIO_PIN_2
#define L1_GPIO_Port GPIOC
#define L2_Pin GPIO_PIN_3
#define L2_GPIO_Port GPIOC
#define K1_Pin GPIO_PIN_0
#define K1_GPIO_Port GPIOA
#define L3_Pin GPIO_PIN_1
#define L3_GPIO_Port GPIOA
#define L4_Pin GPIO_PIN_2
#define L4_GPIO_Port GPIOA
#define L5_Pin GPIO_PIN_3
#define L5_GPIO_Port GPIOA
#define L6_Pin GPIO_PIN_4
#define L6_GPIO_Port GPIOA
#define L7_Pin GPIO_PIN_5
#define L7_GPIO_Port GPIOA
#define ENC_VID_1_Pin GPIO_PIN_6
#define ENC_VID_1_GPIO_Port GPIOA
#define ENC_VID_2_Pin GPIO_PIN_7
#define ENC_VID_2_GPIO_Port GPIOA
#define L8_Pin GPIO_PIN_4
#define L8_GPIO_Port GPIOC
#define VID_BTN_Pin GPIO_PIN_5
#define VID_BTN_GPIO_Port GPIOC
#define BRT_BTN_Pin GPIO_PIN_0
#define BRT_BTN_GPIO_Port GPIOB
#define B1_Pin GPIO_PIN_1
#define B1_GPIO_Port GPIOB
#define B2_Pin GPIO_PIN_7
#define B2_GPIO_Port GPIOE
#define B3_Pin GPIO_PIN_8
#define B3_GPIO_Port GPIOE
#define B4_Pin GPIO_PIN_9
#define B4_GPIO_Port GPIOE
#define B5_Pin GPIO_PIN_10
#define B5_GPIO_Port GPIOE
#define B6_Pin GPIO_PIN_11
#define B6_GPIO_Port GPIOE
#define B7_Pin GPIO_PIN_12
#define B7_GPIO_Port GPIOE
#define B8_Pin GPIO_PIN_13
#define B8_GPIO_Port GPIOE
#define R1_Pin GPIO_PIN_14
#define R1_GPIO_Port GPIOE
#define R2_Pin GPIO_PIN_15
#define R2_GPIO_Port GPIOE
#define R3_Pin GPIO_PIN_10
#define R3_GPIO_Port GPIOB
#define R4_Pin GPIO_PIN_11
#define R4_GPIO_Port GPIOB
#define R5_Pin GPIO_PIN_12
#define R5_GPIO_Port GPIOB
#define R6_Pin GPIO_PIN_13
#define R6_GPIO_Port GPIOB
#define R7_Pin GPIO_PIN_14
#define R7_GPIO_Port GPIOB
#define SW_DAY_Pin GPIO_PIN_11
#define SW_DAY_GPIO_Port GPIOD
#define SW_NT_Pin GPIO_PIN_12
#define SW_NT_GPIO_Port GPIOD
#define SW_MONO_Pin GPIO_PIN_13
#define SW_MONO_GPIO_Port GPIOD
#define R8_Pin GPIO_PIN_14
#define R8_GPIO_Port GPIOD
#define R9_Pin GPIO_PIN_15
#define R9_GPIO_Port GPIOD
#define FLASH_CS_Pin GPIO_PIN_15
#define FLASH_CS_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOD
#define ENC_BRT_1_Pin GPIO_PIN_6
#define ENC_BRT_1_GPIO_Port GPIOB
#define ENC_BRT_2_Pin GPIO_PIN_7
#define ENC_BRT_2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
