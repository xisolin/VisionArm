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
#define KEY2_Pin GPIO_PIN_13
#define KEY2_GPIO_Port GPIOC
#define POWER_U_Pin GPIO_PIN_0
#define POWER_U_GPIO_Port GPIOA
#define TEMP_Pin GPIO_PIN_1
#define TEMP_GPIO_Port GPIOA
#define IN_BM_Pin GPIO_PIN_2
#define IN_BM_GPIO_Port GPIOA
#define IN_BP_Pin GPIO_PIN_3
#define IN_BP_GPIO_Port GPIOA
#define IN_AM_Pin GPIO_PIN_4
#define IN_AM_GPIO_Port GPIOA
#define IN_AP_Pin GPIO_PIN_5
#define IN_AP_GPIO_Port GPIOA
#define ADC_ENC_Pin GPIO_PIN_6
#define ADC_ENC_GPIO_Port GPIOA
#define MT6816_AUX_CS_Pin GPIO_PIN_7
#define MT6816_AUX_CS_GPIO_Port GPIOA
#define BUTTON_2_Pin GPIO_PIN_2
#define BUTTON_2_GPIO_Port GPIOB
#define IN_PWM_B_Pin GPIO_PIN_10
#define IN_PWM_B_GPIO_Port GPIOB
#define IN_PWM_A_Pin GPIO_PIN_11
#define IN_PWM_A_GPIO_Port GPIOB
#define KEY1_Pin GPIO_PIN_12
#define KEY1_GPIO_Port GPIOB
#define MT6816_CS_Pin GPIO_PIN_15
#define MT6816_CS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
