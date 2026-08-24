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
#include "stm32g0xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SYS_LED_Pin GPIO_PIN_0
#define SYS_LED_GPIO_Port GPIOA
#define PSUP_5V_M_Pin GPIO_PIN_1
#define PSUP_5V_M_GPIO_Port GPIOA
#define PSUP_12V_M_Pin GPIO_PIN_2
#define PSUP_12V_M_GPIO_Port GPIOA
#define Current_Measurement_Pin GPIO_PIN_3
#define Current_Measurement_GPIO_Port GPIOA
#define Voltage_Measurement_Pin GPIO_PIN_4
#define Voltage_Measurement_GPIO_Port GPIOA
#define Temperature_Measurement_Pin GPIO_PIN_5
#define Temperature_Measurement_GPIO_Port GPIOA
#define GPIO_OUT_FAN_NOT_READY_Pin GPIO_PIN_1
#define GPIO_OUT_FAN_NOT_READY_GPIO_Port GPIOB
#define GPIO_OUT_PSUP_NOT_READY_Pin GPIO_PIN_6
#define GPIO_OUT_PSUP_NOT_READY_GPIO_Port GPIOC
#define GPIO_OUT_OVER_VOLTAGE_Pin GPIO_PIN_15
#define GPIO_OUT_OVER_VOLTAGE_GPIO_Port GPIOA
#define GPIO_OUT_OVER_CURRENT_Pin GPIO_PIN_5
#define GPIO_OUT_OVER_CURRENT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
