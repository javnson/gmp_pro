/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY3_Pin GPIO_PIN_2
#define KEY3_GPIO_Port GPIOE
#define KEY4_Pin GPIO_PIN_3
#define KEY4_GPIO_Port GPIOE
#define KEY5_Pin GPIO_PIN_4
#define KEY5_GPIO_Port GPIOE
#define KEY6_Pin GPIO_PIN_5
#define KEY6_GPIO_Port GPIOE
#define KEY7_Pin GPIO_PIN_6
#define KEY7_GPIO_Port GPIOE
#define SPEAK_Pin GPIO_PIN_13
#define SPEAK_GPIO_Port GPIOC
#define AD1_Pin GPIO_PIN_0
#define AD1_GPIO_Port GPIOC
#define AD2_Pin GPIO_PIN_1
#define AD2_GPIO_Port GPIOC
#define AD3_Pin GPIO_PIN_2
#define AD3_GPIO_Port GPIOC
#define AD4_Pin GPIO_PIN_3
#define AD4_GPIO_Port GPIOC
#define T2_C1_Pin GPIO_PIN_0
#define T2_C1_GPIO_Port GPIOA
#define TXD_Pin GPIO_PIN_2
#define TXD_GPIO_Port GPIOA
#define RXD_Pin GPIO_PIN_3
#define RXD_GPIO_Port GPIOA
#define DA1_Pin GPIO_PIN_4
#define DA1_GPIO_Port GPIOA
#define DA2_Pin GPIO_PIN_5
#define DA2_GPIO_Port GPIOA
#define T1_C1N_Pin GPIO_PIN_7
#define T1_C1N_GPIO_Port GPIOA
#define AD5_Pin GPIO_PIN_4
#define AD5_GPIO_Port GPIOC
#define AD6_Pin GPIO_PIN_5
#define AD6_GPIO_Port GPIOC
#define KEY8_Pin GPIO_PIN_7
#define KEY8_GPIO_Port GPIOE
#define LED_G_Pin GPIO_PIN_12
#define LED_G_GPIO_Port GPIOE
#define LED_Y_Pin GPIO_PIN_13
#define LED_Y_GPIO_Port GPIOE
#define LED_R_Pin GPIO_PIN_14
#define LED_R_GPIO_Port GPIOE
#define I2C_SCL_Pin GPIO_PIN_10
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_11
#define I2C_SDA_GPIO_Port GPIOB
#define Mz_CS_Pin GPIO_PIN_12
#define Mz_CS_GPIO_Port GPIOB
#define Mz_SCK_Pin GPIO_PIN_13
#define Mz_SCK_GPIO_Port GPIOB
#define Mz_RST_Pin GPIO_PIN_14
#define Mz_RST_GPIO_Port GPIOB
#define Mz_MOSI_Pin GPIO_PIN_15
#define Mz_MOSI_GPIO_Port GPIOB
#define EC_A_Pin GPIO_PIN_12
#define EC_A_GPIO_Port GPIOD
#define EC_B_Pin GPIO_PIN_13
#define EC_B_GPIO_Port GPIOD
#define EC_SW_Pin GPIO_PIN_14
#define EC_SW_GPIO_Port GPIOD
#define PCLK_Pin GPIO_PIN_9
#define PCLK_GPIO_Port GPIOC
#define T1_C1_Pin GPIO_PIN_8
#define T1_C1_GPIO_Port GPIOA
#define RF_CSN_Pin GPIO_PIN_15
#define RF_CSN_GPIO_Port GPIOA
#define RF_IRQ_Pin GPIO_PIN_10
#define RF_IRQ_GPIO_Port GPIOC
#define RF_CE_Pin GPIO_PIN_12
#define RF_CE_GPIO_Port GPIOC
#define OV_D0_Pin GPIO_PIN_0
#define OV_D0_GPIO_Port GPIOD
#define OV_D1_Pin GPIO_PIN_1
#define OV_D1_GPIO_Port GPIOD
#define OV_D2_Pin GPIO_PIN_2
#define OV_D2_GPIO_Port GPIOD
#define OV_D3_Pin GPIO_PIN_3
#define OV_D3_GPIO_Port GPIOD
#define OV_D4_Pin GPIO_PIN_4
#define OV_D4_GPIO_Port GPIOD
#define OV_D5_Pin GPIO_PIN_5
#define OV_D5_GPIO_Port GPIOD
#define OV_D6_Pin GPIO_PIN_6
#define OV_D6_GPIO_Port GPIOD
#define OV_D7_Pin GPIO_PIN_7
#define OV_D7_GPIO_Port GPIOD
#define RF_SCK_Pin GPIO_PIN_3
#define RF_SCK_GPIO_Port GPIOB
#define RF_MISO_Pin GPIO_PIN_4
#define RF_MISO_GPIO_Port GPIOB
#define RF_MOSI_Pin GPIO_PIN_5
#define RF_MOSI_GPIO_Port GPIOB
#define VSYNC_Pin GPIO_PIN_7
#define VSYNC_GPIO_Port GPIOB
#define VSYNC_EXTI_IRQn EXTI9_5_IRQn
#define KEY1_Pin GPIO_PIN_0
#define KEY1_GPIO_Port GPIOE
#define KEY2_Pin GPIO_PIN_1
#define KEY2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
