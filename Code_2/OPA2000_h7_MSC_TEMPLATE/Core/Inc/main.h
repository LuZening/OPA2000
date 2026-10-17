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
#include "stm32h7xx_hal.h"

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
#define PWR_ON_SIG_Pin GPIO_PIN_2
#define PWR_ON_SIG_GPIO_Port GPIOE
#define BAND_SEL4X_Pin GPIO_PIN_3
#define BAND_SEL4X_GPIO_Port GPIOE
#define BAND_SEL3X_Pin GPIO_PIN_5
#define BAND_SEL3X_GPIO_Port GPIOE
#define BUZZER_Pin GPIO_PIN_6
#define BUZZER_GPIO_Port GPIOE
#define BAND_SEL2X_Pin GPIO_PIN_13
#define BAND_SEL2X_GPIO_Port GPIOC
#define PTT_INX_Pin GPIO_PIN_14
#define PTT_INX_GPIO_Port GPIOC
#define PTT_INX_EXTI_IRQn EXTI15_10_IRQn
#define BAND_SEL1X_Pin GPIO_PIN_15
#define BAND_SEL1X_GPIO_Port GPIOC
#define Overdrive_Pin GPIO_PIN_8
#define Overdrive_GPIO_Port GPIOE
#define Overdrive_EXTI_IRQn EXTI9_5_IRQn
#define ALL_FAULT_OUT_Pin GPIO_PIN_12
#define ALL_FAULT_OUT_GPIO_Port GPIOE
#define RF_BIAS_ON_Pin GPIO_PIN_11
#define RF_BIAS_ON_GPIO_Port GPIOB
#define HARDPWR_ERR_Pin GPIO_PIN_12
#define HARDPWR_ERR_GPIO_Port GPIOB
#define HARDPWR_ERR_EXTI_IRQn EXTI15_10_IRQn
#define BAND_TX_Pin GPIO_PIN_14
#define BAND_TX_GPIO_Port GPIOB
#define BAND_RX_Pin GPIO_PIN_15
#define BAND_RX_GPIO_Port GPIOB
#define CPU_READY_Pin GPIO_PIN_8
#define CPU_READY_GPIO_Port GPIOD
#define LCD_BL_Pin GPIO_PIN_9
#define LCD_BL_GPIO_Port GPIOD
#define T_INTX_Pin GPIO_PIN_11
#define T_INTX_GPIO_Port GPIOD
#define T_INTX_EXTI_IRQn EXTI15_10_IRQn
#define ENC_SWX_Pin GPIO_PIN_13
#define ENC_SWX_GPIO_Port GPIOD
#define ENC_SWX_EXTI_IRQn EXTI15_10_IRQn
#define RELAY2_Pin GPIO_PIN_14
#define RELAY2_GPIO_Port GPIOD
#define RELAY1_Pin GPIO_PIN_15
#define RELAY1_GPIO_Port GPIOD
#define FAN1_PWM_Pin GPIO_PIN_8
#define FAN1_PWM_GPIO_Port GPIOC
#define BAND_SEL6X_Pin GPIO_PIN_10
#define BAND_SEL6X_GPIO_Port GPIOC
#define BAND_SEL5X_Pin GPIO_PIN_4
#define BAND_SEL5X_GPIO_Port GPIOD
#define FREQDET_IN_Pin GPIO_PIN_0
#define FREQDET_IN_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
