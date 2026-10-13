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
#define LPF2_Pin GPIO_PIN_2
#define LPF2_GPIO_Port GPIOE
#define LPF3_Pin GPIO_PIN_3
#define LPF3_GPIO_Port GPIOE
#define LPF4_Pin GPIO_PIN_4
#define LPF4_GPIO_Port GPIOE
#define LPF5_Pin GPIO_PIN_5
#define LPF5_GPIO_Port GPIOE
#define BUZZER_Pin GPIO_PIN_6
#define BUZZER_GPIO_Port GPIOE
#define KEY1_WKUP_Pin GPIO_PIN_13
#define KEY1_WKUP_GPIO_Port GPIOC
#define LPF6_Pin GPIO_PIN_14
#define LPF6_GPIO_Port GPIOC
#define PTT_OUT_Pin GPIO_PIN_15
#define PTT_OUT_GPIO_Port GPIOC
#define CTouch_INTX_Pin GPIO_PIN_2
#define CTouch_INTX_GPIO_Port GPIOF
#define CTouch_RSTX_Pin GPIO_PIN_3
#define CTouch_RSTX_GPIO_Port GPIOF
#define REAR_RESERVED1_Pin GPIO_PIN_4
#define REAR_RESERVED1_GPIO_Port GPIOF
#define PTT_INX_Pin GPIO_PIN_5
#define PTT_INX_GPIO_Port GPIOF
#define BACKUP_POWEREDX_Pin GPIO_PIN_15
#define BACKUP_POWEREDX_GPIO_Port GPIOF
#define BACKUP_POWEREDX_EXTI_IRQn EXTI15_10_IRQn
#define ALC_PWM_OUT_Pin GPIO_PIN_0
#define ALC_PWM_OUT_GPIO_Port GPIOG
#define CLEAR_FAULTX_OC_Pin GPIO_PIN_1
#define CLEAR_FAULTX_OC_GPIO_Port GPIOG
#define ETR_FREQDET_Pin GPIO_PIN_7
#define ETR_FREQDET_GPIO_Port GPIOE
#define TFT_DISP_Pin GPIO_PIN_10
#define TFT_DISP_GPIO_Port GPIOE
#define LO_BASE_CLK_Pin GPIO_PIN_11
#define LO_BASE_CLK_GPIO_Port GPIOB
#define HARDPWR_ERR_Pin GPIO_PIN_12
#define HARDPWR_ERR_GPIO_Port GPIOB
#define HARDPWR_ERR_EXTI_IRQn EXTI15_10_IRQn
#define RF_IN_PROTECT_CPU_SIG_Pin GPIO_PIN_13
#define RF_IN_PROTECT_CPU_SIG_GPIO_Port GPIOB
#define CPU_READY_Pin GPIO_PIN_8
#define CPU_READY_GPIO_Port GPIOD
#define LIGHTNING_PROTECTION_B_Pin GPIO_PIN_9
#define LIGHTNING_PROTECTION_B_GPIO_Port GPIOD
#define BL_PWM_Pin GPIO_PIN_14
#define BL_PWM_GPIO_Port GPIOD
#define PUMP_PWM_Pin GPIO_PIN_15
#define PUMP_PWM_GPIO_Port GPIOD
#define RF_BIAS_ON_Pin GPIO_PIN_2
#define RF_BIAS_ON_GPIO_Port GPIOG
#define RELAY2_Pin GPIO_PIN_3
#define RELAY2_GPIO_Port GPIOG
#define RELAY1_Pin GPIO_PIN_4
#define RELAY1_GPIO_Port GPIOG
#define MAIN_PSU_ON_SIG_Pin GPIO_PIN_5
#define MAIN_PSU_ON_SIG_GPIO_Port GPIOG
#define PWR_ON_SIG_Pin GPIO_PIN_7
#define PWR_ON_SIG_GPIO_Port GPIOG
#define FAN1_PWM_Pin GPIO_PIN_8
#define FAN1_PWM_GPIO_Port GPIOC
#define FAN1_SENSEX_Pin GPIO_PIN_4
#define FAN1_SENSEX_GPIO_Port GPIOD
#define PUMP_SENSEX_Pin GPIO_PIN_5
#define PUMP_SENSEX_GPIO_Port GPIOD
#define SDMMC2_CD_Pin GPIO_PIN_15
#define SDMMC2_CD_GPIO_Port GPIOG
#define LIGHTNING_PROTECTION_A_Pin GPIO_PIN_9
#define LIGHTNING_PROTECTION_A_GPIO_Port GPIOB
#define LPF1_Pin GPIO_PIN_1
#define LPF1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
