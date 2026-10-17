/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LPF2_Pin|LPF3_Pin|LPF4_Pin|LPF5_Pin
                          |TFT_DISP_Pin|LPF1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LPF6_Pin|PTT_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, CTouch_RSTX_Pin|REAR_RESERVED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, ALC_PWM_OUT_Pin|RF_BIAS_ON_Pin|RELAY2_Pin|RELAY1_Pin
                          |MAIN_PSU_ON_SIG_Pin|PWR_ON_SIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CLEAR_FAULTX_OC_GPIO_Port, CLEAR_FAULTX_OC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RF_IN_PROTECT_CPU_SIG_Pin|LIGHTNING_PROTECTION_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, CPU_READY_Pin|LIGHTNING_PROTECTION_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LPF2_Pin LPF3_Pin LPF4_Pin LPF5_Pin
                           TFT_DISP_Pin LPF1_Pin */
  GPIO_InitStruct.Pin = LPF2_Pin|LPF3_Pin|LPF4_Pin|LPF5_Pin
                          |TFT_DISP_Pin|LPF1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LPF6_Pin PTT_OUT_Pin */
  GPIO_InitStruct.Pin = LPF6_Pin|PTT_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : CTouch_INTX_Pin */
  GPIO_InitStruct.Pin = CTouch_INTX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CTouch_INTX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CTouch_RSTX_Pin REAR_RESERVED1_Pin */
  GPIO_InitStruct.Pin = CTouch_RSTX_Pin|REAR_RESERVED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PTT_INX_Pin BACKUP_POWEREDX_Pin */
  GPIO_InitStruct.Pin = PTT_INX_Pin|BACKUP_POWEREDX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : ALC_PWM_OUT_Pin */
  GPIO_InitStruct.Pin = ALC_PWM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ALC_PWM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLEAR_FAULTX_OC_Pin */
  GPIO_InitStruct.Pin = CLEAR_FAULTX_OC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CLEAR_FAULTX_OC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HARDPWR_ERR_Pin */
  GPIO_InitStruct.Pin = HARDPWR_ERR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(HARDPWR_ERR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_IN_PROTECT_CPU_SIG_Pin LIGHTNING_PROTECTION_A_Pin */
  GPIO_InitStruct.Pin = RF_IN_PROTECT_CPU_SIG_Pin|LIGHTNING_PROTECTION_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CPU_READY_Pin LIGHTNING_PROTECTION_B_Pin */
  GPIO_InitStruct.Pin = CPU_READY_Pin|LIGHTNING_PROTECTION_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_BIAS_ON_Pin RELAY2_Pin RELAY1_Pin MAIN_PSU_ON_SIG_Pin
                           PWR_ON_SIG_Pin */
  GPIO_InitStruct.Pin = RF_BIAS_ON_Pin|RELAY2_Pin|RELAY1_Pin|MAIN_PSU_ON_SIG_Pin
                          |PWR_ON_SIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : FAN1_SENSEX_Pin PUMP_SENSEX_Pin */
  GPIO_InitStruct.Pin = FAN1_SENSEX_Pin|PUMP_SENSEX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SDMMC2_CD_Pin */
  GPIO_InitStruct.Pin = SDMMC2_CD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SDMMC2_CD_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
