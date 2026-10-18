/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_hal_timebase_sys_tick.c
  * @brief   HAL timebase source file, using system SysTick as source of the time base.
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
#include "stm32h7xx_hal.h"

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  SysTick interrupt handler.
  * @note   This function is called by the HAL timebase module
  *         (HAL_InitTick) and the HAL tick interrupt (HAL_IncTick).
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/**
  * @brief  This function configures SysTick
  * @param  TickPriority: Tick interrupt priority.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  /*SystemCoreClock variable is declared in system_stm32h7xx.c file */
  uint32_t uwTimclock = 0U;
  uint32_t uwPeriod;
  uint32_t uwClockFreq;

  SysTick->CTRL = 0U;
  SysTick->LOAD = 0U;
  SysTick->VAL = 0U;

  uwClockFreq = HAL_RCC_GetHCLKFreq();
  uwPeriod = (1000000U / 1000U);
  uwTimclock = (uint32_t)(((uwClockFreq / 100UL) * uwPeriod));
  uwPeriod = ((uwTimclock / 1000U) >= 0xFFFFFFU) ? 0xFFFFFFU : (uwTimclock / 1000U);

  /* Configure the SysTick interrupt priority */
  if (TickPriority < (1UL << __NVIC_PRIO_BITS))
  {
    NVIC_SetPriority(SysTick_IRQn, TickPriority);
    uwTickPrio = TickPriority;
  }
  else
  {
    return HAL_ERROR;
  }

  /* Reload the SysTick counter value */
  SysTick->LOAD = uwPeriod;

  /* Set the SysTick period */
  SysTick->VAL = 0U;

  /* Enable SysTick IRQ */
  NVIC_EnableIRQ(SysTick_IRQn);

  /* Enable the SysTick_IRQn */
  SysTick->CTRL = (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Suspend Tick increment.
  * @note   Disable the tick increment by disabling SysTick.
  * @retval None
  */
void HAL_SuspendTick(void)
{
  /* Disable SysTick */
  SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk);
  /* Clear the tick increment flag */

}

/**
  * @brief  Resume Tick increment.
  * @note   Enable the tick increment by enabling SysTick.
  * @retval None
  */
void HAL_ResumeTick(void)
{
  /* Enable SysTick */
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}
