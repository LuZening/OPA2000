/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "adc.h"
#include "bdma2.h"
#include "comp.h"
#include "dac.h"
#include "dma.h"
#include "dma2d.h"
#include "dts.h"
#include "fatfs.h"
#include "fdcan.h"
#include "i2c.h"
#include "usart.h"
#include "ltdc.h"
#include "sai.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "boot.h"
#include "config.h"
#include "user_adc.h"
#include "power_control.h"
#include "protection.h"
#include "touch.h"
#include "gui.h"
#include "app_tasks.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  HAL_GPIO_WritePin(CPU_READY_GPIO_Port,CPU_READY_Pin,1);
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_BDMA2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_COMP1_Init();
  MX_COMP2_Init();
  MX_DAC1_Init();
  MX_DAC2_Init();
  MX_DMA2D_Init();
  MX_DTS_Init();
  MX_FDCAN1_Init();
  MX_I2C3_Init();
  MX_LPUART1_UART_Init();
  MX_LTDC_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM7_Init();
  MX_TIM15_Init();
  MX_I2C2_Init();
  MX_SDMMC2_SD_Init();
  MX_UART4_Init();
  MX_SAI2_Init();
  MX_TIM4_Init();
  MX_SPI6_Init();
  MX_FATFS_Init();
  MX_USB_OTG_HS_PCD_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(CPU_READY_GPIO_Port,CPU_READY_Pin,0);
  /* =========================================================================
   *  BOOT SEQUENCE (design spec 2026-10-14)
   *
   *  Step 0 (hardware defaults, before main): RF_BIAS PG2 = LOW,
   *         PWR_ON PG7 = LOW, REAR_RESERVED1 PF4 = HIGH.  Nothing
   *         transmits because the ADC / COMP / protection chain is not
   *         armed yet and g_ptt_ready is still false.
   *
   *  Everything below runs single-threaded - osKernelStart() has NOT
   *  been called - so no RTOS primitive (mutex, queue) may be used here.
   *  That is why config_lock() is deliberately absent below.
   * ====================================================================== */

  /* --- Step 1: pin the safety pins and blank the panel -------------- */
  /* Explicit, not relying on the reset state: RF path de-energised,
   * REAR_RESERVED1 held high, BL_PWM forced to 0 % so the operator
   * never sees a half-drawn screen while we boot. */
  boot_gpio_safe_state();

  /* --- Step 2: load (or rebuild) the persistent config ---------------- */
  bool have_valid_config = config_init();

  /* --- Step 3: probe the touch controller ------------------------------ */
  /* GT911 on I2C3 first, then NS2009 on I2C2; 100 ms per transaction.
   * Single-threaded, so the probe cannot collide with anything. */
  touch_init();

  /* Decide whether the touch driver needs calibration:
   *   - config_init() said "no valid record" -> we just reset to
   *     defaults -> the touch transform is the identity -> prompt.
   *   - a valid record was loaded but its touch transform is still the
   *     identity (operator never finished) -> prompt.
   *   - otherwise -> straight to the main screen. */
  g_calibration_required = !have_valid_config || config_touch_needs_calibration();
  g_touch_calibrated     = !g_calibration_required;

  /* --- Step 4: park every watchdog wide open, then start the comparators - */
  /* MX_COMP1/2_Init() already armed the comparator interrupts, and the DAC
   * references still hold their power-on value of 0 V.  A reference of 0 V
   * means "any input at all is over the limit", so the very first tick of
   * the comparator would latch an over-drive / over-current fault out of pure
   * noise - which is exactly the false alarm seen at power-up.

   * So instead of pushing the operator's real limits now, park AWD1/AWD2/AWD3
   * at full scale and both COMP DAC references at the VDDA rail: nothing in
   * the analog chain can possibly cross them.  boot_apply_thresholds() runs
   * later, from adc_stabilisation_tick(), once the front end has settled and
   * the IM zero-offset calibration has produced a trustworthy reading.  By
   * then CLEAR_FAULTX_OC has been held low for the whole window, so the
   * false alarm can never be latched in the first place. */
  boot_hold_thresholds_open();

  /* --- Enable the two comparators ------------------------------------ */
  /* MX_COMPx_Init() only configures the peripheral; without this the
   * comparators stay disabled and HAL_COMP_TriggerCallback() would never
   * fire, i.e. the COMP1 overdrive and COMP2 over-current protections
   * would be dead code.  Started only after the references above are at the
   * unreachable end of the DAC range, so the first comparison already runs
   * against a threshold that cannot be crossed. */
  if (HAL_COMP_Start_IT(&hcomp1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_COMP_Start_IT(&hcomp2) != HAL_OK) {
    Error_Handler();
  }

  /* --- Step 5: USB device bring-up (moved out of the deleted defaultTask) - */
  /* MX_USB_DEVICE_Init() is safe to call here: it does not depend on
   * FreeRTOS primitives, only registers the USB device descriptors and
   * starts the OTG_FS peripheral.  USB interrupt handlers stay dormant
   * until the NVIC is globally enabled (which happens at the first
   * context switch inside osKernelStart()), so no race exists.
   *
   * Previously this lived in freertos.c's defaultTask, which was a 512 B
   * idle task whose only job was to call MX_USB_DEVICE_Init() once and
   * then spin on osDelay(1).  Folding that one call into the boot
   * sequence retires the task and reclaims 512 B of stack + its TCB. */
//  MX_USB_DEVICE_Init();

  /* --- Step 6: hand over to the scheduler (below) --------------------- */
  /* From here on g_gui_ready / g_ptt_ready take over: the GUI task opens
   * the backlight after its first repaint, and app_adc_task raises
   * g_ptt_ready once the IM zero-offset calibration has finished.  Until
   * both are true, power_control refuses every PTT request. */
  
  HAL_GPIO_WritePin(CPU_READY_GPIO_Port,CPU_READY_Pin,1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /*AXI clock gating */
  RCC->CKGAENR = 0xE003FFFF;

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_CSI
                              |RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB2;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SDMMC
                              |RCC_PERIPHCLK_SAI2A|RCC_PERIPHCLK_SAI2B
                              |RCC_PERIPHCLK_SPI1|RCC_PERIPHCLK_FDCAN
                              |RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 16;
  PeriphClkInitStruct.PLL2.PLL2P = 4;
  PeriphClkInitStruct.PLL2.PLL2Q = 8;
  PeriphClkInitStruct.PLL2.PLL2R = 9;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.PLL3.PLL3M = 6;
  PeriphClkInitStruct.PLL3.PLL3N = 58;
  PeriphClkInitStruct.PLL3.PLL3P = 10;
  PeriphClkInitStruct.PLL3.PLL3Q = 24;
  PeriphClkInitStruct.PLL3.PLL3R = 10;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 8048;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
  PeriphClkInitStruct.Sai2BClockSelection = RCC_SAI2BCLKSOURCE_PLL3;
  PeriphClkInitStruct.Sai2AClockSelection = RCC_SAI2ACLKSOURCE_PLL3;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_PLL3;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief GPIO EXTI Callback for external interrupts
  * @param GPIO_Pin: Pin that triggered the interrupt
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == HARDPWR_ERR_Pin)
  {
    /* Hardware supply fault from the PSU (PB12, EXTI rising edge). */
    protection_hardpwr_err_isr();
  }
  else if (GPIO_Pin == PTT_INX_Pin)
  {
    /* Foot pedal / PTT key edge (PF5, EXTI5, active low). */
    power_control_ptt_isr();
  }
}

/**
  * @brief DMA half complete callback (for ADC double buffering)
  * @param hdma: DMA handle
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* User can add code here if needed */
}

/**
  * @brief DMA complete callback
  * @param hadc ADC handle
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1)
  {
    /* Publish the fresh regular-group results, then compare the channels
       that have no analog watchdog behind them (REV2 / FWD1 / FWD2).
       Runs in interrupt context: integer only, no FreeRTOS, no FPU. */
    adc_process();
    protection_check_adc1_dma();
  }
}

/**
  * @brief Injected conversion complete callback (ADC2: NTC + die temp)
  * @param hadc ADC handle
  * @retval None
  *
  * @details
  *  Reads the three JDR registers (JDR1=NTC core, JDR2=NTC aux,
  *  JDR3=internal temp sensor) and publishes them to the volatile
  *  buffers in user_adc.c.  Even though this callback fires at
  *  JEOS (because EOCSelection=ADC_EOC_SEQ_CONV), JDRx is updated
  *  per single conversion so the read order is rank 1/2/3
  *  regardless.  Reading must happen here - the task side can't,
  *  because task reading vs ISR writing would race without a lock.
  *
  *  Then runs the NTC over-temperature check.  Core NTC has no
  *  analog watchdog, so the over-temperature comparison lives here
  *  rather than in the 100 ms backstop.
  */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC2) {
    /* Order matches user_adc.c's three volatile slots and the ioc
     * InjectedChannel rows:
     *   Rank 1 = ADC_CHANNEL_2          -> adc2_inj_ntc_core_raw
     *   Rank 2 = ADC_CHANNEL_6          -> adc2_inj_ntc_aux_raw
     *   Rank 3 = ADC_CHANNEL_TEMPSENSOR -> adc2_inj_tsense_raw
     *
     * HAL_ADCEx_InjectedGetValue() reads the JDRx register of the
     * rank argument (1-based) - same convention as the ioc fields. */
    adc2_inj_ntc_core_raw = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    adc2_inj_ntc_aux_raw  = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    adc2_inj_tsense_raw   = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
  }
  /* Core NTC has no analog watchdog, so the over-temperature comparison
     lives here rather than in the 100 ms backstop. */
  protection_check_ntc();
}

/* Analog watchdog interrupts.  These fire from ADC1_2_IRQHandler at main
   priority 0, so the shutdown happens before anything else can run. */

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
  (void)hadc;
  protection_awd1_cb();     /* AWD1 -> CH4 RF input power  */
}

void HAL_ADCEx_LevelOutOfWindow2Callback(ADC_HandleTypeDef* hadc)
{
  (void)hadc;
  protection_awd2_cb();     /* AWD2 -> CH5 main current     */
}

void HAL_ADCEx_LevelOutOfWindow3Callback(ADC_HandleTypeDef* hadc)
{
  (void)hadc;
  protection_awd3_cb();     /* AWD3 -> CH6 coupler 1 reverse */
}

/**
  * @brief Comparator trigger callback.
  *
  * Fires from COMP_IRQHandler (main priority 0) via HAL_COMP_IRQHandler()
  * when either comparator's EXTI line fires.  The HAL provides this as a
  * __weak no-op, so without this override BOTH COMP protections are
  * silently dead: the interrupt reaches the HAL and then goes nowhere.
  *
  * COMP1 = RF input envelope overdrive (inverting input is DAC1 CH1,
  *         which boot_apply_thresholds() programs from
  *         g_config.band_rf_in_max_w).
  * COMP2 = main current over-current (inverting input is DAC1 CH2,
  *         programmed from g_config.band_im_max_mA).
  *
  * Both handlers run protection_trigger_locked(), which drops the relays,
  * the high-voltage rail and the PA bias synchronously - see the
  * concurrency note in protection.h.
  */
void HAL_COMP_TriggerCallback(COMP_HandleTypeDef* hcomp)
{
  if (hcomp == NULL)
  {
    return;
  }

  if (hcomp->Instance == COMP1)
  {
    protection_comp1_isr();
  }
  else if (hcomp->Instance == COMP2)
  {
    protection_comp2_isr();
  }
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x08000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256KB;
  MPU_InitStruct.SubRegionDisable = 0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x90000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_128MB;
  MPU_InitStruct.SubRegionDisable = 0x0;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0x20000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_128KB;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER3;
  MPU_InitStruct.BaseAddress = 0x24000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_1MB;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER4;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER5;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_128KB;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER6;
  MPU_InitStruct.BaseAddress = 0x38000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM5)
  {
    /* TIM5 is the 15 ms break-before-make delay for the PTT relay
     * sequence; it is started/stopped entirely from power_control.c. */
    power_relay_timer_callback();
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
