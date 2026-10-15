/**
 * @file    main.h  (host-test shim)
 * @brief   Redirects <main.h> to the HAL stub for the host unit tests.
 *
 * @details User sources such as user_adc.h include "main.h" to reach the
 *          real CubeMX-generated header.  On the host we only need the HAL
 *          stub, so this file shadows the generated Core/Inc/main.h when
 *          User/Tests/stubs is first on the include path.
 *
 *          The GPIO pin macros below mirror Core/Inc/main.h.  They exist
 *          because hw_pinmap.h now aliases its short names onto the
 *          CubeMX-generated ones, so anything including it needs these to
 *          resolve.  Keep them in step with the real header - the GPIO
 *          recorder in stm32h7xx_hal_stub.c treats each GPIO_TypeDef as an
 *          opaque identity, so a wrong port here shows up as a test
 *          asserting on the wrong pin.
 *
 *          Never visible to the firmware build.
 */

#ifndef MAIN_HOST_TEST_SHIM_H
#define MAIN_HOST_TEST_SHIM_H

#include "stm32h7xx_hal_stub.h"

/* ---- mirrors of Core/Inc/main.h -------------------------------------- */

/* LPF band relays */
#define LPF1_Pin GPIO_PIN_1
#define LPF1_GPIO_Port GPIOE
#define LPF2_Pin GPIO_PIN_2
#define LPF2_GPIO_Port GPIOE
#define LPF3_Pin GPIO_PIN_3
#define LPF3_GPIO_Port GPIOE
#define LPF4_Pin GPIO_PIN_4
#define LPF4_GPIO_Port GPIOE
#define LPF5_Pin GPIO_PIN_5
#define LPF5_GPIO_Port GPIOE
#define LPF6_Pin GPIO_PIN_14
#define LPF6_GPIO_Port GPIOC

/* Power / relay */
#define RELAY1_Pin GPIO_PIN_4
#define RELAY1_GPIO_Port GPIOG
#define RELAY2_Pin GPIO_PIN_3
#define RELAY2_GPIO_Port GPIOG
#define MAIN_PSU_ON_SIG_Pin GPIO_PIN_5
#define MAIN_PSU_ON_SIG_GPIO_Port GPIOG
#define PWR_ON_SIG_Pin GPIO_PIN_7
#define PWR_ON_SIG_GPIO_Port GPIOG
#define RF_BIAS_ON_Pin GPIO_PIN_2
#define RF_BIAS_ON_GPIO_Port GPIOG

/* Fault I/O */
#define HARDPWR_ERR_Pin GPIO_PIN_12
#define HARDPWR_ERR_GPIO_Port GPIOB
#define RF_IN_PROTECT_CPU_SIG_Pin GPIO_PIN_13
#define RF_IN_PROTECT_CPU_SIG_GPIO_Port GPIOB
#define CPU_READY_Pin GPIO_PIN_8
#define CPU_READY_GPIO_Port GPIOD
#define LIGHTNING_PROTECTION_A_Pin GPIO_PIN_9
#define LIGHTNING_PROTECTION_A_GPIO_Port GPIOB
#define LIGHTNING_PROTECTION_B_Pin GPIO_PIN_9
#define LIGHTNING_PROTECTION_B_GPIO_Port GPIOD

/* PTT */
#define PTT_OUT_Pin GPIO_PIN_15
#define PTT_OUT_GPIO_Port GPIOC
#define PTT_INX_Pin GPIO_PIN_5
#define PTT_INX_GPIO_Port GPIOF
#define KEY1_WKUP_Pin GPIO_PIN_13
#define KEY1_WKUP_GPIO_Port GPIOC

/* Fan / pump / backlight / ALC / buzzer / ETR */
#define FAN1_PWM_Pin GPIO_PIN_8
#define FAN1_PWM_GPIO_Port GPIOC
#define PUMP_PWM_Pin GPIO_PIN_15
#define PUMP_PWM_GPIO_Port GPIOD
#define BL_PWM_Pin GPIO_PIN_14
#define BL_PWM_GPIO_Port GPIOD
#define FAN1_SENSEX_Pin GPIO_PIN_4
#define FAN1_SENSEX_GPIO_Port GPIOD
#define PUMP_SENSEX_Pin GPIO_PIN_5
#define PUMP_SENSEX_GPIO_Port GPIOD
#define ALC_PWM_OUT_Pin GPIO_PIN_0
#define ALC_PWM_OUT_GPIO_Port GPIOG
#define ETR_FREQDET_Pin GPIO_PIN_7
#define ETR_FREQDET_GPIO_Port GPIOE
#define BUZZER_Pin GPIO_PIN_6
#define BUZZER_GPIO_Port GPIOE

/* Touch / rear */
#define REAR_RESERVED1_Pin GPIO_PIN_4
#define REAR_RESERVED1_GPIO_Port GPIOF
#define CTouch_RSTX_Pin GPIO_PIN_3
#define CTouch_RSTX_GPIO_Port GPIOF
#define CTouch_INTX_Pin GPIO_PIN_2
#define CTouch_INTX_GPIO_Port GPIOF

/* SD card detect */
#define SDMMC2_CD_Pin GPIO_PIN_15
#define SDMMC2_CD_GPIO_Port GPIOG

#endif /* MAIN_HOST_TEST_SHIM_H */