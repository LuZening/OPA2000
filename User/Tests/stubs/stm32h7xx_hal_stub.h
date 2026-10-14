/**
 * @file    stm32h7xx_hal_stub.h
 * @brief   Minimal HAL stub for host-side unit tests.
 *
 * @details Provides the small subset of STM32 HAL types/macros the
 *          Control / Networking algorithms touch, plus a recording
 *          surface so tests can assert "GPIO X was set high" without
 *          pulling in the real CMSIS + HAL.
 *
 *          IMPORTANT: this stub is a *compile-time* replacement for the
 *          real stm32h7xx_hal.h.  It must never be linked into the
 *          firmware - only into the host test binary.
 */

#ifndef STM32H7XX_HAL_STUB_H
#define STM32H7XX_HAL_STUB_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 *  Common status enum (used by ADC + DMA stubs)
 *============================================================================*/

typedef enum {
    HAL_OK       = 0x00U,
    HAL_ERROR    = 0x01U,
    HAL_BUSY     = 0x02U,
    HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;

/*============================================================================
 *  GPIO stub
 *============================================================================*/

typedef struct { uint32_t dummy; } GPIO_TypeDef;

/* Fake GPIO port identities.  hw_pinmap.h refers to GPIOA..GPIOG by name;
 * the tests only care *which* port a pin belongs to, never the real
 * MMIO address, so distinct static objects are enough. */
extern GPIO_TypeDef *const GPIOA;
extern GPIO_TypeDef *const GPIOB;
extern GPIO_TypeDef *const GPIOC;
extern GPIO_TypeDef *const GPIOD;
extern GPIO_TypeDef *const GPIOE;
extern GPIO_TypeDef *const GPIOF;
extern GPIO_TypeDef *const GPIOG;
extern GPIO_TypeDef *const GPIOH;

/* GPIO port bit masks (only 0..15 used by this project). */
#define GPIO_PIN_0      ((uint16_t)0x0001U)
#define GPIO_PIN_1      ((uint16_t)0x0002U)
#define GPIO_PIN_2      ((uint16_t)0x0004U)
#define GPIO_PIN_3      ((uint16_t)0x0008U)
#define GPIO_PIN_4      ((uint16_t)0x0010U)
#define GPIO_PIN_5      ((uint16_t)0x0020U)
#define GPIO_PIN_6      ((uint16_t)0x0040U)
#define GPIO_PIN_7      ((uint16_t)0x0080U)
#define GPIO_PIN_8      ((uint16_t)0x0100U)
#define GPIO_PIN_9      ((uint16_t)0x0200U)
#define GPIO_PIN_10     ((uint16_t)0x0400U)
#define GPIO_PIN_11     ((uint16_t)0x0800U)
#define GPIO_PIN_12     ((uint16_t)0x1000U)
#define GPIO_PIN_13     ((uint16_t)0x2000U)
#define GPIO_PIN_14     ((uint16_t)0x4000U)
#define GPIO_PIN_15     ((uint16_t)0x8000U)

#define GPIO_PIN_RESET  0U
#define GPIO_PIN_SET    1U

typedef enum {
    GPIO_PIN_RESET_E = 0U,
    GPIO_PIN_SET_E   = 1U
} GPIO_PinState;

void     stub_gpio_set      (GPIO_TypeDef *port, uint16_t pin_mask, uint8_t level);
uint8_t  stub_gpio_get_last (GPIO_TypeDef *port, uint16_t pin_mask);
uint32_t stub_gpio_set_count(GPIO_TypeDef *port, uint16_t pin_mask);
void     stub_gpio_reset_counters(void);

void         HAL_GPIO_WritePin (GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
GPIO_PinState HAL_GPIO_ReadPin  (GPIO_TypeDef *port, uint16_t pin);
void         HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin);

/*============================================================================
 *  TIM stub
 *============================================================================*/

typedef struct {
    uint32_t ARR, CCR1, CCR2, CCR3, CCR4, PSC, CNT, SR, DIER, CR1, CR2;
} TIM_TypeDef;

/* Mirrors the real HAL: TIM_HandleTypeDef::Instance is a *pointer*. */
typedef struct {
    TIM_TypeDef  *Instance;
    uint32_t      init_period;
    uint32_t      init_prescaler;
    char          name[16];
    int           pwm_started_ch_mask;
    int           base_started;
} TIM_HandleTypeDef;

#define TIM_CHANNEL_1   0x01U
#define TIM_CHANNEL_2   0x02U
#define TIM_CHANNEL_3   0x04U
#define TIM_CHANNEL_4   0x08U
#define TIM_IT_UPDATE   0x0001U
#define TIM_FLAG_UPDATE 0x0001U

int HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t channel);
int HAL_TIM_PWM_Stop (TIM_HandleTypeDef *htim, uint32_t channel);
int HAL_TIM_Base_Start(TIM_HandleTypeDef *htim);
int HAL_TIM_Base_Stop (TIM_HandleTypeDef *htim);
int HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);
int HAL_TIM_Base_Stop_IT (TIM_HandleTypeDef *htim);
int HAL_TIM_OC_Start   (TIM_HandleTypeDef *htim, uint32_t channel);
int HAL_TIM_OC_Stop    (TIM_HandleTypeDef *htim, uint32_t channel);
int HAL_TIM_OC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t channel,
                         uint32_t *pData, uint32_t Length);
int HAL_TIM_OC_Stop_DMA (TIM_HandleTypeDef *htim, uint32_t channel);

#define __HAL_TIM_SET_COUNTER(htim, c)        ((htim)->Instance->CNT = (c))
#define __HAL_TIM_GET_COUNTER(htim)            ((htim)->Instance->CNT)
#define __HAL_TIM_CLEAR_FLAG(htim, f)         ((htim)->Instance->SR = 0)
#define __HAL_TIM_CLEAR_IT(htim, f)           ((htim)->Instance->SR = 0)
#define __HAL_TIM_SET_COMPARE(htim, ch, v)                    \
    do {                                                      \
        switch (ch) {                                         \
        case TIM_CHANNEL_1: (htim)->Instance->CCR1 = (v); break;\
        case TIM_CHANNEL_2: (htim)->Instance->CCR2 = (v); break;\
        case TIM_CHANNEL_3: (htim)->Instance->CCR3 = (v); break;\
        case TIM_CHANNEL_4: (htim)->Instance->CCR4 = (v); break;\
        default: break;                                        \
        }                                                     \
    } while (0)

/*============================================================================
 *  DMA stub
 *============================================================================*/

typedef struct { uint32_t dummy[10]; } DMA_HandleTypeDef;

typedef enum {
    DMA_PERIPH_TO_MEMORY = 0U,
    DMA_MEMORY_TO_PERIPH = 1U,
    DMA_MEMORY_TO_MEMORY = 2U
} HAL_DMA_TransferDirectionTypeDef;

HAL_StatusTypeDef HAL_DMA_Start(DMA_HandleTypeDef *hdma,
                                uint32_t src, uint32_t dst,
                                uint32_t len,
                                HAL_DMA_TransferDirectionTypeDef dir);
HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma,
                                   uint32_t src, uint32_t dst,
                                   uint32_t len,
                                   HAL_DMA_TransferDirectionTypeDef dir);
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma);

/*============================================================================
 *  ADC stub
 *============================================================================*/

typedef struct { uint32_t dummy[8]; } ADC_HandleTypeDef;

/* ADC channel selectors used by hw_pinmap.h (AWD1/AWD2/AWD3 routing). */
#define ADC_CHANNEL_4           4U
#define ADC_CHANNEL_5           5U
#define ADC_CHANNEL_6           6U
#define ADC_CHANNEL_TEMPSENSOR  16U

/* Mirror of the STM32H7B0 ADC_TypeDef fields the Control code touches.
 * adc_calibration.c programs the analog-watchdog threshold registers
 * directly (one LTR/HTR pair per watchdog on this part). */
typedef struct {
    volatile uint32_t ISR;
    volatile uint32_t IER;
    volatile uint32_t CR;
    volatile uint32_t CFGR;
    volatile uint32_t CFGR2;
    volatile uint32_t SMPR1;
    volatile uint32_t SMPR2;
    volatile uint32_t PCSEL;
    volatile uint32_t LTR1;
    volatile uint32_t HTR1;
    volatile uint32_t LTR2;
    volatile uint32_t HTR2;
    volatile uint32_t LTR3;
    volatile uint32_t HTR3;
} ADC_TypeDef;

extern ADC_TypeDef *const ADC1;
extern ADC_TypeDef *const ADC2;

HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *hadc,
                                    uint32_t *pData, uint32_t Length);
HAL_StatusTypeDef HAL_ADC_Stop_DMA (ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedStart(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedStop (ADC_HandleTypeDef *hadc);

/*============================================================================
 *  Misc stubs
 *============================================================================*/

uint32_t HAL_GetTick(void);
void     HAL_Delay(uint32_t ms);
void     Error_Handler(void);

void     test_wall_clock_reset(void);
void     test_wall_clock_advance(uint32_t ms);
uint32_t test_wall_clock_call_count(void);

#ifdef __cplusplus
}
#endif

#endif /* STM32H7XX_HAL_STUB_H */