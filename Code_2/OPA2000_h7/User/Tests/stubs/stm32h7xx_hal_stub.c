/**
 * @file    stm32h7xx_hal_stub.c
 * @brief   HAL stub implementations + extern peripheral handles for the
 *          host-side test binary.
 *
 * @details GPIO writes are recorded per (port, pin) so tests can assert
 *          both the level and how many times a pin was touched.
 *          TIM stubs count PWM / base start events.
 *          HAL_GetTick() is backed by a fake clock the tests advance
 *          explicitly via test_wall_clock_advance().
 */

#include "stm32h7xx_hal_stub.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/*============================================================================
 *  Extern peripheral handles
 *
 *  power_control.c / user_adc.c / freq_detect.c declare these as extern.
 *  In the firmware they are defined in Core/Src/main.c; in the test
 *  binary we provide them here.
 *============================================================================*/

/* Each handle owns a statically allocated TIM_TypeDef so that
 * htimN.Instance->ARR behaves exactly like the firmware. */
static TIM_TypeDef s_tim1;
static TIM_TypeDef s_tim2;
static TIM_TypeDef s_tim3;
static TIM_TypeDef s_tim4;
static TIM_TypeDef s_tim5;
static TIM_TypeDef s_tim7;
static TIM_TypeDef s_tim15;

TIM_HandleTypeDef htim1  = { &s_tim1,  0, 0, "TIM1",  0, 0 };
TIM_HandleTypeDef htim2  = { &s_tim2,  0, 0, "TIM2",  0, 0 };
TIM_HandleTypeDef htim3  = { &s_tim3,  0, 0, "TIM3",  0, 0 };
TIM_HandleTypeDef htim4  = { &s_tim4,  0, 0, "TIM4",  0, 0 };
TIM_HandleTypeDef htim5  = { &s_tim5,  0, 0, "TIM5",  0, 0 };
TIM_HandleTypeDef htim7  = { &s_tim7,  0, 0, "TIM7",  0, 0 };
TIM_HandleTypeDef htim15 = { &s_tim15, 0, 0, "TIM15", 0, 0 };

DMA_HandleTypeDef hdma_tim2_ch1;
DMA_HandleTypeDef hdma_tim2_ch2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

/* DAC1 handle: boot.c declares it extern and drives it from
 * boot_apply_comp_thresholds().  The registers behind it are here so a
 * test can assert the DAC code that a config threshold produced. */
static DAC_TypeDef s_dac1;
DAC_HandleTypeDef hdac1 = { &s_dac1, 0, 0, "DAC1" };

/* Fake ADC instances - tests inspect LTR/HTR after the AWD setters run. */
static ADC_TypeDef s_adc1;
static ADC_TypeDef s_adc2;

ADC_TypeDef *const ADC1 = &s_adc1;
ADC_TypeDef *const ADC2 = &s_adc2;

/*============================================================================
 *  Fake GPIO port identities
 *============================================================================*/

static GPIO_TypeDef s_gpioA = { 0 };
static GPIO_TypeDef s_gpioB = { 0 };
static GPIO_TypeDef s_gpioC = { 0 };
static GPIO_TypeDef s_gpioD = { 0 };
static GPIO_TypeDef s_gpioE = { 0 };
static GPIO_TypeDef s_gpioF = { 0 };
static GPIO_TypeDef s_gpioG = { 0 };
static GPIO_TypeDef s_gpioH = { 0 };

GPIO_TypeDef *const GPIOA = &s_gpioA;
GPIO_TypeDef *const GPIOB = &s_gpioB;
GPIO_TypeDef *const GPIOC = &s_gpioC;
GPIO_TypeDef *const GPIOD = &s_gpioD;
GPIO_TypeDef *const GPIOE = &s_gpioE;
GPIO_TypeDef *const GPIOF = &s_gpioF;
GPIO_TypeDef *const GPIOG = &s_gpioG;

/* EXTI controller: the firmware masks the digital fault lines per line
 * through IMR1, and the test needs to see that mask. */
static EXTI_TypeDef s_exti = { 0 };
EXTI_TypeDef *const EXTI = &s_exti;
GPIO_TypeDef *const GPIOH = &s_gpioH;

/*============================================================================
 *  GPIO recording
 *============================================================================*/

typedef struct {
    uintptr_t port_addr;
    uint16_t  pin_mask;
    uint8_t   last_level;
    uint32_t  count;
} gpio_event_t;

#define GPIO_EVENT_MAX 128U
static gpio_event_t s_gpio_events[GPIO_EVENT_MAX];
static uint32_t      s_gpio_event_count = 0U;

static gpio_event_t *find_or_create_event(GPIO_TypeDef *port, uint16_t pin)
{
    for (uint32_t i = 0; i < s_gpio_event_count; i++) {
        if (s_gpio_events[i].port_addr == (uintptr_t)port &&
            s_gpio_events[i].pin_mask == pin) {
            return &s_gpio_events[i];
        }
    }
    if (s_gpio_event_count >= GPIO_EVENT_MAX) return NULL;
    gpio_event_t *e = &s_gpio_events[s_gpio_event_count++];
    e->port_addr  = (uintptr_t)port;
    e->pin_mask   = pin;
    e->last_level = 0xFFU;      /* undefined */
    e->count      = 0U;
    return e;
}

void stub_gpio_set(GPIO_TypeDef *port, uint16_t pin, uint8_t level)
{
    gpio_event_t *e = find_or_create_event(port, pin);
    if (!e) return;
    e->last_level = level;
    e->count++;
}

uint8_t stub_gpio_get_last(GPIO_TypeDef *port, uint16_t pin)
{
    for (uint32_t i = 0; i < s_gpio_event_count; i++) {
        if (s_gpio_events[i].port_addr == (uintptr_t)port &&
            s_gpio_events[i].pin_mask == pin) {
            return s_gpio_events[i].last_level;
        }
    }
    return 0xFFU;
}

uint32_t stub_gpio_set_count(GPIO_TypeDef *port, uint16_t pin)
{
    for (uint32_t i = 0; i < s_gpio_event_count; i++) {
        if (s_gpio_events[i].port_addr == (uintptr_t)port &&
            s_gpio_events[i].pin_mask == pin) {
            return s_gpio_events[i].count;
        }
    }
    return 0U;
}

void stub_gpio_reset_counters(void)
{
    /* Clear the write counters but keep the last recorded level, so a test
     * can establish a known pin state (e.g. via power_init_outputs) and
     * still count only the writes it triggered afterwards. */
    for (uint32_t i = 0; i < s_gpio_event_count; i++) {
        s_gpio_events[i].count = 0U;
    }
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    stub_gpio_set(port, pin, (uint8_t)state);
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    uint8_t last = stub_gpio_get_last(port, pin);
    return (last == 0xFFU) ? GPIO_PIN_RESET_E
                           : (last ? GPIO_PIN_SET_E : GPIO_PIN_RESET_E);
}

void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin)
{
    uint8_t cur = stub_gpio_get_last(port, pin);
    stub_gpio_set(port, pin, (cur == GPIO_PIN_SET_E) ? GPIO_PIN_RESET_E
                                                      : GPIO_PIN_SET_E);
}

/*============================================================================
 *  TIM stub
 *============================================================================*/

/*============================================================================
 *  DAC stub
 *==========================================================================*/

HAL_StatusTypeDef HAL_DAC_SetValue(DAC_HandleTypeDef *hdac, uint32_t Channel,
                                   uint32_t Alignment, uint32_t Data)
{
    (void)Alignment;
    if (hdac == NULL || hdac->Instance == NULL) {
        return HAL_ERROR;
    }
    /* The real HAL writes one of the two right-aligned data holding
     * registers and nothing else - no pin is driven until Start(). */
    if (Channel == DAC_CHANNEL_1) {
        hdac->Instance->DHR12R1 = Data & 0xFFFU;
    } else {
        hdac->Instance->DHR12R2 = Data & 0xFFFU;
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef *hdac, uint32_t Channel)
{
    if (hdac == NULL || hdac->Instance == NULL) {
        return HAL_ERROR;
    }
    hdac->Instance->CR |= (Channel == DAC_CHANNEL_1) ? 0x01U : 0x02U;
    return HAL_OK;
}

/*============================================================================
 *  COMP stub
 *==========================================================================*/

static COMP_TypeDef s_comp1 = { 0U };
static COMP_TypeDef s_comp2 = { 0U };

COMP_HandleTypeDef hcomp1 = { &s_comp1 };
COMP_HandleTypeDef hcomp2 = { &s_comp2 };

HAL_StatusTypeDef HAL_COMP_Start_IT(COMP_HandleTypeDef *hcomp)
{
    if (hcomp == NULL || hcomp->Instance == NULL) {
        return HAL_ERROR;
    }
    /* EN bit, as the real HAL would set.  user_adc.c only checks the
     * return value, but leaving the register observable keeps the stub
     * honest. */
    hcomp->Instance->CR |= 0x00000004U;
    return HAL_OK;
}

int HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t channel)
{
    if (htim) htim->pwm_started_ch_mask |= (int)channel;
    return 0;
}

int HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t channel)
{
    if (htim) htim->pwm_started_ch_mask &= ~(int)channel;
    return 0;
}

int HAL_TIM_Base_Start(TIM_HandleTypeDef *htim)
{
    if (htim) htim->base_started = 1;
    return 0;
}

int HAL_TIM_Base_Stop(TIM_HandleTypeDef *htim)
{
    if (htim) htim->base_started = 0;
    return 0;
}

int HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim)
{
    if (htim) htim->base_started = 1;
    return 0;
}

int HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *htim)
{
    if (htim) htim->base_started = 0;
    return 0;
}

int HAL_TIM_OC_Start(TIM_HandleTypeDef *htim, uint32_t channel)
{
    if (htim) htim->pwm_started_ch_mask |= (int)channel;
    return 0;
}

int HAL_TIM_OC_Stop(TIM_HandleTypeDef *htim, uint32_t channel)
{
    if (htim) htim->pwm_started_ch_mask &= ~(int)channel;
    return 0;
}

int HAL_TIM_OC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t channel,
                         uint32_t *pData, uint32_t Length)
{
    (void)pData;
    (void)Length;
    if (htim) htim->pwm_started_ch_mask |= (int)channel;
    return 0;
}

int HAL_TIM_OC_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t channel)
{
    if (htim) htim->pwm_started_ch_mask &= ~(int)channel;
    return 0;
}

/*============================================================================
 *  DMA stub
 *============================================================================*/

HAL_StatusTypeDef HAL_DMA_Start(DMA_HandleTypeDef *hdma,
                                uint32_t src, uint32_t dst,
                                uint32_t len,
                                HAL_DMA_TransferDirectionTypeDef dir)
{
    (void)hdma; (void)src; (void)dst; (void)len; (void)dir;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma,
                                   uint32_t src, uint32_t dst,
                                   uint32_t len,
                                   HAL_DMA_TransferDirectionTypeDef dir)
{
    (void)hdma; (void)src; (void)dst; (void)len; (void)dir;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    return HAL_OK;
}

/*============================================================================
 *  ADC stub
 *============================================================================*/

HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *hadc,
                                    uint32_t *pData, uint32_t Length)
{
    (void)hadc; (void)pData; (void)Length;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    return HAL_OK;
}

/* Track which variant the firmware called.  Reset by the test, observed
 * by the test - the property being guarded is exactly which one
 * adc_injected_tick() reaches for, because the polling variant
 * silently disables the NTC over-temperature protection. */
static uint32_t s_inj_start_it_calls     = 0U;
static uint32_t s_inj_start_poll_calls   = 0U;

uint32_t test_injected_start_it_calls(void)   { return s_inj_start_it_calls; }
uint32_t test_injected_start_poll_calls(void) { return s_inj_start_poll_calls; }
void     test_injected_reset_counters(void)   { s_inj_start_it_calls = 0U; s_inj_start_poll_calls = 0U; }

HAL_StatusTypeDef HAL_ADCEx_InjectedStart(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    s_inj_start_poll_calls++;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef *hadc)
{
    /* _IT variant exists to enable the JEOS interrupt; on the host we
     * only need the linkage.  The host tests that care about the
     * interrupt path stub a counter here rather than rely on the
     * callback firing, because the callback target lives in main.c
     * which is not part of the host build. */
    (void)hadc;
    s_inj_start_it_calls++;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_ADCEx_InjectedStop(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_ADCEx_InjectedStop_IT(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    return HAL_OK;
}

/*============================================================================
 *  Tick + Error_Handler stubs
 *============================================================================*/

static uint32_t s_tick_ms    = 0U;
static uint32_t s_tick_calls = 0U;
/* How many times a test drove the core into its parking loop.  Reset with
 * the fake clock so a test can assert power_soft_off() reached it. */
static uint32_t s_wfi_count  = 0U;

void test_wall_clock_reset(void)   { s_tick_ms = 0U; s_tick_calls = 0U; s_wfi_count = 0U; }
void test_wall_clock_advance(uint32_t ms) { s_tick_ms += ms; s_tick_calls++; }
uint32_t test_wall_clock_call_count(void) { return s_tick_calls; }
uint32_t test_wfi_count(void)     { return s_wfi_count; }

uint32_t HAL_GetTick(void) { return s_tick_ms; }

void HAL_Delay(uint32_t ms)
{
    /* Tests never really sleep; just move the fake clock forward. */
    s_tick_ms += ms;
}

void __WFI(void)
{
    /* The firmware's __WFI parks the core forever; here it just returns so
     * a test that reaches power_soft_off()'s parking loop can observe the
     * shutdown sequence and carry on.  Count it so a test can assert the
     * sequence really did get as far as parking the CPU. */
    s_wfi_count++;
}

void Error_Handler(void)
{
    fprintf(stderr, "[TEST] Error_Handler invoked -> aborting test run\n");
    abort();
}