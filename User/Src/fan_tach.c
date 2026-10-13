/**
 * @file fan_tach.c
 * @brief Fan / pump TACH (RPM) detection - 500 us polling in TIM7 ISR.
 *
 * @details Pulse counting:
 *   - ISR reads GPIOD->IDR for PD4 / PD5.
 *   - Tracks previous level.  Falling edge (high->low) increments a
 *     per-channel pulse counter.
 *   - Counters are reset by the task side every 100 ms.
 *   - RPM = count * 60 / poles / (window_seconds)
 *     where window = 100 ms, poles = 2 -> RPM = count * 300.
 *
 * The ISR is integer-only (no FPU).  TIM7 is configured at 500 us
 * (prescaler = 19, period = 4999).
 */

#include "fan_tach.h"
#include "stm32h7xx_hal.h"

/* TIM7: 200 MHz / 20 = 10 MHz, period = 4999 -> 500 us tick. */
#define FAN_TACH_TIM_PRESCALER  19U
#define FAN_TACH_TIM_PERIOD     4999U

/* Typical PC fan = 2 poles. */
#define FAN_POLES               2U
#define FAN_WINDOW_MS           100U

/*============================================================================
 *  Private state (volatile for ISR/task sharing)
 *============================================================================*/

/* Pulse count accumulated during the current window. */
static volatile uint32_t s_fan_pulse_count  = 0U;
static volatile uint32_t s_pump_pulse_count = 0U;

/* Last sampled GPIO level (active high - active low signal with pull-up
 * gives a high idle and a low pulse, so we detect falling edges). */
static volatile uint8_t  s_fan_last_level  = 1U;
static volatile uint8_t  s_pump_last_level = 1U;

/* Latest RPM computed by the task.  Read-only for GUI. */
static volatile uint32_t s_fan_rpm  = 0U;
static volatile uint32_t s_pump_rpm = 0U;

/*============================================================================
 *  Public API
 *============================================================================*/

void fan_tach_init(void)
{
    s_fan_pulse_count  = 0U;
    s_pump_pulse_count = 0U;
    s_fan_last_level   = 1U;
    s_pump_last_level  = 1U;
    s_fan_rpm          = 0U;
    s_pump_rpm         = 0U;

    /* PD4 / PD5 are already configured as GPIO_Input with PULLUP in ioc;
     * nothing else to do here. */
}

void fan_tach_start(void)
{
    HAL_TIM_Base_Stop_IT(&htim7);
    htim7.Instance->PSC = FAN_TACH_TIM_PRESCALER;
    htim7.Instance->ARR = FAN_TACH_TIM_PERIOD;
    __HAL_TIM_SET_COUNTER(&htim7, 0);
    __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
    HAL_TIM_Base_Start_IT(&htim7);
}

void fan_tach_stop(void)
{
    HAL_TIM_Base_Stop_IT(&htim7);
}

void fan_tach_tick(void)
{
    /* Read both TACH inputs as plain GPIO levels. */
    uint8_t fan_now  = (HAL_GPIO_ReadPin(FAN1_SENSE_GPIO_Port,
                                        FAN1_SENSE_Pin) == GPIO_PIN_SET) ? 1U : 0U;
    uint8_t pump_now = (HAL_GPIO_ReadPin(PUMP_SENSE_GPIO_Port,
                                        PUMP_SENSE_Pin) == GPIO_PIN_SET) ? 1U : 0U;

    /* Falling-edge detect: previous=1 and current=0 -> count++. */
    if (s_fan_last_level == 1U && fan_now == 0U) {
        ++s_fan_pulse_count;
    }
    if (s_pump_last_level == 1U && pump_now == 0U) {
        ++s_pump_pulse_count;
    }
    s_fan_last_level  = fan_now;
    s_pump_last_level = pump_now;

    /* Task side reads s_fan_pulse_count every 100 ms and computes RPM. */
}

uint32_t fan_tach_get_fan_rpm(void)  { return s_fan_rpm;  }
uint32_t fan_tach_get_pump_rpm(void) { return s_pump_rpm; }

/*============================================================================
 *  Task-side periodic RPM calculation (100 ms tick)
 *  Not in ISR; integer math, no FPU.
 *============================================================================*/
void fan_tach_calculate_rpm(void)
{
    /* Read pulse counts atomically by stopping the TIM7 interrupt
     * briefly.  Acceptable because 100 ms / 500 us = 200 ticks = ~1 ms
     * max gap; no fan-pulse is missed during that small window. */
    __disable_irq();
    uint32_t fan_count  = s_fan_pulse_count;
    uint32_t pump_count = s_pump_pulse_count;
    s_fan_pulse_count  = 0U;
    s_pump_pulse_count = 0U;
    __enable_irq();

    /* RPM = count * 60 / (poles * window_seconds)
     *      = count * 60 / (2 * 0.1)
     *      = count * 300  */
    s_fan_rpm  = fan_count  * 60U / (FAN_POLES * FAN_WINDOW_MS / 1000U);
    s_pump_rpm = pump_count * 60U / (FAN_POLES * FAN_WINDOW_MS / 1000U);
}