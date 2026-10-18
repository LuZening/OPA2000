/**
 * @file fan_tach.c
 * @brief Fan / pump TACH (RPM) detection - edge counting in the TIM7 ISR.
 *
 * @details Pulse counting:
 *   - fan_tach_tick() is called from TIM7_IRQHandler, which freq_detect.c
 *     has already programmed for a 100 us update tick.  fan_tach never
 *     touches PSC/ARR itself.
 *   - The ISR reads PD4 / PD5 and counts falling edges.  The pins are
 *     GPIO inputs with a pull-up, so the line idles high and each tach
 *     pulse pulls it low.
 *   - fan_tach_calculate_rpm() runs from the monitor task, snapshots the
 *     counters, and divides by the elapsed tick count measured with
 *     HAL_GetTick().  Using the measured window rather than a compile-time
 *     constant means the reading cannot silently go wrong when the
 *     caller's period changes.
 *
 *   RPM = pulses * 60 / (PPR * window_seconds), with PPR per channel -
 *   see FAN_POLES / PUMP_POLES in fan_tach.h.
 *
 * The ISR is integer-only (no FPU).
 */

#include "fan_tach.h"
#include "stm32h7xx_hal.h"

/* FAN_POLES / PUMP_POLES come from fan_tach.h - that is the single place
 * to edit them when a part's pulse-per-revolution figure is known. */

/* Sanity bounds on the measured window.  A window shorter than this
 * cannot resolve anything useful (at 12000 rpm / 2 PPR that is still
 * 40 pulses, but the division gets noisy), and a window longer than a
 * few seconds means the caller stalled - in both cases holding the last
 * reading is more honest than publishing a wild number. */
#define FAN_WINDOW_MIN_MS       200U
#define FAN_WINDOW_MAX_MS       5000U

/*============================================================================
 *  Private state (volatile for ISR/task sharing)
 *==========================================================================*/

/* Pulse count accumulated during the current window. */
static volatile uint32_t s_fan_pulse_count  = 0U;
static volatile uint32_t s_pump_pulse_count = 0U;

/* Last sampled GPIO level.  Active-low signal with a pull-up gives a
 * high idle and a low pulse, so we detect falling edges. */
static volatile uint8_t  s_fan_last_level  = 1U;
static volatile uint8_t  s_pump_last_level = 1U;

/* Latest RPM computed by the task.  Read-only for GUI / CAT. */
static volatile uint32_t s_fan_rpm  = 0U;
static volatile uint32_t s_pump_rpm = 0U;

/* HAL_GetTick() at the previous snapshot, for the measured window. */
static uint32_t s_last_calc_tick = 0U;
/* The first snapshot has no previous timestamp to measure against. */
static uint8_t  s_have_last_tick = 0U;

/*============================================================================
 *  Public API
 *==========================================================================*/

void fan_tach_init(void)
{
    s_fan_pulse_count  = 0U;
    s_pump_pulse_count = 0U;
    s_fan_last_level   = 1U;
    s_pump_last_level  = 1U;
    s_fan_rpm          = 0U;
    s_pump_rpm         = 0U;
    s_last_calc_tick   = HAL_GetTick();
    s_have_last_tick   = 1U;

    /* PD4 / PD5 are already configured as GPIO_Input with PULLUP in ioc;
     * nothing else to do here.  TIM7 belongs to freq_detect.c. */
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
}

uint32_t fan_tach_get_fan_rpm(void)  { return s_fan_rpm;  }
uint32_t fan_tach_get_pump_rpm(void) { return s_pump_rpm; }

uint32_t fan_tach_get_fan2_rpm(void)
{
#if FAN2_TACH_MIRRORS_FAN1
    /* No PWM channel and no TACH pin of its own - see fan_tach.h. */
    return s_fan_rpm;
#else
    return s_pump_rpm;
#endif
}

/*============================================================================
 *  Task-side periodic RPM calculation
 *  Not in ISR; integer math, no FPU.
 *============================================================================*/
void fan_tach_calculate_rpm(void)
{
    uint32_t now  = HAL_GetTick();
    uint32_t win_ms;

    /* Read pulse counts atomically by masking interrupts briefly.  This
     * is a handful of instructions, so no tach pulse is lost - and even
     * if one were, it would cost 30 RPM for one window. */
    __disable_irq();
    uint32_t fan_count  = s_fan_pulse_count;
    uint32_t pump_count = s_pump_pulse_count;
    s_fan_pulse_count  = 0U;
    s_pump_pulse_count = 0U;
    __enable_irq();

    if (s_have_last_tick == 0U) {
        /* First call after boot: consume the window, publish nothing. */
        s_last_calc_tick = now;
        s_have_last_tick = 1U;
        return;
    }

    win_ms = now - s_last_calc_tick;   /* unsigned, wraps correctly */
    s_last_calc_tick = now;

    if ((win_ms < FAN_WINDOW_MIN_MS) || (win_ms > FAN_WINDOW_MAX_MS)) {
        /* Window too short to mean anything, or the caller stalled.
         * Keep the previous reading rather than publish noise. */
        return;
    }

    /* RPM = count * 60 / (PPR * window_s)
     *      = count * 60 * 1000 / (PPR * window_ms)
     * The uint64_t keeps count * 60000 from overflowing if the window
     * is long and the fan is fast.  Each channel uses its own PPR - see
     * the FAN_POLES / PUMP_POLES note at the top of this file. */
    s_fan_rpm  = (uint32_t)(((uint64_t)fan_count  * 60000ULL) /
                            (FAN_POLES  * win_ms));
    s_pump_rpm = (uint32_t)(((uint64_t)pump_count * 60000ULL) /
                            (PUMP_POLES * win_ms));
}
