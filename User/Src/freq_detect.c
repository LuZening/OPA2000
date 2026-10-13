/**
 * @file freq_detect.c
 * @brief Frequency counter + band mismatch detector implementation.
 *
 * @details TIM1 is configured (by CubeMX) in External Clock Mode 2 with
 *          ETR as clock source and prescaler = 31.  TIM7 update fires
 *          every 100 us and triggers freq_detect_isr() which:
 *            - reads TIM1->CNT
 *            - subtracts the previous count (with 16-bit wrap handling)
 *            - publishes g_radio_freq_hz = delta * 10000
 *            - if freq > 1 MHz and measured band != g_current_band and
 *              g_power_state == TX, calls protection_trigger_locked()
 *              with ALERT_FREQ_MISMATCH.
 *
 *          The task-side wrapper freq_detect_check() runs in the
 *          Control task (10 ms tick) and updates the LPF band
 *          via switch_band() once the measured band has stabilised.
 */

#include "freq_detect.h"
#include "protection.h"
#include "power_control.h"
#include "stm32h7xx_hal.h"

/*============================================================================
 *  Private state (volatile for ISR/task sharing)
 *============================================================================*/

/* Latest measured frequency in Hz - written by ISR, read by task */
static volatile uint32_t s_measured_freq = 0U;

/* TIM1 counter snapshot for delta computation in the ISR. */
static volatile uint16_t s_last_counter = 0U;
static volatile bool     s_first_check  = true;

/* Band-mismatch latched flag (ISR sets, task reads+clears). */
static volatile bool     s_mismatch_latched = false;
static volatile bool     s_mismatch_alerted = false;

/*============================================================================
 *  TIM7 / TIM1 configuration
 *============================================================================*/

/* TIM7: APB1 timer clock = 200 MHz, prescaler = 19 -> 10 MHz, period 999
 * = 100 us.  Same configuration as old code; kept here for clarity. */
#define FREQDET_TIM7_PRESCALER    19U
#define FREQDET_TIM7_PERIOD       999U

/* TIM1 prescaler set in hw_pinmap.h (#define ETR_FREQDET_PRESCALER 31). */

/*============================================================================
 *  Public API
 *============================================================================*/

void freq_detect_init(void)
{
    s_measured_freq      = 0U;
    s_last_counter       = 0U;
    s_first_check        = true;
    s_mismatch_latched   = false;
    s_mismatch_alerted   = false;

    /* Configure TIM1 ETR (already started by HAL_TIM_Base_Start in main.c
     * via CubeMX).  Just make sure the counter is reset. */
    __HAL_TIM_SET_COUNTER(&htim1, 0);
}

void freq_detect_start(void)
{
    /* Configure TIM7 100 us tick.  Same pattern as legacy code. */
    HAL_TIM_Base_Stop_IT(&htim7);
    htim7.Instance->PSC = FREQDET_TIM7_PRESCALER;
    htim7.Instance->ARR = FREQDET_TIM7_PERIOD;
    __HAL_TIM_SET_COUNTER(&htim7, 0);
    __HAL_TIM_CLEAR_FLAG(&htim7, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&htim7);

    /* Reset TIM1 ETR counter + flip the first-check flag. */
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    s_first_check  = true;
    s_last_counter = 0U;
}

void freq_detect_stop(void)
{
    HAL_TIM_Base_Stop_IT(&htim7);
    s_mismatch_alerted = false;
}

void freq_detect_isr(void)
{
    uint16_t cur = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);

    if (s_first_check) {
        s_last_counter = cur;
        s_first_check  = false;
        return;
    }

    /* 16-bit counter wrap handling. */
    uint16_t delta = (cur >= s_last_counter)
                     ? (uint16_t)(cur - s_last_counter)
                     : (uint16_t)((0xFFFFU - s_last_counter) + cur + 1U);
    s_last_counter = cur;

    /* freq = delta / 100us = delta * 10000 Hz */
    uint32_t freq = (uint32_t)delta * 10000U;
    s_measured_freq = freq;
    g_radio_freq_hz = freq;

    /* Band-mismatch check only valid when we have a real signal and
     * the amp is currently transmitting. */
    if (freq >= FREQDET_MIN_VALID_FREQ_HZ &&
        g_power_state == POWER_STATE_TX &&
        !s_mismatch_latched) {
        BandType measured = frequency_to_band(freq);
        if (measured != BAND_NONE &&
            measured != BAND_FAULT &&
            measured != g_current_band) {
            s_mismatch_latched = true;
            protection_trigger_locked(ALERT_FREQ_MISMATCH);
        }
    }
}

void freq_detect_check(void)
{
    /* LPF band follow: switch band when measured band differs. */
    BandType measured = frequency_to_band(s_measured_freq);
    if (measured != BAND_NONE && measured != BAND_FAULT &&
        measured != g_current_band) {
        switch_band(measured);
    }

    /* Alert state machine: clear the latch once conditions normalise. */
    if (s_mismatch_latched) {
        if (s_measured_freq < FREQDET_MIN_VALID_FREQ_HZ ||
            g_power_state != POWER_STATE_TX) {
            /* Signal gone or no longer transmitting - clear latch. */
            s_mismatch_latched = false;
            s_mismatch_alerted = false;
        }
    }
}

uint32_t freq_detect_get_frequency(void) { return s_measured_freq; }