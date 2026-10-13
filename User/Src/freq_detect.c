/**
 * @file freq_detect.c
 * @brief Frequency Detector Implementation
 *
 * TIM4 counts external pulses on FREQDET_IN (PE0) via ETR.
 * TIM7 fires every 100us - in its ISR we read TIM4 counter,
 * calculate frequency, and compare against expected band.
 * If mismatch is detected with a valid signal (>1MHz), protection
 * is triggered immediately (PTT cut, fault output).
 */

#include "freq_detect.h"
#include "protection.h"
#include "comm.h"
#include "power_control.h"
#include "stm32h7xx_hal.h"

/*============================================================================
 * Private Variables
 *============================================================================*/

/* Current frequency measurement (updated by ISR, read by task) */
static volatile uint32_t measured_frequency = 0;
static volatile BandType detected_band = BAND_NONE;

/* TIM4 counter value from last ISR tick */
static uint16_t last_counter = 0;
static bool first_check = true;

/* Frequency mismatch flag - set by ISR, cleared by task */
static volatile bool freq_mismatch_detected = false;

/* Alert already shown (prevent repeated popups) */
static bool freq_mismatch_alert_shown = false;

/*============================================================================
 * Public Functions
 *============================================================================*/

/**
 * @brief Initialize frequency detector
 */
void freq_detect_init(void)
{
    measured_frequency = 0;
    detected_band = BAND_NONE;
    first_check = true;
    freq_mismatch_detected = false;
    freq_mismatch_alert_shown = false;
}

/**
 * @brief Start frequency detection
 *
 * Starts TIM4 ETR counter and TIM7 100us interval timer.
 */
void freq_detect_start(void)
{
    /* Reset TIM4 ETR counter */
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    HAL_TIM_Base_Start(&htim4);

    /* Configure TIM7 for 100us period */
    HAL_TIM_Base_Stop(&htim7);
    __HAL_TIM_SET_PRESCALER(&htim7, FREQDET_TIM7_PRESCALER);
    __HAL_TIM_SET_AUTORELOAD(&htim7, FREQDET_TIM7_PERIOD);
    __HAL_TIM_SET_COUNTER(&htim7, 0);

    /* Clear pending interrupt flag before starting */
    __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);

    /* Start TIM7 with interrupt enabled */
    HAL_TIM_Base_Start_IT(&htim7);

    first_check = true;
    freq_mismatch_detected = false;
    freq_mismatch_alert_shown = false;
}

/**
 * @brief Stop frequency detection
 */
void freq_detect_stop(void)
{
    HAL_TIM_Base_Stop_IT(&htim7);
    HAL_TIM_Base_Stop(&htim4);
    freq_mismatch_alert_shown = false;
}

/**
 * @brief Frequency detection ISR - called from TIM7 IRQ every 100us
 *
 * Reads TIM4 ETR counter, calculates frequency, compares against
 * expected band. If mismatch with a valid signal (>1MHz), triggers
 * protection immediately via direct GPIO writes.
 *
 * @note ISR context - NO FreeRTOS APIs, NO HAL function calls that
 *       might use OS primitives.
 */
void freq_detect_isr(void)
{
    uint16_t current_counter;
    uint16_t count_diff;
    uint32_t freq_hz;
    BandType expected_band;

    /* Read TIM4 ETR counter */
    current_counter = __HAL_TIM_GET_COUNTER(&htim4);

    if (first_check) {
        last_counter = current_counter;
        first_check = false;
        return;
    }

    /* Calculate difference (handle 16-bit wrap-around) */
    if (current_counter >= last_counter) {
        count_diff = current_counter - last_counter;
    } else {
        count_diff = (0xFFFF - last_counter) + current_counter;
    }
    last_counter = current_counter;

    /* Convert to frequency: count per 100us * 10000 = Hz */
    freq_hz = (uint32_t)count_diff * FREQDET_MULTIPLIER;
    measured_frequency = freq_hz;

    /* Only check band mismatch if we have a valid signal (>1MHz) */
    if (freq_hz > FREQDET_MIN_VALID_FREQ_HZ && g_power_state == POWER_STATE_TX) {
        detected_band = frequency_to_band(freq_hz);
        expected_band = frequency_to_band(comm_get_radio_frequency());

        if (detected_band != expected_band && detected_band != BAND_NONE
            && expected_band != BAND_NONE && !freq_mismatch_detected) {
            /* Mismatch detected with valid signal - trigger protection */
            freq_mismatch_detected = true;
            protection_freq_mismatch();
        }
    }
}

/**
 * @brief Task-level frequency check
 *
 * Handles GUI alert state and band updates.
 * Called from protect task (not timing-critical).
 */
void freq_detect_check(void)
{
    /* Update band from ISR-measured frequency */
    BandType fb = frequency_to_band(measured_frequency);

    if (g_current_band != fb && fb != BAND_NONE && fb != BAND_FAULT) {
        power_set_band(fb);
    }

    /* Handle mismatch alert - show once, clear when bands re-align */
    if (freq_mismatch_detected) {
        /* Re-check current status */
        BandType expected = frequency_to_band(comm_get_radio_frequency());
        BandType actual = frequency_to_band(measured_frequency);

        if (actual == expected || actual == BAND_NONE || actual == BAND_FAULT
            || measured_frequency < FREQDET_MIN_VALID_FREQ_HZ) {
            /* Condition cleared */
            freq_mismatch_detected = false;
            freq_mismatch_alert_shown = false;
        } else if (!freq_mismatch_alert_shown) {
            freq_mismatch_alert_shown = true;
            /* Alert flag already set by ISR, GUI task will show it */
        }
    }
}

/**
 * @brief Get measured frequency (read by task)
 */
uint32_t freq_detect_get_frequency(void)
{
    return measured_frequency;
}

/**
 * @brief Get detected band
 */
BandType freq_detect_get_band(void)
{
    return detected_band;
}
