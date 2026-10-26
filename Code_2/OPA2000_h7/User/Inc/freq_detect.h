/**
 * @file freq_detect.h
 * @brief Frequency counter (TIM1 ETR) and band-mismatch detector.
 *
 * @details TIM1 is configured in External Clock Mode 2 (ETR as clock
 *          source, prescaler = 31 from hw_pinmap.h).  The 16-bit
 *          counter increments on every rising edge of PE7
 *          (FREQDET_TIM7_PRESCALER divider skipped -- TIM1 clocked
 *          directly from APB2).
 *
 *          TIM7 update ISR fires every 100 us and reads the counter.
 *          Frequency (Hz) = count * 10000 (1 s / 100 us).
 *
 *          When the measured band differs from g_current_band and the
 *          signal is above 1 MHz, protection_trigger_locked() is
 *          called immediately (ISR-safe; no FreeRTOS API).
 */

#ifndef FREQ_DETECT_H
#define FREQ_DETECT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "opa2000.h"
#include "hw_pinmap.h"

/* HAL timer handles (defined in main.c) */
extern TIM_HandleTypeDef htim1;   /*!< TIM1 ETR counter (PE7)  */
extern TIM_HandleTypeDef htim7;   /*!< TIM7 100 us tick         */

/*============================================================================
 *  Public API
 *============================================================================*/

/* Initialise TIM7 100 us period + arm TIM1 ETR counter. */
void freq_detect_init(void);
/* Start the 100 us sampling (TIM7 interrupt). */
void freq_detect_start(void);
/* Stop the sampling. */
void freq_detect_stop(void);

/* TIM7 update ISR - reads TIM1 counter, updates g_radio_freq_hz and
 * calls protection_trigger_locked() on band mismatch.  ISR context,
 * no FreeRTOS API. */
void freq_detect_isr(void);

/* Task-side wrapper: updates LPF band selection from the latest
 * measurement and clears the alert once bands re-align. */
void freq_detect_check(void);

/* Latest measured frequency in Hz.  May be 0 if no signal. */
uint32_t freq_detect_get_frequency(void);

/* Whether that frequency is a live measurement right now, as opposed to a
 * value the counter is still holding from before the input went quiet or
 * noisy.  Checks the ham-band range AND that the sliding window has not
 * been rejecting for a while AND that enough samples have arrived since
 * start().  A plain range test is not enough: s_measured_freq is only
 * overwritten on a successful accept, so a dead input keeps reporting the
 * last real value indefinitely.  Callers displaying the number should use
 * this and show it as stale rather than live. */
bool freq_detect_is_measurement_valid(void);

#ifdef HOST_TEST_NATIVE
/* Test hooks for the sliding window.  idx in 0..2. */
uint16_t freq_detect_test_window(uint8_t idx);
uint8_t  freq_detect_test_window_fill(void);
uint16_t freq_detect_test_accepted_lsb(void);

/* The sampling window the frequency conversion rests on, mirrored from
 * freq_detect.c.  Exposed so a test can assert the relationship between
 * the window and the Hz-per-LSB factor instead of taking it on trust -
 * getting this wrong makes every published frequency low by the same
 * ratio, with nothing else to notice it. */
#define FREQDET_TEST_TIM7_PRESCALER    19U
#define FREQDET_TEST_TIM7_PERIOD       1249U   /* 1250 ticks = 100 us */
#endif

#ifdef __cplusplus
}
#endif

#endif /* FREQ_DETECT_H */