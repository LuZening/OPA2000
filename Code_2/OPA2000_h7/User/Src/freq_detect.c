/**
 * @file freq_detect.c
 * @brief Frequency counter + band mismatch detector implementation.
 *
 * @details TIM1 is configured (by CubeMX) in External Clock Mode 2 with
 *          ETR as clock source and prescaler = 31.  TIM7 update fires
 *          every 100 us and triggers freq_detect_isr() which:
 *            - reads TIM1->CNT
 *            - subtracts the previous count (with 16-bit wrap handling)
 *            - pushes the sample through a 3-deep sliding window
 *            - publishes g_radio_freq_hz once the window has a stable
 *              value (at least 2 of 3 samples agree within +/- 2 LSB)
 *            - if the window says the amp is transmitting on a
 *              different band than the operator asked for, calls
 *              protection_trigger_locked() with ALERT_FREQ_MISMATCH
 *
 *          Band-mismatch robustness (the lesson learned the hard way
 *          with the original single-sample version):
 *
 *            1. The ETR line carries a 1.8..30 MHz RF signal divided
 *               down by a /32 prescaler in the LO board.  The TIM1
 *               counter sees events at F_RF / 32; the 100 us sampling
 *               window (see FREQDET_TIM7_PERIOD) therefore has a
 *               +/- 1 LSB quantisation of 1 part in 10000 = 10 kHz per
 *               LSB.  Worst-case quantisation error on the displayed
 *               frequency is +/- 1 LSB (a single 10 kHz step), and
 *               since the loop can be off by half a step in either
 *               direction, we accept consecutive samples that agree
 *               within +/- 2 LSB (= 20 kHz).
 *
 *            2. The measurement therefore dithers around the real
 *               value by +/- 2 LSB.  When the radio is parked on a
 *               band edge (e.g. exactly 14.000000 MHz on the 20m/17m
 *               boundary) the displayed value crosses the boundary
 *               with each sample and the band classifier oscillates
 *               between BAND_20M_17M and BAND_15M_10M.  The single-
 *               sample detector would then trip ALERT_FREQ_MISMATCH
 *               once per boundary crossing - unacceptable.
 *
 *               Mitigation:  the band classifier used by the
 *               mismatch check widens each band interval by BAND_EPS_HZ
 *               on both sides (a guard band) so that a measured
 *               frequency sitting within +/- BAND_EPS_HZ of a band
 *               edge is treated as belonging to whichever band the
 *               radio is already on.  BAND_EPS_HZ must be larger
 *               than the +/- 2 LSB quantisation band (10 kHz * 2)
 *               - we set it to 500 kHz, which also gives ample
 *               margin against a slow PLL drift or a slightly out-
 *               of-tune exciter.
 *
 *               frequency_to_band() (the raw classifier used by the
 *               LPF-band auto-follow) is left untouched so the GUI
 *               and LPF logic keep their crisp edges; only the
 *               mismatch check uses the widened version.
 *
 *          The task-side wrapper freq_detect_check() runs in the
 *          Control task (10 ms tick) and updates the LPF band
 *          via switch_band() once the measured band has stabilised.
 */
#include <stdint.h>
#include "freq_detect.h"
#include "protection.h"
#include "power_control.h"
#include "stm32h7xx_hal.h"

/*============================================================================
 *  Private state (volatile for ISR/task sharing)
 *============================================================================*/

/* Latest published frequency in Hz - written by ISR, read by task.
 * Only updated once the sliding window agrees; otherwise holds the
 * last accepted value (or 0 right after boot). */
static volatile uint32_t s_measured_freq = 0U;

/* Latest raw sample, even if it has not been accepted by the
 * window yet.  Used by host tests and by debug. */
static volatile uint32_t s_raw_sample    = 0U;

/* TIM1 counter snapshot for delta computation in the ISR. */
static volatile uint16_t s_last_counter = 0U;
static volatile bool     s_first_check  = true;

/* 3-deep sliding window of raw samples (in TIM1-CNT LSB units, i.e.
 * pre-multiplication by 10000).  Reading and writing happens entirely
 * from the ISR context so no locking is required; the volatile is
 * only here to be defensive against the compiler optimising across
 * the ISR boundary. */
static volatile uint16_t s_window[3] = { 0U, 0U, 0U };
static volatile uint8_t  s_window_fill = 0U;  /*!< samples in window */

/* The accepted frequency in raw LSB units (kept next to the window so
 * a debug reader can see what made it through the filter). */
static volatile uint16_t s_accepted_lsb = 0U;

/* Consecutive ticks the sliding window REJECTED (spread > SPREAD_MAX).
 * Reset to 0 by every accept.  This is what tells a reader that
 * s_measured_freq is still a real measurement rather than a leftover:
 * once the input is unplugged every delta collapses to 0, and three
 * zeros have a spread of 0 - which the spread test happily ACCEPTS as
 * "0 Hz is perfectly consistent".  That accept would overwrite the last
 * good reading, so the reject counter alone would not catch it either.
 * What does catch it is the published value itself falling outside
 * [FREQDET_MIN_VALID_FREQ_HZ, FREQDET_MAX_VALID_FREQ_HZ); see
 * freq_detect_get_validity().  This counter covers the other half of
 * the problem - a reading that is in range but no longer being refreshed
 * because the input went noisy (spark, relay bounce, a partial
 * disconnect) and every window since has been discarded. */
static volatile uint16_t s_reject_run = 0U;

/* Band-mismatch latched flag (ISR sets, task reads+clears). */
static volatile bool     s_mismatch_latched = false;
static volatile bool     s_mismatch_alerted = false;

/*============================================================================
 *  TIM7 / TIM1 configuration
 *============================================================================*/

/* TIM7 tick = the sampling window of the frequency measurement.
 *
 * Clock: SYSCLK 250 MHz (HSE 25 / PLLM 2 * PLLN 40 / PLLP 2).  APB1 is
 * 125 MHz and TIM7 sits on APB1 with a prescaler, so the timer clock is
 * 2 x APB1 = 250 MHz.  Prescaler 19 -> /20 = 12.5 MHz.
 *
 * ARR 1249 gives 1250 ticks = 100 us.  This is the number the whole
 * conversion rests on: 1 TIM1 count in 100 us is 1 part in 10000 of the
 * divided-down ETR signal, i.e. 10 kHz per LSB, which is what
 * FREQDET_LSB_HZ below and every tolerance in this file are written
 * against.
 *
 * ARR 999 (which is what CubeMX put here) would be 1000 ticks = 80 us at
 * the real clock, and the published frequency would then come out 20% LOW
 * - 14.000 MHz would read as 11.2 MHz and every band decision would be
 * made on the wrong number. */
#define FREQDET_TIM7_PRESCALER    19U
#define FREQDET_TIM7_PERIOD       1249U

/*!< Hz represented by one TIM1 count in one sampling window.
 *
 *   window_hz = 250 MHz / ((ARR+1) * (PSC+1)) = 250e6 / (1250 * 20) = 10000
 *   One count inside that window is one whole window's worth of ETR
 *   cycles, i.e. exactly window_hz Hz.  With the ARR above this is
 *   10 kHz, which is what SPREAD_MAX_LSB and BAND_EPS_HZ are written
 *   against.  Deriving it (instead of writing 10000) means changing the
 *   window length cannot leave the conversion quietly stale - the test
 *   the_sampling_window_is_100us exists to keep this honest. */
#define FREQDET_LSB_HZ  (250000000UL                                   \
                       / ((FREQDET_TIM7_PERIOD + 1U) * (FREQDET_TIM7_PRESCALER + 1U)))

/* Sliding-window parameters.  The window holds 3 samples; we accept
 * the result when the *spread* (max - min across the filled slots) is
 * <= SPREAD_MAX_LSB.  With LSB = 10 kHz and SPREAD_MAX_LSB = 2 the
 * accepted frequency agrees with the true value within +/- 1 LSB. */
#define FREQDET_WINDOW_SIZE       3U
#define FREQDET_SPREAD_MAX_LSB    2U

/* Consecutive rejected windows that make the published frequency stale.
 *
 * With a 3-deep window and a spread tolerance of 2 LSB, one glitch (a
 * single sample landing 3+ LSB away) only shifts the window contents; the
 * next tick usually re-accepts.  Requiring a short run rather than a
 * single rejection keeps an ordinary transient from flickering the
 * display grey.
 *
 * At 100 us per tick, 5 consecutive rejects is 500 us of continuous
 * disagreement - far longer than any relay bounce or spark, and short
 * enough that a genuine signal re-validates almost immediately once it
 * comes back. */
#define FREQDET_STALE_REJECTS     5U

/* Band-edge guard band for the mismatch check.  See the file comment
 * for the rationale; raw frequency_to_band() does NOT use this. */
#define BAND_EPS_HZ               500000U   /*!< 500 kHz */

/*============================================================================
 *  Private helpers
 *============================================================================*/

/* Sort s_window[0..fill-1] in place using a 3-element insertion sort.
 * The window is tiny and the function is called from the ISR; an O(n^2)
 * sort is faster here than any binary-search or heap dance, and the
 * compiler keeps the whole thing in registers. */
static void sort_window(uint16_t *w, uint8_t n)
{
    for (uint8_t i = 1U; i < n; i++) {
        uint16_t key = w[i];
        uint8_t  j   = i;
        while (j > 0U && w[j - 1U] > key) {
            w[j] = w[j - 1U];
            j--;
        }
        w[j] = key;
    }
}

/* Map an accepted frequency (Hz) into a band, but with each band
 * widened by BAND_EPS_HZ on both sides.  The point is to absorb the
 * quantisation band (see the file comment) so a measurement parked on
 * a band edge does not bounce between two band classifiers. */
static BandType frequency_to_band_wide(uint32_t frequency_hz)
{
    /* The band table lives in opa2000.h.  We use signed offsets so
     * the comparison does not underflow when the lower band goes
     * below 0; the input is always > 0 in practice. */
    const uint32_t eps = BAND_EPS_HZ;

    /* Same precedence order as frequency_to_band(), but with each
     * MIN shifted down by eps and each MAX shifted up by eps. */
    if (frequency_hz + eps < BAND_160M_MIN)  return BAND_NONE;
    if (frequency_hz <= BAND_160M_MAX + eps) return BAND_160M;
    if (frequency_hz < BAND_80M_MIN - eps)   return BAND_FAULT;
    if (frequency_hz <= BAND_80M_MAX + eps)  return BAND_80M;
    if (frequency_hz < BAND_40M_30M_MIN - eps) return BAND_FAULT;
    if (frequency_hz <= BAND_40M_30M_MAX + eps) return BAND_40M_30M;
    if (frequency_hz < BAND_20M_17M_MIN - eps)  return BAND_FAULT;
    if (frequency_hz <= BAND_20M_17M_MAX + eps) return BAND_20M_17M;
    if (frequency_hz < BAND_15M_10M_MIN - eps)  return BAND_FAULT;
    if (frequency_hz <= BAND_15M_10M_MAX + eps) return BAND_15M_10M;
    return BAND_FAULT;
}

/*============================================================================
 *  Public API
 *============================================================================*/

void freq_detect_init(void)
{
    s_measured_freq      = 0U;
    s_raw_sample         = 0U;
    s_last_counter       = 0U;
    s_first_check        = true;
    s_window[0]          = 0U;
    s_window[1]          = 0U;
    s_window[2]          = 0U;
    s_window_fill        = 0U;
    s_accepted_lsb       = 0U;
    s_reject_run         = 0U;
    s_mismatch_latched   = false;
    s_mismatch_alerted   = false;

    /* Configure TIM1 ETR (already started by HAL_TIM_Base_Start in main.c
     * via CubeMX).  Just make sure the counter is reset. */
    __HAL_TIM_SET_COUNTER(&htim1, 0);
}

void freq_detect_start(void)
{
    /* Configure TIM7 100 us tick. */
    HAL_TIM_Base_Stop_IT(&htim7);
    htim7.Instance->PSC = FREQDET_TIM7_PRESCALER;
    htim7.Instance->ARR = FREQDET_TIM7_PERIOD;
    __HAL_TIM_SET_COUNTER(&htim7, 0);
    __HAL_TIM_CLEAR_FLAG(&htim7, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&htim7);

    /* Reset TIM1 ETR counter + flip the first-check flag and clear
     * the sliding window so we do not start mid-stream. */
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    s_first_check  = true;
    s_last_counter = 0U;
    s_window_fill  = 0U;
    s_accepted_lsb = 0U;
    s_reject_run   = 0U;
    s_measured_freq = 0U;
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

    /* Always publish the raw sample for debug / tests, even before the
     * window accepts it - the host test fixture injects a delta
     * directly and wants to read it back.
     *
     * The multiplier is derived from FREQDET_TIM7_PERIOD, so the two can
     * never drift apart: change the sampling window and the Hz-per-LSB
     * follows, instead of the frequency silently reading low. */
    s_raw_sample = (uint32_t)delta * FREQDET_LSB_HZ;

    /* ----------------------------------------------------------------
     * Sliding window:  push delta into s_window (FIFO).  Once we have
     * at least 2 samples and the spread is <= FREDQDET_SPREAD_MAX_LSB
     * (= 2 LSB), accept the median as the canonical frequency.
     *
     * Median of 3 is robust against one outlier (relay bounce, EMI
     * pulse) - if one of the three differs from the other two by
     * more than 2 LSB the spread is > 2 LSB and we discard the whole
     * window for that tick; a fresh sample fills the slot.
     * ---------------------------------------------------------------- */
    if (s_window_fill < FREQDET_WINDOW_SIZE) {
        s_window[s_window_fill++] = delta;
    } else {
        s_window[0] = s_window[1];
        s_window[1] = s_window[2];
        s_window[2] = delta;
    }

    if (s_window_fill >= 2U) {
        uint16_t sorted[FREQDET_WINDOW_SIZE];
        uint8_t  n = (s_window_fill <= FREQDET_WINDOW_SIZE)
                     ? s_window_fill : FREQDET_WINDOW_SIZE;
        for (uint8_t i = 0U; i < n; i++) {
            sorted[i] = s_window[i];
        }
        sort_window(sorted, n);
        uint16_t spread = (uint16_t)(sorted[n - 1U] - sorted[0U]);
        if (spread <= FREQDET_SPREAD_MAX_LSB) {
            /* Median.  For n == 2 we take the lower (== average within
             * 2 LSB); for n == 3 we take the middle element. */
            uint16_t median = (n == 2U) ? sorted[0U]
                                        : sorted[1U];
            s_accepted_lsb = median;
            uint32_t freq  = (uint32_t)median * FREQDET_LSB_HZ;
            s_measured_freq = freq;
            g_radio_freq_hz = freq;
            s_reject_run = 0U;
        }
        /* else: spread too big, keep the previous s_measured_freq.
         * The LSB delta is +/- 1; a spread of 3+ LSB is a real glitch
         * (relay bounce, spark, etc.) and we wait for it to settle.
         *
         * Reject is a *pure* window-maintenance step:  it does not
         * publish a frequency, does not advance any protection
         * state, and does NOT participate in band-mismatch
         * detection.  The mismatch check below is gated on a
         * successful accept by tracking whether s_measured_freq
         * changed this tick (see s_freq_published_this_tick).
         *
         * The run counter saturates rather than wrapping: a reader only
         * ever asks "has it been stale for a while", and wrapping back to
         * 0 would briefly make a long-dead input look fresh again. */
        else {
            if (s_reject_run < 0xFFFFU) {
                s_reject_run++;
            }
            return;
        }
    }
    else {
        /* First 1-2 samples after start - no published frequency yet,
         * no band-mismatch decision can be made. */
        return;
    }

    /* ----------------------------------------------------------------
     * Band-mismatch check.  Reached only when this tick published a
     * new frequency (window accepted the median).  Guarded by:
     *
     *  1. Sample is in the valid RF range (1 MHz .. 60 MHz).  Below
     *     1 MHz means "no signal" or "noise" - not a band we are
     *     supposed to be on.  Above 60 MHz means a harmonic / spur /
     *     out-of-band spur; we do not want to alert on those either
     *     because frequency_to_band() returns BAND_FAULT for them
     *     and the only action would be a false-positive shutdown.
     *     This is what keeps idle RFI out: the counter has to see a
     *     real, stable carrier in a real ham band before it can
     *     disagree with the selected band.
     *  2. The latch is not already set, so we do not pile up
     *     protection_trigger_locked() calls while the operator has
     *     not yet dismissed.
     *
     * NOT gated on the keying state.  A frequency that is measurably
     * wrong for the selected band means the LPF will not pass what
     * the radio is actually transmitting, and that is equally wrong
     * whether the operator has keyed or not: a receiver keyed into
     * the wrong passband, a mis-wired LPF, or an RF source feeding
     * the counter that is not the radio at all are all faults the
     * operator needs told about.  The valid-frequency window above
     * is what distinguishes "a real signal on a real band" from
     * "noise on the counter input", and that check does not depend
     * on the amplifier's state.
     *
     * The band classification uses the wide-edge version
     * (frequency_to_band_wide) so a measurement sitting in the
     * quantisation band right on top of a boundary does not bounce.
     * ---------------------------------------------------------------- */
    if (!s_mismatch_latched &&
        s_measured_freq >= FREQDET_MIN_VALID_FREQ_HZ &&
        s_measured_freq <  FREQDET_MAX_VALID_FREQ_HZ) {
        BandType measured = frequency_to_band_wide(s_measured_freq);
        if (measured != BAND_NONE &&
            measured != BAND_FAULT &&
            measured != g_current_band) {
            s_mismatch_latched = true;
            protection_note_source(PROT_SRC_FREQ);
            protection_trigger_locked(ALERT_FREQ_MISMATCH);
        }
    }
}

void freq_detect_check(void)
{
    /* LPF band follow: use the *raw* (non-widened) classifier so the
     * LPF switches crisply when the radio leaves a band; the
     * quantisation oscillation lives only in the mismatch path. */
    BandType measured = frequency_to_band(s_measured_freq);
    if (measured != BAND_NONE && measured != BAND_FAULT &&
        measured != g_current_band) {
        switch_band(measured);
    }

    /* Clear the latch once the conditions that set it are gone: the
     * measured frequency fell out of the valid window (carrier
     * dropped / LO lost / counter back to noise), or the radio moved
     * back onto the selected band.
     *
     * The keying state deliberately plays no part here.  It used to
     * ("carrier dropped -> clear"), which was correct when the
     * mismatch check only ran while keyed; now that the check runs
     * unconditionally, keying is not what set the latch and so it
     * cannot be what clears it either.  Releasing the latch also
     * requires the operator to dismiss the alert - this only re-arms
     * the detector, it does not clear a standing alert. */
    if (s_mismatch_latched) {
        if (s_measured_freq < FREQDET_MIN_VALID_FREQ_HZ ||
            s_measured_freq >= FREQDET_MAX_VALID_FREQ_HZ ||
            frequency_to_band_wide(s_measured_freq) == g_current_band) {
            s_mismatch_latched = false;
            s_mismatch_alerted = false;
        }
    }
}

uint32_t freq_detect_get_frequency(void) { return s_measured_freq; }

/* Is s_measured_freq a live measurement right now?
 *
 * Three conditions, all of which a plain range check would miss:
 *
 *  1. In range.  Outside [FREQDET_MIN_VALID_FREQ_HZ,
 *     FREQDET_MAX_VALID_FREQ_HZ) it is not a ham-band frequency at all -
 *     the input is floating, or the counter is seeing its own noise.
 *
 *  2. The window has not been rejecting.  s_measured_freq deliberately
 *     holds its last value when the sliding window discards a sample, so
 *     a reading can be arbitrarily old and still look perfect to a range
 *     check.  A run of rejections means the input went noisy - spark,
 *     relay bounce, a half-open cable - and the number on screen is a
 *     leftover, not a measurement.
 *
 *  3. Enough samples have ever landed.  Right after start() the window
 *     holds fewer than FREQDET_WINDOW_SIZE entries and nothing has been
 *     accepted yet.
 *
 * Lives here rather than in the GUI because the thresholds and the window
 * parameters are both private to this file: a caller re-deriving the rule
 * would silently disagree the moment either changed. */
bool freq_detect_is_measurement_valid(void)
{
    if (s_measured_freq < FREQDET_MIN_VALID_FREQ_HZ ||
        s_measured_freq >= FREQDET_MAX_VALID_FREQ_HZ) {
        return false;
    }
    if (s_reject_run >= FREQDET_STALE_REJECTS) {
        return false;
    }
    return s_window_fill >= 2U;
}

/* HOST_TEST_NATIVE hook: read the raw last sample even when the sliding
 * window has not accepted it yet.  Useful for the unit tests that
 * inject a single delta and want to assert what landed in the window. */
#if defined(HOST_TEST_NATIVE)
uint16_t freq_detect_test_window(uint8_t idx)
{
    return (idx < FREQDET_WINDOW_SIZE) ? s_window[idx] : 0U;
}
uint8_t  freq_detect_test_window_fill(void) { return s_window_fill; }
uint16_t freq_detect_test_accepted_lsb(void) { return s_accepted_lsb; }
#endif