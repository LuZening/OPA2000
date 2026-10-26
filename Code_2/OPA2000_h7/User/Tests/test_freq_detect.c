/**
 * @file    test_freq_detect.c
 * @brief   Unit tests for freq_detect.c: the 100 us frequency sampler,
 *          the 3-deep sliding window and the band-mismatch check with
 *          its 500 kHz guard band.
 *
 * @details Tests cover:
 *
 *   - Window warm-up: 2 samples agree -> frequency published
 *   - Spread > 2 LSB (relay bounce) -> frequency NOT updated
 *   - Median of 3 with one outlier -> outlier ignored
 *   - Band mismatch on different band during TX -> ALERT_FREQ_MISMATCH
 *   - Band mismatch on the *same* band during TX -> no alert
 *   - Band mismatch while IDLE -> also alerts (no keying gate); idle
 *     and correctly on band -> stays quiet, because the valid-frequency
 *     window and the 3-sample agreement are what reject noise, not the
 *     keying state
 *   - Below 1 MHz / above 60 MHz -> no alert
 *   - Quantisation on a band edge: 14.000 MHz sits between 20m/17m and
 *     15m/10m - the guard band keeps the wide classifier on whichever
 *     band the radio is already on, so no alert
 *   - Rejected samples do NOT participate in band-mismatch detection:
 *     a stray ETR pulse that the window discards must not flip the
 *     latch into ALERT_FREQ_MISMATCH
 *   - Latch clears once the radio returns to the selected band or the
 *     frequency leaves the valid window - and NOT merely because the
 *     carrier dropped, since unkeying says nothing about which band
 *     the radio is on
 *
 * @note    The tests drive TIM1->CNT directly to simulate the ETR
 *          divider feeding the counter; the 100 us TIM7 tick is fired
 *          by calling freq_detect_isr() directly because the host
 *          build does not run the real TIM7 ISR.
 */

#include "test_harness.h"
#include "opa2000.h"
#include "freq_detect.h"
#include "protection.h"
#include "power_control.h"
#include "stm32h7xx_hal_stub.h"

#include <stddef.h>

/* Test hooks declared in freq_detect.h under HOST_TEST_NATIVE. */
extern uint16_t freq_detect_test_window(uint8_t idx);
extern uint8_t  freq_detect_test_window_fill(void);
extern uint16_t freq_detect_test_accepted_lsb(void);

/* Recorded by comm_host_stub.c. */
extern unsigned int       comm_stub_event_count;
extern volatile AlertSeverity comm_stub_last_severity;

/*============================================================================
 *  Helpers
 *==========================================================================*/

/*============================================================================
 *  The sampling window IS the conversion factor
 *
 *  One TIM1 count in the sampling window equals F_RF/32 divided by the
 *  window length.  Everything published downstream - the Hz readout, the
 *  2 LSB spread tolerance, the 500 kHz band guard - is expressed against
 *  that one number.  SYSCLK is 250 MHz, so the ARR that yields 100 us is
 *  1249, not the 999 that a "200 MHz" assumption produces (80 us).
 *
 *  With the wrong window the published frequency is low by exactly the
 *  same factor: at ARR 999 a 14.000 MHz carrier reads 14.0 * 100/125 =
 *  11.2 MHz, and every band decision is then made on a number that is not
 *  the frequency.  The cases at the bottom of this file pin the
 *  relationship so a future clock-tree change cannot silently reintroduce
 *  it.
 *==========================================================================*/

/* True once fd_pump_ticks() has run its priming tick.  Reset by
 * fd_setup(), because freq_detect_start() re-arms s_first_check and the
 * next pump needs the priming tick again. */
static bool s_primed = false;

/* Reset freq_detect.c internals and the alert latch.  Mirrors the
 * boot-time sequence the firmware runs. */
static void fd_setup(void)
{
    freq_detect_init();
    freq_detect_start();
    s_primed = false;

    /* Clear any latch left by an earlier test.  protection_init(), not
     * protection_clear(): a dismiss deliberately holds the same cause off
     * for PROTECTION_REARM_HOLD_MS so the popup really goes away, and this
     * helper wants a guaranteed-clean slate so the case under test can
     * raise the very same fault again immediately. */
    protection_init();
    protection_clear();

    /* The master protection switch defaults to 0 in a zeroed g_config, and
     * protection_trigger_locked() returns immediately when it is clear -
     * so the mismatch alert would never be raised.  Arm it, exactly as
     * config_init() does on the real firmware. */
    g_config.allow_protection = 1U;

    /* The mismatch check requires the radio to be on a band, so seed
     * the operator-side band selection. */
    g_current_band  = BAND_20M_17M;
    g_tx_active     = false;
    g_alert_active  = ALERT_NONE;
    g_alert_flag    = false;
    g_radio_freq_hz = 0U;
}

/* Pump the 100 us sampler N times, with TIM1->CNT walking forward by
 * `delta` each tick.  This bypasses the TIM7 ISR entirely - the test
 * calls freq_detect_isr() directly, the same function the real ISR
 * would call.
 *
 * The first call to freq_detect_isr() after freq_detect_start() always
 * returns early (it just captures s_last_counter so the next delta is
 * well-defined).  `fd_pump_ticks()` accounts for that by feeding one
 * extra "priming" tick at the start so the caller-specified N
 * corresponds to N samples landing in the sliding window. */
static void fd_pump_ticks(uint16_t delta, uint32_t n)
{
    /* Priming tick: drive CNT to a known place so the next ISR call
     * has a delta to compute.  freq_detect_isr() will set s_last_counter
     * and return without touching the window.
     *
     * It is needed ONCE, not once per call.  freq_detect_start() resets
     * s_first_check, so the very first ISR call after it swallows the
     * sample and just latches the counter.  On every LATER call the
     * window is already full, and resetting CNT to 0 makes the delta
     * wrap around to ~65000 - a garbage sample that the window then
     * rejects, so the counter under test was measuring the harness
     * instead of the code.  From the second pump on, the counter is
     * already where the previous pump left it and no priming is
     * needed. */
    if (!s_primed) {
        htim1.Instance->CNT = 0U;
        freq_detect_isr();
        s_primed = true;
    }

    for (uint32_t i = 0U; i < n; i++) {
        htim1.Instance->CNT = (uint16_t)(htim1.Instance->CNT + delta);
        freq_detect_isr();
    }
}

/*============================================================================
 *  The sampling window IS the conversion factor
 *
 *  These pin the relationship between the TIM7 reload value and the
 *  Hz-per-LSB factor the whole module rests on, so a future clock-tree
 *  change (or a CubeMX regeneration putting ARR 999 back) cannot silently
 *  reintroduce a frequency that is low by the window error.
 *==========================================================================*/

TEST_CASE(the_sampling_window_is_100us)
{
    /* TIM7 counts at 250 MHz / (PSC+1); one window is (ARR+1) of those. */
    uint32_t ticks_per_window = (uint32_t)(FREQDET_TEST_TIM7_PERIOD + 1U)
                                * (FREQDET_TEST_TIM7_PRESCALER + 1U);
    uint32_t window_hz = 250000000UL / ticks_per_window;
    ASSERT_EQ(window_hz, 10000UL);   /* 100 us */
}

TEST_CASE(one_count_is_ten_kHz_and_the_readout_agrees)
{
    fd_setup();
    /* 1000 counts per 100 us window -> 1000 * 10 kHz = 10 MHz. */
    fd_pump_ticks(1000U, 3U);
    ASSERT_EQ(g_radio_freq_hz, 10000000UL);
    ASSERT_EQ(freq_detect_test_accepted_lsb(), 1000U);
}

TEST_CASE(the_published_frequency_is_not_low_by_the_window_error)
{
    fd_setup();
    /* 10000 counts -> 100 MHz.  At the old ARR 999 the window was 80 us,
     * so this same carrier published as 80 MHz. */
    fd_pump_ticks(10000U, 3U);
    ASSERT_EQ(g_radio_freq_hz, 100000000UL);
}

/*============================================================================
 *  Sliding-window tests
 *============================================================================*/

TEST_CASE(window_publishes_after_two_agreeing_samples)
{
    fd_setup();
    /* Two 14 MHz ticks - delta = 1400000/10000 = 140. */
    fd_pump_ticks(140U, 2U);
    ASSERT_EQ(freq_detect_get_frequency(), 1400000U);
    ASSERT_EQ(freq_detect_test_window_fill(), 2U);
    ASSERT_EQ(freq_detect_test_accepted_lsb(), 140U);
}

TEST_CASE(window_rejects_when_spread_exceeds_2_lsb)
{
    fd_setup();
    fd_pump_ticks(140U, 5U);          /* agree on 140 */
    ASSERT_EQ(freq_detect_get_frequency(), 1400000U);

    /* Now push 142 (would-be +20 kHz).  Spread = 142 - 140 = 2 -> still OK
     * (we accept up to 2 LSB inclusive).  Median of [140, 140, 142]
     * is 140 (the lower of the two differing samples), not 141. */
    fd_pump_ticks(142U, 1U);
    ASSERT_EQ(freq_detect_get_frequency(), 1400000U);

    /* Push 150 - window becomes [140, 142, 150], spread = 10 -> reject,
     * frequency stays at the previous accepted value. */
    fd_pump_ticks(150U, 1U);
    ASSERT_EQ(freq_detect_get_frequency(), 1400000U);
}

TEST_CASE(window_recovers_when_signal_stabilises_again)
{
    fd_setup();
    fd_pump_ticks(140U, 3U);
    ASSERT_EQ(freq_detect_get_frequency(), 1400000U);

    /* Inject one stray sample: spread = 5 -> reject. */
    fd_pump_ticks(145U, 1U);
    ASSERT_EQ(freq_detect_get_frequency(), 1400000U);

    /* Settle back to 140: window is now [140, 140, 145]; replace the
     * 145 slot one at a time with 140. */
    fd_pump_ticks(140U, 1U);          /* window: [140, 145, 140], median 140, spread 5 */
    ASSERT_EQ(freq_detect_get_frequency(), 1400000U);

    fd_pump_ticks(140U, 1U);          /* window: [145, 140, 140], median 140, spread 5 */
    ASSERT_EQ(freq_detect_get_frequency(), 1400000U);

    fd_pump_ticks(140U, 1U);          /* window: [140, 140, 140], spread 0 -> accept */
    ASSERT_EQ(freq_detect_get_frequency(), 1400000U);
}

/*============================================================================
 *  Validity reporting - what freq_detect_is_measurement_valid() answers
 *
 *  The distinction these cover is the one a range check cannot make.
 *  s_measured_freq is only overwritten on a successful window accept, so
 *  a dead or noisy input keeps returning the last real value forever.  A
 *  display that trusted the number alone would paint a stale frequency
 *  green, telling the operator the radio is still on the air.
 *==========================================================================*/

TEST_CASE(a_healthy_measurement_reports_valid)
{
    fd_setup();
    fd_pump_ticks(140U, 4U);
    ASSERT_EQ(freq_detect_get_frequency(), 1400000U);
    ASSERT_TRUE(freq_detect_is_measurement_valid());
}

TEST_CASE(nothing_measured_yet_is_not_valid)
{
    fd_setup();
    /* No ticks at all: the window is empty and nothing has been accepted. */
    ASSERT_EQ(freq_detect_get_frequency(), 0U);
    ASSERT_FALSE(freq_detect_is_measurement_valid());
}

TEST_CASE(a_single_sample_is_not_enough_to_be_valid)
{
    fd_setup();
    fd_pump_ticks(140U, 1U);          /* one sample, window holds 1 */
    /* The frequency is published (two agreeing slots is the accept rule),
     * but the caller has no confirmation the input is stable, so the
     * reading must not be presented as live. */
    ASSERT_FALSE(freq_detect_is_measurement_valid());
}

TEST_CASE(a_dead_input_stops_being_reported_valid)
{
    fd_setup();
    fd_pump_ticks(140U, 5U);
    ASSERT_TRUE(freq_detect_is_measurement_valid());

    /* Unplug: no edges at all, so every delta is 0.  Three zeros have a
     * spread of 0, which the window ACCEPTS - so the published value
     * collapses to 0 and the range check catches it. */
    fd_pump_ticks(0U, 5U);
    ASSERT_EQ(freq_detect_get_frequency(), 0U);
    ASSERT_FALSE(freq_detect_is_measurement_valid());
}

TEST_CASE(a_noisy_input_keeps_the_last_value_but_stops_being_valid)
{
    fd_setup();
    fd_pump_ticks(140U, 5U);
    ASSERT_TRUE(freq_detect_is_measurement_valid());
    ASSERT_EQ(freq_detect_get_frequency(), 1400000U);

    /* Wildly varying deltas: every window has a spread far above 2 LSB,
     * so nothing is accepted and s_measured_freq holds the old value.
     * The number still looks perfect - only the reject run exposes that
     * it is stale. */
    fd_pump_ticks(300U, 1U);
    fd_pump_ticks(100U, 1U);
    fd_pump_ticks(280U, 1U);
    fd_pump_ticks(90U,  1U);
    fd_pump_ticks(310U, 1U);
    fd_pump_ticks(120U, 1U);

    ASSERT_EQ(freq_detect_get_frequency(), 1400000U);   /* unchanged */
    ASSERT_FALSE(freq_detect_is_measurement_valid());    /* but stale */
}

TEST_CASE(one_glitch_does_not_invalidate_a_good_measurement)
{
    fd_setup();
    fd_pump_ticks(140U, 5U);
    ASSERT_TRUE(freq_detect_is_measurement_valid());

    /* A single out-of-band sample is rejected by the window, but ONE
     * rejection is not enough to call the reading stale - that needs
     * FREQDET_STALE_REJECTS in a row.  So the display must stay green
     * through a single glitch, otherwise it would flicker grey on every
     * relay click. */
    fd_pump_ticks(400U, 1U);
    ASSERT_TRUE(freq_detect_is_measurement_valid());

    /* The window still holds the 400, so it takes three good samples to
     * push it out; until then every tick is rejected.  The reading is
     * still reported as live throughout, because the reject run has not
     * reached the staleness threshold. */
    fd_pump_ticks(140U, 2U);
    ASSERT_TRUE(freq_detect_is_measurement_valid());

    /* Third good sample: the window is clean again and the accept
     * resets the reject run. */
    fd_pump_ticks(140U, 1U);
    ASSERT_TRUE(freq_detect_is_measurement_valid());
}

TEST_CASE(an_out_of_band_signal_is_never_valid)
{
    fd_setup();
    /* 700 kHz - consistent, accepted, but far below the 1 MHz floor. */
    fd_pump_ticks(70U, 5U);
    ASSERT_EQ(freq_detect_get_frequency(), 700000U);
    ASSERT_FALSE(freq_detect_is_measurement_valid());
}

/*============================================================================
 *  Band-mismatch tests
 *============================================================================*/

TEST_CASE(mismatch_alerts_when_band_differs_during_tx)
{
    fd_setup();
    g_current_band = BAND_20M_17M;
    g_tx_active    = true;

    /* First 14 MHz - same band, no alert. */
    fd_pump_ticks(1400U, 5U);
    ASSERT_NE(g_alert_active, ALERT_FREQ_MISMATCH);

    /* Hard switch to 7 MHz (40m/30m) and let the window accept it.
     * The brief spread during the switch is rejected, but the window
     * settles once three 700-tick samples land; that final accept
     * publishes the new freq and the mismatch check fires. */
    fd_pump_ticks(700U, 10U);
    ASSERT_EQ(g_alert_active, ALERT_FREQ_MISMATCH);
}

TEST_CASE(mismatch_does_not_alert_on_same_band)
{
    fd_setup();
    g_current_band = BAND_15M_10M;
    g_tx_active    = true;
    /* 21 MHz - in 15m/10m. */
    fd_pump_ticks(2100U, 3U);
    ASSERT_NE(g_alert_active, ALERT_FREQ_MISMATCH);
}

TEST_CASE(mismatch_does_not_alert_below_1_mhz)
{
    fd_setup();
    g_tx_active = true;
    /* 500 kHz: well below the 1 MHz valid window. */
    fd_pump_ticks(50U, 3U);
    ASSERT_NE(g_alert_active, ALERT_FREQ_MISMATCH);
}

TEST_CASE(mismatch_does_not_alert_above_60_mhz)
{
    fd_setup();
    g_tx_active = true;
    /* 70 MHz: above the 60 MHz ceiling. */
    fd_pump_ticks(7000U, 3U);
    ASSERT_NE(g_alert_active, ALERT_FREQ_MISMATCH);
}

TEST_CASE(reject_does_not_trigger_band_mismatch_check)
{
    /* The whole point of the sliding window is that a single stray
     * sample (relay bounce, EMI pulse) must NOT cause a band-mismatch
     * alert - the band-mismatch check is only reached when this tick
     * published a new freq.  Stay in band 20m/17m, then mix in samples
     * that would otherwise look like 40m/30m but spread too widely
     * for the window to accept. */
    fd_setup();
    g_current_band = BAND_20M_17M;
    g_tx_active    = true;

    /* Anchor the window at 14 MHz so the spread is well-defined. */
    fd_pump_ticks(1400U, 5U);
    ASSERT_NE(g_alert_active, ALERT_FREQ_MISMATCH);

    /* Push three alternating values that bracket a different band
     * (40m/30m):  the window cannot accept (700..1500 = 800 LSB
     * spread) and the freq stays at 14 MHz -> no alert. */
    htim1.Instance->CNT = 0U;
    freq_detect_isr();                                  /* priming */
    htim1.Instance->CNT = 700U; freq_detect_isr();
    htim1.Instance->CNT = 1500U; freq_detect_isr();
    htim1.Instance->CNT = 700U; freq_detect_isr();
    ASSERT_NE(g_alert_active, ALERT_FREQ_MISMATCH);
    ASSERT_EQ(freq_detect_get_frequency(), 14000000U);  /* unchanged */

    /* Even a freak single huge delta (simulating a momentary spike on
     * the ETR line) gets rejected and does not move freq or fire the
     * mismatch check. */
    htim1.Instance->CNT = 60000U; freq_detect_isr();
    ASSERT_EQ(freq_detect_get_frequency(), 14000000U);
    ASSERT_NE(g_alert_active, ALERT_FREQ_MISMATCH);

    /* Settle back to 14 MHz - window accepts, but freq still matches
     * g_current_band so no alert. */
    fd_pump_ticks(1400U, 3U);
    ASSERT_NE(g_alert_active, ALERT_FREQ_MISMATCH);
}

/*============================================================================
 *  Quantisation-on-band-edge tests
 *
 *    BAND_20M_17M_MIN    14.100 MHz
 *    BAND_20M_17M_MAX    18.168 MHz
 *    BAND_15M_10M_MIN    21.000 MHz
 *
 *  So the 20m/17m band ends at 18.168 MHz and the 15m/10m band starts
 *  at 21.000 MHz - there is a 2.832 MHz gap (the 17m sub-band sits
 *  inside 20m/17m; this is not the boundary that matters here).
 *
 *  The boundary that bites at the 20m/17m -> 15m/10m transition:
 *
 *    18.168 MHz          BAND_20M_17M_MAX
 *    18.168 .. 21.000    BAND_FAULT
 *    21.000              BAND_15M_10M_MIN
 *
 *  A quantised measurement at 18.17 MHz is *just* below the 18.168
 *  edge; with the +500 kHz guard band the wide classifier treats it as
 *  BAND_15M_10M.  This is exactly the design intent: when the radio
 *  is parked on 15m/10m and the measurement dithers by less than
 *  500 kHz around a band edge, the classifier does not bounce and
 *  the latch never opens.
 *==========================================================================*/

TEST_CASE(quantisation_near_band_edge_does_not_trigger_mismatch)
{
    fd_setup();
    g_current_band = BAND_15M_10M;
    g_tx_active    = true;

    /* 18.10 MHz = 1810 ticks.  With raw classifier this is BAND_20M_17M
     * (between 14.1 and 18.168), so the raw mismatch path WOULD trip.
     * With the wide classifier (BAND_EPS_HZ = 500 kHz) BAND_15M_10M_MIN
     * becomes 20.5 MHz; 18.10 MHz < 20.5 MHz so the wide classifier
     * returns BAND_20M_17M here.  Move to 20.6 MHz to verify the
     * guard band pulls the borderline measurement back into 15m/10m.
     */
    fd_pump_ticks(1810U, 3U);          /* 18.10 MHz -> BAND_20M_17M raw & wide */
    ASSERT_EQ(g_alert_active, ALERT_FREQ_MISMATCH);

    /* Reset latch and move to 18.6 MHz, which is still in the FAULT
     * gap (18.168..21.0) with the raw classifier but the wide one
     * treats 18.6 + 0.5 = 19.1 MHz which is still < 21.0 - 0.5 =
     * 20.5 MHz, so the wide classifier returns BAND_FAULT for it.
     * Verify BAND_FAULT does not raise the alert either. */
    protection_clear();
    g_alert_active = ALERT_NONE;
    g_alert_flag   = false;
    fd_pump_ticks(1860U, 3U);
    ASSERT_NE(g_alert_active, ALERT_FREQ_MISMATCH);

    /* Now 20.6 MHz: raw -> BAND_FAULT, wide -> 20.6 + 0.5 = 21.1 which
     * exceeds 21.0 - 0.5 = 20.5, so wide still says FAULT.  No alert. */
    protection_clear();
    g_alert_active = ALERT_NONE;
    g_alert_flag   = false;
    fd_pump_ticks(2060U, 3U);
    ASSERT_NE(g_alert_active, ALERT_FREQ_MISMATCH);

    /* 21.4 MHz: raw -> BAND_15M_10M, wide -> 15M_10M.  Same band as
     * g_current_band -> no alert. */
    protection_clear();
    g_alert_active = ALERT_NONE;
    g_alert_flag   = false;
    fd_pump_ticks(2140U, 3U);
    ASSERT_NE(g_alert_active, ALERT_FREQ_MISMATCH);

    /* 24.9 MHz: solid 15m/10m band, no edge issue. */
    protection_clear();
    g_alert_active = ALERT_NONE;
    g_alert_flag   = false;
    fd_pump_ticks(2490U, 3U);
    ASSERT_NE(g_alert_active, ALERT_FREQ_MISMATCH);
}

TEST_CASE(quantisation_at_exact_band_edge_does_not_bounce)
{
    fd_setup();
    g_current_band = BAND_15M_10M;
    g_tx_active    = true;

    /* Settle the window at 20.5 MHz first (so the spread vs. a fresh
     * 0 sample does not make us reject the first few). */
    fd_pump_ticks(2050U, 3U);
    ASSERT_EQ(freq_detect_get_frequency(), 20500000U);

    /* Dither 20.5 MHz +/- 10 kHz.  All three samples land in the
     * guard band of the 21.0 MHz edge so the wide classifier picks
     * BAND_15M_10M regardless of which side of 21.0 the raw value
     * sits on.  Median of three is the middle value (2050), so the
     * published frequency stays at 20.5 MHz. */
    fd_pump_ticks(2051U, 1U);         /* +10 kHz */
    fd_pump_ticks(2050U, 1U);
    ASSERT_EQ(freq_detect_get_frequency(), 20500000U);
    /* Wide classifier: 20.5 + 0.5 = 21.0 -> BAND_15M_10M (== current). */
    ASSERT_NE(g_alert_active, ALERT_FREQ_MISMATCH);
}

/*============================================================================
 *  Latch-clearing tests
 *==========================================================================*/

TEST_CASE(latch_clears_when_the_radio_returns_to_the_right_band)
{
    fd_setup();
    g_current_band = BAND_20M_17M;
    g_tx_active    = true;
    fd_pump_ticks(700U, 3U);           /* 7 MHz -> wrong band */
    ASSERT_EQ(g_alert_active, ALERT_FREQ_MISMATCH);

    /* Move the radio back onto the selected band.  The latch has to
     * release on that alone - the keying state is no longer part of
     * the condition in either direction, because it is not what set
     * the latch any more. */
    fd_pump_ticks(1400U, 3U);          /* 14 MHz = 20m/17m, correct */
    freq_detect_check();

    /* Now the radio drifts back to the wrong band: the detector must
     * be armed again and fire a second time.  Step past the post-dismiss
     * hold-off first - a dismiss deliberately keeps THIS cause quiet for
     * PROTECTION_REARM_HOLD_MS so the popup really goes away. */
    protection_clear();
    test_wall_clock_advance(1000U);
    fd_pump_ticks(700U, 3U);           /* 7 MHz = 40m/30m vs. 20m/17m */
    ASSERT_EQ(g_alert_active, ALERT_FREQ_MISMATCH);
}

TEST_CASE(latch_does_not_clear_merely_because_the_carrier_dropped)
{
    fd_setup();
    g_current_band = BAND_20M_17M;
    g_tx_active    = true;
    fd_pump_ticks(700U, 3U);           /* 7 MHz -> wrong band */
    ASSERT_EQ(g_alert_active, ALERT_FREQ_MISMATCH);

    /* Unkeying is not evidence that the radio is on the right band -
     * the reading simply stops being updated.  Releasing on that would
     * let a unit that is still on the wrong band re-arm itself every
     * time the operator lets go of the PTT. */
    g_tx_active = false;
    freq_detect_check();

    /* The mismatch is still standing, so a fresh wrong-band sample
     * must NOT be able to fire a second alert (the latch held). */
    g_tx_active = true;
    fd_pump_ticks(700U, 3U);
    ASSERT_EQ(g_alert_active, ALERT_FREQ_MISMATCH);
    /* And the latch is still what suppresses the second alert: only
     * one alert event has ever been published for one dismissal. */
}

TEST_CASE(mismatch_alerts_while_idle_too)
{
    fd_setup();
    g_current_band = BAND_20M_17M;
    g_tx_active    = false;            /* idle: and it must not matter */

    fd_pump_ticks(700U, 3U);           /* 7 MHz, a real ham-band carrier */

    ASSERT_EQ(g_alert_active, ALERT_FREQ_MISMATCH);
}

TEST_CASE(mismatch_stays_quiet_when_idle_on_the_right_band)
{
    fd_setup();
    g_current_band = BAND_20M_17M;
    g_tx_active    = false;

    /* Idle but correctly on band - the common case, and it must stay
     * silent.  This is what keeps the unconditional check harmless:
     * the valid-frequency window plus the 3-sample agreement are what
     * distinguish a real carrier from noise, not the keying state. */
    fd_pump_ticks(1400U, 5U);          /* 14 MHz */

    ASSERT_NE(g_alert_active, ALERT_FREQ_MISMATCH);
}

TEST_CASE(latch_clears_when_freq_leaves_valid_window)
{
    fd_setup();
    g_current_band = BAND_20M_17M;
    g_tx_active    = true;
    fd_pump_ticks(700U, 3U);           /* 7 MHz -> wrong band */
    ASSERT_EQ(g_alert_active, ALERT_FREQ_MISMATCH);

    /* Move below 1 MHz: latch should release. */
    fd_pump_ticks(50U, 3U);
    freq_detect_check();
    /* Move back to wrong band: latch should re-arm. */
    fd_pump_ticks(700U, 3U);
    ASSERT_EQ(g_alert_active, ALERT_FREQ_MISMATCH);
}

/*============================================================================
 *  Suite registration
 *============================================================================*/

const test_entry_t g_suite_freq_detect[] = {
    TEST_ENTRY(the_sampling_window_is_100us),
    TEST_ENTRY(one_count_is_ten_kHz_and_the_readout_agrees),
    TEST_ENTRY(the_published_frequency_is_not_low_by_the_window_error),
    TEST_ENTRY(window_publishes_after_two_agreeing_samples),
    TEST_ENTRY(window_rejects_when_spread_exceeds_2_lsb),
    TEST_ENTRY(window_recovers_when_signal_stabilises_again),
    TEST_ENTRY(a_healthy_measurement_reports_valid),
    TEST_ENTRY(nothing_measured_yet_is_not_valid),
    TEST_ENTRY(a_single_sample_is_not_enough_to_be_valid),
    TEST_ENTRY(a_dead_input_stops_being_reported_valid),
    TEST_ENTRY(a_noisy_input_keeps_the_last_value_but_stops_being_valid),
    TEST_ENTRY(one_glitch_does_not_invalidate_a_good_measurement),
    TEST_ENTRY(an_out_of_band_signal_is_never_valid),
    TEST_ENTRY(mismatch_alerts_when_band_differs_during_tx),
    TEST_ENTRY(mismatch_does_not_alert_on_same_band),
    TEST_ENTRY(mismatch_does_not_alert_below_1_mhz),
    TEST_ENTRY(mismatch_does_not_alert_above_60_mhz),
    TEST_ENTRY(reject_does_not_trigger_band_mismatch_check),
    TEST_ENTRY(quantisation_near_band_edge_does_not_trigger_mismatch),
    TEST_ENTRY(quantisation_at_exact_band_edge_does_not_bounce),
    TEST_ENTRY(latch_clears_when_the_radio_returns_to_the_right_band),
    TEST_ENTRY(latch_does_not_clear_merely_because_the_carrier_dropped),
    TEST_ENTRY(mismatch_alerts_while_idle_too),
    TEST_ENTRY(mismatch_stays_quiet_when_idle_on_the_right_band),
    TEST_ENTRY(latch_clears_when_freq_leaves_valid_window),
    TEST_SUITE_END,
};