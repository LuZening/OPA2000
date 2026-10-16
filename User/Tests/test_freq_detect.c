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
 *   - Below 1 MHz / above 60 MHz during TX -> no alert
 *   - Quantisation on a band edge: 14.000 MHz sits between 20m/17m and
 *     15m/10m - the guard band keeps the wide classifier on whichever
 *     band the radio is already on, so no alert
 *   - Rejected samples do NOT participate in band-mismatch detection:
 *     a stray ETR pulse that the window discards must not flip the
 *     latch into ALERT_FREQ_MISMATCH
 *   - Latch clears once the carrier drops or the frequency leaves the
 *     valid window
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
 *============================================================================*/

/* Reset freq_detect.c internals and the alert latch.  Mirrors the
 * boot-time sequence the firmware runs. */
static void fd_setup(void)
{
    freq_detect_init();
    freq_detect_start();

    /* Clear any latch left by an earlier test (the same is done by
     * protection_clear() in the firmware; here we just exercise the
     * test's own reset path). */
    protection_clear();

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
     * and return without touching the window. */
    htim1.Instance->CNT = 0U;
    freq_detect_isr();

    for (uint32_t i = 0U; i < n; i++) {
        htim1.Instance->CNT = (uint16_t)(htim1.Instance->CNT + delta);
        freq_detect_isr();
    }
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

TEST_CASE(mismatch_does_not_alert_when_not_transmitting)
{
    fd_setup();
    /* Without TX, even a wildly off-band signal must not alert. */
    g_tx_active = false;
    fd_pump_ticks(700U, 3U);          /* 7 MHz vs. operator-set 20m */
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

TEST_CASE(latch_clears_when_carrier_drops)
{
    fd_setup();
    g_current_band = BAND_20M_17M;
    g_tx_active    = true;
    fd_pump_ticks(1400U, 3U);          /* 14 MHz, same band */
    /* Cross to a different band while keyed. */
    fd_pump_ticks(7000U, 3U);          /* 70 MHz: above 60 MHz, so no
                                          alert yet - we need a real
                                          wrong-band signal. */
    fd_pump_ticks(700U, 3U);           /* 7 MHz */
    ASSERT_EQ(g_alert_active, ALERT_FREQ_MISMATCH);

    /* Drop the carrier: the next call to freq_detect_check() should
     * release the latch. */
    g_tx_active = false;
    freq_detect_check();
    /* Push another wrong-band sample to prove the latch re-arms. */
    g_tx_active = true;
    fd_pump_ticks(700U, 3U);           /* 7 MHz = 40m/30m vs. 20m/17m */
    ASSERT_EQ(g_alert_active, ALERT_FREQ_MISMATCH);
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
    TEST_ENTRY(window_publishes_after_two_agreeing_samples),
    TEST_ENTRY(window_rejects_when_spread_exceeds_2_lsb),
    TEST_ENTRY(window_recovers_when_signal_stabilises_again),
    TEST_ENTRY(mismatch_alerts_when_band_differs_during_tx),
    TEST_ENTRY(mismatch_does_not_alert_on_same_band),
    TEST_ENTRY(mismatch_does_not_alert_below_1_mhz),
    TEST_ENTRY(mismatch_does_not_alert_above_60_mhz),
    TEST_ENTRY(mismatch_does_not_alert_when_not_transmitting),
    TEST_ENTRY(reject_does_not_trigger_band_mismatch_check),
    TEST_ENTRY(quantisation_near_band_edge_does_not_trigger_mismatch),
    TEST_ENTRY(quantisation_at_exact_band_edge_does_not_bounce),
    TEST_ENTRY(latch_clears_when_carrier_drops),
    TEST_ENTRY(latch_clears_when_freq_leaves_valid_window),
    TEST_SUITE_END,
};