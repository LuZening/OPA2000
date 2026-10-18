/**
 * @file    test_user_adc.c
 * @brief   Unit tests for the REAL user_adc.c implementation.
 *
 * @details Covers
 *            - adc_init() resets the calibration state machine
 *            - adc_im_zero_offset_tick() 50-sample window + retry policy
 *            - adc_process() publishes engineering units into g_adc_data
 *
 *          The tests drive the real state machine by seeding the ADC1 DMA
 *          buffer; no logic is duplicated here.
 */

#include "test_harness.h"
#include "user_adc.h"
#include "adc_calibration.h"

#include <stdint.h>
#include <string.h>

/* CC6920 zero-offset window: 0 A = 1.65 V = 43254 codes, +-40 mV = +-1048. */
#define IM_ZERO_EXPECTED     43254
#define IM_ZERO_TOLERANCE    1048
#define IM_SAMPLE_COUNT      50
#define IM_TICKS_PER_PASS    (IM_SAMPLE_COUNT + 1)   /* 50 samples + 1 validate */

/* Power-on settling window, in ADC-task ticks.  Must match
 * STAB_SETTLE_TICKS in user_adc.c: 20 x 50 ms = 1.0 s. */
#define STAB_SETTLE_TICKS    20

/* Reset the module and seed the IM rank of the newest DMA row. */
static void im_feed(uint16_t raw)
{
    adc1_dma_buf[(ADC_DMA_BUFFER_SIZE - 1U) * ADC1_RANK_COUNT + ADC1_RANK_IM] = raw;
}

static void im_reset(void)
{
    adc_init();
    stub_gpio_reset_counters();
    g_alert_flag   = false;
    g_alert_active = ALERT_NONE;
}

/* Run one full sampling+validation pass, seeding `raw` before every tick. */
static void im_run_pass(uint16_t raw)
{
    for (int i = 0; i < IM_TICKS_PER_PASS; i++) {
        im_feed(raw);
        (void)adc_im_zero_offset_tick();
    }
}

/*============================================================================
 *  adc_init()
 *==========================================================================*/

TEST_CASE(adc_init_rearms_calibration_and_blocks_rf)
{
    im_reset();

    ASSERT_FALSE(g_config.im_calibrated);
    ASSERT_FALSE(g_allow_rf);
}

/*============================================================================
 *  adc_im_zero_offset_tick() - success paths
 *==========================================================================*/

TEST_CASE(im_calibration_succeeds_nominal)
{
    im_reset();

    for (int i = 0; i < IM_SAMPLE_COUNT; i++) {
        im_feed(IM_ZERO_EXPECTED);
        ASSERT_FALSE(adc_im_zero_offset_tick());
    }
    /* 51st tick performs the validation. */
    im_feed(IM_ZERO_EXPECTED);
    ASSERT_TRUE(adc_im_zero_offset_tick());

    ASSERT_EQ(g_config.im_zero_offset_adc, IM_ZERO_EXPECTED);
    ASSERT_EQ(g_config.im_calibrated, 1);
    ASSERT_TRUE(g_allow_rf);
}

TEST_CASE(im_calibration_succeeds_at_upper_tolerance_edge)
{
    im_reset();
    im_run_pass((uint16_t)(IM_ZERO_EXPECTED + IM_ZERO_TOLERANCE));

    ASSERT_EQ(g_config.im_calibrated, 1);
    ASSERT_TRUE(g_allow_rf);
    ASSERT_FALSE(g_alert_flag);
}

TEST_CASE(im_calibration_succeeds_at_lower_tolerance_edge)
{
    im_reset();
    im_run_pass((uint16_t)(IM_ZERO_EXPECTED - IM_ZERO_TOLERANCE));

    ASSERT_EQ(g_config.im_calibrated, 1);
    ASSERT_TRUE(g_allow_rf);
    ASSERT_FALSE(g_alert_flag);
}

TEST_CASE(im_calibration_averages_sampled_window)
{
    /* Alternate +/- 50 codes around the expectation: the mean must land
     * on the expectation, proving the state machine averages all 50
     * samples instead of using the last one. */
    im_reset();

    for (int i = 0; i < IM_SAMPLE_COUNT; i++) {
        im_feed((i & 1) ? (uint16_t)(IM_ZERO_EXPECTED - 50)
                        : (uint16_t)(IM_ZERO_EXPECTED + 50));
        adc_im_zero_offset_tick();
    }
    im_feed(IM_ZERO_EXPECTED);
    ASSERT_TRUE(adc_im_zero_offset_tick());

    ASSERT_EQ(g_config.im_zero_offset_adc, IM_ZERO_EXPECTED);
}

TEST_CASE(im_calibration_short_circuits_once_calibrated)
{
    im_reset();
    im_run_pass(IM_ZERO_EXPECTED);
    ASSERT_TRUE(g_config.im_calibrated);

    /* Further ticks must be no-ops that keep reporting success even if
     * the sensor now reads garbage. */
    im_run_pass(1000);
    ASSERT_EQ(g_config.im_zero_offset_adc, IM_ZERO_EXPECTED);
    ASSERT_TRUE(adc_im_zero_offset_tick());
    ASSERT_FALSE(g_alert_flag);
}

/*============================================================================
 *  adc_im_zero_offset_tick() - retry / fault path
 *==========================================================================*/

TEST_CASE(im_calibration_retries_three_times_then_alarms)
{
    const uint16_t bad = (uint16_t)(IM_ZERO_EXPECTED + 3000);

    im_reset();

    /* Pass 1 */
    im_run_pass(bad);
    ASSERT_FALSE(g_config.im_calibrated);
    ASSERT_FALSE(g_allow_rf);
    ASSERT_FALSE(g_alert_flag);

    /* Pass 2 */
    im_run_pass(bad);
    ASSERT_FALSE(g_config.im_calibrated);
    ASSERT_FALSE(g_alert_flag);

    /* Pass 3 -> give up */
    im_run_pass(bad);
    ASSERT_FALSE(g_config.im_calibrated);
    ASSERT_FALSE(g_allow_rf);
    ASSERT_TRUE(g_alert_flag);
    ASSERT_EQ(g_alert_active, ALERT_HARDWARE_FAULT);

    /* Once given up the machine stays idle and must not re-arm itself. */
    im_run_pass(IM_ZERO_EXPECTED);
    ASSERT_FALSE(g_config.im_calibrated);
}

TEST_CASE(im_calibration_recovers_on_second_pass)
{
    im_reset();

    im_run_pass((uint16_t)(IM_ZERO_EXPECTED + 3000));
    ASSERT_FALSE(g_config.im_calibrated);

    im_run_pass(IM_ZERO_EXPECTED);
    ASSERT_EQ(g_config.im_calibrated, 1);
    ASSERT_TRUE(g_allow_rf);
    ASSERT_FALSE(g_alert_flag);
}

/*============================================================================
 *  Power-on stabilisation gate (adc_stabilisation_tick / _done)
 *
 *  The amplifier must not treat its own analog front end as trustworthy
 *  during the first second after power-up.  These cases pin the three
 *  properties that make that safe:
 *
 *    - nothing is armed and PTT stays blocked for the whole window,
 *    - CLEAR_FAULTX_OC is held low for every tick of it, so nothing can
 *      latch an alarm from the settling transient,
 *    - the real limits go out and the pin is released only after the IM
 *      zero-offset calibration has actually passed.
 *
 *  A "tick" here is one ADC-task iteration, i.e. 50 ms; STAB_SETTLE_TICKS
 *  (20) is therefore 1.0 s, matching the spec.
 *==========================================================================*/

/* Ticks the settling window with the IM rank already at its true zero, so
 * the calibration can pass as soon as it is allowed to start. */
static void stab_tick_settling(int count)
{
    for (int i = 0; i < count; i++) {
        im_feed(IM_ZERO_EXPECTED);
        ASSERT_FALSE(adc_stabilisation_tick());
        ASSERT_FALSE(adc_stabilisation_done());
    }
}

/* Ticks past the settling window, seeding `raw` before each one. */
static bool stab_tick_calibrating(uint16_t raw, int count)
{
    bool armed = false;
    for (int i = 0; i < count; i++) {
        im_feed(raw);
        armed = adc_stabilisation_tick();
    }
    return armed;
}

static uint8_t clear_faultx_level(void)
{
    return stub_gpio_get_last(CLEAR_FAULTX_OC_GPIO_Port, CLEAR_FAULTX_OC_Pin);
}

TEST_CASE(stabilisation_blocks_the_first_second_entirely)
{
    im_reset();

    /* One tick short of the window must still be un-armed. */
    stab_tick_settling(STAB_SETTLE_TICKS - 1);

    ASSERT_FALSE(adc_stabilisation_done());
    ASSERT_FALSE(g_config.im_calibrated);
    ASSERT_FALSE(g_allow_rf);
}

TEST_CASE(stabilisation_holds_clear_fault_low_for_the_whole_window)
{
    im_reset();
    /* Put the pin somewhere other than the level we are about to assert, so
     * a missing write cannot accidentally look like a pass. */
    stub_gpio_set(CLEAR_FAULTX_OC_GPIO_Port, CLEAR_FAULTX_OC_Pin, 1U);

    stab_tick_settling(STAB_SETTLE_TICKS);

    ASSERT_EQ(clear_faultx_level(), 0);
    /* Still low one tick into calibration, i.e. held across the boundary. */
    (void)stab_tick_calibrating(IM_ZERO_EXPECTED, 1);
    ASSERT_EQ(clear_faultx_level(), 0);
}

TEST_CASE(stabilisation_arms_only_after_the_calibration_passes)
{
    im_reset();
    stab_tick_settling(STAB_SETTLE_TICKS);

    /* 50 sampling ticks then the validation tick, all fed the true zero.
     * The last of them is the one that arms, which is what makes this the
     * earliest possible moment the readings may be believed. */
    ASSERT_TRUE(stab_tick_calibrating(IM_ZERO_EXPECTED, IM_TICKS_PER_PASS));

    ASSERT_TRUE(adc_stabilisation_done());
    ASSERT_EQ(g_config.im_calibrated, 1);
    ASSERT_TRUE(g_allow_rf);
    ASSERT_FALSE(g_alert_flag);
}

TEST_CASE(stabilisation_releases_clear_fault_when_it_arms)
{
    im_reset();
    stub_gpio_set(CLEAR_FAULTX_OC_GPIO_Port, CLEAR_FAULTX_OC_Pin, 0U);

    stab_tick_settling(STAB_SETTLE_TICKS);
    ASSERT_TRUE(stab_tick_calibrating(IM_ZERO_EXPECTED, IM_TICKS_PER_PASS));

    /* Released, not left low: the latch must work normally from now on. */
    ASSERT_EQ(clear_faultx_level(), 1);
}

TEST_CASE(stabilisation_holds_clear_fault_low_when_calibration_fails)
{
    const uint16_t bad = (uint16_t)(IM_ZERO_EXPECTED + 3000);

    im_reset();
    stub_gpio_set(CLEAR_FAULTX_OC_GPIO_Port, CLEAR_FAULTX_OC_Pin, 1U);

    stab_tick_settling(STAB_SETTLE_TICKS);

    /* Three failed passes -> ALERT_HARDWARE_FAULT, but still not armed and
     * still holding the pin low, so a bad sensor cannot both alarm and then
     * leave a latched fault behind for the next power cycle. */
    for (int pass = 0; pass < 3; pass++) {
        (void)stab_tick_calibrating(bad, IM_TICKS_PER_PASS);
    }

    ASSERT_TRUE(g_alert_flag);
    ASSERT_EQ(g_alert_active, ALERT_HARDWARE_FAULT);
    ASSERT_FALSE(adc_stabilisation_done());
    ASSERT_FALSE(g_allow_rf);
    ASSERT_EQ(clear_faultx_level(), 0);
}

TEST_CASE(stabilisation_stays_armed_and_is_idempotent)
{
    im_reset();
    stab_tick_settling(STAB_SETTLE_TICKS);
    ASSERT_TRUE(stab_tick_calibrating(IM_ZERO_EXPECTED, IM_TICKS_PER_PASS));
    ASSERT_TRUE(adc_stabilisation_done());

    /* The recorded zero must survive further ticks: a later ARMED tick must
     * not re-run the calibration and overwrite it with a fresh average. */
    uint32_t offset = g_config.im_zero_offset_adc;

    for (int i = 0; i < 200; i++) {
        im_feed(1000U);              /* garbage on the bus */
        ASSERT_TRUE(adc_stabilisation_tick());
    }

    ASSERT_TRUE(adc_stabilisation_done());
    ASSERT_EQ(g_config.im_zero_offset_adc, offset);
    ASSERT_FALSE(g_alert_flag);
}

TEST_CASE(stabilisation_restarts_when_adc_init_reruns)
{
    im_reset();
    stab_tick_settling(STAB_SETTLE_TICKS);
    ASSERT_TRUE(stab_tick_calibrating(IM_ZERO_EXPECTED, IM_TICKS_PER_PASS));
    ASSERT_TRUE(adc_stabilisation_done());

    /* A soft restart re-runs adc_init(); the gate must go back to square one
     * rather than carry the armed state over from the previous run. */
    adc_init();

    ASSERT_FALSE(adc_stabilisation_done());
    ASSERT_FALSE(g_config.im_calibrated);
    ASSERT_FALSE(g_allow_rf);
}

/*============================================================================
 *  adc_process()
 *==========================================================================*/

TEST_CASE(adc_process_averages_the_rolling_dma_window)
{
    im_reset();
    opa2000_init();

    /* 15 rows at 100 codes, newest row at 30000 -> adc_process() must
     * report the 16-sample rolling average (15*100 + 30000)/16 = 1968. */
    for (uint16_t row = 0U; row < ADC_DMA_BUFFER_SIZE; row++) {
        for (uint8_t r = 0U; r < ADC1_RANK_COUNT; r++) {
            adc1_dma_buf[row * ADC1_RANK_COUNT + r] = 100U;
        }
    }
    adc1_dma_buf[(ADC_DMA_BUFFER_SIZE - 1U) * ADC1_RANK_COUNT + ADC1_RANK_VM] = 30000U;

    adc_process();

    ASSERT_EQ(g_adc_data.adc1_raw[ADC1_RANK_VM], 1968);
    ASSERT_EQ(g_adc_data.vm_mV, adc1_raw_to_vm_mV(1968));
}

TEST_CASE(adc_process_publishes_all_engineering_units)
{
    im_reset();
    opa2000_init();

    /* Fill every DMA row with the same code per rank so the 16-sample
     * rolling average equals the seeded value exactly. */
    for (uint8_t r = 0U; r < ADC1_RANK_COUNT; r++) {
        uint16_t code = 1000U;
        switch (r) {
        case ADC1_RANK_RF_IN:   code = 20000U; break;
        case ADC1_RANK_REV1:    code = 4000U;  break;
        case ADC1_RANK_FWD1:    code = 30000U; break;
        case ADC1_RANK_REV2:    code = 6000U;  break;
        case ADC1_RANK_FWD2:    code = 40000U; break;
        case ADC1_RANK_VBIAS:   code = 26000U; break;
        case ADC1_RANK_KEY_ADC: code = 12345U; break;
        default: break;
        }
        for (uint16_t row = 0U; row < ADC_DMA_BUFFER_SIZE; row++) {
            adc1_dma_buf[row * ADC1_RANK_COUNT + r] = code;
        }
    }

    adc2_inj_ntc_core_raw = 20000U;
    adc2_inj_ntc_aux_raw  = 21000U;
    adc2_inj_tsense_raw   = 17000U;
    for (uint16_t row = 0U; row < ADC_DMA_BUFFER_SIZE; row++) {
        adc2_dma_buf[row * 2U]     = 1234U;
        adc2_dma_buf[row * 2U + 1U] = 5678U;
    }

    adc_process();

    ASSERT_EQ(g_adc_data.rf_in_mW, adc1_raw_to_rf_in_mW(20000));
    ASSERT_EQ(g_adc_data.rev1_mW,  adc1_raw_to_rev1_mW(4000));
    ASSERT_EQ(g_adc_data.fwd1_mW,  adc1_raw_to_fwd1_mW(30000));
    ASSERT_EQ(g_adc_data.rev2_mW,  adc1_raw_to_rev2_mW(6000));
    ASSERT_EQ(g_adc_data.fwd2_mW,  adc1_raw_to_fwd2_mW(40000));
    ASSERT_EQ(g_adc_data.vbias_mV, adc1_raw_to_vbias_sense_mV(26000));
    ASSERT_EQ(g_adc_data.key_adc,  12345);
    ASSERT_EQ(g_adc_data.ntc_core_tenth_c,
              adc2_inj_raw_to_core_ntc_tenth_c(20000));
    ASSERT_EQ(g_adc_data.ntc_aux_tenth_c,
              adc2_inj_raw_to_aux_ntc_tenth_c(21000));
    ASSERT_EQ(g_adc_data.mcu_int_tenth_c,
              adc2_inj_raw_to_mcu_internal_tenth_c(17000));
    ASSERT_EQ(g_adc_data.adc2_if_pa_raw,    1234);
    ASSERT_EQ(g_adc_data.adc2_if_radio_raw, 5678);
}

TEST_CASE(adc_process_derives_swr_from_fwd2_and_rev2)
{
    im_reset();
    opa2000_init();

    /* 400 W forward and 44 W reflected -> |G| = 0.256 -> SWR 1.69 */
    for (uint16_t row = 0U; row < ADC_DMA_BUFFER_SIZE; row++) {
        for (uint8_t r = 0U; r < ADC1_RANK_COUNT; r++) {
            adc1_dma_buf[row * ADC1_RANK_COUNT + r] = 0U;
        }
        adc1_dma_buf[row * ADC1_RANK_COUNT + ADC1_RANK_FWD2] = 40000U;
        adc1_dma_buf[row * ADC1_RANK_COUNT + ADC1_RANK_REV2] = 4400U;
    }

    adc_process();

    ASSERT_TRUE(g_adc_data.fwd2_mW > 0);
    ASSERT_EQ(g_adc_data.swr_x100,
              calculate_swr_x100(g_adc_data.fwd2_mW, g_adc_data.rev2_mW));
    ASSERT_GE(g_adc_data.swr_x100, 160);
    ASSERT_LE(g_adc_data.swr_x100, 180);
}

TEST_CASE(adc_process_increments_sequence_counter)
{
    im_reset();
    opa2000_init();
    g_adc_data.seq = 100U;

    adc_process();
    ASSERT_EQ(g_adc_data.seq, 101);
    adc_process();
    ASSERT_EQ(g_adc_data.seq, 102);
}

/*============================================================================
 *  Peak hold (coupler 2 -> GUI bar graphs)
 *
 *  Contract: instantaneous attack, exponential release with tau = 500 ms,
 *  driven exclusively by adc_peak_hold_tick() from task context.  The holds
 *  are a pure function of the coupler reading and the elapsed time - they
 *  must never depend on PTT / g_tx_active.
 *
 *  Each case publishes a coupler reading with adc_process(), advances the
 *  fake clock, then ticks the hold: exactly what the ADC task does.
 *==========================================================================*/

/* Fill the whole rolling window with one code for every rank, so
 * adc_process() reports exactly that code's engineering value. */
static void seed_all_ranks(uint16_t code)
{
    for (uint16_t row = 0U; row < ADC_DMA_BUFFER_SIZE; row++) {
        for (uint8_t r = 0U; r < ADC1_RANK_COUNT; r++) {
            adc1_dma_buf[row * ADC1_RANK_COUNT + r] = code;
        }
    }
}

/* Seed only the two coupler-2 ranks; everything else reads zero. */
static void seed_coupler2(uint16_t fwd2_code, uint16_t rev2_code)
{
    for (uint16_t row = 0U; row < ADC_DMA_BUFFER_SIZE; row++) {
        for (uint8_t r = 0U; r < ADC1_RANK_COUNT; r++) {
            adc1_dma_buf[row * ADC1_RANK_COUNT + r] = 0U;
        }
        adc1_dma_buf[row * ADC1_RANK_COUNT + ADC1_RANK_FWD2] = fwd2_code;
        adc1_dma_buf[row * ADC1_RANK_COUNT + ADC1_RANK_REV2] = rev2_code;
    }
}

/* Publish a coupler reading and advance the hold by dt_ms. */
static void feed_coupler2(uint16_t fwd2_code, uint16_t rev2_code, uint32_t dt_ms)
{
    seed_coupler2(fwd2_code, rev2_code);
    adc_process();
    if (dt_ms > 0U) {
        test_wall_clock_advance(dt_ms);
    }
    adc_peak_hold_tick();
}

static void peak_reset(void)
{
    adc_init();
    opa2000_init();
    test_wall_clock_reset();
    g_tx_active = false;   /* default: idle, which must not matter */
}

TEST_CASE(peak_hold_attacks_instantly_to_the_new_maximum)
{
    peak_reset();

    feed_coupler2(40000U, 4400U, 50U);
    ASSERT_EQ(g_adc_data.fwd2_peak_mW, g_adc_data.fwd2_mW);
    int32_t first = g_adc_data.fwd2_peak_mW;
    ASSERT_TRUE(first > 0);

    /* A bigger sample takes over with no delay, whatever the interval. */
    feed_coupler2(45000U, 4400U, 5000U);
    ASSERT_EQ(g_adc_data.fwd2_peak_mW, g_adc_data.fwd2_mW);
    ASSERT_GT(g_adc_data.fwd2_peak_mW, first);
}

TEST_CASE(peak_hold_releases_by_one_time_constant)
{
    peak_reset();

    feed_coupler2(40000U, 0U, 50U);
    int32_t held = g_adc_data.fwd2_peak_mW;
    ASSERT_TRUE(held > 0);

    /* One tau = 500 ms later the hold must be at 1/e = 36.8 %. */
    feed_coupler2(0U, 0U, 500U);
    int32_t after = g_adc_data.fwd2_peak_mW;

    /* +-1.1 points: wide enough for the float rounding, tight enough to
     * prove the release is neither linear (near 0) nor frozen (unchanged). */
    ASSERT_GT(after, (int32_t)((int64_t)held * 357 / 1000));
    ASSERT_LT(after, (int32_t)((int64_t)held * 379 / 1000));
}

TEST_CASE(peak_hold_release_depends_on_elapsed_time_not_on_tick_count)
{
    peak_reset();

    feed_coupler2(40000U, 0U, 50U);
    int32_t held = g_adc_data.fwd2_peak_mW;
    ASSERT_TRUE(held > 0);

    /* 1.5 s = 3 tau -> e^-3 = 4.98 %.  Once as a single 1500 ms step and
     * three times as 500 ms steps must land in the same place: the curve is
     * a function of elapsed time, not of how often the task happens to run. */
    feed_coupler2(0U, 0U, 1500U);
    int32_t one_step = g_adc_data.fwd2_peak_mW;

    peak_reset();
    feed_coupler2(40000U, 0U, 50U);
    ASSERT_EQ(g_adc_data.fwd2_peak_mW, held);
    for (int i = 0; i < 3; i++) {
        feed_coupler2(0U, 0U, 500U);
    }
    int32_t three_steps = g_adc_data.fwd2_peak_mW;

    /* e^-3 = 4.98 %, allow 4.5..5.5 % */
    ASSERT_GT(one_step,   (int32_t)((int64_t)held * 45 / 1000));
    ASSERT_LT(one_step,   (int32_t)((int64_t)held * 55 / 1000));
    /* float32 has ~7 significant digits; one part in 10 000 is generous. */
    ASSERT_GT(three_steps, one_step - held / 10000);
    ASSERT_LT(three_steps, one_step + held / 10000);
}

TEST_CASE(peak_hold_never_falls_below_a_rising_input)
{
    peak_reset();

    feed_coupler2(40000U, 0U, 50U);
    int32_t held = g_adc_data.fwd2_peak_mW;

    /* 2 s of decay puts the hold well under this, so the new sample wins. */
    feed_coupler2(0U, 0U, 2000U);
    ASSERT_LT(g_adc_data.fwd2_peak_mW, held);
    int32_t decayed = g_adc_data.fwd2_peak_mW;

    feed_coupler2(30000U, 0U, 0U);
    ASSERT_EQ(g_adc_data.fwd2_peak_mW, g_adc_data.fwd2_mW);
    ASSERT_GT(g_adc_data.fwd2_peak_mW, decayed);
}

TEST_CASE(peak_hold_tracks_forward_and_reverse_independently)
{
    peak_reset();

    feed_coupler2(40000U, 2000U, 50U);
    int32_t fwd = g_adc_data.fwd2_peak_mW;
    int32_t rev = g_adc_data.rev2_peak_mW;
    ASSERT_GT(fwd, rev * 10);

    /* Tuning spike: reverse jumps 20x while forward collapses.  Over the
     * 100 ms step the forward hold has not yet decayed past the new (tiny)
     * forward sample, so it must still be falling - that is the release
     * doing its job - while the reverse hold jumps to the new maximum
     * immediately. */
    feed_coupler2(2000U, 40000U, 100U);
    ASSERT_LT(g_adc_data.fwd2_peak_mW, fwd);
    ASSERT_GT(g_adc_data.fwd2_peak_mW, g_adc_data.fwd2_mW);
    ASSERT_EQ(g_adc_data.rev2_peak_mW, g_adc_data.rev2_mW);
    ASSERT_GT(g_adc_data.rev2_peak_mW, rev);
}

TEST_CASE(peak_hold_ignores_the_keying_state)
{
    /* Two runs with identical coupler data must produce identical holds,
     * whether or not the amplifier is keyed.  This is the property the
     * display needs in order to be an honest instrument. */
    peak_reset();
    g_tx_active = false;
    feed_coupler2(40000U, 4400U, 50U);
    feed_coupler2(20000U, 2200U, 200U);
    int32_t idle_fwd = g_adc_data.fwd2_peak_mW;
    int32_t idle_rev = g_adc_data.rev2_peak_mW;

    peak_reset();
    g_tx_active = true;
    feed_coupler2(40000U, 4400U, 50U);
    feed_coupler2(20000U, 2200U, 200U);

    ASSERT_EQ(g_adc_data.fwd2_peak_mW, idle_fwd);
    ASSERT_EQ(g_adc_data.rev2_peak_mW, idle_rev);
}

TEST_CASE(peak_hold_is_not_reset_by_keying_edges)
{
    peak_reset();

    /* Build a hold, then key and unkey repeatedly with no new data.  A
     * keying-triggered reset would zero it; a measurement must not. */
    feed_coupler2(40000U, 0U, 50U);
    int32_t held = g_adc_data.fwd2_peak_mW;
    ASSERT_TRUE(held > 0);

    for (int i = 0; i < 6; i++) {
        g_tx_active = !g_tx_active;
        feed_coupler2(0U, 0U, 100U);
    }
    /* Six 100 ms steps = 600 ms = 1.2 tau -> e^-1.2 = 0.3010.  The tolerance
     * is 0.5 % of the held value, which absorbs the float32 rounding of
     * the hold (kept as int32_t, so each step also truncates). */
    int32_t expected = (int32_t)((double)held * 0.30119421191220214);
    int32_t tol       = held / 200 + 1;      /* 0.5 % */
    ASSERT_GT(g_adc_data.fwd2_peak_mW, expected - tol);
    ASSERT_LT(g_adc_data.fwd2_peak_mW, expected + tol);
}

TEST_CASE(peak_hold_reports_a_nonzero_idle_reading)
{
    /* An idle amplifier whose coupler still shows a signal must display
     * that signal: it may be front-end drift or interfering RF, and hiding
     * it would hide a fault. */
    peak_reset();
    g_tx_active = false;

    feed_coupler2(40000U, 40000U, 50U);

    ASSERT_EQ(g_adc_data.fwd2_peak_mW, g_adc_data.fwd2_mW);
    ASSERT_EQ(g_adc_data.rev2_peak_mW, g_adc_data.rev2_mW);
    ASSERT_GT(g_adc_data.fwd2_peak_mW, 0);
}

TEST_CASE(peak_hold_decays_to_zero_when_the_coupler_goes_quiet)
{
    peak_reset();

    feed_coupler2(40000U, 0U, 50U);
    ASSERT_GT(g_adc_data.fwd2_peak_mW, 0);

    /* 20 tau: the hold must reach exactly 0, not a denormal leftover that
     * would keep a bar permanently lit. */
    feed_coupler2(0U, 0U, 10000U);
    ASSERT_EQ(g_adc_data.fwd2_peak_mW, 0);
    ASSERT_EQ(g_adc_data.rev2_peak_mW, 0);
}

TEST_CASE(peak_hold_survives_a_zero_interval)
{
    peak_reset();

    feed_coupler2(40000U, 0U, 50U);
    int32_t held = g_adc_data.fwd2_peak_mW;
    ASSERT_TRUE(held > 0);

    /* dt = 0 means "no time has passed", so no release - not a division by
     * zero and not a snap back to the instantaneous sample. */
    feed_coupler2(0U, 0U, 0U);
    ASSERT_EQ(g_adc_data.fwd2_peak_mW, held);
    feed_coupler2(0U, 0U, 0U);
    ASSERT_EQ(g_adc_data.fwd2_peak_mW, held);
}

TEST_CASE(peak_hold_clamps_a_very_long_stall)
{
    peak_reset();

    feed_coupler2(40000U, 0U, 50U);
    int32_t held = g_adc_data.fwd2_peak_mW;
    ASSERT_TRUE(held > 0);

    /* 30 s is clamped to PEAK_MAX_STEP_MS (10 s) so a debugger pause or an
     * interrupt storm cannot turn into one huge jump.  The result must be
     * exactly what a 10 s release gives, i.e. already zero. */
    feed_coupler2(0U, 0U, 30000U);
    ASSERT_EQ(g_adc_data.fwd2_peak_mW, 0);
    ASSERT_LT(held, 2000000);   /* only to keep the fixture honest */
}

TEST_CASE(peak_hold_gui_can_derive_a_consistent_swr)
{
    peak_reset();

    /* 400 W forward, 44 W reflected.  The GUI builds its own SWR from the
     * two holds; it must stay in the same band as the instantaneous one,
     * which is what "three bars agree" means. */
    feed_coupler2(40000U, 4400U, 50U);

    int32_t swr_from_peaks = calculate_swr_x100(g_adc_data.fwd2_peak_mW,
                                                g_adc_data.rev2_peak_mW);
    ASSERT_GE(swr_from_peaks, 160);
    ASSERT_LE(swr_from_peaks, 180);
}

TEST_CASE(peak_hold_decays_monotonically_towards_zero)
{
    peak_reset();

    feed_coupler2(40000U, 0U, 50U);
    int32_t prev = g_adc_data.fwd2_peak_mW;
    for (int i = 0; i < 10; i++) {
        feed_coupler2(0U, 0U, 200U);
        ASSERT_LE(g_adc_data.fwd2_peak_mW, prev);
        ASSERT_GE(g_adc_data.fwd2_peak_mW, 0);
        prev = g_adc_data.fwd2_peak_mW;
    }
}

TEST_CASE(peak_hold_is_independent_of_which_context_publishes)
{
    /* adc_process() also runs from the DMA interrupt, at a rate unrelated
     * to the task tick.  The hold must therefore be driven purely by its
     * own elapsed-time accounting and must not assume one publish per
     * tick. */
    peak_reset();

    /* Publish ten times inside one 50 ms window, then tick once. */
    for (int i = 0; i < 10; i++) {
        seed_coupler2(40000U, 0U);
        adc_process();
    }
    test_wall_clock_advance(50U);
    adc_peak_hold_tick();
    int32_t burst = g_adc_data.fwd2_peak_mW;
    ASSERT_GT(burst, 0);

    peak_reset();
    feed_coupler2(40000U, 0U, 50U);

    /* Same data, same elapsed time -> same hold. */
    ASSERT_EQ(g_adc_data.fwd2_peak_mW, burst);
}

/*============================================================================
 *  Suite table
 *==========================================================================*/

const test_entry_t g_suite_user_adc[] = {
    TEST_ENTRY(adc_init_rearms_calibration_and_blocks_rf),
    TEST_ENTRY(im_calibration_succeeds_nominal),
    TEST_ENTRY(im_calibration_succeeds_at_upper_tolerance_edge),
    TEST_ENTRY(im_calibration_succeeds_at_lower_tolerance_edge),
    TEST_ENTRY(im_calibration_averages_sampled_window),
    TEST_ENTRY(im_calibration_short_circuits_once_calibrated),
    TEST_ENTRY(im_calibration_retries_three_times_then_alarms),
    TEST_ENTRY(im_calibration_recovers_on_second_pass),
    TEST_ENTRY(stabilisation_blocks_the_first_second_entirely),
    TEST_ENTRY(stabilisation_holds_clear_fault_low_for_the_whole_window),
    TEST_ENTRY(stabilisation_arms_only_after_the_calibration_passes),
    TEST_ENTRY(stabilisation_releases_clear_fault_when_it_arms),
    TEST_ENTRY(stabilisation_holds_clear_fault_low_when_calibration_fails),
    TEST_ENTRY(stabilisation_stays_armed_and_is_idempotent),
    TEST_ENTRY(stabilisation_restarts_when_adc_init_reruns),
    TEST_ENTRY(adc_process_averages_the_rolling_dma_window),
    TEST_ENTRY(adc_process_publishes_all_engineering_units),
    TEST_ENTRY(adc_process_derives_swr_from_fwd2_and_rev2),
    TEST_ENTRY(adc_process_increments_sequence_counter),
    TEST_ENTRY(peak_hold_attacks_instantly_to_the_new_maximum),
    TEST_ENTRY(peak_hold_releases_by_one_time_constant),
    TEST_ENTRY(peak_hold_release_depends_on_elapsed_time_not_on_tick_count),
    TEST_ENTRY(peak_hold_never_falls_below_a_rising_input),
    TEST_ENTRY(peak_hold_tracks_forward_and_reverse_independently),
    TEST_ENTRY(peak_hold_ignores_the_keying_state),
    TEST_ENTRY(peak_hold_is_not_reset_by_keying_edges),
    TEST_ENTRY(peak_hold_reports_a_nonzero_idle_reading),
    TEST_ENTRY(peak_hold_decays_to_zero_when_the_coupler_goes_quiet),
    TEST_ENTRY(peak_hold_survives_a_zero_interval),
    TEST_ENTRY(peak_hold_clamps_a_very_long_stall),
    TEST_ENTRY(peak_hold_gui_can_derive_a_consistent_swr),
    TEST_ENTRY(peak_hold_decays_monotonically_towards_zero),
    TEST_ENTRY(peak_hold_is_independent_of_which_context_publishes),
    TEST_SUITE_END
};