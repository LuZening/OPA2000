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
#include <math.h>    /* exp() - the expected peak-hold release is derived */

/* IM_ZERO_EXPECTED / IM_ZERO_TOLERANCE come from adc_calibration.h -
 * they are shared with adc_calibration.c, which fits the AWD2 threshold
 * to the same plausible band. */
#define IM_SAMPLE_COUNT      50
#define IM_TICKS_PER_PASS    (IM_SAMPLE_COUNT + 1)   /* 50 samples + 1 validate */

/* DAC1 / COMP1 / COMP2 handles, defined by the HAL stub.  Observed
 * directly by the comparator bring-up cases below. */
extern DAC_HandleTypeDef hdac1;
extern COMP_HandleTypeDef hcomp1;
extern COMP_HandleTypeDef hcomp2;

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
 *  adc_presettle() - the power-on silence
 *==========================================================================*/

TEST_CASE(presettle_powers_the_converters_without_taking_a_sample)
{
    /* The silence only works if the converters are up but the DMA is NOT
     * running: a started DMA would fill the buffer with the cold-conversion
     * transient, which is exactly what the silence is meant to keep out. */
    ADC1->CR = 0U;
    ADC2->CR = 0U;

    adc_presettle();

    ASSERT_TRUE((ADC1->CR & ADC_CR_ADEN) != 0U);
    ASSERT_TRUE((ADC2->CR & ADC_CR_ADEN) != 0U);
    ASSERT_TRUE((ADC1->CR & ADC_CR_ADVREGEN) != 0U);
    ASSERT_TRUE((ADC2->CR & ADC_CR_ADVREGEN) != 0U);

    /* No conversion has been requested: neither ADSTART nor JADSTART is
     * set, so the hardware is idle. */
    ASSERT_EQ(ADC1->CR & (1UL << 2), 0U);   /* ADSTART */
    ASSERT_EQ(ADC2->CR & (1UL << 2), 0U);   /* ADSTART */
    ASSERT_EQ(ADC2->CR & (1UL << 3), 0U);   /* JADSTART */

    /* The silence window is long enough to cover the reference settling
     * and still short enough not to delay the operator. */
    ASSERT_GE(ADC_PRESETTLE_MS, 50U);
    ASSERT_LE(ADC_PRESETTLE_MS, 500U);

    ADC1->CR = 0U;
    ADC2->CR = 0U;
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
    /* The calibration is a MEASUREMENT step.  It must not touch the
     * transmit permission - g_allow_rf is the power-on path's and
     * protection.c's business. */
    ASSERT_FALSE(g_allow_rf);
}

TEST_CASE(im_calibration_succeeds_at_upper_tolerance_edge)
{
    im_reset();
    im_run_pass((uint16_t)(IM_ZERO_EXPECTED + IM_ZERO_TOLERANCE));

    ASSERT_EQ(g_config.im_calibrated, 1);
    ASSERT_FALSE(g_allow_rf);
    ASSERT_FALSE(g_alert_flag);
}

TEST_CASE(im_calibration_succeeds_at_lower_tolerance_edge)
{
    im_reset();
    im_run_pass((uint16_t)(IM_ZERO_EXPECTED - IM_ZERO_TOLERANCE));

    ASSERT_EQ(g_config.im_calibrated, 1);
    ASSERT_FALSE(g_allow_rf);
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

TEST_CASE(im_calibration_retries_forever_and_never_alarms)
{
    const uint16_t bad = (uint16_t)(IM_ZERO_EXPECTED + 3000);

    im_reset();

    /* A zero-offset that will not settle is a warm-up artefact far more
     * often than a hardware failure, so it no longer raises an alert after
     * a fixed number of attempts - it keeps sampling. */
    for (int pass = 0; pass < 20; pass++) {
        im_run_pass(bad);
        ASSERT_FALSE(g_config.im_calibrated);
        ASSERT_FALSE(g_alert_flag);    /* and it never raises an alarm */
    }
    ASSERT_EQ(g_alert_active, ALERT_NONE);

    /* Still trying: a later good pass must be able to succeed. */
    im_run_pass(IM_ZERO_EXPECTED);
    ASSERT_TRUE(g_config.im_calibrated);
    ASSERT_FALSE(g_allow_rf);   /* the calibration never grants it */
}

TEST_CASE(im_calibration_recovers_on_second_pass)
{
    im_reset();

    im_run_pass((uint16_t)(IM_ZERO_EXPECTED + 3000));
    ASSERT_FALSE(g_config.im_calibrated);

    im_run_pass(IM_ZERO_EXPECTED);
    ASSERT_EQ(g_config.im_calibrated, 1);
    ASSERT_FALSE(g_allow_rf);
    ASSERT_FALSE(g_alert_flag);
}

/*============================================================================
 *  Sensor model detection, inside the zero-offset window
 *
 *  The model is not asked for separately: the sensor's resting output IS
 *  its zero point, and the voltage that resting point sits at is what
 *  tells the two families apart.  So the identification happens in the
 *  same 50-sample window that measures the offset.
 *==========================================================================*/

TEST_CASE(im_calibration_identifies_the_bidirectional_sensor)
{
    im_reset();
    ASSERT_EQ(g_config.im_sensor_type, IM_SENSOR_UNKNOWN);

    im_run_pass(IM_ZERO_EXPECTED);

    ASSERT_EQ(g_config.im_calibrated, 1);
    ASSERT_EQ(g_config.im_sensor_type, IM_SENSOR_BIDIRECTIONAL);
    ASSERT_EQ(g_config.im_zero_offset_adc, IM_ZERO_EXPECTED);
}

TEST_CASE(im_calibration_identifies_the_unidirectional_sensor)
{
    /* 0.3 V is 7864 codes on a 2.5 V reference; 5000 codes is 190 mV,
     * a typical resting level for a one-way sensor. */
    const uint16_t uni_rest = 5000U;

    im_reset();
    im_run_pass(uni_rest);

    ASSERT_EQ(g_config.im_calibrated, 1);
    ASSERT_EQ(g_config.im_sensor_type, IM_SENSOR_UNIDIRECTIONAL);
    ASSERT_EQ(g_config.im_zero_offset_adc, uni_rest);
}

TEST_CASE(im_calibration_never_accepts_an_unrecognised_sensor)
{
    /* Somewhere between the two windows: a disconnected sensor, a broken
     * one, or a node stuck at an unexpected voltage.  It must NOT be
     * mistaken for a working sensor, so the calibration keeps sampling and
     * the current protection stays parked. */
    const uint16_t neither = 20000U;

    im_reset();

    for (int pass = 0; pass < 5; pass++) {
        im_run_pass(neither);
        ASSERT_FALSE(g_config.im_calibrated);
        ASSERT_EQ(g_config.im_sensor_type, IM_SENSOR_UNKNOWN);
    }

    /* A good reading later still works. */
    im_run_pass(IM_ZERO_EXPECTED);
    ASSERT_EQ(g_config.im_calibrated, 1);
    ASSERT_EQ(g_config.im_sensor_type, IM_SENSOR_BIDIRECTIONAL);
}

TEST_CASE(im_calibration_accepts_a_unidirectional_sensor_resting_at_ground)
{
    /* A one-way sensor whose output really does sit at 0 V.  It is a
     * legitimate sensor, so it must be recognised as one and its resting
     * point measured - the current conversion and the trip point are both
     * "the measured zero plus the limit", so they are correct wherever
     * the zero turned out to be. */
    im_reset();
    im_run_pass(0U);

    ASSERT_EQ(g_config.im_calibrated, 1);
    ASSERT_EQ(g_config.im_sensor_type, IM_SENSOR_UNIDIRECTIONAL);
    ASSERT_EQ(g_config.im_zero_offset_adc, 0U);
}

/*============================================================================
 *  Power-on stabilisation gate (adc_stabilisation_tick / _done)
 *
 *  The 100 ms power-on silence lives OUTSIDE this function (adc_presettle()
 *  + vTaskDelay in the task).  What this function owns is the moment the
 *  protection chain goes live and the amplifier becomes transmittable.
 *
 *  Two things are deliberately INDEPENDENT here:
 *
 *    - the amplifier may be keyed (g_allow_rf), and
 *    - the main-current zero offset is calibrated.
 *
 *  The cases below pin that separation, because getting it wrong in the
 *  other direction is what turned a drifting current sensor into a bricked
 *  amplifier.
 *
 *  A "tick" here is one ADC-task iteration, i.e. 50 ms.
 *==========================================================================*/

/* Tick the stabilisation machine until it reports ARMED, so a case does not
 * have to care how many settling ticks the comparators need.  Returns the
 * tick count, or 0 if it never arms (which would be a failure). */
static uint32_t stab_tick_until_armed(void)
{
    for (uint32_t i = 0; i < 200U; i++) {
        if (adc_stabilisation_tick()) {
            return i + 1U;
        }
    }
    return 0U;
}

/* Ticks the stabilisation machine with the IM rank already at its true
 * zero, so the background calibration can pass as soon as it has enough
 * samples.  Runs one full sampling+validation pass and returns the latest
 * stabilisation state.
 *
 * The pass is counted AFTER the gate has armed, not from the first tick:
 * the comparator settling window consumes ticks of its own, and a fixed
 * budget would silently shorten the calibration window instead of just
 * delaying it. */
static bool stab_tick_calibrating(uint16_t raw, int count)
{
    (void)stab_tick_until_armed();

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

TEST_CASE(stabilisation_arms_on_the_first_tick_after_the_silence)
{
    im_reset();

    /* The silence is over and the DMA is running, so the amplifier becomes
     * transmittable on the very first tick - the transmit permission does
     * NOT wait for the comparator settling window or for the zero-offset
     * calibration.  The zero-offset calibration has not even taken a
     * sample yet. */
    ASSERT_FALSE(adc_stabilisation_tick());
    ASSERT_TRUE(g_allow_rf);
    ASSERT_FALSE(g_config.im_calibrated);
    ASSERT_FALSE(g_alert_flag);

    /* adc_stabilisation_done() reports the narrower condition: the
     * comparators are enabled, so the analog protection chain is fully
     * live.  It comes true at the end of the settling window. */
    ASSERT_FALSE(adc_stabilisation_done());
    ASSERT_TRUE(stab_tick_until_armed() > 0U);
    ASSERT_TRUE(adc_stabilisation_done());
    ASSERT_FALSE(g_config.im_calibrated);
    ASSERT_FALSE(g_alert_flag);
}

TEST_CASE(stabilisation_arms_even_when_the_offset_will_never_settle)
{
    const uint16_t bad = (uint16_t)(IM_ZERO_EXPECTED + 3000);

    im_reset();

    /* A current sensor that reads nonsense must NOT be able to brick the
     * amplifier.  The protection chain goes live, the amplifier may be
     * keyed, and the only thing lost is the current reading - which
     * stays at 0 (uncalibrated) and shows "--" on the panel. */
    for (int pass = 0; pass < 20; pass++) {
        im_feed(bad);
        (void)adc_stabilisation_tick();
    }

    ASSERT_TRUE(stab_tick_until_armed() > 0U);
    ASSERT_TRUE(adc_stabilisation_done());
    ASSERT_TRUE(g_allow_rf);
    ASSERT_FALSE(g_config.im_calibrated);
    /* Still no alarm: a sensor that will not settle is a warm-up
     * artefact, and alarming on it would only train the operator to
     * ignore alarms. */
    ASSERT_FALSE(g_alert_flag);
    ASSERT_EQ(g_alert_active, ALERT_NONE);
}

TEST_CASE(stabilisation_releases_clear_fault_when_it_arms)
{
    im_reset();
    stub_gpio_set(CLEAR_FAULTX_OC_GPIO_Port, CLEAR_FAULTX_OC_Pin, 0U);

    ASSERT_TRUE(stab_tick_until_armed() > 0U);

    /* Released, not left low: the latch must work normally from now on. */
    ASSERT_EQ(clear_faultx_level(), 1);
}

TEST_CASE(stabilisation_calibrates_in_the_background_after_arming)
{
    im_reset();

    /* Transmittable on the first tick, with the calibration still empty. */
    ASSERT_FALSE(adc_stabilisation_tick());
    ASSERT_TRUE(g_allow_rf);
    ASSERT_FALSE(g_config.im_calibrated);

    /* The calibration then runs in the background and succeeds on its own. */
    (void)stab_tick_calibrating(IM_ZERO_EXPECTED, IM_TICKS_PER_PASS);
    ASSERT_EQ(g_config.im_calibrated, 1);
    ASSERT_EQ(g_config.im_zero_offset_adc, IM_ZERO_EXPECTED);
    /* And it still did not touch the transmit permission. */
    ASSERT_TRUE(g_allow_rf);
    ASSERT_FALSE(g_alert_flag);
}

TEST_CASE(stabilisation_stays_armed_and_is_idempotent)
{
    im_reset();
    ASSERT_TRUE(stab_tick_calibrating(IM_ZERO_EXPECTED, IM_TICKS_PER_PASS));
    ASSERT_TRUE(stab_tick_until_armed() > 0U);
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
    ASSERT_TRUE(g_allow_rf);
    ASSERT_FALSE(g_alert_flag);
}

/*============================================================================
 *  The current protection follows the zero-offset calibration
 *
 *  The zero-offset calibration is a measurement step, but the main-current
 *  protection is expressed RELATIVE to its result - AWD2's threshold and
 *  COMP2's reference voltage are both "zero offset plus the limit".  So
 *  the current protection is off until the calibration has produced a
 *  usable number, and comes alive when it does.
 *
 *  Everything else is unaffected: the amplifier may be keyed, and the
 *  other protections stay armed.
 *==========================================================================*/

TEST_CASE(current_protection_is_off_until_the_calibration_succeeds)
{
    im_reset();

    /* Transmittable, but the current protection is not live yet - the
     * offset has not been measured. */
    ASSERT_FALSE(adc_stabilisation_tick());
    ASSERT_TRUE(g_allow_rf);
    ASSERT_FALSE(adc_im_protection_valid());

    /* The watchdog is parked wide open, so it cannot trip on a threshold
     * that was computed from an unmeasured offset. */
    adc_awd_set_im_mA(30000);
    ASSERT_EQ(ADC1->HTR2, ADC_FULL_SCALE);

    /* The calibration now succeeds in the background. */
    (void)stab_tick_calibrating(IM_ZERO_EXPECTED, IM_TICKS_PER_PASS);
    ASSERT_EQ(g_config.im_calibrated, 1);

    /* ... and the current protection is live from now on. */
    ASSERT_TRUE(adc_im_protection_valid());
    adc_awd_set_im_mA(30000);
    ASSERT_EQ(ADC1->HTR2, IM_ZERO_EXPECTED + 20761U);
}

TEST_CASE(current_protection_stays_off_while_the_offset_never_settles)
{
    const uint16_t bad = (uint16_t)(IM_ZERO_EXPECTED + 3000);

    im_reset();
    ASSERT_FALSE(adc_stabilisation_tick());
    ASSERT_TRUE(stab_tick_until_armed() > 0U);

    for (int pass = 0; pass < 20; pass++) {
        (void)stab_tick_calibrating(bad, IM_TICKS_PER_PASS);
    }

    /* The offset never settled, so the current protection never came
     * live - but the amplifier is still keyable and nothing alarmed. */
    ASSERT_FALSE(g_config.im_calibrated);
    ASSERT_FALSE(adc_im_protection_valid());
    ASSERT_TRUE(adc_stabilisation_done());
    ASSERT_TRUE(g_allow_rf);
    ASSERT_FALSE(g_alert_flag);
}

TEST_CASE(stabilisation_restarts_when_adc_init_reruns)
{
    im_reset();
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
 *  Comparator bring-up must not produce an edge
 *
 *  COMP1 (RF overdrive) and COMP2 (main overcurrent) are the two protection
 *  sources that have no software-side gate: they fire straight from the
 *  comparator IRQ into protection_trigger_locked().  So the ONLY thing
 *  standing between a power-up transient and a latched P0 shutdown is the
 *  moment at which HAL_COMP_Start_IT() runs.
 *
 *  The bug this pins: boot_arm_thresholds() pushed the operator's REAL
 *  reference onto the DAC and HAL_COMP_Start_IT() ran a few statements later
 *  - both inside the same STAB_ARMING branch.  A comparator evaluates
 *  continuously the instant it is enabled, so with the D5/R40 peak detector
 *  still charging out of its reset floor, COMP1's very first comparison
 *  already saw input > reference and latched ALERT_OVERDRIVE at every cold
 *  boot.  The amplifier could then not be keyed until the operator dismissed
 *  a fault that never happened.
 *
 *  The fix keeps the comparators DISABLED through the settling window and
 *  enables them only once the analog side has had time to reach its real
 *  resting level.  A comparator that has never been enabled cannot produce
 *  an edge, whatever its reference says.
 *==========================================================================*/

/* DAC CH1 data holding register - what COMP1 would see on its inverting
 * input.  Mirrors the real HAL, where SetValue() writes DHR12R and Start()
 * is what puts the channel on the pin.
 *
 * Asserted in DAC codes rather than millivolts: the round trip is quantised
 * (4096 codes over 3300 mV), so 2000 mV does not come back out as exactly
 * 2000 and a test that demands it would be asserting the DAC's resolution
 * rather than the bring-up order. */
static uint32_t dac_code1(void) { return hdac1.Instance->DHR12R1; }

static bool comp1_running(void) { return (hcomp1.Instance->CR & 0x4U) != 0U; }
static bool comp2_running(void) { return (hcomp2.Instance->CR & 0x4U) != 0U; }

/* The stub's EN bit is write-only - nothing clears it between cases - so a
 * case that needs a genuine "not enabled yet" has to say so.  That matches
 * the real peripheral too: nothing on the power-on path calls
 * HAL_COMP_Stop(), so within one power cycle the bit genuinely stays set
 * from the arming tick onwards. */
static void comp_force_stopped(void)
{
    hcomp1.Instance->CR &= ~0x4U;
    hcomp2.Instance->CR &= ~0x4U;
}

TEST_CASE(the_comparators_stay_disabled_through_the_settling_window)
{
    im_reset();
    comp_force_stopped();
    g_config.comp1_ref_mV = 2000U;

    /* The arming tick: limits go in, but the comparators must NOT start. */
    ASSERT_FALSE(adc_stabilisation_tick());

    for (int i = 0; i < 19; i++) {
        ASSERT_FALSE(adc_stabilisation_tick());
        ASSERT_FALSE(comp1_running());
        ASSERT_FALSE(comp2_running());
    }

    /* The window closes on the next tick, and only now do they start. */
    ASSERT_TRUE(adc_stabilisation_tick());
    ASSERT_TRUE(comp1_running());
    ASSERT_TRUE(comp2_running());
}

TEST_CASE(the_comparators_enable_against_an_already_settled_input)
{
    im_reset();
    comp_force_stopped();
    g_config.comp1_ref_mV = 2000U;

    /* The reference is programmed early - at the arming tick - because a
     * reference on a disabled comparator is inert.  That gives the analog
     * side a whole settling window with the threshold already in place,
     * so the first comparison the comparator makes is against an input
     * that has finished charging. */
    ASSERT_FALSE(adc_stabilisation_tick());
    uint32_t armed_code = dac_code1();
    ASSERT_TRUE(armed_code > 0U);
    ASSERT_NE(armed_code, DAC_FULL_SCALE);   /* not the unreachable rail */
    ASSERT_FALSE(comp1_running());

    for (int i = 0; i < 20; i++) {
        (void)adc_stabilisation_tick();
    }

    ASSERT_TRUE(comp1_running());
    ASSERT_EQ(dac_code1(), armed_code);      /* unchanged across the enable */
}

TEST_CASE(no_comparator_edge_can_latch_before_the_silence_ends)
{
    im_reset();
    comp_force_stopped();

    /* Fresh from adc_init(): nothing armed, and critically no comparator
     * enabled - so protection_comp1_isr() / protection_comp2_isr() have
     * nothing to report even if an edge were delivered. */
    ASSERT_FALSE(adc_stabilisation_done());
    ASSERT_FALSE(comp1_running());
    ASSERT_FALSE(comp2_running());

    (void)adc_stabilisation_tick();
    ASSERT_FALSE(adc_stabilisation_done());   /* still inside the window */
    ASSERT_FALSE(comp1_running());

    for (int i = 0; i < 20; i++) {
        (void)adc_stabilisation_tick();
    }
    ASSERT_TRUE(adc_stabilisation_done());
    ASSERT_TRUE(comp1_running());
}

/*============================================================================
 *  adc_process()
 *
 *  Two modes:
 *
 *    - DEBUG:   the most-recent DMA row, no averaging.  Useful when
 *               stepping through the ISR; the bar graph would dance
 *               but the developer can see exactly what landed there.
 *
 *    - RELEASE: the rolling mean across the whole 16-row window.
 *               Trades a few cycles of latency for a much quieter
 *               number on the bar graph.
 *
 *  The expected value is decided by the build mode.
 *==========================================================================*/

TEST_CASE(adc_process_averages_or_picks_the_latest)
{
    im_reset();
    opa2000_init();

    /* 15 rows at 100 codes, newest row at 30000:
     *   DEBUG:    report 30000 (the last row).
     *   RELEASE:  report (15*100 + 30000)/16 = 1968 (the rolling mean). */
    for (uint16_t row = 0U; row < ADC_DMA_BUFFER_SIZE; row++) {
        for (uint8_t r = 0U; r < ADC1_RANK_COUNT; r++) {
            adc1_dma_buf[row * ADC1_RANK_COUNT + r] = 100U;
        }
    }
    adc1_dma_buf[(ADC_DMA_BUFFER_SIZE - 1U) * ADC1_RANK_COUNT + ADC1_RANK_VM] = 30000U;

    adc_process();

#if defined(DEBUG)
    ASSERT_EQ(g_adc_data.adc1_raw[ADC1_RANK_VM], 30000U);
    ASSERT_EQ(g_adc_data.vm_mV, adc1_raw_to_vm_mV(30000U));
#else
    ASSERT_EQ(g_adc_data.adc1_raw[ADC1_RANK_VM], 1968U);
    ASSERT_EQ(g_adc_data.vm_mV, adc1_raw_to_vm_mV(1968U));
#endif
}

TEST_CASE(the_four_coupler_power_channels_report_the_window_peak)
{
    im_reset();
    opa2000_init();

    /* Quiet window carrying RF on ONE row only, plus a slow channel (IM) on
     * that same row.  A mean would report code/16 for everything; the four
     * coupler ranks must report the peak and the slow channel must keep the
     * mean.  That single difference is the whole contract: the power
     * readouts (and the coupler protection that compares them) answer "how
     * much RF was there", not "how much was there on average". */
    for (uint16_t row = 0U; row < ADC_DMA_BUFFER_SIZE; row++) {
        for (uint8_t r = 0U; r < ADC1_RANK_COUNT; r++) {
            adc1_dma_buf[row * ADC1_RANK_COUNT + r] = 0U;
        }
    }
    adc1_dma_buf[ADC1_RANK_FWD1] = 30000U;
    adc1_dma_buf[ADC1_RANK_REV1] = 3000U;
    adc1_dma_buf[ADC1_RANK_FWD2] = 40000U;
    adc1_dma_buf[ADC1_RANK_REV2] = 4000U;
    adc1_dma_buf[ADC1_RANK_IM]   = 3200U;

    adc_process();

#if !defined(DEBUG)
    /* Peak: the full reading, not 1/16 of it. */
    ASSERT_EQ(g_adc_data.adc1_raw[ADC1_RANK_FWD1], 30000U);
    ASSERT_EQ(g_adc_data.adc1_raw[ADC1_RANK_REV1], 3000U);
    ASSERT_EQ(g_adc_data.adc1_raw[ADC1_RANK_FWD2], 40000U);
    ASSERT_EQ(g_adc_data.adc1_raw[ADC1_RANK_REV2], 4000U);
    ASSERT_EQ(g_adc_data.fwd2_mW, adc1_raw_to_fwd2_mW(40000U));
    ASSERT_EQ(g_adc_data.rev2_mW, adc1_raw_to_rev2_mW(4000U));
    ASSERT_EQ(g_adc_data.fwd1_mW, adc1_raw_to_fwd1_mW(30000U));
    ASSERT_EQ(g_adc_data.rev1_mW, adc1_raw_to_rev1_mW(3000U));

    /* Everything else stays on the rolling mean - here 3200/16 = 200. */
    ASSERT_EQ(g_adc_data.adc1_raw[ADC1_RANK_IM], 200U);
    ASSERT_EQ(g_adc_data.im_mA, adc1_raw_to_im_mA(200U));
#endif
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

    /* 400 W forward and 25 W reflected -> |G| = sqrt(25/400) = 0.25
     * -> SWR = (1+0.25)/(1-0.25) = 1.667.
     *
     * The raw codes are DERIVED from those powers through the public
     * inverse conversion, not hard-coded.  A hard-coded code only means
     * "400 W" for as long as the front-end curve stays exactly as it was;
     * when the curve changes - and it has, the OPA2000 divider network is
     * not the OPA1000 one - the fixture silently starts testing a
     * different pair of powers and the assertion below fails for a reason
     * that has nothing to do with the code under test. */
    uint16_t fwd2_code = adc_dc_mW_to_raw(400000U);
    uint16_t rev2_code = adc_dc_mW_to_raw(25000U);

    for (uint16_t row = 0U; row < ADC_DMA_BUFFER_SIZE; row++) {
        for (uint8_t r = 0U; r < ADC1_RANK_COUNT; r++) {
            adc1_dma_buf[row * ADC1_RANK_COUNT + r] = 0U;
        }
        adc1_dma_buf[row * ADC1_RANK_COUNT + ADC1_RANK_FWD2] = fwd2_code;
        adc1_dma_buf[row * ADC1_RANK_COUNT + ADC1_RANK_REV2] = rev2_code;
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
 *  Contract: instantaneous attack, exponential release with tau = PEAK_TAU_MS,
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

/* Expected fraction of the held value left after dt_ms, from the published
 * time constant.  Deriving it keeps these cases honest when tau moves. */
static int32_t peak_expected_after(int32_t held, uint32_t dt_ms)
{
    double tau_s = (double)PEAK_TAU_MS / 1000.0;
    return (int32_t)((double)held * exp(-((double)dt_ms / 1000.0) / tau_s));
}

TEST_CASE(peak_hold_releases_by_one_time_constant)
{
    peak_reset();

    feed_coupler2(40000U, 0U, 50U);
    int32_t held = g_adc_data.fwd2_peak_mW;
    ASSERT_TRUE(held > 0);

    /* One tau later the hold must be at 1/e = 36.8 %. */
    feed_coupler2(0U, 0U, PEAK_TAU_MS);
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

    /* One tau in a single step must equal one tau taken as ten tenths: the
     * curve is a function of elapsed time, not of how often the task
     * happens to run.  Kept inside one clamp window on purpose - a step
     * longer than PEAK_MAX_STEP_MS is a different contract, covered below. */
    feed_coupler2(0U, 0U, PEAK_TAU_MS);
    int32_t one_step = g_adc_data.fwd2_peak_mW;

    peak_reset();
    feed_coupler2(40000U, 0U, 50U);
    ASSERT_EQ(g_adc_data.fwd2_peak_mW, held);
    for (int i = 0; i < 10; i++) {
        feed_coupler2(0U, 0U, PEAK_TAU_MS / 10U);
    }
    int32_t ten_steps = g_adc_data.fwd2_peak_mW;

    /* Both land on e^-1.  The int32_t store truncates once per step, so ten
     * steps may sit a few milliwatts lower than one. */
    int32_t expect = peak_expected_after(held, PEAK_TAU_MS);
    ASSERT_LE(one_step, expect);
    ASSERT_GT(one_step, expect - held / 1000);
    ASSERT_LE(ten_steps, expect);
    ASSERT_GT(ten_steps, expect - held / 1000);
}

TEST_CASE(peak_hold_never_falls_below_a_rising_input)
{
    peak_reset();

    feed_coupler2(40000U, 0U, 50U);
    int32_t held = g_adc_data.fwd2_peak_mW;

    /* Two tau of decay puts the hold well under the new sample, so the
     * sample wins.  Two separate one-tau steps, not one 2*tau step, so the
     * PEAK_MAX_STEP_MS clamp never enters the picture. */
    feed_coupler2(0U, 0U, PEAK_TAU_MS);
    feed_coupler2(0U, 0U, PEAK_TAU_MS);
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
    /* Six 100 ms steps = 600 ms, i.e. 0.12 tau with the published constant.
     * The tolerance is 0.5 % of the held value, which absorbs the float32
     * rounding of the hold (kept as int32_t, so each step also truncates). */
    int32_t expected = peak_expected_after(held, 600U);
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
     * would keep a bar permanently lit.  One step is clamped to
     * PEAK_MAX_STEP_MS, so this takes several passes (each = 2 tau). */
    for (int i = 0; i < 10; i++) {
        feed_coupler2(0U, 0U, PEAK_MAX_STEP_MS);
    }
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

    /* A 30 s stall is clamped to PEAK_MAX_STEP_MS, so the result must be
     * exactly what one clamped step gives - the 30 s that actually elapsed
     * must NOT drive the decay, or a debugger pause would produce a jump no
     * runtime path can explain. */
    feed_coupler2(0U, 0U, 30000U);
    int32_t stalled = g_adc_data.fwd2_peak_mW;

    peak_reset();
    feed_coupler2(40000U, 0U, 50U);
    ASSERT_EQ(g_adc_data.fwd2_peak_mW, held);
    feed_coupler2(0U, 0U, PEAK_MAX_STEP_MS);

    ASSERT_EQ(g_adc_data.fwd2_peak_mW, stalled);
    /* And that clamped step is a plain one-clamp release, not zero: the
     * expected value follows the published tau, so this stays correct
     * whatever PEAK_TAU_MS is set to. */
    int32_t expect = peak_expected_after(held, PEAK_MAX_STEP_MS);
    int32_t tol    = held / 1000 + 1;
    ASSERT_GT(stalled, expect - tol);
    ASSERT_LT(stalled, expect + tol);
}

TEST_CASE(peak_hold_gui_can_derive_a_consistent_swr)
{
    peak_reset();

    /* 400 W forward, 25 W reflected -> SWR 1.667.  The GUI builds its
     * own SWR from the two holds; it must stay in the same band as the
     * instantaneous one, which is what "three bars agree" means.  The
     * codes come from the public inverse conversion so the fixture states
     * its intent in watts, not in raw counts tied to one particular front
     * end. */
    feed_coupler2(adc_dc_mW_to_raw(400000U), adc_dc_mW_to_raw(25000U), 50U);

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
 *  ADC2 injected conversions - the interrupt path
 *
 *  The injected sequence is configured software-start with no trigger, so
 *  it runs once and parks.  Re-arming every 100 ms is what keeps the NTC
 *  readings fresh, and that re-arm must be the _IT variant so the JEOS
 *  interrupt fires - the non-_IT variant starts the sequence but never
 *  enables the interrupt, which means the JDRs are never read out and the
 *  NTC protection path is dead code.
 *
 *  The HAL stub exposes counters for both variants; the test that follows
 *  asserts the firmware reaches for _IT, not the polling one.  If somebody
 *  reverts back to the polling variant, this case fails loudly and
 *  immediately, because that is the failure mode the operator would
 *  otherwise only discover at the bench with an over-temperature event.
 *==========================================================================*/

TEST_CASE(injected_rearm_uses_the_it_variant)
{
    test_injected_reset_counters();
    test_wall_clock_reset();
    test_wall_clock_advance(1000U);   /* sit well outside any 100 ms window */
    adc_init();                        /* s_inj_last_tick now 0, "now" = 1000 */

    /* now - 0 = 1000, which is NOT < 100, so the first call goes through.
     * Subsequent ticks within the period must be no-ops. */
    adc_injected_tick();

    /* The bug being guarded against is exactly the polling variant, so
     * this is the line that catches the regression.  _IT count must be
     * 1, the polling count must be 0. */
    ASSERT_EQ(test_injected_start_it_calls(),   1U);
    ASSERT_EQ(test_injected_start_poll_calls(), 0U);

    /* Within the 100 ms window: no second fire. */
    test_wall_clock_advance(50U);
    adc_injected_tick();
    ASSERT_EQ(test_injected_start_it_calls(),   1U);
    ASSERT_EQ(test_injected_start_poll_calls(), 0U);

    /* Past the window: a second fire is allowed, again via _IT. */
    test_wall_clock_advance(100U);
    adc_injected_tick();
    ASSERT_EQ(test_injected_start_it_calls(),   2U);
    ASSERT_EQ(test_injected_start_poll_calls(), 0U);
}

/*============================================================================
 *  Suite table
 *==========================================================================*/

const test_entry_t g_suite_user_adc[] = {
    TEST_ENTRY(adc_init_rearms_calibration_and_blocks_rf),
    TEST_ENTRY(presettle_powers_the_converters_without_taking_a_sample),
    TEST_ENTRY(im_calibration_succeeds_nominal),
    TEST_ENTRY(im_calibration_succeeds_at_upper_tolerance_edge),
    TEST_ENTRY(im_calibration_succeeds_at_lower_tolerance_edge),
    TEST_ENTRY(im_calibration_averages_sampled_window),
    TEST_ENTRY(im_calibration_short_circuits_once_calibrated),
    TEST_ENTRY(im_calibration_retries_forever_and_never_alarms),
    TEST_ENTRY(im_calibration_recovers_on_second_pass),
    TEST_ENTRY(im_calibration_identifies_the_bidirectional_sensor),
    TEST_ENTRY(im_calibration_identifies_the_unidirectional_sensor),
    TEST_ENTRY(im_calibration_never_accepts_an_unrecognised_sensor),
    TEST_ENTRY(im_calibration_accepts_a_unidirectional_sensor_resting_at_ground),
    TEST_ENTRY(stabilisation_arms_on_the_first_tick_after_the_silence),
    TEST_ENTRY(stabilisation_arms_even_when_the_offset_will_never_settle),
    TEST_ENTRY(stabilisation_releases_clear_fault_when_it_arms),
    TEST_ENTRY(stabilisation_calibrates_in_the_background_after_arming),
    TEST_ENTRY(stabilisation_stays_armed_and_is_idempotent),
    TEST_ENTRY(current_protection_is_off_until_the_calibration_succeeds),
    TEST_ENTRY(current_protection_stays_off_while_the_offset_never_settles),
    TEST_ENTRY(stabilisation_restarts_when_adc_init_reruns),
    TEST_ENTRY(no_comparator_edge_can_latch_before_the_silence_ends),
    TEST_ENTRY(the_comparators_stay_disabled_through_the_settling_window),
    TEST_ENTRY(the_comparators_enable_against_an_already_settled_input),
    TEST_ENTRY(adc_process_averages_or_picks_the_latest),
    TEST_ENTRY(the_four_coupler_power_channels_report_the_window_peak),
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
    TEST_ENTRY(injected_rearm_uses_the_it_variant),
    TEST_SUITE_END
};