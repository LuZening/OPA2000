/**
 * @file    test_awd_switch.c
 * @brief   Tests for the USE_AWD compile-time switch.
 *
 * @details This file is compiled into a SEPARATE host binary
 *          (host_tests_noawd) whose copy of adc_calibration.c was built
 *          with -DOPA2000_NO_AWD, i.e. with the ADC analog watchdogs
 *          switched out of the build.
 *
 *          The main host_tests binary cannot cover this: the switch is a
 *          COMPILE-TIME one, so the behaviour under test only exists in a
 *          build that was configured differently.  Two binaries, one
 *          source tree.
 *
 *          What is being asserted:
 *            - the thresholds are parked at full scale, so the watchdog
 *              physically cannot trip (this is the half that stops the
 *              hardware);
 *            - the lower thresholds are still zero, so a falling-edge
 *              crossing cannot latch either;
 *            - everything ELSE about the module still works - the switch
 *              must only affect the watchdog thresholds, not the current
 *              conversion or the COMP2 reference derivation.
 *
 *          The converse (the watchdogs ARE live in the default build) is
 *          asserted by test_adc_calibration.c, which links the default
 *          adc_calibration.c.
 */

#include "test_harness.h"
#include "adc_calibration.h"
#include "opa2000.h"
#include "stm32h7xx_hal_stub.h"

/* The whole point of this binary: the watchdogs must be OUT here.  If the
 * build flags are wrong, every assertion below would still "pass" against
 * a live watchdog, so the switch state is checked first and loudly. */
#if defined(USE_AWD)
#error "This test binary must be built with OPA2000_NO_AWD (watchdogs OUT)"
#endif

/*============================================================================
 *  The switch state itself
 *==========================================================================*/

TEST_CASE(the_watchdogs_are_out_of_this_build)
{
    /* USE_AWD must not be defined here.  The #error above already fails
     * the compile, so reaching this line means the configuration is what
     * it should be.  Assert it anyway - it documents the intent and it
     * catches a build that reaches the file through a different header
     * path. */
#ifdef USE_AWD
    ASSERT_TRUE(0);
#else
    ASSERT_TRUE(1);
#endif
}

/*============================================================================
 *  The thresholds are parked where they cannot trip
 *==========================================================================*/

TEST_CASE(awd1_is_parked_at_full_scale_when_the_watchdogs_are_out)
{
    opa2000_init();

    /* The live build would compute a threshold from the RF-in curve.
     * Here it must be full scale regardless of the limit asked for. */
    adc_awd_set_rf_in_mW(2000000);
    ASSERT_EQ(ADC1->HTR1, ADC_FULL_SCALE);
    ASSERT_EQ(ADC1->LTR1, 0);
}

TEST_CASE(awd2_is_parked_at_full_scale_when_the_watchdogs_are_out)
{
    /* A fully calibrated sensor, so the live build WOULD fit a real
     * threshold.  This is the case that matters: a stale or wrong zero
     * offset is exactly what put AWD2 in the wrong place, and with the
     * watchdogs out the threshold must not exist at all. */
    opa2000_init();
    g_config.im_calibrated      = 1U;
    g_config.im_zero_offset_adc = 43254U;
    g_config.im_sensor_type     = IM_SENSOR_BIDIRECTIONAL;

    adc_awd_set_im_mA(30000);               /* 30 A */
    ASSERT_EQ(ADC1->HTR2, ADC_FULL_SCALE);
    ASSERT_EQ(ADC1->LTR2, 0);
}

TEST_CASE(awd3_is_parked_at_full_scale_when_the_watchdogs_are_out)
{
    opa2000_init();

    /* Even with a bench-calibrated coupler, which in the live build
     * would produce a real threshold. */
    g_config.rev1.gain_uW_per_code = 200U;
    g_config.rev1.offset_mW        = 0;

    adc_awd_set_rev1_mW(200);
    ASSERT_EQ(ADC1->HTR3, ADC_FULL_SCALE);
    ASSERT_EQ(ADC1->LTR3, 0);
}

/*============================================================================
 *  The switch must not leak into anything else
 *==========================================================================*/

TEST_CASE(the_current_conversion_still_works_with_the_watchdogs_out)
{
    opa2000_init();
    g_config.im_calibrated      = 1U;
    g_config.im_zero_offset_adc = 43254U;
    g_config.im_sensor_type     = IM_SENSOR_BIDIRECTIONAL;

    /* The reading is what the operator sees; it must not depend on the
     * watchdog switch at all. */
    ASSERT_EQ(adc1_raw_to_im_mA(43254U), 0);
    /* 6920 codes = 10 A, but the inverse conversion is integer-only:
     * 6920 * 25000000 / 17301240 = 9999.9 mA, so it truncates to 9999. */
    ASSERT_EQ(adc1_raw_to_im_mA(43254U + 6920U), 9999);
}

TEST_CASE(the_comp2_reference_is_still_derived_with_the_watchdogs_out)
{
    opa2000_init();
    g_config.im_calibrated      = 1U;
    g_config.im_zero_offset_adc = 43254U;
    g_config.im_sensor_type     = IM_SENSOR_BIDIRECTIONAL;
    g_config.band_im_max_mA     = 30000U;

    /* COMP2 is a separate peripheral from the watchdogs: switching AWD
     * out must not disturb the comparator reference derivation. */
    uint32_t ref = adc_im_comp2_ref_mV(2442U, 30000U);
    ASSERT_TRUE(ref > 2000U);
    ASSERT_TRUE(ref <= 2500U);
}

/*============================================================================
 *  Suite table
 *==========================================================================*/

const test_entry_t g_suite_awd_switch[] = {
    TEST_ENTRY(the_watchdogs_are_out_of_this_build),
    TEST_ENTRY(awd1_is_parked_at_full_scale_when_the_watchdogs_are_out),
    TEST_ENTRY(awd2_is_parked_at_full_scale_when_the_watchdogs_are_out),
    TEST_ENTRY(awd3_is_parked_at_full_scale_when_the_watchdogs_are_out),
    TEST_ENTRY(the_current_conversion_still_works_with_the_watchdogs_out),
    TEST_ENTRY(the_comp2_reference_is_still_derived_with_the_watchdogs_out),
    TEST_SUITE_END
};
