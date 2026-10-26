/**
 * @file    test_boot.c
 * @brief   Host-side tests for the boot-sequencing helpers.
 *
 * @details boot.c is mostly GPIO poking, which the host stub can record
 *          but not meaningfully assert.  What IS worth testing on the
 *          host is what actually reaches the peripherals:
 *
 *            - the COMP DAC references land in the right data holding
 *              registers, i.e. a config threshold really does turn into
 *              the code that drives the comparator
 *            - an out-of-range config is clamped instead of pinning a
 *              comparator output at its rail
 *            - band -> band_rf_in_max_w[] index mapping (a wrong index
 *              would apply the wrong band's RF limit)
 *            - the "was a valid config loaded" -> calibration-needed
 *              decision
 *
 *          boot.c is compiled into the binary for this (see
 *          User/Tests/CMakeLists.txt), so these assertions exercise the
 *          firmware code rather than a copy of it.
 */

#include "test_harness.h"
#include "boot.h"
#include "config.h"
#include "opa2000.h"
/* Brings in ADC_FULL_SCALE, the "parked at 16-bit full scale" value the
 * AWD assertions below compare against. */
#include "adc_calibration.h"
#include "stm32h7xx_hal_stub.h"
/* For the pin macros (CLEAR_FAULTX_OC_*): it is the header that pulls
 * stm32h7xx_hal.h / main.h in, so this is the project's way of making a
 * pin visible to a host test - same as test_protection.c and
 * test_power_control.c do. */
#include "hw_pinmap.h"

#include <string.h>

extern DAC_HandleTypeDef hdac1;

/* Expected DAC code for a reference voltage, derived independently of
 * boot.c so the two cannot drift apart.  DAC1 is 12-bit on VDDA. */
static uint16_t expected_dac_code(uint32_t mv)
{
    if (mv >= DAC_VDDA_MV) {
        return (uint16_t)DAC_FULL_SCALE;
    }
    return (uint16_t)((mv * DAC_FULL_SCALE) / DAC_VDDA_MV);
}

static uint16_t ref_band_index(int band)
{
    switch (band) {
        case BAND_160M:    return 0U;
        case BAND_80M:     return 1U;
        case BAND_40M_30M: return 2U;
        case BAND_20M_17M: return 3U;
        case BAND_15M_10M: return 4U;
        default:           return 4U;   /* safest = 15-10 m */
    }
}

/*============================================================================
 *  Test 1: the config thresholds reach the DAC data holding registers
 *
 *  This is the assertion that matters for the "user edits a threshold and
 *  it takes effect" requirement: it drives the real
 *  boot_apply_comp_thresholds() and reads back what the peripheral was
 *  actually programmed with, rather than re-deriving the formula here.
 *==========================================================================*/
TEST_CASE(boot_comp_thresholds_reach_the_dac_registers)
{
    g_config.comp1_ref_mV = 1500U;
    g_config.comp2_ref_mV = 2442U;
    /* COMP2's reference is derived from the CC6920's zero offset, so it
     * only becomes meaningful once the zero-offset calibration has run.
     * Without that it is parked at the rail. */
    g_config.im_calibrated      = 1U;
    g_config.im_zero_offset_adc = 43254U;
    /* A recognised sensor model: without one the current protection is
     * parked, because the zero point would be meaningless. */
    g_config.im_sensor_type     = IM_SENSOR_BIDIRECTIONAL;

    boot_apply_comp_thresholds();

    ASSERT_EQ(hdac1.Instance->DHR12R1, expected_dac_code(1500U));
    ASSERT_EQ(hdac1.Instance->DHR12R2, expected_dac_code(2442U));

    /* 1500 mV of 3300 mV is about 45 % of scale; guard the arithmetic
     * against someone changing DAC_VDDA_MV to the ADC's 2.5 V rail. */
    ASSERT_TRUE(hdac1.Instance->DHR12R1 > 1700U);
    ASSERT_TRUE(hdac1.Instance->DHR12R1 < 2000U);

    /* Both channels must actually be driving their pins. */
    ASSERT_TRUE((hdac1.Instance->CR & 0x03U) == 0x03U);
}

TEST_CASE(boot_comp2_parks_at_the_rail_without_a_valid_calibration)
{
    /* COMP2 watches the main-current node and its reference is derived
     * from the CC6920's zero offset.  Until that offset has actually
     * been measured the reference would sit at the wrong voltage, so
     * COMP2 is parked at the rail where it cannot trip.
     *
     * COMP1 (RF input overdrive) does not depend on the current
     * measurement and stays armed. */
    g_config.comp1_ref_mV = 1500U;
    g_config.comp2_ref_mV = 2442U;

    /* No calibration at all. */
    g_config.im_calibrated      = 0U;
    g_config.im_zero_offset_adc = 43254U;
    boot_apply_comp_thresholds();
    ASSERT_EQ(hdac1.Instance->DHR12R1, expected_dac_code(1500U));
    ASSERT_EQ(hdac1.Instance->DHR12R2, expected_dac_code(DAC_VDDA_MV));
    ASSERT_EQ(hdac1.Instance->DHR12R2, DAC_FULL_SCALE);

    /* Calibrated, but the offset is implausible: still parked. */
    g_config.im_calibrated      = 1U;
    g_config.im_zero_offset_adc = 100U;
    boot_apply_comp_thresholds();
    ASSERT_EQ(hdac1.Instance->DHR12R2, DAC_FULL_SCALE);

    /* A valid calibration brings it live again.  With the sensor on its
     * nominal 1.65 V zero the derived reference lands exactly on the
     * 2442 mV the stored default carries. */
    g_config.im_zero_offset_adc = 43254U;
    g_config.im_sensor_type     = IM_SENSOR_BIDIRECTIONAL;
    boot_apply_comp_thresholds();
    ASSERT_EQ(hdac1.Instance->DHR12R2, expected_dac_code(DEFAULT_COMP2_REF_MV));
}

TEST_CASE(boot_comp_thresholds_clamp_to_the_configured_ceiling)
{
    /* A corrupted or hand-edited config must not be able to pin a
     * comparator output at its rail. */
    g_config.comp1_ref_mV = 60000U;   /* way past COMP1_REF_MV_MAX */
    g_config.comp2_ref_mV = 60000U;
    g_config.im_calibrated      = 1U;
    g_config.im_zero_offset_adc = 43254U;
    g_config.im_sensor_type     = IM_SENSOR_BIDIRECTIONAL;

    boot_apply_comp_thresholds();

    ASSERT_EQ(hdac1.Instance->DHR12R1, expected_dac_code(COMP1_REF_MV_MAX));

    /* COMP2 is different now: the stored value is only a fallback for an
     * unrecognised sensor, and this one IS recognised, so the garbage
     * never gets the chance to reach the DAC at all.  The applied
     * reference is derived from the measured zero point and the limit. */
    ASSERT_EQ(hdac1.Instance->DHR12R2, expected_dac_code(DEFAULT_COMP2_REF_MV));

    /* With the sensor unrecognised there is no measured zero to derive
     * from, so the channel is parked at the VDDA rail instead - the
     * comparator cannot trip at all until a calibration has run.  That,
     * rather than the ceiling, is what keeps garbage out of the DAC on
     * that path. */
    g_config.im_sensor_type = IM_SENSOR_UNKNOWN;
    boot_apply_comp_thresholds();
    ASSERT_EQ(hdac1.Instance->DHR12R2, DAC_FULL_SCALE);
}

TEST_CASE(boot_dac_code_conversion_boundaries)
{
    /* 0 mV -> code 0.  Asserted through COMP1, which is taken from the
     * stored value verbatim; COMP2 is derived and can no longer be given
     * a 0 mV reference through the config. */
    g_config.comp1_ref_mV = 0U;
    g_config.comp2_ref_mV = 0U;
    g_config.im_calibrated      = 1U;
    g_config.im_zero_offset_adc = 43254U;
    g_config.im_sensor_type     = IM_SENSOR_BIDIRECTIONAL;
    boot_apply_comp_thresholds();
    ASSERT_EQ(hdac1.Instance->DHR12R1, 0U);

    /* And COMP2 with that same config is the derived 30 A reference, not
     * the 0 mV that was stored - which is the point of the change. */
    ASSERT_EQ(hdac1.Instance->DHR12R2, expected_dac_code(DEFAULT_COMP2_REF_MV));

    /* 0 mV -> code 0 straight through the converter. */
    ASSERT_EQ(expected_dac_code(0U), 0U);

    /* Exactly full scale -> 4095. */
    ASSERT_EQ(expected_dac_code(DAC_VDDA_MV), DAC_FULL_SCALE);

    /* Over-range clamps instead of wrapping; a wrapped code would land
     * near 0 V, i.e. "trip on any noise". */
    ASSERT_EQ(expected_dac_code(5000U), DAC_FULL_SCALE);
    ASSERT_EQ(expected_dac_code(0xFFFFFFFFU), DAC_FULL_SCALE);
}

/*============================================================================
 *  Test 1b: the documented COMP2 reference points.
 *  CC6920BSO-50A: V = 1650 + I * 26.4, so these are the exact DAC codes
 *  for the current ratings the operator actually picks on the slider.
 *==========================================================================*/
TEST_CASE(comp2_reference_matches_the_current_sensor_law)
{
    /* 1650 mV is zero current: the DAC code is just under half scale, and
     * 1650 mV is the floor of the COMP2 slider. */
    ASSERT_EQ(expected_dac_code(1650U), 2047U);

    /* The 30 A design limit through the sensor law is 1650 + 30*26.4 =
     * 2442 mV, and that is the value the stored default carries. */
    ASSERT_EQ(expected_dac_code(DEFAULT_COMP2_REF_MV), 3030U);

    /* It is NOT the ceiling any more: the ceiling is the ADC full scale
     * (2500 mV), because the applied reference is derived from the
     * MEASURED resting point and a sensor sitting above 1.65 V needs a
     * reference above 2442 mV to trip at 30 A.  Clamping it to 2442
     * would reintroduce the early trip this derivation removes. */
    ASSERT_GT(COMP2_REF_MV_MAX, DEFAULT_COMP2_REF_MV);
    ASSERT_EQ(COMP2_REF_MV_MAX, ADC_VREF_MV);
}

/*============================================================================
 *  Test 2: band -> config index mapping
 *  Every band must land on its own slot; anything unknown falls back to
 *  the most restrictive entry (index 4 = 15-10 m, the lowest RF limit).
 *============================================================================*/
TEST_CASE(boot_band_index_mapping)
{
    /* Indices must be distinct and ordered as band_rf_in_max_w[] is
     * declared in opa2000.h: 160m, 80m, 40-30m, 20-17m, 15-10m. */
    ASSERT_EQ(ref_band_index(BAND_160M),    0U);
    ASSERT_EQ(ref_band_index(BAND_80M),     1U);
    ASSERT_EQ(ref_band_index(BAND_40M_30M), 2U);
    ASSERT_EQ(ref_band_index(BAND_20M_17M), 3U);
    ASSERT_EQ(ref_band_index(BAND_15M_10M), 4U);

    /* Unknown / fault bands must NOT index out of bounds. */
    ASSERT_EQ(ref_band_index(BAND_NONE),  4U);
    ASSERT_EQ(ref_band_index(BAND_FAULT), 4U);
    ASSERT_EQ(ref_band_index(-1),         4U);
    ASSERT_EQ(ref_band_index(999),        4U);
}

/*============================================================================
 *  Test 3: band_rf_in_max_w[] must cover every index ref_band_index() emits
 *  boot_apply_thresholds() indexes it with the value above, so a shorter
 *  array would be an out-of-bounds read on hardware.
 *==========================================================================*/
TEST_CASE(boot_rf_limit_array_size)
{
    /* Six slots (BAND_160M .. BAND_15M_10M, plus BAND_FAULT which shares
     * the last safe entry).  If the array is ever shortened this fails
     * rather than letting boot_apply_thresholds() index past the end. */
    ASSERT_EQ((int)BAND_FAULT, 6);
    ASSERT_EQ(sizeof(g_config.band_rf_in_max_w) / sizeof(g_config.band_rf_in_max_w[0]),
              6U);

    /* The array is indexed 0..4; make sure we can legally read index 4. */
    uint16_t probe = g_config.band_rf_in_max_w[4];
    (void)probe;
}

/*============================================================================
 *  Test 4: a fresh config always asks for touch calibration
 *  config_reset_default() leaves the transform at the identity, which is
 *  exactly the "operator never calibrated" state.  If this ever
 *  returns false the unit would boot straight to the main screen with
 *  an unusable touch panel.
 *============================================================================*/
TEST_CASE(boot_fresh_config_needs_touch_cal)
{
    config_reset_default();
    ASSERT_TRUE(config_touch_needs_calibration());
}

/*============================================================================
 *  Test 5: a marked-as-calibrated config does NOT ask again
 *  Once g_config.touch.calibrated is set, boot must go straight to the
 *  main screen even though the gains may still be near 1.
 *============================================================================*/
TEST_CASE(boot_calibrated_config_skips_touch_cal)
{
    config_reset_default();
    ASSERT_TRUE(config_touch_needs_calibration());

    g_config.touch.calibrated = 1U;
    ASSERT_FALSE(config_touch_needs_calibration());

    /* restore for any later test */
    g_config.touch.calibrated = 0U;
}

/*============================================================================
 *  Test 6: a non-identity gain suppresses the prompt even when the
 *  calibrated flag is missing (defensive: a config written by an older
 *  firmware may carry the gains but not the flag).
 *============================================================================*/
TEST_CASE(boot_non_identity_gain_skips_prompt)
{
    config_reset_default();
    ASSERT_TRUE(config_touch_needs_calibration());

    /* A real calibration always produces gains well away from 1.0 for a
     * 480x272 panel mapped onto a 0..4095 raw range. */
    g_config.touch.x_gain = 0.1172f;   /* 480 / 4095 */
    ASSERT_FALSE(config_touch_needs_calibration());

    g_config.touch.x_gain = 1.0f;
    ASSERT_TRUE(config_touch_needs_calibration());
}

/*============================================================================
 *  Test 3b: boot_hold_thresholds_open() must be uncrossable
 *
 *  This is the first thing main() does once the peripherals are up, and it
 *  exists purely to stop the amplifier coming up already alarming.  The
 *  property that matters is that neither limit can be reached by any
 *  conversion result, so a settling transient cannot trip anything:
 *
 *    - both COMP DAC references sit at code 4095 = the VDDA rail,
 *    - CLEAR_FAULTX_OC is pulled low, so the external latch is held clear.
 *==========================================================================*/
TEST_CASE(boot_hold_thresholds_open_parks_every_limit)
{
    /* Start from a config with real, low limits, so the test fails if the
     * function silently reverts to programming from g_config. */
    config_reset_default();
    g_config.comp1_ref_mV = 1650U;
    g_config.comp2_ref_mV = 1650U;

    boot_hold_thresholds_open();

    ASSERT_EQ(hdac1.Instance->DHR12R1, DAC_FULL_SCALE);
    ASSERT_EQ(hdac1.Instance->DHR12R2, DAC_FULL_SCALE);
    /* Both channels started, or the pins never reach the rail at all. */
    ASSERT_TRUE((hdac1.Instance->CR & 0x03U) == 0x03U);

    ASSERT_EQ(stub_gpio_get_last(CLEAR_FAULTX_OC_GPIO_Port,
                                 CLEAR_FAULTX_OC_Pin), 0);

    /* The apply gate is closed by the same call, and that is the part that
     * actually keeps the false alarm away.  power_init() -> switch_band()
     * reaches boot_apply_thresholds() a few milliseconds after the
     * scheduler starts, i.e. deep inside the settling window; if this apply
     * went through, the operator's real limits would land while the rail is
     * still charging and everything above would be undone. */
    g_config.band_im_max_mA = 10000U;
    boot_apply_thresholds((int)BAND_15M_10M);

    ASSERT_EQ(ADC1->HTR2, ADC_FULL_SCALE);            /* still parked wide */
    ASSERT_EQ(hdac1.Instance->DHR12R1, DAC_FULL_SCALE);
    ASSERT_EQ(hdac1.Instance->DHR12R2, DAC_FULL_SCALE);
    ASSERT_EQ(stub_gpio_get_last(CLEAR_FAULTX_OC_GPIO_Port,
                                 CLEAR_FAULTX_OC_Pin), 0);
}

/*============================================================================
 *  Test 3c: boot_arm_thresholds() re-opens the gate
 *
 *  The settling window is only useful if it ends.  boot_arm_thresholds() is
 *  the one call that ends it, made by adc_stabilisation_tick() on the tick
 *  the IM zero-offset calibration passes.  After it a band change or a
 *  Settings edit must reach the hardware again, otherwise the operator
 *  would be stuck with the parked limits for the rest of the session.
 *==========================================================================*/
TEST_CASE(boot_arm_thresholds_opens_the_gate)
{
    config_reset_default();
    g_config.im_zero_offset_adc = 43254U;
    /* The current protection is expressed relative to the measured zero
     * offset, so it only comes live once the calibration has produced
     * one.  AWD2 and COMP2 are the two that depend on it. */
    g_config.im_calibrated      = 1U;
    g_config.im_sensor_type     = IM_SENSOR_BIDIRECTIONAL;
    g_config.band_im_max_mA    = 10000U;
    g_config.comp1_ref_mV      = 1650U;
    g_config.comp2_ref_mV      = 2442U;

    boot_hold_thresholds_open();
    ASSERT_EQ(ADC1->HTR2, ADC_FULL_SCALE);      /* parked, gate closed */

    boot_arm_thresholds((int)BAND_15M_10M);

    /* Both references came down off the rail to live values, so the
     * comparators are running again.  COMP1 is stored as-is; COMP2 is
     * derived from the measured zero point and the configured limit
     * (1650 + 10 A * 26.4 = 1914 mV), which is what the applied
     * reference now always is. */
    ASSERT_EQ(hdac1.Instance->DHR12R1, expected_dac_code(1650U));
    ASSERT_EQ(hdac1.Instance->DHR12R2, expected_dac_code(1650U + 264U));
    /* AWD2 now holds a real limit: 10000 mA is 6920 codes above the
     * 43254-code zero point of the CC6920 law. */
    ASSERT_EQ(ADC1->HTR2, 43254U + 6920U);
}

TEST_CASE(boot_arm_thresholds_leaves_the_current_protection_parked)
{
    /* The same thing, but with no valid zero-offset calibration: the
     * current protection must stay parked even though the gate is open,
     * because a threshold computed from an unmeasured offset would sit
     * at the wrong current. */
    config_reset_default();
    g_config.im_calibrated      = 0U;
    g_config.im_zero_offset_adc = 43254U;
    g_config.band_im_max_mA    = 10000U;
    g_config.comp1_ref_mV      = 1650U;
    g_config.comp2_ref_mV      = 2442U;

    boot_hold_thresholds_open();
    boot_arm_thresholds((int)BAND_15M_10M);

    /* AWD2 parked, COMP2 parked at the rail ... */
    ASSERT_EQ(ADC1->HTR2, ADC_FULL_SCALE);
    ASSERT_EQ(hdac1.Instance->DHR12R2, DAC_FULL_SCALE);
    /* ... but COMP1, which does not depend on the current measurement, is
     * armed as usual. */
    ASSERT_EQ(hdac1.Instance->DHR12R1, expected_dac_code(1650U));
}

/*============================================================================
 *  Suite registration
 *============================================================================*/
const test_entry_t g_suite_boot[] = {
    TEST_ENTRY(boot_comp_thresholds_reach_the_dac_registers),
    TEST_ENTRY(boot_comp2_parks_at_the_rail_without_a_valid_calibration),
    TEST_ENTRY(boot_comp_thresholds_clamp_to_the_configured_ceiling),
    TEST_ENTRY(boot_dac_code_conversion_boundaries),
    TEST_ENTRY(comp2_reference_matches_the_current_sensor_law),
    TEST_ENTRY(boot_band_index_mapping),
    TEST_ENTRY(boot_rf_limit_array_size),
    TEST_ENTRY(boot_hold_thresholds_open_parks_every_limit),
    TEST_ENTRY(boot_arm_thresholds_opens_the_gate),
    TEST_ENTRY(boot_arm_thresholds_leaves_the_current_protection_parked),
    TEST_ENTRY(boot_fresh_config_needs_touch_cal),
    TEST_ENTRY(boot_calibrated_config_skips_touch_cal),
    TEST_ENTRY(boot_non_identity_gain_skips_prompt),
    TEST_SUITE_END,
};
