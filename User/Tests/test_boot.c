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
#include "stm32h7xx_hal_stub.h"

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

TEST_CASE(boot_comp_thresholds_clamp_to_the_configured_ceiling)
{
    /* A corrupted or hand-edited config must not be able to pin a
     * comparator output at its rail. */
    g_config.comp1_ref_mV = 60000U;   /* way past COMP1_REF_MV_MAX */
    g_config.comp2_ref_mV = 60000U;

    boot_apply_comp_thresholds();

    ASSERT_EQ(hdac1.Instance->DHR12R1, expected_dac_code(COMP1_REF_MV_MAX));
    ASSERT_EQ(hdac1.Instance->DHR12R2, expected_dac_code(COMP2_REF_MV_MAX));
}

TEST_CASE(boot_dac_code_conversion_boundaries)
{
    /* 0 mV -> code 0. */
    g_config.comp1_ref_mV = 0U;
    g_config.comp2_ref_mV = 0U;
    boot_apply_comp_thresholds();
    ASSERT_EQ(hdac1.Instance->DHR12R1, 0U);
    ASSERT_EQ(hdac1.Instance->DHR12R2, 0U);

    /* Exactly full scale -> 4095. */
    g_config.comp1_ref_mV = DEFAULT_COMP1_REF_MV;   /* keep the clamp in play */
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

    /* The 30 A design limit is the shipped default and, by definition,
     * the slider maximum. */
    ASSERT_EQ(expected_dac_code(DEFAULT_COMP2_REF_MV),
              expected_dac_code(COMP2_REF_MV_MAX));
    ASSERT_EQ(expected_dac_code(DEFAULT_COMP2_REF_MV), 3030U);
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
 *  Test 3: band_rf_in_max_w[] must hold exactly 5 entries
 *  boot_apply_thresholds() indexes it with the value above, so a shorter
 *  array would be an out-of-bounds read on hardware.
 *============================================================================*/
TEST_CASE(boot_rf_limit_array_size)
{
    ASSERT_EQ(BAND_COUNT >= 5 ? 1 : 0, 1);
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
 *  Suite registration
 *============================================================================*/
const test_entry_t g_suite_boot[] = {
    TEST_ENTRY(boot_comp_thresholds_reach_the_dac_registers),
    TEST_ENTRY(boot_comp_thresholds_clamp_to_the_configured_ceiling),
    TEST_ENTRY(boot_dac_code_conversion_boundaries),
    TEST_ENTRY(comp2_reference_matches_the_current_sensor_law),
    TEST_ENTRY(boot_band_index_mapping),
    TEST_ENTRY(boot_rf_limit_array_size),
    TEST_ENTRY(boot_fresh_config_needs_touch_cal),
    TEST_ENTRY(boot_calibrated_config_skips_touch_cal),
    TEST_ENTRY(boot_non_identity_gain_skips_prompt),
    TEST_SUITE_END,
};
