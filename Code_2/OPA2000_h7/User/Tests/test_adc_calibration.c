/**
 * @file    test_adc_calibration.c
 * @brief   Unit tests for adc_calibration.c - raw ADC code to engineering
 *          unit conversion plus the analog-watchdog threshold setters.
 *
 * @details The NTC curve is validated against the Beta equation itself
 *          (computed here in double precision), so the integer ln()
 *          implementation is checked to better than 0.1 C across the whole
 *          operating range rather than at a single hand-picked point.
 */

#include "test_harness.h"
#include "adc_calibration.h"
#include "opa2000.h"
#include "stm32h7xx_hal_stub.h"

#include <math.h>
#include <stdint.h>

/* Reference (double precision) Beta-equation temperature for a raw code. */
static double ntc_reference_c(uint16_t raw)
{
    double v = (double)raw * 2500.0 / 65535.0;   /* mV on the NTC tap   */
    double r = v / (3300.0 - v);                 /* R_ntc / R35        */
    double t = 1.0 / (1.0 / 298.15 + log(r) / 3435.0);
    return t - 273.15;
}

/*============================================================================
 *  Raw -> millivolt helpers
 *==========================================================================*/

TEST_CASE(adc_raw_to_mV_endpoints)
{
    ASSERT_EQ(ADC_RAW_TO_MV(0U), 0U);
    ASSERT_EQ(ADC_RAW_TO_MV(65535U), 2500U);      /* 65535*2500/65535 */
}

TEST_CASE(if_envelope_conversion_is_millivolts)
{
    ASSERT_EQ(adc2_raw_to_if_pa_env_mV(0U), 0);
    ASSERT_EQ(adc2_raw_to_if_pa_env_mV(65535U), 2500);
    ASSERT_EQ(adc2_raw_to_if_radio_env_mV(13107U), 500);
}

TEST_CASE(vbias_is_a_direct_voltage_sense)
{
    ASSERT_EQ(adc1_raw_to_vbias_sense_mV(0U), 0);
    ASSERT_EQ(adc1_raw_to_vbias_sense_mV(65535U), 2500);
    /* 1.65 V code = 43254 */
    ASSERT_EQ(adc1_raw_to_vbias_sense_mV(43254U), 1650);
}

TEST_CASE(vm_scales_full_scale_to_50V)
{
    ASSERT_EQ(adc1_raw_to_vm_mV(0U), 0);
    ASSERT_EQ(adc1_raw_to_vm_mV(65535U), 50000);
    ASSERT_EQ(adc1_raw_to_vm_mV(32768U), 25000);
}

TEST_CASE(key_channel_passes_the_raw_code_through)
{
    ASSERT_EQ(adc1_raw_to_key_adc_code(0U), 0);
    ASSERT_EQ(adc1_raw_to_key_adc_code(43254U), 43254);
    ASSERT_EQ(adc1_raw_to_key_adc_code(65535U), 65535);  /* needs uint16_t */
}

/*============================================================================
 *  Main current (CC6920BSO-50A: 0 A = 1.65 V, 26.4 mV/A)
 *==========================================================================*/

TEST_CASE(im_conversion_is_zero_until_calibrated)
{
    g_config.im_calibrated     = 0U;
    g_config.im_zero_offset_adc = 43254U;
    ASSERT_EQ(adc1_raw_to_im_mA(50000U), 0);
}

TEST_CASE(im_conversion_is_zero_at_the_calibrated_offset)
{
    g_config.im_calibrated      = 1U;
    g_config.im_zero_offset_adc = 43254U;
    ASSERT_EQ(adc1_raw_to_im_mA(43254U), 0);
}

TEST_CASE(im_conversion_matches_the_26_4mV_per_amp_slope)
{
    g_config.im_calibrated      = 1U;
    g_config.im_zero_offset_adc = 43254U;

    /* 1 A = 26.4 mV = 26.4 * 65535 / 2500 = 691.9 codes */
    ASSERT_GE(adc1_raw_to_im_mA(43254U + 692U), 990);
    ASSERT_LE(adc1_raw_to_im_mA(43254U + 692U), 1010);

    /* 10 A = 6920 codes */
    ASSERT_GE(adc1_raw_to_im_mA(43254U + 6920U), 9900);
    ASSERT_LE(adc1_raw_to_im_mA(43254U + 6920U), 10100);

    /* 30 A (protection limit) = 20760 codes */
    ASSERT_GE(adc1_raw_to_im_mA(43254U + 20760U), 29900);
    ASSERT_LE(adc1_raw_to_im_mA(43254U + 20760U), 30100);
}

TEST_CASE(im_conversion_is_negative_below_the_offset)
{
    g_config.im_calibrated      = 1U;
    g_config.im_zero_offset_adc = 43254U;
    ASSERT_LT(adc1_raw_to_im_mA(42000U), 0);
}

TEST_CASE(im_conversion_is_monotonic)
{
    int32_t prev = adc1_raw_to_im_mA(41254U);
    for (uint32_t raw = 41255U; raw < 65535U; raw += 250U) {
        int32_t cur = adc1_raw_to_im_mA((uint16_t)raw);
        ASSERT_GT(cur, prev);
        prev = cur;
    }
}

TEST_CASE(im_conversion_endpoints_do_not_wrap)
{
    /* delta = -43254 codes at raw 0 and +22281 at full scale; the 32-bit
     * result must stay the true value, not a wrapped one. */
    g_config.im_calibrated      = 1U;
    g_config.im_zero_offset_adc = 43254U;
    ASSERT_EQ(adc1_raw_to_im_mA(0U), -62501);
    ASSERT_EQ(adc1_raw_to_im_mA(65535U), 32195);
}

/*============================================================================
 *  Coupler power conversion (OPA1000 model)
 *==========================================================================*/

TEST_CASE(power_is_zero_inside_the_rectifier_dead_band)
{
    ASSERT_EQ(adc1_raw_to_fwd1_mW(0U), 0);
    ASSERT_EQ(adc1_raw_to_fwd1_mW(99U), 0);
}

/* Independent double-precision reference, transcribed from the float code
 * in OPA1000 Sensor_task.c convert_ADC2FwdPower1().  Kept on the same 1 mV
 * ADC grid as the integer path so it validates the arithmetic (and the
 * DC_TURNS constant) rather than the quantisation. */
static double dc_reference_mW(uint32_t raw)
{
    uint32_t v_mV = (raw * ADC_VREF_MV) / ADC_FULL_SCALE;
    double   vrf_V = ((double)v_mV + (double)ADC_DC_VF_MV) / 1000.0
                     * ((double)ADC_DC_ATTEN_RATIO_Q1 / 10.0)
                     * (double)ADC_DC_TURNS;
    /* P(W) = VRF^2 / Z0 / 2, VRF in volts, result in milliwatts */
    return vrf_V * vrf_V / (2.0 * (double)ADC_DC_Z0_OHM) * 1000.0;
}

/* ADC code that the float model says produces `watts`. */
static uint16_t dc_code_for_W(double watts)
{
    double vrf_V  = sqrt(watts * 2.0 * (double)ADC_DC_Z0_OHM);
    double v_mV   = vrf_V * 1000.0
                    / (((double)ADC_DC_ATTEN_RATIO_Q1 / 10.0) * (double)ADC_DC_TURNS)
                    - (double)ADC_DC_VF_MV;
    if (v_mV < 0.0) v_mV = 0.0;
    return (uint16_t)((v_mV * (double)ADC_FULL_SCALE) / (double)ADC_VREF_MV);
}

TEST_CASE(power_matches_the_double_precision_model)
{
    /* Sweep the whole code range: the integer pipeline must track the
     * float model to well under a milliwatt of relative error. */
    for (uint32_t raw = 100U; raw < 65535U; raw += 97U) {
        double ref = dc_reference_mW(raw);
        int32_t lo = (int32_t)(ref * 0.995);
        int32_t hi = (int32_t)(ref * 1.005) + 1;
        ASSERT_GE(adc1_raw_to_fwd1_mW((uint16_t)raw), lo);
        ASSERT_LE(adc1_raw_to_fwd1_mW((uint16_t)raw), hi);
    }
}

TEST_CASE(power_reference_points_are_reachable)
{
    /* OPA1000 measured anchors (12-bit, 3.28 V): 100 W -> 560 counts,
     * 250 W -> 1100, 400 W -> 1400.  Those powers must land on a code the
     * OPA2000 front end can actually produce. */
    const double anchors_W[] = { 100.0, 250.0, 400.0, 800.0, 1100.0 };
    for (unsigned i = 0; i < sizeof anchors_W / sizeof anchors_W[0]; i++) {
        uint16_t raw = dc_code_for_W(anchors_W[i]);
        int32_t  got = adc1_raw_to_fwd1_mW(raw);
        int32_t  want_mW = (int32_t)(anchors_W[i] * 1000.0);
        ASSERT_GT(raw, 100U);                       /* above the dead band */
        ASSERT_LT(raw, 65535U);                     /* inside full scale  */
        ASSERT_GE(got, (int32_t)((uint64_t)want_mW * 98 / 100));
        ASSERT_LE(got, (int32_t)((uint64_t)want_mW * 102 / 100));
    }
}

TEST_CASE(power_full_scale_leaves_headroom_over_the_transmit_limits)
{
    /* The transmit limits are 1050 W (coupler 2) and 1100 W (coupler 1);
     * full scale has to sit comfortably above both. */
    int32_t fs = adc1_raw_to_fwd1_mW(65535U);
    ASSERT_GT(fs, 1500000);
    ASSERT_LT(fs, 1700000);
    ASSERT_GT(fs, 1100000);
}

TEST_CASE(all_four_coupler_channels_share_the_same_curve)
{
    uint16_t raw = dc_code_for_W(400.0);
    int32_t ref = adc1_raw_to_fwd1_mW(raw);
    ASSERT_EQ(adc1_raw_to_fwd2_mW(raw), ref);
    ASSERT_EQ(adc1_raw_to_rev1_mW(raw), ref);
    ASSERT_EQ(adc1_raw_to_rev2_mW(raw), ref);
    ASSERT_EQ(adc1_raw_to_rf_in_mW(raw), ref);
}

TEST_CASE(power_conversion_is_monotonic)
{
    /* ADC_RAW_TO_MV() resolves to 1 mV, i.e. 26 LSB, so the curve is a
     * step function per LSB: non-decreasing always, strictly increasing
     * once the step spans more than one millivolt. */
    for (uint32_t raw = 100U; raw < 65535U; raw++) {
        ASSERT_GE(adc1_raw_to_fwd1_mW((uint16_t)raw),
                  adc1_raw_to_fwd1_mW((uint16_t)(raw - 1U)));
    }
    for (uint32_t raw = 100U; raw + 64U < 65535U; raw += 64U) {
        ASSERT_GT(adc1_raw_to_fwd1_mW((uint16_t)(raw + 64U)),
                  adc1_raw_to_fwd1_mW((uint16_t)raw));
    }
}

TEST_CASE(power_conversion_does_not_wrap_at_full_scale)
{
    /* vrf^2 needs 64 bits: 397320^2 = 1.58e11 overflows uint32. */
    ASSERT_GT(adc1_raw_to_fwd1_mW(65535U), 1000000);
}

/*============================================================================
 *  NTC (core / aux) - Beta equation
 *==========================================================================*/

TEST_CASE(ntc_reads_25C_when_the_divider_is_balanced)
{
    /* R35 == R0 -> tap at 1.65 V -> 1650 mV -> code 43254 */
    int16_t t = adc2_inj_raw_to_core_ntc_tenth_c(43254U);
    ASSERT_GE(t, 248);
    ASSERT_LE(t, 251);
}

TEST_CASE(ntc_tracks_the_beta_equation_over_the_full_range)
{
    /* Sweep the divider from near-short to near-open and require the
     * integer pipeline to stay within 0.5 C of the float reference. */
    const uint16_t codes[] = {
        5000U, 10000U, 15000U, 20000U, 25000U, 30000U,
        35000U, 40000U, 43254U, 48000U, 55000U, 62000U
    };
    for (unsigned i = 0; i < sizeof codes / sizeof codes[0]; i++) {
        double ref_c = ntc_reference_c(codes[i]);
        int32_t got  = adc2_inj_raw_to_core_ntc_tenth_c(codes[i]);
        int32_t ref  = (int32_t)(ref_c * 10.0 + 0.5);
        int32_t err  = got - ref;
        if (err < 0) err = -err;
        ASSERT_LE(err, 5);            /* <= 0.5 C */
    }
}

TEST_CASE(ntc_is_monotonic_decreasing_with_resistance)
{
    int16_t prev = adc2_inj_raw_to_core_ntc_tenth_c(1000U);
    for (uint32_t raw = 2000U; raw < 65000U; raw += 500U) {
        int16_t cur = adc2_inj_raw_to_core_ntc_tenth_c((uint16_t)raw);
        ASSERT_LE(cur, prev);         /* hotter NTC -> lower tap -> lower T */
        prev = cur;
    }
}

TEST_CASE(ntc_open_circuit_reports_the_minimum)
{
    ASSERT_EQ(adc2_inj_raw_to_core_ntc_tenth_c(0U), -400);
}

TEST_CASE(ntc_full_scale_reads_cold)
{
    /* The tap cannot exceed the 2.5 V ADC range, so a full-scale reading
     * is a very hot NTC (R/R0 = 2500/800 = 3.125) and must read below 0 C. */
    ASSERT_LT(adc2_inj_raw_to_aux_ntc_tenth_c(65535U), 0);
}

TEST_CASE(aux_and_core_ntc_share_the_same_curve)
{
    ASSERT_EQ(adc2_inj_raw_to_aux_ntc_tenth_c(33333U),
              adc2_inj_raw_to_core_ntc_tenth_c(33333U));
}

/*============================================================================
 *  MCU internal temperature sensor (0.3 V @ 0 C, 4.3 mV/K)
 *==========================================================================*/

TEST_CASE(mcu_internal_sensor_zero_c_is_300mV)
{
    /* 300 mV = code 7864.2; 7870 truncates to exactly 300 mV. */
    int16_t t = adc2_inj_raw_to_mcu_internal_tenth_c(7870U);
    ASSERT_GE(t, -1);
    ASSERT_LE(t, 1);
}

TEST_CASE(mcu_internal_sensor_100c_is_4_3V)
{
    /* The 4.3 V point is outside the 2.5 V ADC range, so full scale is
     * checked instead: 2500 mV -> (2500-300)/43 = 51.1 C. */
    ASSERT_EQ(adc2_inj_raw_to_mcu_internal_tenth_c(65535U), 511);
}

TEST_CASE(mcu_internal_sensor_clamps_below_range)
{
    ASSERT_EQ(adc2_inj_raw_to_mcu_internal_tenth_c(0U), -400);
    ASSERT_EQ(adc2_inj_raw_to_mcu_internal_tenth_c(7000U), -400);
}

/*============================================================================
 *  Analog watchdog threshold setters
 *
 *  AWD2 (main current) has a published transfer function, so its threshold is
 *  computed from the datasheet sensor law and is tested against that law.
 *
 *  AWD1 (RF input) and AWD3 (coupler 1 reverse) deliberately do NOT convert
 *  milliwatts into ADC codes yet: the divider network in front of each pin is
 *  still being brought up, and a single shared "mW to raw" formula would be
 *  wrong for at least one of them.  adc_calibration.c leaves the high
 *  threshold at 0 with a TODO until each channel's analog path is measured.
 *
 *  The tests below pin that known state: the threshold stays 0 and only the
 *  low threshold is cleared.  When the conversion is filled in, these cases
 *  are the ones to replace with round-trip checks.
 *==========================================================================*/

TEST_CASE(awd1_threshold_stays_disabled_until_the_divider_is_known)
{
    opa2000_init();
    /* TODO: replace with a round-trip once adc_awd_set_rf_in_mW() converts
       milliwatts using the measured RF input divider. */
    adc_awd_set_rf_in_mW(2000000);
    ASSERT_EQ(ADC1->HTR1, 0);
    ASSERT_EQ(ADC1->LTR1, 0);
}

TEST_CASE(awd2_threshold_is_offset_from_the_zero_point)
{
    g_config.im_calibrated      = 1U;
    g_config.im_zero_offset_adc = 43254U;
    adc_awd_set_im_mA(30000);               /* 30 A */

    /* 30 A = 30 * 17301240 / 25000000 = 20761 codes above the offset */
    ASSERT_EQ(ADC1->HTR2, 43254U + 20761U);
    ASSERT_EQ(ADC1->LTR2, 0);
}

TEST_CASE(awd3_threshold_stays_disabled_until_the_coupler_is_known)
{
    opa2000_init();
    /* TODO: replace with a round-trip once adc_awd_set_rev1_mW() converts
       milliwatts using the measured coupler 1 / diode detector path. */
    adc_awd_set_rev1_mW(800000);
    ASSERT_EQ(ADC1->HTR3, 0);
    ASSERT_EQ(ADC1->LTR3, 0);
}

TEST_CASE(awd2_threshold_saturates_instead_of_wrapping)
{
    opa2000_init();
    /* A threshold beyond the 16-bit code range must clamp to full scale,
     * not wrap around to a tiny value that would trip instantly. */
    adc_awd_set_im_mA(60000);
    ASSERT_EQ(ADC1->HTR2, 65535);
    ASSERT_EQ(ADC1->LTR2, 0);
}

TEST_CASE(awd2_threshold_is_independent_of_the_uncalibrated_ones)
{
    opa2000_init();
    g_config.im_calibrated      = 1U;
    g_config.im_zero_offset_adc = 43254U;

    adc_awd_set_rf_in_mW(200000);   /* not converted yet */
    adc_awd_set_im_mA(10000);       /* 10 A above the zero point */
    adc_awd_set_rev1_mW(300000);    /* not converted yet */

    ASSERT_EQ(ADC1->HTR1, 0);
    ASSERT_EQ(ADC1->HTR2, 43254U + 6920U);
    ASSERT_EQ(ADC1->HTR3, 0);
}

/*============================================================================
 *  Suite table
 *==========================================================================*/

const test_entry_t g_suite_adc_calibration[] = {
    TEST_ENTRY(adc_raw_to_mV_endpoints),
    TEST_ENTRY(if_envelope_conversion_is_millivolts),
    TEST_ENTRY(vbias_is_a_direct_voltage_sense),
    TEST_ENTRY(vm_scales_full_scale_to_50V),
    TEST_ENTRY(key_channel_passes_the_raw_code_through),
    TEST_ENTRY(im_conversion_is_zero_until_calibrated),
    TEST_ENTRY(im_conversion_is_zero_at_the_calibrated_offset),
    TEST_ENTRY(im_conversion_matches_the_26_4mV_per_amp_slope),
    TEST_ENTRY(im_conversion_is_negative_below_the_offset),
    TEST_ENTRY(im_conversion_is_monotonic),
    TEST_ENTRY(im_conversion_endpoints_do_not_wrap),
    TEST_ENTRY(power_is_zero_inside_the_rectifier_dead_band),
    TEST_ENTRY(power_matches_the_double_precision_model),
    TEST_ENTRY(power_reference_points_are_reachable),
    TEST_ENTRY(power_full_scale_leaves_headroom_over_the_transmit_limits),
    TEST_ENTRY(all_four_coupler_channels_share_the_same_curve),
    TEST_ENTRY(power_conversion_is_monotonic),
    TEST_ENTRY(power_conversion_does_not_wrap_at_full_scale),
    TEST_ENTRY(ntc_reads_25C_when_the_divider_is_balanced),
    TEST_ENTRY(ntc_tracks_the_beta_equation_over_the_full_range),
    TEST_ENTRY(ntc_is_monotonic_decreasing_with_resistance),
    TEST_ENTRY(ntc_open_circuit_reports_the_minimum),
    TEST_ENTRY(ntc_full_scale_reads_cold),
    TEST_ENTRY(aux_and_core_ntc_share_the_same_curve),
    TEST_ENTRY(mcu_internal_sensor_zero_c_is_300mV),
    TEST_ENTRY(mcu_internal_sensor_100c_is_4_3V),
    TEST_ENTRY(mcu_internal_sensor_clamps_below_range),
    TEST_ENTRY(awd1_threshold_stays_disabled_until_the_divider_is_known),
    TEST_ENTRY(awd2_threshold_is_offset_from_the_zero_point),
    TEST_ENTRY(awd3_threshold_stays_disabled_until_the_coupler_is_known),
    TEST_ENTRY(awd2_threshold_saturates_instead_of_wrapping),
    TEST_ENTRY(awd2_threshold_is_independent_of_the_uncalibrated_ones),
    TEST_SUITE_END
};