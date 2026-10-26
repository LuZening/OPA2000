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

/* Reference (double precision) Beta-equation temperature for a raw code.
 *
 * Board wiring is 3V3 -> NTC -> tap -> 2.2k -> GND, so the tap rises with
 * temperature and the divider has to be inverted the other way round from
 * the usual pull-up arrangement:
 *     v = Vcc * 2.2k / (R_ntc + 2.2k)   =>   R_ntc = 2.2k * (Vcc - v) / v
 */
static double ntc_reference_c(uint16_t raw)
{
    double v = (double)raw * 2500.0 / 65535.0;   /* mV on the NTC tap   */
    double r;
    double t;

    if (v <= 0.0) {
        return -40.0;                            /* open -> clamped low */
    }
    r = 2200.0 * (3300.0 - v) / v;              /* R_ntc in ohms      */
    t = 1.0 / (1.0 / 298.15 + log(r / 10000.0) / 3950.0);
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

TEST_CASE(vm_undoes_the_300k_over_10k_divider)
{
    /* 300k upper / 10k lower -> V_pin = Vm / 31, so the 2.5 V reference
     * full scale is 2.5 * 31 = 77.5 V on the supply rail. */
    ASSERT_EQ(adc1_raw_to_vm_mV(0U), 0);
    ASSERT_EQ(adc1_raw_to_vm_mV(65535U), 77500);
    ASSERT_EQ(adc1_raw_to_vm_mV(32768U), 38750);

    /* A known operating point, to catch a divider that is close but wrong:
     * 13.8 V -> V_pin = 445.16 mV -> code 11668 -> back to 13 798 mV.  The
     * 2 mV difference is the code quantisation of a 1.18 mV-per-LSB input,
     * not an error in the ratio. */
    ASSERT_EQ(adc1_raw_to_vm_mV(11668U), 13798);

    /* The old 1:20 mapping would have returned 50 000 mV at full scale;
     * assert the difference explicitly so a regression back to it is loud. */
    ASSERT_NE(adc1_raw_to_vm_mV(65535U), 50000);
}

TEST_CASE(vm_full_scale_follows_the_reference_and_the_divider)
{
    /* The conversion is "pin voltage, then x31".  Written as two steps here
     * so the test states the physical chain rather than restating the
     * implementation's single multiply. */
    uint32_t pin_mV = (65535U * 2500U) / 65535U;   /* 2500 mV at full scale */
    ASSERT_EQ(pin_mV * 31U, (uint32_t)adc1_raw_to_vm_mV(65535U));
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

/*============================================================================
 *  The scale factor, spelled out - guards against a factor-of-ten
 *
 *  A current reading that is out by a power of ten is the most damaging
 *  kind of conversion bug: nothing crashes, the number still moves in the
 *  right direction, and it stays plausible.  The operator just believes
 *  the amplifier draws ten times what it draws, and the protection trips
 *  at a tenth of its rated current (or not at all).
 *
 *  This case derives the expected answer from first principles - the
 *  sensor's published slope, the reference voltage and the code width -
 *  and then checks it against BOTH directions of the conversion, so a
 *  constant changed in one place and not the other is caught too.
 *==========================================================================*/

TEST_CASE(the_im_scale_factor_is_one_milliamp_per_code_point_445)
{
    g_config.im_calibrated      = 1U;
    g_config.im_zero_offset_adc = 43254U;
    /* Needed by adc_im_protection_valid(), which ma_to_adc_code() consults:
     * without a recognised model the watchdog parks at full scale instead of
     * being fitted, and the round-trip below would read 32 A of nonsense. */
    g_config.im_sensor_type     = IM_SENSOR_BIDIRECTIONAL;

    /* Derive the expected scale from the hardware, not from the
     * implementation:
     *   1 LSB        = 2500 mV / 65535 codes   ~= 38.1 uV
     *   sensor slope = 26.4 mV/A               = 26400 uV/A
     *   codes per A  = 26400 uV / 38.1 uV      ~= 692                */
    const uint32_t microV_per_code = 2500000U / 65535U;   /* 38 uV */
    const uint32_t uV_per_amp     = 26400U;               /* 26.4 mV/A */
    const uint32_t codes_per_amp  = uV_per_amp / microV_per_code;  /* 694 */

    /* 694 codes must be 1000 mA.  Integer division inside the conversion
     * costs at most one code, i.e. 1.4 mA, so the assert is on 0.1 A. */
    ASSERT_EQ(adc1_raw_to_im_mA(43254U + codes_per_amp) / 10U, 100U);

    /* 400 codes - the jump the operator saw on the Sensors screen - is
     * 0.578 A, NOT 5.78 A.  The old GUI formatted the milliamp value as
     * centiamps (dividing by 100 instead of 1000), which is exactly a
     * factor of ten too large; this is the number that bug produced. */
    int32_t four_hundred = adc1_raw_to_im_mA(43254U + 400U);
    ASSERT_EQ(four_hundred / 10U, 57U);       /* 57.x -> 0.57x A */
    ASSERT_LT(four_hundred, 1000);            /* well under 1 A  */

    /* And the inverse agrees, so the watchdog trips at the same current the
     * display shows.  The round trip truncates twice - once in each
     * direction - so it lands 1 mA low (10 000 -> code 6920 above the
     * offset -> 9999 mA).  One LSB of current is 1.4 mA, so a +/-10 mA
     * window is tighter than the quantisation and still catches a
     * factor-of-ten or factor-of-two error, which is what this is here for. */
    adc_awd_set_im_mA(10000U);
    int32_t at_watchdog = adc1_raw_to_im_mA((uint16_t)ADC1->HTR2);
    ASSERT_GE(at_watchdog, 9990);
    ASSERT_LE(at_watchdog, 10010);
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

/* Independent double-precision reference for the OPA2000 front end:
 *
 *   coupler tap --(33:1 transformer)--> --(120R/330R)--> --(Vf=200mV)-->
 *   --(220k/100k)--> --buffer--> ADC
 *
 * transcribed in floating point so it validates the integer arithmetic
 * (and the resistor constants) rather than repeating them.  Kept on the
 * same 1 mV ADC grid as the integer path so it tests the arithmetic and
 * not the quantisation. */
static double dc_reference_mW(uint32_t raw)
{
    uint32_t v_mV = (raw * ADC_VREF_MV) / ADC_FULL_SCALE;
    double   v_det_V = (double)v_mV / 1000.0
                       * ((double)ADC_DC_POST_GAIN_Q1 / 10.0);
    double   v_pre_V = v_det_V + (double)ADC_DC_VF_MV / 1000.0;
    double   vrf_V   = v_pre_V * ((double)ADC_DC_PRE_GAIN_Q1 / 10.0);
    /* P(W) = VRF^2 / Z0 / 2, VRF in volts, result in milliwatts */
    return vrf_V * vrf_V / (2.0 * (double)ADC_DC_Z0_OHM) * 1000.0;
}

/* ADC code that the float model says produces `watts`. */
static uint16_t dc_code_for_W(double watts)
{
    double vrf_V  = sqrt(watts * 2.0 * (double)ADC_DC_Z0_OHM);
    double v_pre_V = vrf_V / ((double)ADC_DC_PRE_GAIN_Q1 / 10.0);
    double v_det_V = v_pre_V - (double)ADC_DC_VF_MV / 1000.0;
    double v_mV    = v_det_V * 1000.0 / ((double)ADC_DC_POST_GAIN_Q1 / 10.0);
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
    /* The OPA2000 front end has to be able to represent every power the
     * amplifier is rated for, and to do it inside the 16-bit code space. */
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
     * full scale has to sit above both.  With the OPA2000 chain (33 turns,
     * 120R/330R, 220k/100k) full scale is about 1362 W, so the 1100 W
     * limit lands at roughly 90 % of the ADC range. */
    int32_t fs = adc1_raw_to_fwd1_mW(65535U);
    ASSERT_GT(fs, 1300000);
    ASSERT_LT(fs, 1420000);
    ASSERT_GT(fs, 1100000);

    /* The 1100 W limit must not sit so close to the top that the detector
     * runs out of headroom before the fault is noticed. */
    uint16_t raw_1100 = adc_dc_mW_to_raw(1100000U);
    ASSERT_LT(raw_1100, 65535U);
    ASSERT_GT(raw_1100, 55000U);      /* above 84 % of full scale */
}

TEST_CASE(all_four_coupler_channels_share_the_same_curve)
{
    /* With no bench trim written, all four couplers follow the OPA1000
     * base model unchanged.  A zeroed trim means "not calibrated", not
     * "gain zero", so it must not scale the reading down. */
    uint16_t raw = dc_code_for_W(400.0);
    int32_t ref = adc1_raw_to_fwd1_mW(raw);
    ASSERT_GT(ref, 0);
    ASSERT_EQ(adc1_raw_to_fwd2_mW(raw), ref);
    ASSERT_EQ(adc1_raw_to_rev1_mW(raw), ref);
    ASSERT_EQ(adc1_raw_to_rev2_mW(raw), ref);
    ASSERT_EQ(adc1_raw_to_rf_in_mW(raw), ref);
}

TEST_CASE(each_coupler_applies_its_own_trim)
{
    uint16_t raw = dc_code_for_W(400.0);
    int32_t base = adc1_raw_to_fwd1_mW(raw);

    /* The trim is an ABSOLUTE fit against the raw code, in the same
     * units the calibration screen stores: micro-watts per code for the
     * slope, milli-watts for the intercept.  So a trim that says
     * "1000 uW/code" means 1 mW per code, NOT a x1.0 scaling of the
     * base model - at 400 W the base model reads ~400000 mW while the
     * trim says something completely different. */
    g_config.fwd1.gain_uW_per_code = 1000U;   /* 1 mW per code */
    g_config.fwd1.offset_mW        = 0;
    g_config.fwd2.gain_uW_per_code = 2000U;   /* 2 mW per code */
    g_config.fwd2.offset_mW        = 0;
    g_config.rev1.gain_uW_per_code = 1000U;
    g_config.rev1.offset_mW        = 25;      /* +25 mW */
    g_config.rev2.gain_uW_per_code = 1000U;
    g_config.rev2.offset_mW        = 0;

    ASSERT_EQ(adc1_raw_to_fwd1_mW(raw), (int32_t)raw);
    ASSERT_EQ(adc1_raw_to_fwd2_mW(raw), (int32_t)raw * 2);
    ASSERT_EQ(adc1_raw_to_rev1_mW(raw), (int32_t)raw + 25);
    ASSERT_EQ(adc1_raw_to_rev2_mW(raw), (int32_t)raw);

    /* The RF input detector has no coupler, so it must ignore all four
     * trims - otherwise calibrating coupler 2 would silently move the
     * RF input power reading too. */
    ASSERT_EQ(adc1_raw_to_rf_in_mW(raw), base);

    /* Back to uncalibrated. */
    g_config.fwd1.gain_uW_per_code = 0U;
    g_config.fwd1.offset_mW        = 0;
    g_config.fwd2.gain_uW_per_code = 0U;
    g_config.fwd2.offset_mW        = 0;
    g_config.rev1.gain_uW_per_code = 0U;
    g_config.rev1.offset_mW        = 0;
    g_config.rev2.gain_uW_per_code = 0U;
    g_config.rev2.offset_mW        = 0;
}

TEST_CASE(a_negative_trim_never_reads_as_power)
{
    uint16_t raw = dc_code_for_W(400.0);

    /* A bench trim can overshoot; a negative result must be clamped to
     * zero rather than wrapping into a huge unsigned power. */
    g_config.fwd1.gain_uW_per_code = 1000U;
    g_config.fwd1.offset_mW        = -1000000;
    ASSERT_EQ(adc1_raw_to_fwd1_mW(raw), 0);

    g_config.fwd1.gain_uW_per_code = 0U;
    g_config.fwd1.offset_mW        = 0;
}

TEST_CASE(a_calibrated_coupler_ignores_the_rectifier_dead_band)
{
    /* The dead band belongs to the OPA1000 diode model.  A bench fit
     * already carries the rectifier's own offset, so a calibrated
     * coupler must report from code 0 upwards - otherwise the operator
     * calibrates the coupler and then sees a dead zone that the
     * calibration never mentioned. */
    g_config.fwd1.gain_uW_per_code = 1000U;
    g_config.fwd1.offset_mW        = 0;

    ASSERT_EQ(adc1_raw_to_fwd1_mW(0U), 0);
    ASSERT_EQ(adc1_raw_to_fwd1_mW(1U), 1);
    ASSERT_EQ(adc1_raw_to_fwd1_mW(99U), 99);

    g_config.fwd1.gain_uW_per_code = 0U;
    g_config.fwd1.offset_mW        = 0;
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
    /* vrf^2 needs 64 bits: 369000^2 = 1.36e11 overflows uint32. */
    ASSERT_GT(adc1_raw_to_fwd1_mW(65535U), 1000000);
}

/*============================================================================
 *  Bidirectional conversion: mW -> raw -> mW
 *
 *  The whole point of routing both directions through one function is that
 *  the inverse never disagrees with the forward.  These cases pin that
 *  down: the code the inverse produces must read back as the power that
 *  was asked for, to within the resolution of the chain.
 *==========================================================================*/

TEST_CASE(the_inverse_produces_a_code_that_reads_back_as_the_requested_power)
{
    /* Every one of the powers the amplifier is rated for, plus the limits
     * the protection chain compares against. */
    const uint32_t powers_mW[] = {
        1000U, 10000U, 100000U, 250000U, 400000U, 550000U,
        800000U, 1000000U, 1050000U, 1100000U, 1300000U
    };

    for (unsigned i = 0; i < sizeof powers_mW / sizeof powers_mW[0]; i++) {
        uint32_t want = powers_mW[i];
        uint16_t raw  = adc_dc_mW_to_raw(want);
        int32_t  got  = adc_dc_raw_to_mW(raw);

        /* The inverse returns the FIRST code that reaches the limit, so the
         * reading is at or just above the request - never below it, or the
         * watchdog would trip late. */
        ASSERT_GE((uint32_t)got, want);
        /* And it must not overshoot by more than one code's worth of
         * power, or the limit would be effectively somewhere else. */
        ASSERT_LE((uint32_t)got, want + 5000U);
    }
}

TEST_CASE(the_inverse_and_the_forward_agree_across_the_whole_range)
{
    /* Sweep the code space: for every code, the power it reads must map
     * back to that same code.  This is the property that makes it safe to
     * compute a watchdog threshold from a power the operator typed. */
    for (uint32_t raw = 200U; raw < 65535U; raw += 331U) {
        int32_t  mW = adc_dc_raw_to_mW((uint16_t)raw);
        if (mW <= 0) continue;
        uint16_t back = adc_dc_mW_to_raw((uint32_t)mW);

        /* The forward curve is a step function: several codes can read the
         * same power, and the inverse must return the first of them.  So
         * the round trip lands on the same code or a LOWER one - never
         * above, which would mean a different power than the one asked
         * for.  It can also land on the very same code, when that code is
         * already the first of its plateau. */
        ASSERT_LE((uint32_t)back, raw);
        /* And the power itself must be preserved exactly: the code the
         * inverse returns has to read back as the same milliwatts. */
        ASSERT_EQ(adc_dc_raw_to_mW(back), mW);
    }
}

TEST_CASE(the_inverse_saturates_instead_of_wrapping)
{
    /* A power beyond what the chain can represent must clamp to full
     * scale, not wrap around to a small code that would let the fault
     * slip through unnoticed. */
    ASSERT_EQ(adc_dc_mW_to_raw(2000000U), 65535U);
    ASSERT_EQ(adc_dc_mW_to_raw(0xFFFFFFFFU), 65535U);
}

TEST_CASE(the_public_api_matches_the_per_channel_couplers)
{
    /* adc_dc_raw_to_mW() is the nominal chain, which is what the four
     * directional couplers use until they are bench-calibrated.  The two
     * must not drift apart. */
    for (uint32_t raw = 100U; raw < 65535U; raw += 997U) {
        int32_t a = adc_dc_raw_to_mW((uint16_t)raw);
        ASSERT_EQ(a, adc1_raw_to_fwd1_mW((uint16_t)raw));
        ASSERT_EQ(a, adc1_raw_to_fwd2_mW((uint16_t)raw));
        ASSERT_EQ(a, adc1_raw_to_rev2_mW((uint16_t)raw));
    }
}

TEST_CASE(the_chain_adds_the_rectifier_drop_between_the_two_dividers)
{
    /* The Schottky drop sits BETWEEN the two dividers, so it is scaled by
     * the transformer ratio but NOT by the 220k/100k pair.  Getting that
     * wrong is a silent error: the curve still rises monotonically, it is
     * just wrong by a factor.  Pin the two stages down individually. */
    uint32_t raw = ADC_DC_DEADBAND_CODE;      /* the first conducting code */

    /* v_adc at the pin, then the detector node in tenths of a millivolt
     * (the "deci" suffix: deci- = 10^-1), then the peak at the tap. */
    uint32_t v_adc = (raw * ADC_VREF_MV) / ADC_FULL_SCALE;
    uint32_t det_deci = v_adc * ADC_DC_POST_GAIN_Q1;
    uint32_t pre_deci = det_deci + (ADC_DC_VF_MV * 10U);
    uint32_t v_rf  = (uint32_t)(((uint64_t)pre_deci * ADC_DC_PRE_GAIN_Q1) / 100U);
    int32_t  want  = (int32_t)(((int64_t)v_rf * v_rf) / 100000);

    ASSERT_EQ(adc1_raw_to_fwd1_mW((uint16_t)raw), want);
    ASSERT_GT(want, 0);

    /* The drop is what makes the reading non-zero at all: without it the
     * same code would read zero.  So the drop has to be there, and it has
     * to be big enough to matter at the bottom of the range. */
    ASSERT_GT(want, 100);

    /* And the dead band really is below this code. */
    ASSERT_EQ(adc1_raw_to_fwd1_mW((uint16_t)(raw - 1U)), 0);
}

/*============================================================================
 *  NTC (core / aux) - Beta equation
 *
 *  Board wiring: 3V3 -> NTC -> tap -> 2.2k -> GND.  The NTC is on the high
 *  side, so the tap voltage RISES with temperature - the opposite of the
 *  pull-up arrangement these tests used to describe.
 *==========================================================================*/

TEST_CASE(ntc_reads_25C_at_the_expected_code)
{
    /* R_ntc = 10k at 25 C, so the tap is
     *   3300 * 2200 / (10000 + 2200) = 594.6 mV -> code 15599.
     * The old pull-up wiring balanced the divider at mid-scale (43254);
     * this one sits at roughly a quarter scale, which is the single
     * clearest sign that the topology really did change. */
    int16_t t = adc2_inj_raw_to_core_ntc_tenth_c(15599U);
    ASSERT_GE(t, 248);
    ASSERT_LE(t, 251);
}

TEST_CASE(the_tap_rises_with_temperature)
{
    /* The whole point of the pull-down wiring: hotter NTC -> smaller R ->
     * divider dominated by the 2.2k -> HIGHER tap voltage.  If this ever
     * inverts, the protection threshold becomes a trip-on-cold fault and
     * a genuinely hot amplifier reads as cold. */
    ASSERT_LT(adc2_inj_raw_to_core_ntc_tenth_c(10000U),    /* ~10 C */
              adc2_inj_raw_to_core_ntc_tenth_c(30000U));   /* ~50 C */
}

TEST_CASE(ntc_tracks_the_beta_equation_over_the_full_range)
{
    /* Sweep the divider across the codes the hardware can actually
     * produce and require the integer pipeline to stay within 0.5 C of
     * the float reference. */
    const uint16_t codes[] = {
        2000U, 5000U, 9234U, 12000U, 15599U, 20000U,
        23912U, 30000U, 36734U, 43254U, 49273U, 55000U,
        59715U, 62000U, 65000U
    };
    for (unsigned i = 0; i < sizeof codes / sizeof codes[0]; i++) {
        double ref_c = ntc_reference_c(codes[i]);
        int32_t got  = adc2_inj_raw_to_core_ntc_tenth_c(codes[i]);
        int32_t ref  = (int32_t)(ref_c * 10.0);
        int32_t err  = got - ref;
        if (err < 0) err = -err;
        ASSERT_LE(err, 2);            /* <= 0.2 C */
    }
}

TEST_CASE(ntc_is_monotonic_increasing_with_the_tap_voltage)
{
    /* Rising tap == rising temperature.  Sweeping the whole usable code
     * range must never step backwards, because a non-monotonic curve
     * would let protection_check_ntc() fire and clear repeatedly as the
     * reading jitters across a fold. */
    int16_t prev = adc2_inj_raw_to_core_ntc_tenth_c(2000U);
    for (uint32_t raw = 2500U; raw < 65000U; raw += 250U) {
        int16_t cur = adc2_inj_raw_to_core_ntc_tenth_c((uint16_t)raw);
        ASSERT_GE(cur, prev);
        prev = cur;
    }
}

TEST_CASE(ntc_open_circuit_reports_the_minimum)
{
    /* NTC disconnected -> the 2.2k pulls the tap to GND -> 0 mV, which
     * this wiring reads as "absurdly cold" and clamps to the floor. */
    ASSERT_EQ(adc2_inj_raw_to_core_ntc_tenth_c(0U), -400);
}

TEST_CASE(ntc_full_scale_reads_hot_not_cold)
{
    /* The tap cannot exceed the 2.5 V ADC range.  Full scale therefore
     * means the NTC is down to 704 ohm, which a 10K B3950 part reaches at
     * about 99.7 C - HOT, and still under the 120 C sanity clamp.  Under
     * the old pull-up wiring the same code read below freezing; that
     * inversion is what this assertion exists to catch. */
    int16_t t = adc2_inj_raw_to_aux_ntc_tenth_c(65535U);
    ASSERT_GT(t, 900);               /* > 90 C, unmistakably hot */
    ASSERT_LT(t, 1050);              /* and below the 120 C clamp */
}

TEST_CASE(ntc_protection_threshold_is_reachable_on_this_wiring)
{
    /* The protection limit is 70 C (MAX_NTC_TEMP_C).  Under the pull-down
     * wiring with a 10K B3950 part, 70 C sits at code 48053 (R_ntc = 1.76k),
     * about 73% of full scale - comfortably inside the ADC range, so a
     * genuine overheat is reachable and the protection can actually fire.
     *
     * The expectation is derived from the macro rather than pasted in, so
     * moving the limit does not quietly turn this into a test of nothing. */
    int16_t at_limit = adc2_inj_raw_to_core_ntc_tenth_c(48053U);
    ASSERT_GE(at_limit, (int32_t)(MAX_NTC_TEMP_C * 10) - 5);
    ASSERT_LE(at_limit, (int32_t)(MAX_NTC_TEMP_C * 10) + 5);
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
 *  The watchdog must trip at exactly the power the operator sees on the
 *  screen, on every channel.  That means the threshold setter and the
 *  reading must go through the same conversion, and its exact inverse.
 *
 *  AWD2 (main current) has a published transfer function, so its threshold
 *  is computed from the datasheet sensor law and is tested against that
 *  law.
 *
 *  AWD1 (RF input) and AWD3 (coupler 1 reverse) have channel-specific
 *  front-end gains (ADC_DC_POST_GAIN_RF_IN_Q1 and ADC_DC_POST_GAIN_REV1_Q1)
 *  that live in adc_calibration.h.  Zero means "not measured yet" and parks
 *  the watchdog at full scale; non-zero drives the watchdog through the
 *  same curve the display reads.  REV1 also wins the threshold from the
 *  bench trim if the operator has calibrated that coupler.
 *
 *  The tests below pin both the safe state (0 gain -> full scale) and
 *  the working state (non-zero gain -> threshold from the same curve as
 *  the reading).
 *==========================================================================*/

TEST_CASE(awd1_threshold_parks_wide_open_until_the_divider_is_known)
{
    opa2000_init();
    /* ADC_DC_POST_GAIN_RF_IN_Q1 is 0 in the shipped header.  The watchdog
     * cannot trip on a curve nobody has verified, so HTR sits at full
     * scale. */
    ASSERT_EQ(ADC_DC_POST_GAIN_RF_IN_Q1, 0U);

    adc_awd_set_rf_in_mW(2000000);
    ASSERT_EQ(ADC1->HTR1, ADC_FULL_SCALE);
    ASSERT_EQ(ADC1->LTR1, 0);
}

TEST_CASE(awd2_threshold_is_offset_from_the_zero_point)
{
    g_config.im_calibrated      = 1U;
    g_config.im_zero_offset_adc = 43254U;
    /* A recognised sensor: without one the current protection is parked,
     * because the zero point would be meaningless. */
    g_config.im_sensor_type     = IM_SENSOR_BIDIRECTIONAL;
    adc_awd_set_im_mA(30000);               /* 30 A */

    /* 30 A = 30 * 17301240 / 25000000 = 20761 codes above the offset */
    ASSERT_EQ(ADC1->HTR2, 43254U + 20761U);
    ASSERT_EQ(ADC1->LTR2, 0);
}

TEST_CASE(awd3_threshold_parks_wide_open_until_either_input_is_known)
{
    opa2000_init();
    /* Both ADC_DC_POST_GAIN_REV1_Q1 and g_config.rev1.gain_uW_per_code are 0
     * in the default config.  The watchdog parks wide open. */
    ASSERT_EQ(ADC_DC_POST_GAIN_REV1_Q1, 0U);
    ASSERT_EQ(g_config.rev1.gain_uW_per_code, 0);

    adc_awd_set_rev1_mW(800000);
    ASSERT_EQ(ADC1->HTR3, ADC_FULL_SCALE);
    ASSERT_EQ(ADC1->LTR3, 0);
}

TEST_CASE(awd3_threshold_uses_the_bench_trim_when_available)
{
    opa2000_init();
    /* A bench-calibrated coupler: the trim wins over the divider.  The
     * fit is "200 uW per code" = 0.2 mW per code, offset 0. */
    g_config.rev1.gain_uW_per_code = 200U;
    g_config.rev1.offset_mW        = 0;

    /* 800 W = 800000 mW -> code 4000000, past full scale, so the
     * threshold saturates instead of wrapping. */
    adc_awd_set_rev1_mW(800000);
    ASSERT_EQ(ADC1->HTR3, ADC_FULL_SCALE);

    /* A limit inside the converter's range lands exactly: 200 mW at
     * 0.2 mW per code is code 1000. */
    adc_awd_set_rev1_mW(200);
    ASSERT_EQ(ADC1->HTR3, 1000U);
    ASSERT_EQ(ADC1->LTR3, 0);

    g_config.rev1.gain_uW_per_code = 0U;
    g_config.rev1.offset_mW        = 0;
}

TEST_CASE(awd2_threshold_saturates_instead_of_wrapping)
{
    opa2000_init();
    /* Calibrated, so the threshold really is computed - otherwise this
     * case would pass for the wrong reason (parked, not saturated). */
    g_config.im_calibrated      = 1U;
    g_config.im_zero_offset_adc = 43254U;
    /* A threshold beyond the 16-bit code range must clamp to full scale,
     * not wrap around to a tiny value that would trip instantly. */
    adc_awd_set_im_mA(60000);
    ASSERT_EQ(ADC1->HTR2, 65535);
    ASSERT_EQ(ADC1->LTR2, 0);
}

TEST_CASE(awd2_threshold_parks_wide_open_without_a_valid_calibration)
{
    opa2000_init();
    /* AWD2's trip point is the zero offset plus the current limit, so a
     * stale or garbage offset would put the hardware line at the wrong
     * current - worse than having no line at all, because a wrong line
     * trips on a good amplifier or misses a real overcurrent.
     *
     * So the current protection is off until the zero-offset calibration
     * has produced a usable number. */
    ASSERT_FALSE(adc_im_protection_valid());

    /* No calibration at all: parked wide open, whatever the stored
     * offset says. */
    g_config.im_calibrated      = 0U;
    g_config.im_zero_offset_adc = 43254U;    /* the nominal value */
    adc_awd_set_im_mA(30000);
    ASSERT_EQ(ADC1->HTR2, ADC_FULL_SCALE);
    ASSERT_EQ(ADC1->LTR2, 0);

    /* Calibrated, but the offset is outside the plausible band: still
     * parked, because a nonsense offset would place the trip point at a
     * nonsense current. */
    g_config.im_calibrated      = 1U;
    g_config.im_zero_offset_adc = 100U;      /* far below the band */
    adc_awd_set_im_mA(30000);
    ASSERT_EQ(ADC1->HTR2, ADC_FULL_SCALE);

    g_config.im_zero_offset_adc = 60000U;    /* far above the band */
    adc_awd_set_im_mA(30000);
    ASSERT_EQ(ADC1->HTR2, ADC_FULL_SCALE);
}

TEST_CASE(awd2_threshold_uses_a_measured_offset)
{
    opa2000_init();
    /* A measured offset inside the band produces a real trip point: the
     * offset plus the current limit.  30 A = 20761 codes. */
    g_config.im_calibrated      = 1U;
    g_config.im_zero_offset_adc = 43254U;
    g_config.im_sensor_type     = IM_SENSOR_BIDIRECTIONAL;
    ASSERT_TRUE(adc_im_protection_valid());
    adc_awd_set_im_mA(30000);
    ASSERT_EQ(ADC1->HTR2, 43254U + 20761U);

    /* A different measured offset moves the hardware line with it - that
     * is the whole point of re-fitting after the calibration. */
    g_config.im_zero_offset_adc = 43000U;
    adc_awd_set_im_mA(30000);
    ASSERT_EQ(ADC1->HTR2, 43000U + 20761U);

    /* And the edges of the plausible band still fit in the code space. */
    g_config.im_zero_offset_adc = IM_ZERO_MAX;
    adc_awd_set_im_mA(30000);
    ASSERT_EQ(ADC1->HTR2, IM_ZERO_MAX + 20761U);
    ASSERT_LT(IM_ZERO_MAX + 20761U, ADC_FULL_SCALE);
}

TEST_CASE(awd_thresholds_round_trip_with_the_reading)
{
    /* The strongest property the conversion has: whatever the operator
     * sees as a power, the watchdog trip point is at or below it.  The
     * point of routing both through dc_raw_to_mW/dc_mW_to_raw is that
     * they cannot disagree - so the watchdog ALWAYS trips on a reading
     * that the operator would describe as "over the limit" and never
     * on one they would describe as "well under it".  This is checked
     * by reading the threshold, computing the power that sits on it via
     * the public conversion, and confirming the relationship. */
    opa2000_init();
    /* Force the trim fields to a known state - opa2000_init() does not
     * touch the coupler calibration struct. */
    g_config.fwd1.gain_uW_per_code = 0U;
    g_config.fwd2.gain_uW_per_code = 0U;
    g_config.rev1.gain_uW_per_code = 0U;
    g_config.rev2.gain_uW_per_code = 0U;
    g_config.fwd1.offset_mW = 0;
    g_config.fwd2.offset_mW = 0;
    g_config.rev1.offset_mW = 0;
    g_config.rev2.offset_mW = 0;

    /* IM (AWD2): the sensor has a published transfer function, so the
     * round-trip is exact. */
    g_config.im_calibrated      = 1U;
    g_config.im_zero_offset_adc = 43254U;
    g_config.im_sensor_type     = IM_SENSOR_BIDIRECTIONAL;
    adc_awd_set_im_mA(10000);
    /* Read back the codes as if they were a fresh conversion: the power
     * at HTR2 must read 10 A (or close, since the inverse uses integer
     * arithmetic). */
    int32_t mA_at_htr = adc1_raw_to_im_mA((uint16_t)ADC1->HTR2);
    ASSERT_GE(mA_at_htr,  9990);
    ASSERT_LE(mA_at_htr, 10010);

    /* The rev1 trim: exact inverse, no approximation.  A gain of
     * 100000 uW/code is 100 mW per code, so a 30 W limit sits on
     * code 300 - and adc1_raw_to_rev1_mW(300) reads back 30 W. */
    g_config.rev1.gain_uW_per_code = 100000U;
    g_config.rev1.offset_mW        = 0;
    adc_awd_set_rev1_mW(30000);           /* 30 W */
    ASSERT_EQ(ADC1->HTR3, 300U);
    ASSERT_EQ(adc1_raw_to_rev1_mW((uint16_t)ADC1->HTR3), 30000);

    /* A limit past what the trim can express saturates rather than
     * wrapping: 300 W would be code 3000, but 3 kW would be 30000 and
     * 300 kW would overflow the converter entirely. */
    adc_awd_set_rev1_mW(300000000);
    ASSERT_EQ(ADC1->HTR3, ADC_FULL_SCALE);

    g_config.rev1.gain_uW_per_code = 0U;
    g_config.rev1.offset_mW        = 0;
}

TEST_CASE(awd_thresholds_dont_interfere_with_each_other)
{
    opa2000_init();
    g_config.im_calibrated      = 1U;
    g_config.im_zero_offset_adc = 43254U;
    g_config.im_sensor_type     = IM_SENSOR_BIDIRECTIONAL;

    /* RF input parks wide open; REV1 has no known divider or trim, also
     * parks wide open.  Only IM produces a real threshold. */
    adc_awd_set_rf_in_mW(200000);
    adc_awd_set_im_mA(10000);
    adc_awd_set_rev1_mW(300000);

    ASSERT_EQ(ADC1->HTR1, ADC_FULL_SCALE);
    ASSERT_EQ(ADC1->HTR2, 43254U + 6920U);
    ASSERT_EQ(ADC1->HTR3, ADC_FULL_SCALE);
}

/*============================================================================
 *  Sensor model detection
 *
 *  Two current sensors are fitted to the same board and they are told
 *  apart by their own resting output: the bidirectional part rests at
 *  1.65 V, the unidirectional one below 0.3 V.  The firmware samples that
 *  resting point during the power-on zero-offset window, so no operator
 *  action and no board-variant flag is involved.
 *==========================================================================*/
TEST_CASE(sensor_model_is_told_apart_by_its_resting_level)
{
    /* The bidirectional part: 1.65 V, and anywhere inside the 40 mV band
     * around it. */
    ASSERT_EQ(adc_im_detect_sensor(IM_ZERO_EXPECTED), IM_SENSOR_BIDIRECTIONAL);
    ASSERT_EQ(adc_im_detect_sensor(IM_ZERO_MIN),     IM_SENSOR_BIDIRECTIONAL);
    ASSERT_EQ(adc_im_detect_sensor(IM_ZERO_MAX),     IM_SENSOR_BIDIRECTIONAL);

    /* The unidirectional part: below 0.3 V, down to and including 0 V -
     * a one-way sensor whose output sits at ground is still a one-way
     * sensor, and its resting point is measured like any other. */
    ASSERT_EQ(adc_im_detect_sensor(0U), IM_SENSOR_UNIDIRECTIONAL);
    ASSERT_EQ(adc_im_detect_sensor(IM_UNI_REST_MAX_CODE), IM_SENSOR_UNIDIRECTIONAL);
    ASSERT_EQ(adc_im_detect_sensor(5000U),                IM_SENSOR_UNIDIRECTIONAL);

    /* Anything between the two windows is neither: a disconnected sensor,
     * a broken one, or a node stuck at some rail voltage.  Reporting
     * UNKNOWN is the honest answer - the current protection must not be
     * armed against a number nobody understands. */
    ASSERT_EQ(adc_im_detect_sensor(IM_ZERO_MAX + 1U),  IM_SENSOR_UNKNOWN);
    ASSERT_EQ(adc_im_detect_sensor(IM_UNI_REST_MAX_CODE + 1U), IM_SENSOR_UNKNOWN);
    ASSERT_EQ(adc_im_detect_sensor(20000U),            IM_SENSOR_UNKNOWN);
    ASSERT_EQ(adc_im_detect_sensor(65535U),            IM_SENSOR_UNKNOWN);
}

TEST_CASE(sensor_model_decides_whether_the_current_protection_is_trustworthy)
{
    /* Not calibrated at all: nothing to trust, whatever the model says. */
    g_config.im_calibrated      = 0U;
    g_config.im_sensor_type     = IM_SENSOR_BIDIRECTIONAL;
    g_config.im_zero_offset_adc = 43254U;
    ASSERT_FALSE(adc_im_protection_valid());

    /* Bidirectional, offset inside the band: live. */
    g_config.im_calibrated = 1U;
    ASSERT_TRUE(adc_im_protection_valid());

    /* Bidirectional, offset outside the band: a nonsense zero point
     * would place the trip point at a nonsense current. */
    g_config.im_zero_offset_adc = IM_ZERO_MAX + 1U;
    ASSERT_FALSE(adc_im_protection_valid());
    g_config.im_zero_offset_adc = IM_ZERO_MIN - 1U;
    ASSERT_FALSE(adc_im_protection_valid());

    /* Unidirectional, offset below the 0.3 V ceiling: live. */
    g_config.im_sensor_type     = IM_SENSOR_UNIDIRECTIONAL;
    g_config.im_zero_offset_adc = 5000U;
    ASSERT_TRUE(adc_im_protection_valid());

    /* Unidirectional, offset above the ceiling: not something such a
     * sensor can produce. */
    g_config.im_zero_offset_adc = IM_UNI_REST_MAX_CODE + 1U;
    ASSERT_FALSE(adc_im_protection_valid());

    /* Unidirectional, offset at ground: a one-way sensor whose output
     * really does sit at 0 V.  The trip point is "the measured zero plus
     * the limit", so it is correct wherever the zero turned out to be. */
    g_config.im_zero_offset_adc = 0U;
    ASSERT_TRUE(adc_im_protection_valid());

    /* Unrecognised model: never live, even with a plausible offset. */
    g_config.im_sensor_type     = IM_SENSOR_UNKNOWN;
    g_config.im_zero_offset_adc = 43254U;
    ASSERT_FALSE(adc_im_protection_valid());
}

TEST_CASE(both_sensor_families_share_the_same_sensitivity)
{
    /* The current conversion is "raw minus the measured resting point,
     * times the fixed sensitivity", so the slope does not depend on the
     * model - only the resting point does.  264 tenths of a mV per amp
     * is 26.4 mV/A, and it is carried in tenths precisely because 26.4
     * is not a whole number of millivolts. */
    ASSERT_EQ(adc_im_mv_per_amp_q1(), 264U);

    /* The same 26.4 mV/A on both parts: 1 A is 692 codes. */
    g_config.im_calibrated      = 1U;
    g_config.im_sensor_type     = IM_SENSOR_BIDIRECTIONAL;
    g_config.im_zero_offset_adc = 43254U;
    ASSERT_GE(adc1_raw_to_im_mA(43254U + 692U),  990);
    ASSERT_LE(adc1_raw_to_im_mA(43254U + 692U), 1010);

    g_config.im_sensor_type     = IM_SENSOR_UNIDIRECTIONAL;
    g_config.im_zero_offset_adc = 5000U;
    ASSERT_GE(adc1_raw_to_im_mA(5000U + 692U),  990);
    ASSERT_LE(adc1_raw_to_im_mA(5000U + 692U), 1010);
}

TEST_CASE(the_measured_zero_point_is_reported_in_millivolts)
{
    g_config.im_zero_offset_adc = 43254U;      /* 1.65 V */
    ASSERT_EQ(adc_im_zero_mV(), 1650U);

    g_config.im_zero_offset_adc = 0U;          /* 0 V */
    ASSERT_EQ(adc_im_zero_mV(), 0U);

    /* 7864 codes = 0.3 V on a 2.5 V reference (299 mV after the integer
     * truncation, one mV below the 0.3 V ceiling it came from). */
    g_config.im_zero_offset_adc = IM_UNI_REST_MAX_CODE;
    ASSERT_EQ(adc_im_zero_mV(), 299U);
    ASSERT_LE(adc_im_zero_mV(), IM_UNI_REST_MAX_MV);
}

TEST_CASE(the_comp2_reference_follows_the_fitted_sensor)
{
    /* The bidirectional sensor on a PERFECT 1.65 V resting point: the
     * derived reference lands on the 2442 mV the default configuration
     * stores, so nothing changes for a correctly fitted part. */
    g_config.im_sensor_type     = IM_SENSOR_BIDIRECTIONAL;
    g_config.im_zero_offset_adc = 43254U;
    ASSERT_EQ(adc_im_zero_mV(), 1650U);
    ASSERT_EQ(adc_im_comp2_ref_mV(2442U, 30000U), 2442U);

    /* The unidirectional sensor: the stored value would trip at nearly
     * twice the intended current, so the reference is derived from the
     * MEASURED zero point and the configured limit:
     *     ref = zero_mV + limit_mA * 26.4 / 1000
     * With a 5000-code resting point (190 mV) and a 30 A limit
     * (30 * 26.4 = 792 mV) that is 982 mV. */
    g_config.im_sensor_type     = IM_SENSOR_UNIDIRECTIONAL;
    g_config.im_zero_offset_adc = 5000U;
    ASSERT_EQ(adc_im_zero_mV(), 190U);
    ASSERT_EQ(adc_im_comp2_ref_mV(2442U, 30000U), 190U + 792U);

    /* An unrecognised sensor keeps the stored value - it is never used,
     * because adc_im_protection_valid() parks the channel. */
    g_config.im_sensor_type     = IM_SENSOR_UNKNOWN;
    ASSERT_EQ(adc_im_comp2_ref_mV(2442U, 30000U), 2442U);
}

TEST_CASE(a_bidirectional_sensor_off_nominal_moves_the_reference_with_its_zero)
{
    /* The stored reference is written against a NOMINAL 1.65 V baseline,
     * but the calibration accepts anything within IM_ZERO_TOLERANCE of it
     * - a couple of hundred millivolts, i.e. several amps.  A sensor
     * resting ABOVE 1.65 V therefore has a 2442 mV reference sitting
     * CLOSER than the configured limit, while AWD2 (built on the measured
     * zero) still waits for the full 30 A.  The comparator is the faster
     * of the two and nothing can veto it, so that gap is a standing
     * source of CURRENT_HIGH popups at a current the amplifier never
     * exceeded - which is exactly the complaint that motivated this.
     *
     * Every offset below is inside the tolerance band on purpose: outside
     * it adc_im_protection_valid() is false and both protections are
     * parked, which would make the test vacuous. */
    g_config.im_calibrated  = 1U;
    g_config.im_sensor_type = IM_SENSOR_BIDIRECTIONAL;

    /* --- Resting HIGH: the dangerous direction ---------------------- */
    g_config.im_zero_offset_adc = 44000U;        /* 1678 mV, ~1.1 A high */
    ASSERT_EQ(adc_im_zero_mV(), 1678U);

    /* The derived reference is what actually trips at 30 A, and it sits
     * ABOVE the stored one because this sensor already sits 28 mV higher
     * than the 1.65 V the stored value was written against.  Left at
     * 2442 mV, the comparator would have fired at
     * (2442 - 1678) / 26.4 = 28.9 A - more than an amp early, on every
     * transmission. */
    ASSERT_EQ(adc_im_comp2_ref_mV(2442U, 30000U), 1678U + 792U);
    ASSERT_GT(adc_im_comp2_ref_mV(2442U, 30000U), 2442U);
    ASSERT_EQ(adc_im_comp2_ref_mV(2442U, 30000U) - 2442U, 28U);

    /* And the watchdog agrees with it at the configured limit, because
     * both are built from the same measured point and the same 26.4
     * mV/A.  This is the agreement that was missing before. */
    adc_awd_set_im_mA(30000U);
    ASSERT_EQ(ADC1->HTR2, 44000U + 20761U);
    uint32_t htr_mV = ADC_RAW_TO_MV((uint16_t)ADC1->HTR2);
    ASSERT_GE(htr_mV, 1678U + 792U - 2U);
    ASSERT_LE(htr_mV, 1678U + 792U + 2U);

    /* --- Resting LOW: trips late, which AWD2 still catches ---------- */
    g_config.im_zero_offset_adc = 40000U;        /* 1525 mV, ~4.7 A low */
    ASSERT_EQ(adc_im_zero_mV(), 1525U);
    ASSERT_EQ(adc_im_comp2_ref_mV(2442U, 30000U), 1525U + 792U);

    adc_awd_set_im_mA(30000U);
    htr_mV = ADC_RAW_TO_MV((uint16_t)ADC1->HTR2);
    ASSERT_GE(htr_mV, 1525U + 792U - 2U);
    ASSERT_LE(htr_mV, 1525U + 792U + 2U);

    /* --- Beyond what the node can reach ------------------------------ */
    /* 48000 codes is 1831 mV: still inside the tolerance band, but
     * 1831 + 792 = 2623 mV is past the 2500 mV the ADC node can even
     * reach.  The ceiling clamps it, so the backstop simply never fires
     * while AWD2 still holds the 30 A line - a degraded backstop rather
     * than a dangerous early trip, which is the right way round to fail. */
    g_config.im_zero_offset_adc = 48000U;
    ASSERT_EQ(adc_im_zero_mV(), 1831U);
    ASSERT_EQ(adc_im_comp2_ref_mV(2442U, 30000U), COMP2_REF_MV_MAX);
}

TEST_CASE(the_comparator_and_the_watchdog_trip_at_the_same_current)
{
    /* The comparator is the backstop for the watchdog, so both have to
     * fire at the SAME current - otherwise the operator gets two
     * different limits and neither of them is the one on the label.
     *
     * That is why the comparator reference is built from the same
     * sensitivity the AWD2 threshold and the current reading use: 26.4
     * mV/A.  Rounding it to 26 mV/A would put the backstop 1.6 % away
     * from the limit it is backing up. */
    const uint16_t limit_mA = 30000U;

    g_config.im_calibrated      = 1U;
    g_config.im_sensor_type     = IM_SENSOR_UNIDIRECTIONAL;
    g_config.im_zero_offset_adc = 5000U;      /* 190 mV resting point */

    /* The comparator reference sits at the voltage the sensor reaches at
     * the configured limit. */
    uint32_t ref_mV = adc_im_comp2_ref_mV(2442U, limit_mA);
    ASSERT_EQ(ref_mV, 190U + 792U);

    /* And the watchdog threshold sits at the code the sensor reaches at
     * the same limit: 30 A is 20761 codes above the resting point. */
    adc_awd_set_im_mA(limit_mA);
    ASSERT_EQ(ADC1->HTR2, 5000U + 20761U);

    /* Converting the watchdog's own trip point back to millivolts has to
     * land on the comparator's reference - that is the whole point of
     * routing both through the same transfer function. */
    uint32_t htr_mV = ADC_RAW_TO_MV((uint16_t)ADC1->HTR2);
    ASSERT_GE(htr_mV, ref_mV - 2U);
    ASSERT_LE(htr_mV, ref_mV + 2U);

    /* The current reading at the watchdog's trip point is the limit
     * itself. */
    int32_t mA_at_htr = adc1_raw_to_im_mA((uint16_t)ADC1->HTR2);
    ASSERT_GE(mA_at_htr, 29900);
    ASSERT_LE(mA_at_htr, 30100);
}

TEST_CASE(the_comp2_reference_cannot_exceed_the_dac_ceiling)
{
    /* A limit that would put the reference past the comparator's range
     * is clamped, not wrapped. */
    g_config.im_sensor_type     = IM_SENSOR_UNIDIRECTIONAL;
    g_config.im_zero_offset_adc = 7864U;       /* 300 mV */
    ASSERT_EQ(adc_im_comp2_ref_mV(2442U, 500000U), COMP2_REF_MV_MAX);
}

/*============================================================================
 *  Suite table
 *==========================================================================*/

const test_entry_t g_suite_adc_calibration[] = {
    TEST_ENTRY(adc_raw_to_mV_endpoints),
    TEST_ENTRY(if_envelope_conversion_is_millivolts),
    TEST_ENTRY(vbias_is_a_direct_voltage_sense),
    TEST_ENTRY(vm_undoes_the_300k_over_10k_divider),
    TEST_ENTRY(vm_full_scale_follows_the_reference_and_the_divider),
    TEST_ENTRY(key_channel_passes_the_raw_code_through),
    TEST_ENTRY(im_conversion_is_zero_until_calibrated),
    TEST_ENTRY(im_conversion_is_zero_at_the_calibrated_offset),
    TEST_ENTRY(im_conversion_matches_the_26_4mV_per_amp_slope),
    TEST_ENTRY(the_im_scale_factor_is_one_milliamp_per_code_point_445),
    TEST_ENTRY(im_conversion_is_negative_below_the_offset),
    TEST_ENTRY(im_conversion_is_monotonic),
    TEST_ENTRY(im_conversion_endpoints_do_not_wrap),
    TEST_ENTRY(power_is_zero_inside_the_rectifier_dead_band),
    TEST_ENTRY(power_matches_the_double_precision_model),
    TEST_ENTRY(power_reference_points_are_reachable),
    TEST_ENTRY(power_full_scale_leaves_headroom_over_the_transmit_limits),
    TEST_ENTRY(all_four_coupler_channels_share_the_same_curve),
    TEST_ENTRY(each_coupler_applies_its_own_trim),
    TEST_ENTRY(a_negative_trim_never_reads_as_power),
    TEST_ENTRY(a_calibrated_coupler_ignores_the_rectifier_dead_band),
    TEST_ENTRY(power_conversion_is_monotonic),
    TEST_ENTRY(power_conversion_does_not_wrap_at_full_scale),
    TEST_ENTRY(the_inverse_produces_a_code_that_reads_back_as_the_requested_power),
    TEST_ENTRY(the_inverse_and_the_forward_agree_across_the_whole_range),
    TEST_ENTRY(the_inverse_saturates_instead_of_wrapping),
    TEST_ENTRY(the_public_api_matches_the_per_channel_couplers),
    TEST_ENTRY(the_chain_adds_the_rectifier_drop_between_the_two_dividers),
    TEST_ENTRY(ntc_reads_25C_at_the_expected_code),
    TEST_ENTRY(the_tap_rises_with_temperature),
    TEST_ENTRY(ntc_tracks_the_beta_equation_over_the_full_range),
    TEST_ENTRY(ntc_is_monotonic_increasing_with_the_tap_voltage),
    TEST_ENTRY(ntc_open_circuit_reports_the_minimum),
    TEST_ENTRY(ntc_full_scale_reads_hot_not_cold),
    TEST_ENTRY(ntc_protection_threshold_is_reachable_on_this_wiring),
    TEST_ENTRY(aux_and_core_ntc_share_the_same_curve),
    TEST_ENTRY(mcu_internal_sensor_zero_c_is_300mV),
    TEST_ENTRY(mcu_internal_sensor_100c_is_4_3V),
    TEST_ENTRY(mcu_internal_sensor_clamps_below_range),
    TEST_ENTRY(awd1_threshold_parks_wide_open_until_the_divider_is_known),
    TEST_ENTRY(awd2_threshold_is_offset_from_the_zero_point),
    TEST_ENTRY(awd3_threshold_parks_wide_open_until_either_input_is_known),
    TEST_ENTRY(awd3_threshold_uses_the_bench_trim_when_available),
    TEST_ENTRY(awd2_threshold_saturates_instead_of_wrapping),
    TEST_ENTRY(awd2_threshold_parks_wide_open_without_a_valid_calibration),
    TEST_ENTRY(awd2_threshold_uses_a_measured_offset),
    TEST_ENTRY(awd_thresholds_round_trip_with_the_reading),
    TEST_ENTRY(awd_thresholds_dont_interfere_with_each_other),
    TEST_ENTRY(sensor_model_is_told_apart_by_its_resting_level),
    TEST_ENTRY(sensor_model_decides_whether_the_current_protection_is_trustworthy),
    TEST_ENTRY(both_sensor_families_share_the_same_sensitivity),
    TEST_ENTRY(the_measured_zero_point_is_reported_in_millivolts),
    TEST_ENTRY(the_comp2_reference_follows_the_fitted_sensor),
    TEST_ENTRY(a_bidirectional_sensor_off_nominal_moves_the_reference_with_its_zero),
    TEST_ENTRY(the_comparator_and_the_watchdog_trip_at_the_same_current),
    TEST_ENTRY(the_comp2_reference_cannot_exceed_the_dac_ceiling),
    TEST_SUITE_END
};