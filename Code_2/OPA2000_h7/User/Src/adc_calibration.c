/**
 * @file    adc_calibration.c
 * @brief   ADC raw code -> engineering unit conversions + AWD thresholds.
 *
 * @details All conversions are integer-only so they are safe to call from
 *          ISRs and from the protection hot path (no FPU, no libm).
 *
 *          Nominal (uncalibrated) front-end constants live in this file and
 *          MUST be replaced by the bench calibration of the couplers /
 *          dividers before the product ships.  See
 *          OPA2000_H7_固件设计要求.txt section [ADC] for the electrical
 *          basis of each channel.
 *
 * @version 1.0
 * @date    2026-10
 * @author  Control Agent (wt-control branch)
 */

#include "adc_calibration.h"
#include "opa2000.h"
#include "stm32h7xx_hal.h"

/*============================================================================
 *  Nominal front-end constants
 *==========================================================================*/

/* Main supply voltage sense: 300k upper / 10k lower divider, buffered.
 *
 *   V_pin = Vm * 10k / (300k + 10k) = Vm / 31
 *   Vm    = V_pin * 31
 *
 * The ADC reference is 2.5 V, so code 65535 means V_pin = 2.5 V and
 * Vm = 2.5 * 31 = 77.5 V.  Derive the full scale from the resistors rather
 * than writing 77500: a bare constant gives no hint about where it came
 * from, and the previous 50000 (a 1:20 divider that is not on this board)
 * was exactly that kind of untraceable number. */
#define VM_SENSE_R_TOP_OHMS        300000L
#define VM_SENSE_R_BOTTOM_OHMS     10000L
#define VM_SENSE_DIVIDER_RATIO     ((VM_SENSE_R_TOP_OHMS + VM_SENSE_R_BOTTOM_OHMS) \
                                    / VM_SENSE_R_BOTTOM_OHMS)             /* 31 */
#define VM_NOMINAL_FULL_SCALE_MV   ((int64_t)ADC_VREF_MV * VM_SENSE_DIVIDER_RATIO)

/* NTC front end: 3V3 -> NTC -> tap -> R_pull (2.2k) -> GND.
 *
 * The NTC is on the HIGH side and the fixed resistor pulls the tap DOWN,
 * so the tap voltage RISES with temperature: a hot NTC has a small
 * resistance and the divider is dominated by the 2.2k, which pushes the
 * node up towards 3V3.  The inverse of the more common pull-up arrangement,
 * and getting it backwards is not a small error - it inverts the whole
 * scale, so 90 C reads as -60 C and the over-temperature protection never
 * fires while the amplifier cooks.
 *
 *   v_adc = Vcc * R_pull / (R_ntc + R_pull)
 *   =>    R_ntc = R_pull * (Vcc - v_adc) / v_adc
 *
 * Note the (Vcc - v) / v, NOT v / (Vcc - v): that is the whole difference
 * between this wiring and a pull-up.
 */
#define NTC_SUPPLY_MV                 3300L
#define NTC_R0_OHMS                   10000L     /*!< NTC @ 25 C          */
#define NTC_PULL_OHMS                 2200L      /*!< pull-down to GND    */
#define NTC_BETA                      3950L      /*!< 10K B3950 part      */
#define NTC_T0_K                      29815L      /*!< 29.815 * 1000 */
#define NTC_INV_T0_Q30                3601348    /*!< round(2^30 / 298.15 K) */

/* Sanity clamp for the temperature outputs (0.1 C).
 *
 * The upper clamp is above what the hardware can produce: with this divider
 * the tap reaches the ADC's 2.5 V full scale at R_ntc = 704 ohm, which the
 * Beta equation puts at about 99.7 C for a 10K B3950 part.  Anything the
 * front end can report is below that, so this is a "cannot happen" guard
 * rather than a real reading. */
#define TEMP_MAX_TENTH_C              1200

/* Lower clamp.  Open circuit (NTC disconnected) pulls the tap to GND
 * through the 2.2k, i.e. 0 mV, which this divider would otherwise read as
 * an astronomically cold NTC.  -40 C is the bottom of the range the
 * protection logic cares about. */
#define TEMP_MIN_TENTH_C              (-400)
/* 0.1 C = 273.15 K, rounded up by half a 0.1 C step. */
#define KELVIN_TO_TENTH_C             2732

/*============================================================================
 *  Internal helpers
 *==========================================================================*/

/* ln(2) in Q16.16. */
#define LN2_Q16   45426

/**
 * @brief ln(x) for x given as unsigned Q16.16, result signed Q16.16.
 *
 * Uses ln(m) = 2*atanh(z), z = (m-1)/(m+1), with m normalised to [1,2)
 * so |z| <= 1/3 and the odd series converges in 4 terms (< 1e-6 error).
 */
static int32_t ln_q16_fixed(uint32_t x_q16)
{
    int32_t k = 0;
    int32_t z, z2, z3, z5, z7;

    while (x_q16 < 0x10000u) { x_q16 <<= 1; k--; }
    while (x_q16 >= 0x20000u) { x_q16 >>= 1; k++; }

    z = (int32_t)(((int64_t)(x_q16 - 0x10000u) << 16) / (x_q16 + 0x10000u));
    z2 = (int32_t)(((int64_t)z * z) >> 16);
    z3 = (int32_t)(((int64_t)z2 * z) >> 16);
    z5 = (int32_t)(((int64_t)z3 * z2) >> 16);
    z7 = (int32_t)(((int64_t)z5 * z3) >> 16);

    return 2 * (z + z3 / 3 + z5 / 5 + z7 / 7) + k * LN2_Q16;
}

/**
 * @brief NTC divider -> temperature using the Beta equation.
 *
 * @param raw ADC code on the NTC tap
 * @return temperature in 0.1 C, clamped to [TEMP_MIN_TENTH_C,
 *         TEMP_MAX_TENTH_C]
 */
static int16_t ntc_raw_to_tenth_c(uint16_t raw)
{
    uint32_t v_mv = ADC_RAW_TO_MV(raw);
    uint32_t r_ratio_q16;
    int32_t  ln_q16;
    int32_t  ln_over_b_q30;
    int32_t  inv_q30;
    int64_t  t_q30;
    int32_t  tenth;

    /* Pull-down topology: R_ntc = R_pull * (Vcc - v) / v.
     *
     * The subtraction has to come first and it has to be 64-bit-clean:
     * v_mv tops out at 2500 (the ADC range) so the 32-bit form would be
     * fine here, but writing it as (Vcc - v) makes the sign and the
     * direction of the curve obvious at a glance, which is what stops
     * someone "fixing" it back to the pull-up form later.
     *
     * Q16 keeps the ratio in a form ln_q16_fixed() can take directly. */
    if (v_mv == 0u) {
        return TEMP_MIN_TENTH_C;   /* NTC open: tap pulled to GND */
    }
    r_ratio_q16 = (uint32_t)((((uint64_t)(NTC_SUPPLY_MV - v_mv) * NTC_PULL_OHMS) << 16)
                            / ((uint64_t)v_mv * NTC_R0_OHMS));
    if (r_ratio_q16 == 0u) return TEMP_MIN_TENTH_C;

    /* r_ratio_q16 is R_ntc / R0 already, because the R_pull/R0 scaling is
     * folded into the expression above.  With a pull-up it would have been
     * R_ntc/R_pull and needed the pull-up's own value correcting afterwards;
     * here the two resistors do not cancel (2.2k vs 10k) and folding them
     * in once is the only place the arithmetic happens. */
    ln_q16 = ln_q16_fixed(r_ratio_q16);

    ln_over_b_q30 = (int32_t)(((int64_t)ln_q16 * 16384) / NTC_BETA);
    inv_q30 = NTC_INV_T0_Q30 + ln_over_b_q30;

    /* inv_q30 goes to 0 as the temperature goes to infinity, which would
     * make the divide below blow up.  Clamp it to the temperature the
     * divider can actually report (TEMP_MAX) before dividing. */
    {
    int32_t inv_min_q30 = (int32_t)(((int64_t)1 << 60)
                       / (((int64_t)(TEMP_MAX_TENTH_C + KELVIN_TO_TENTH_C))
                          << 30 / 10));
    if (inv_q30 < inv_min_q30) inv_q30 = inv_min_q30;
    }

    t_q30 = ((int64_t)1 << 60) / inv_q30;                /* Kelvin, Q30 */
    tenth = (int32_t)((t_q30 * 10) >> 30) - KELVIN_TO_TENTH_C;

    if (tenth < TEMP_MIN_TENTH_C) tenth = TEMP_MIN_TENTH_C;
    if (tenth > TEMP_MAX_TENTH_C) tenth = TEMP_MAX_TENTH_C;
    return (int16_t)tenth;
}

/* OPA2000 directional-coupler model, integer form (see adc_calibration.h
 * section 1b for the derivation from the board's resistor values):
 *   v_det_mV = adc_mV * POST_GAIN_Q1 / 10      (undo the 220k/100k pair)
 *   v_pre_mV = v_det_mV + Vf                  (undo the Schottky drop)
 *   vrf_mV   = v_pre_mV * PRE_GAIN_Q1 / 10     (undo 120/330 and 33 turns)
 *   P_mW     = vrf_mV^2 / 100000               (Z0 / 2 = 100 ohm)
 *
 * `trim` is that coupler's bench calibration.  It is an ABSOLUTE fit
 * against the raw ADC code, not a scale factor on the base model:
 *
 *   P_mW = raw * gain_uW_per_code / 1000 + offset_mW
 *
 * which is the same formula the coupler calibration screen displays and
 * stores (gui.c: power_mW = raw * gain_uW_per_code / 1000 + offset_mW),
 * and the one documented on the coupler_t type in opa2000.h.  The gain
 * is micro-watts per ADC code, so it carries the units of the whole
 * curve - applying it to the base model's output instead would be a
 * dimensional error.
 *
 * The four couplers are separate parts with separate transformers, diode
 * drops and tap positions, so they are NOT interchangeable: a trim written
 * for coupler 2 says nothing about coupler 1.  Passing the trim in here is
 * what makes adc1_raw_to_fwd1_mW() and adc1_raw_to_fwd2_mW() actually
 * reflect their own calibration instead of sharing one curve.
 *
 * A trim that is entirely zero (the default from config_reset_default())
 * means "not calibrated": the nominal model is used instead, so an
 * uncalibrated unit behaves exactly as it did before the trims existed.
 * A calibrated unit always has a non-zero gain - the calibration screen
 * derives it from two measured points, and a zero slope would mean the
 * two points were identical, which the screen rejects.
 *
 * `post_gain_q1` is the channel's own front-end gain (adc_calibration.h
 * section 1c): the factor that scales the detector's output up to the
 * ADC pin.  0 means "not measured yet" and selects the nominal
 * ADC_DC_POST_GAIN_Q1, which is what the four directional couplers use.
 *
 * The trim, when present, wins over the nominal gain: it is measured on
 * the actual board, so it is always the better description of it.
 *
 * RETURN VALUE: forward power in mW, 0 below the rectifier dead band.
 * Monotonic in \p raw - the curve never goes down as the code goes up -
 * which is what lets dc_mW_to_raw() binary-search it.
 *
 * \p post_gain_q1 is the channel's own detector-to-ADC divider, Q1 (times
 * ten).  0 means "not measured yet" and selects ADC_DC_POST_GAIN_Q1, the
 * nominal 220k/100k pair of this board. */
static int32_t dc_raw_to_mW(uint16_t raw,
                             const volatile coupler_t *trim,
                             uint32_t post_gain_q1)
{
    /* The "deci" suffix means one tenth of a millivolt (deci- = 10^-1),
     * i.e. these two are scaled by 10 relative to millivolts.  See the
     * block comment below for why the extra digit is needed. */
    uint32_t det_deci;   /* detector node, in 1/10 mV */
    uint32_t pre_deci;   /* detector node + Schottky drop, in 1/10 mV */
    uint32_t vrf_mV;
    int64_t  p_mW;

    if (trim != NULL && trim->gain_uW_per_code != 0) {
        /* Bench-calibrated coupler: straight line through the raw code.
         * The dead band does not apply here - the fit already carries the
         * rectifier's own offset. */
        p_mW = ((int64_t)raw * trim->gain_uW_per_code) / 1000
             + trim->offset_mW;
        if (p_mW < 0) p_mW = 0;  /* a negative trim must not read as power */
        return (int32_t)p_mW;
    }

    if (raw < ADC_DC_DEADBAND_CODE) return 0;

    if (post_gain_q1 == 0U) {
        post_gain_q1 = ADC_DC_POST_GAIN_Q1;   /* nominal board divider */
    }

    /* Undo the chain one stage at a time, in the reverse of the order the
     * signal travels.  The order matters: the Schottky drop sits BETWEEN
     * the two dividers, so it has to be added back before the transformer
     * ratio is applied, not after.  Adding it at the far end (as the old
     * single-gain model did) would scale the 200 mV by the wrong factor
     * and shift the whole curve.
     *
     * v_det = v_adc * post_gain_q1 / 10
     * v_pre = v_det + Vf
     * vrf   = v_pre * pre_gain_q1 / 10
     *
     * The detector node is carried in TENTHS of a millivolt (the "deci"
     * suffix: deci- = 10^-1), not whole millivolts.  That is not cosmetic:
     * the detector node is where the smallest signal lives, and truncating
     * it to a whole millivolt there would throw away a tenth of the very
     * quantity that is being measured.  At 100 W - where the detector node
     * sits at about 70 mV - a whole-millivolt truncation is already 1.4 %
     * of the reading, and at the low end it dominates the result
     * completely.  In tenths the only rounding left is the final division,
     * worth half a millivolt at the tap against a full scale of 369 V.
     *
     * Because post_gain_q1 is itself a Q1 value (i.e. the divider ratio
     * times ten), multiplying by it here is EXACT: 3.2 x 10 = 32 lands on a
     * whole number of tenths, so nothing is lost between the ADC pin and
     * the detector node.
     *
     * Bounds: v_adc is at most 2500 mV and post_gain_q1 is a Q1 value, so
     * det_deci is at most a few hundred thousand; the product below
     * stays far inside 64 bits even for a large override. */
    det_deci = (uint32_t)ADC_RAW_TO_MV(raw) * post_gain_q1;   /* 1/10 mV */
    pre_deci = det_deci + ((uint32_t)ADC_DC_VF_MV * 10U);     /* 1/10 mV */
    vrf_mV   = (uint32_t)(((uint64_t)pre_deci * (uint32_t)ADC_DC_PRE_GAIN_Q1) / 100U);

    p_mW = ((int64_t)vrf_mV * vrf_mV) / 100000;

    if (p_mW < 0) p_mW = 0;
    return (int32_t)p_mW;
}

/* Inverse of dc_raw_to_mW(): milliwatts -> ADC code, saturating at full
 * scale.  Used to turn the configured protection limits into AWD
 * thresholds, which live in ADC code space.
 *
 * RETURN VALUE: the smallest code whose FORWARD reading is at least
 * \p mw.  That is deliberately the safe direction - the threshold trips
 * at or just above the limit, never below it.  A limit that trips a
 * hair late is a nuisance; one that trips early cuts the amplifier off
 * while it is still inside its limit.
 *
 * The trim path is inverted exactly: the fit is a straight line, so the
 * code that sits on the limit is (mw - offset) * 1000 / gain.
 *
 * The nominal path has to be solved the long way round, because the
 * forward curve is a square and the integer divisions inside it do not
 * invert exactly.  Rather than solve the algebra and hope the two agree
 * to the last code, this walks the forward function itself over the
 * whole 16-bit range and picks the code whose reading is the first one
 * to reach the limit.  That way the watchdog trips at exactly the power
 * the operator sees on the screen - the two can never disagree, which
 * is the whole point of putting them through one function. */
static uint32_t dc_mW_to_raw(uint32_t mw,
                              const volatile coupler_t *trim,
                              uint32_t post_gain_q1)
{
    if (trim != NULL && trim->gain_uW_per_code != 0) {
        /* Straight line: exact inverse.  offset_mW may be negative, so
         * the arithmetic is done in 64 bit. */
        int64_t span = (int64_t)mw - (int64_t)trim->offset_mW;
        if (span <= 0) return 0U;
        {
            int64_t raw = (span * 1000) / trim->gain_uW_per_code;
            if (raw < 0) raw = 0;
            if (raw > (int64_t)ADC_FULL_SCALE) raw = (int64_t)ADC_FULL_SCALE;
            return (uint32_t)raw;
        }
    }

    if (mw == 0U) return 0U;

    if (post_gain_q1 == 0U) {
        post_gain_q1 = ADC_DC_POST_GAIN_Q1;   /* nominal board divider */
    }

    /* Walk the forward curve.  It is monotonic in raw, so a binary search
     * over the code range finds the first code whose reading reaches the
     * limit.  This costs ~16 evaluations rather than a sweep over
     * 65536 codes.
     *
     * The search calls dc_raw_to_mW() ITSELF rather than repeating the
     * arithmetic.  That is the whole point: the forward curve contains
     * two integer divisions that do not invert exactly, so a hand-written
     * inverse would land on a code whose reading is a hair's breadth
     * above or below the limit, and the watchdog would then trip at a
     * different power than the operator sees on the screen.  Driving both
     * directions from one function makes that impossible by
     * construction. */
    {
        uint32_t lo = 0U;
        uint32_t hi = ADC_FULL_SCALE;

        while (lo < hi) {
            uint32_t mid = lo + (hi - lo) / 2U;
            if ((uint32_t)dc_raw_to_mW((uint16_t)mid, NULL, post_gain_q1) >= mw) {
                hi = mid;
            } else {
                lo = mid + 1U;
            }
        }
        return lo;
    }
}

/*============================================================================
 *  ADC1 conversions
 *==========================================================================*/

int32_t adc1_raw_to_rf_in_mW(uint16_t raw)
{
    /* The RF input detector (D5/R40 peak detector) has no coupler, so it
     * does not follow the OPA1000 DC model.  Same curve for now; the real
     * slope belongs in the RF-input calibration once the bench numbers
     * are available.  No trim either - there is no fifth coupler to
     * calibrate, and borrowing one of the four would silently couple the
     * RF-input reading to an unrelated part.
     *
     * The one thing that IS specific to this channel is its front-end
     * gain (ADC_DC_POST_GAIN_RF_IN_Q1), because the D5/R40 network has its
     * own divider.  Until that number is filled in the nominal OPA1000
     * gain is used, exactly as before. */
    return dc_raw_to_mW(raw, NULL, ADC_DC_POST_GAIN_RF_IN_Q1);
}

/* Each of the four couplers carries its own bench trim.  They are four
 * separate transformers with their own diode drops and tap positions, so
 * they are deliberately NOT share one curve: a coupler-2 correction
 * applied to coupler 1 would be a calibration error with no symptom to
 * give it away. */
int32_t adc1_raw_to_fwd1_mW(uint16_t raw) { return dc_raw_to_mW(raw, &g_config.fwd1, 0U); }
int32_t adc1_raw_to_fwd2_mW(uint16_t raw) { return dc_raw_to_mW(raw, &g_config.fwd2, 0U); }
int32_t adc1_raw_to_rev1_mW(uint16_t raw) { return dc_raw_to_mW(raw, &g_config.rev1, ADC_DC_POST_GAIN_REV1_Q1); }
int32_t adc1_raw_to_rev2_mW(uint16_t raw) { return dc_raw_to_mW(raw, &g_config.rev2, 0U); }

/*============================================================================
 *  Directional-coupler power, both directions (public)
 *
 *  The nominal chain with no bench trim - i.e. what the board does before
 *  any coupler has been calibrated.  See adc_calibration.h section 2a for
 *  the contract.
 *
 *  WHY THERE IS A REVERSE DIRECTION AT ALL
 *  --------------------------------------
 *  A conversion function normally needs only one direction, because the
 *  reading is what the firmware consumes.  The exception is a channel that
 *  has an ANALOG WATCHDOG behind it: the watchdog's threshold register
 *  holds an ADC CODE, not a wattage, so a limit the operator configures in
 *  watts has to be converted BACK into code space before it can be
 *  programmed.  On this board those channels are exactly:
 *
 *      AWD1  RF input power      -> adc_dc_mW_to_raw()
 *      AWD2  main current       -> ma_to_adc_code()
 *      AWD3  coupler-1 reverse  -> adc_dc_mW_to_raw()
 *
 *  Every other channel (NTC, PA bias, main voltage, the IF envelopes, the
 *  key resistor) is compared in SOFTWARE, in engineering units, against
 *  the forward reading - so it needs the forward direction only.  The rule
 *  to remember when adding a channel: write the inverse if and only if
 *  there is a hardware threshold to program.
 *
 *  Note that the reverse is NOT a hot path.  It runs when a watchdog
 *  threshold is programmed: at power-on, and again whenever the band
 *  changes or the operator edits a protection limit (see
 *  boot_apply_thresholds()).  A few microseconds there is free.
 *==========================================================================*/

int32_t adc_dc_raw_to_mW(uint16_t raw)
{
    /* No trim, nominal front end.  The dead band applies here (a code
     * below ADC_DC_DEADBAND_CODE reads 0 W: the detector has not started
     * conducting, so the reading is noise). */
    return dc_raw_to_mW(raw, NULL, 0U);
}

uint16_t adc_dc_mW_to_raw(uint32_t mW)
{
    /* The inverse is defined as "the smallest code whose reading reaches
     * the requested power", and it is computed by asking the FORWARD
     * function - not by solving the algebra.  That is deliberate: the
     * forward curve contains two integer divisions that do not invert
     * exactly, so a hand-written inverse would land on a code whose
     * reading is a hair's breadth off the limit, and a watchdog programmed
     * from it would trip at a different power than the panel shows.
     *
     * The rounding direction is the safe one: the returned code reads at
     * or just ABOVE the requested power, never below it.  A threshold that
     * trips slightly late is a nuisance; one that trips early would cut
     * the amplifier off while it is still within its limit.
     *
     * Saturates at ADC_FULL_SCALE for a power the chain cannot represent
     * (full scale is about 1362 W), so an unreachable limit parks the
     * watchdog wide open rather than wrapping to a small code that would
     * let the fault slip through. */
    return (uint16_t)dc_mW_to_raw(mW, NULL, 0U);
}


int32_t adc1_raw_to_im_mA(uint16_t raw)
{
    int64_t ma;

    /* 0 A = the sensor's own resting output, 26.4 mV/A.
     * I_mA = delta_code * 2500 / 65535 * 1000 / 26.4
     *       = delta_code * 25000000 / 17301240
     *
     * The formula is the same for both sensor families: it is always
     * "raw minus the MEASURED resting point", and the resting point is
     * what differs (1.65 V for the bidirectional part, near 0 V for the
     * unidirectional one).  That is why the calibration has to run
     * before this can produce a meaningful number, and why the sensor
     * model is detected during that same window.
     *
     * 17301240 = 25000000 * 26.4 / 1000. */
    if (g_config.im_calibrated == 0u) return 0;

    ma = ((int64_t)((int32_t)raw - (int32_t)g_config.im_zero_offset_adc)
          * 25000000) / 17301240;

    /* The sensor measures +-50 A; anything beyond +-100 A is a wiring
     * fault and must not be allowed to wrap the int32 result. */
    if (ma >  100000) ma =  100000;
    if (ma < -100000) ma = -100000;
    return (int32_t)ma;
}

int32_t adc1_raw_to_vbias_sense_mV(uint16_t raw)
{
    /* Direct sense (R32 series + C65 low-pass), no divider. */
    return (int32_t)ADC_RAW_TO_MV(raw);
}

int32_t adc1_raw_to_vm_mV(uint16_t raw)
{
    /* Pin voltage then undo the divider:
     *   V_pin = raw * 2500 / 65535
     *   Vm    = V_pin * 31
     * folded into one 64-bit multiply by the full-scale constant, which is
     * derived from the resistor values above.  A 0-code input reads 0 V and
     * full scale reads 77 500 mV; nothing here can overflow int32 because
     * 65535 * 77500 is about 5.1e9, so the product is taken in int64. */
    return (int32_t)(((int64_t)raw * VM_NOMINAL_FULL_SCALE_MV) / ADC_FULL_SCALE);
}

uint16_t adc1_raw_to_key_adc_code(uint16_t raw) { return raw; }

/*============================================================================
 *  ADC2 conversions
 *==========================================================================*/

int16_t adc2_raw_to_if_pa_env_mV(uint16_t raw)
{
    return (int16_t)ADC_RAW_TO_MV(raw);
}

int16_t adc2_raw_to_if_radio_env_mV(uint16_t raw)
{
    return (int16_t)ADC_RAW_TO_MV(raw);
}

int16_t adc2_inj_raw_to_core_ntc_tenth_c(uint16_t raw)
{
    return ntc_raw_to_tenth_c(raw);
}

int16_t adc2_inj_raw_to_aux_ntc_tenth_c(uint16_t raw)
{
    return ntc_raw_to_tenth_c(raw);
}

int16_t adc2_inj_raw_to_mcu_internal_tenth_c(uint16_t raw)
{
    /* STM32 internal sensor: 0.3 V at 0 C, 4.3 mV/K slope.
     * T_C = (mV - 300) / 43  ->  T_tenth = (mV - 300) * 10 / 43 */
    int32_t mv = (int32_t)ADC_RAW_TO_MV(raw);
    int32_t tenth;

    if (mv < 300) return TEMP_MIN_TENTH_C;              /* below sensor range */
    tenth = ((mv - 300) * 10) / 43;
    if (tenth < TEMP_MIN_TENTH_C) tenth = TEMP_MIN_TENTH_C;
    return (int16_t)tenth;
}

/*============================================================================
 *  ADC analog watchdog thresholds
 *
 *  The STM32H7B0 ADC has one threshold pair per watchdog (LTR1/HTR1,
 *  LTR2/HTR2, LTR3/HTR3), so AWD1/AWD2/AWD3 can carry independent limits.
 *  Writing the threshold registers directly is safe while conversions run
 *  (only the CFGR watchdog routing must stay unchanged).
 *==========================================================================*/

uint8_t adc_im_detect_sensor(uint16_t raw)
{
    uint32_t code = (uint32_t)raw;

    /* The bidirectional sensor rests at 1.65 V (code 43254) and the
     * unidirectional one below 0.3 V (code 7864).  The two windows are
     * far apart, so neither can be mistaken for the other. */
    if (code >= IM_ZERO_MIN && code <= IM_ZERO_MAX) {
        return IM_SENSOR_BIDIRECTIONAL;
    }
    if (code <= IM_UNI_REST_MAX_CODE) {
        return IM_SENSOR_UNIDIRECTIONAL;
    }
    /* Neither signature.  A disconnected sensor, a broken one, a node
     * stuck at a rail voltage, or a reference voltage that is not what
     * the firmware assumes all land here.  Reporting UNKNOWN is the
     * honest answer: the current protection stays parked rather than
     * being armed against a number we do not understand. */
    return IM_SENSOR_UNKNOWN;
}

uint32_t adc_im_mv_per_amp_q1(void)
{
    /* Both fitted families share the 26.4 mV/A slope the current path
     * is built around (the CC6920BSO-50A transfer function), so this is
     * one number today.  It lives behind an accessor so that a family
     * with a different slope only has to change here - and so that the
     * AWD2 threshold, the comparator reference and the current reading
     * cannot disagree about it.
     *
     * Returned in TENTHS of a mV per amp, because 26.4 is not a whole
     * number of millivolts.  Rounding it to 26 would put the
     * comparator's trip point 1.6 % away from the watchdog's, and the
     * backstop would then fire at a different current than the limit it
     * exists to back up. */
    return 264U;   /* 26.4 mV/A */
}

uint32_t adc_im_zero_mV(void)
{
    /* The measured resting point of the fitted sensor, in millivolts. */
    return ((uint32_t)g_config.im_zero_offset_adc * ADC_VREF_MV) / ADC_FULL_SCALE;
}

uint32_t adc_im_comp2_ref_mV(uint32_t configured_mV, uint32_t limit_mA)
{
    /* The comparator reference is "the voltage at which the current
     * reaches the limit", and it is DERIVED - never taken from the
     * operator's stored value, which the GUI lets them trim on the bench:
     *
     *     ref = measured_zero_mV + limit_mA * mV_per_A / 1000
     *
     * BOTH sensor families go through this.  A stored reference is
     * expressed against a NOMINAL 1.65 V baseline, while AWD2 and the
     * current readout are both expressed against the MEASURED resting
     * point.  The two agree only if the fitted sensor really does sit at
     * exactly 1.65 V, and the firmware never required that: the zero
     * offset calibration accepts anything within IM_ZERO_TOLERANCE of
     * 1.65 V, which is a couple of hundred millivolts - i.e. several
     * amps of current error.
     *
     * So on a sensor resting HIGH, a 2442 mV stored reference sits CLOSER
     * than the configured limit: a sensor at 1831 mV would trip at
     * (2442 - 1831) / 26.4 = 23 A, i.e. 7 A early, while AWD2 - which
     * uses the measured zero - still waited for the full 30 A.  The
     * hardware comparator is the faster of the two and nothing downstream
     * can veto it, so the amplifier sat there tripping CURRENT_HIGH at a
     * current it was perfectly happy with.  That was a standing source of
     * nuisance alarms.  Deriving both from the same measured point is what
     * makes the backstop agree with the number on screen.
     *
     * The opposite offset moves the other way: a sensor resting LOW trips
     * LATE (2442 against a 1449 mV zero is 37.6 A).  AWD2 still catches
     * that at 30 A, so a late backstop is a degraded one rather than a
     * dangerous one - which is the asymmetry that makes it worth fixing
     * rather than tolerating.
     *
     * The unidirectional sensor is the same idea taken to its extreme:
     * its baseline is below 0.3 V, so 2442 mV would trip at nearly twice
     * the intended current - a backstop that backs nothing up.
     *
     * configured_mV survives only as the fallback for an unrecognised
     * sensor, where no measured zero exists.  The caller parks the
     * channel in that case (adc_im_protection_valid() is false), so the
     * number never arms anything. */
    if (g_config.im_sensor_type == IM_SENSOR_UNKNOWN) {
        return configured_mV;
    }

    uint64_t ref = (uint64_t)adc_im_zero_mV()
                  + ((uint64_t)limit_mA * adc_im_mv_per_amp_q1()) / 10000U;
    return (ref > (uint64_t)COMP2_REF_MV_MAX) ? (uint32_t)COMP2_REF_MV_MAX
                                              : (uint32_t)ref;
}

bool adc_im_protection_valid(void)
{
    uint32_t off = (uint32_t)g_config.im_zero_offset_adc;

    /* Two conditions, and both are needed:
     *
     *   1. The calibration has COMPLETED.  g_config.im_calibrated is set
     *      only by the zero-offset state machine, and it is cleared on
     *      every boot (adc_init()), so this says "measured on this
     *      unit, this session".
     *
     *   2. The offset it produced is inside the plausible band FOR THE
     *      DETECTED SENSOR.  A corrupted or hand-edited config could
     *      carry a plausible flag with a nonsense offset, and a nonsense
     *      offset would place the trip point at a nonsense current.
     *
     * Only the current protection depends on this.  The amplifier may
     * be keyed and every other protection stays armed regardless - see
     * the block comment in adc_calibration.h. */
    if (g_config.im_calibrated == 0U) {
        return false;
    }

    switch (g_config.im_sensor_type) {
    case IM_SENSOR_BIDIRECTIONAL:
        /* The CC6920's resting point is 1.65 V nominally, and the
         * measured offset has to be inside the tolerance band around it. */
        return (off >= IM_ZERO_MIN) && (off <= IM_ZERO_MAX);

    case IM_SENSOR_UNIDIRECTIONAL:
        /* The one-way sensor rests below 0.3 V, and its resting point is
         * measured wherever it actually sits - down to 0 V.  The
         * conversion is always "raw minus the measured resting point",
         * so the trip point is correct wherever the zero turned out to
         * be. */
        return (off <= IM_UNI_REST_MAX_CODE);

    case IM_SENSOR_UNKNOWN:
    default:
        /* The sensor was not recognised, so the resting point is not
         * known and the current cannot be measured or protected. */
        return false;
    }
}

static uint32_t ma_to_adc_code(uint32_t ma)
{
    /* inverse of I_mA = delta_code * 25000000 / 17301240 */
    uint32_t delta = (uint32_t)(((uint64_t)ma * 17301240u + 12500000u) / 25000000u);
    uint32_t off   = (uint32_t)g_config.im_zero_offset_adc;
    uint32_t code;

    /* The trip point of AWD2 is the zero offset plus the current limit,
     * so it is only meaningful once the offset has been MEASURED.  Until
     * then the stored offset is whatever the factory default or the last
     * saved config left behind - and a stale one would put the hardware
     * line at the wrong current, which is worse than having no line at
     * all (a wrong line trips on a good amplifier, or misses a real
     * overcurrent).
     *
     * So: no valid calibration parks the watchdog wide open.  It cannot
     * trip, and the moment the calibration measures a good offset the
     * threshold is re-fitted (boot_apply_thresholds() from
     * adc_stabilisation_tick()). */
    if (!adc_im_protection_valid()) {
        return ADC_FULL_SCALE;
    }

    code = off + delta;
    return (code > ADC_FULL_SCALE) ? ADC_FULL_SCALE : code;
}

/*============================================================================
 *  Analog watchdog threshold setters
 *
 *  USE_AWD (see opa2000.h) takes the three watchdogs out of the build.
 *
 *  Two things have to happen for that, and they are NOT the same thing:
 *
 *    1. The firmware must stop reacting  - main.c's HAL_ADC*LevelOutOfWindow*
 *       callbacks become empty, so a trip no longer reaches
 *       protection_trigger_locked().
 *    2. The HARDWARE must stop tripping - the thresholds have to be parked
 *       at full scale.  Otherwise the ADC raises its interrupt on every
 *       single conversion above the threshold, and ADC1_2_IRQHandler runs
 *       at main priority 0: an interrupt storm that starves the scheduler.
 *
 *  Only (2) is done here, and it is done through awd_threshold() rather
 *  than by #ifdef-ing each function body.  Two reasons: the code stays
 *  readable, and the static helpers below (ma_to_adc_code, dc_mW_to_raw)
 *  keep being referenced, so the compiler does not warn about them being
 *  unused in a USE_AWD-off build.
 *==========================================================================*/

/*!< Returns `value` when the watchdog is live, ADC_FULL_SCALE when the
 *   USE_AWD switch has taken it out of the build.  A threshold at full
 *   scale can never be crossed by a conversion result, so the watchdog
 *   cannot trip at all. */
static inline uint32_t awd_threshold(uint32_t value)
{
#ifdef USE_AWD
    return value;
#else
    (void)value;
    return ADC_FULL_SCALE;
#endif
}

void adc_awd_set_rf_in_mW(uint32_t max_mW)
{
    uint32_t htr;

    /* If the D5/R40 divider has been measured (ADC_DC_POST_GAIN_RF_IN_Q1 != 0),
     * the watchdog is driven by the same curve the display reads, so the
     * hardware trip point and the value on the screen cannot disagree.
     * Until then the high threshold sits at full scale and the watchdog
     * cannot trip on a curve nobody has verified.  Whichever way it is,
     * LTR is zero so a falling-edge crossing can never latch. */
    if (ADC_DC_POST_GAIN_RF_IN_Q1 != 0U) {
        htr = dc_mW_to_raw(max_mW, NULL, ADC_DC_POST_GAIN_RF_IN_Q1);
    } else {
        htr = ADC_FULL_SCALE;
    }
    ADC1->HTR1 = awd_threshold(htr);
    ADC1->LTR1 = 0u;
}

void adc_awd_set_im_mA(uint16_t max_mA)
{
    ADC1->HTR2 = awd_threshold(ma_to_adc_code(max_mA));
    ADC1->LTR2 = 0u;
}

/*! AWD3 watches coupler 1 reverse power (PA -> LPF side).  The bench
 *  trim is checked first; if the operator has calibrated this coupler it
 *  wins.  Otherwise the divider constant drives the watchdog.  0 means
 *  the divider is unknown and the watchdog stays parked wide open. */
void adc_awd_set_rev1_mW(uint32_t max_mW)
{
    uint32_t htr;

    if (g_config.rev1.gain_uW_per_code != 0) {
        htr = dc_mW_to_raw(max_mW, &g_config.rev1, 0U);
    } else if (ADC_DC_POST_GAIN_REV1_Q1 != 0U) {
        htr = dc_mW_to_raw(max_mW, NULL, ADC_DC_POST_GAIN_REV1_Q1);
    } else {
        htr = ADC_FULL_SCALE;
    }
    ADC1->HTR3 = awd_threshold(htr);
    ADC1->LTR3 = 0u;
}
