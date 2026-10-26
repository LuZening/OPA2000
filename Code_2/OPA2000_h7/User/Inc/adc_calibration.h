/**
 * @file    adc_calibration.h
 * @brief   ADC raw -> engineering-unit conversion functions and
 *          calibration constants.
 *
 * @details Every ADC channel in the OPA2000 amplifier is converted
 *          from raw 16-bit value (0..65535) to a physical engineering
 *          unit (mW, mA, mV, 0.1 C, ADC code) using the prototype
 *          functions in this header.
 *
 *          The conversion is performed in **integer arithmetic** to
 *          keep determinism and avoid floating-point cost in
 *          display/protection hot paths.
 *
 *          Slope / offset coefficients are stored in `config_t` (see
 *          config.h).  Initial values are placeholders pending
 *          hardware calibration; production units MUST run the
 *          coupler calibration state machine before relying on
 *          absolute power values.
 *
 * @version 1.0
 * @date    2026-10
 * @author  Control Agent (wt-control branch)
 */

#ifndef ADC_CALIBRATION_H
#define ADC_CALIBRATION_H

#include <stdint.h>
#include <stdbool.h>
#include "hw_pinmap.h"
/* For the IM_SENSOR_* model identifiers, which live next to the
 * im_sensor_type field they describe. */
#include "opa2000.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===========================================================================
 *  1. ADC reference / constants
 * ===========================================================================*/
#define ADC_VREF_MV             2500U       /*!< VREF+ = VREFBUF 2.5V      */
#define ADC_FULL_SCALE          65535U      /*!< 16-bit ADC full scale    */
#define ADC_RESOLUTION_BITS     16U

/*! Convert raw 16-bit ADC sample to millivolts at the ADC pin. */
#define ADC_RAW_TO_MV(raw)   (((uint32_t)(raw) * ADC_VREF_MV) / ADC_FULL_SCALE)

/* ===========================================================================
 *  1b. Directional-coupler front end (OPA2000 hardware)
 *
 *      The signal chain from the coupler tap to the ADC pin, as built on
 *      this board:
 *
 *        RF tap
 *          -> 33:1 step-down transformer            (DC_TURNS)
 *          -> divider 1: 120 R upper, 330 R lower   (DIV1)
 *          -> Schottky detector, Vf = 200 mV        (VF_MV)
 *          -> peak-hold capacitor (ideal: no droop, no time constant)
 *          -> divider 2: 220 k upper, 100 k lower   (DIV2)
 *          -> buffer
 *          -> ADC pin
 *
 *      Forward, from the ADC pin back up to the coupler tap:
 *
 *        v_det_mV = v_adc_mV * (DIV2_UP + DIV2_DOWN) / DIV2_DOWN
 *        v_pre_mV = v_det_mV + VF_MV                  (undo the Schottky drop)
 *        v_tap_mV = v_pre_mV * (DIV1_UP + DIV1_DOWN) / DIV1_DOWN * DC_TURNS
 *
 *        P_mW = v_tap_mV^2 / 100000                  (Z0 / 2 = 100 ohm)
 *
 *      The tap voltage is a PEAK voltage, so the power is V^2/(2*Z0) with
 *      Z0 = 50 ohm - the same convention the previous generation used, and
 *      the same reason: a coupler taps a fraction of a sinusoidal line.
 *
 *      Both gain factors are exact integers in Q1 (tenths), so the whole
 *      chain collapses without any rounding of its own:
 *
 *        pre_gain_q1  = DC_TURNS * (DIV1_UP+DIV1_DOWN) * 10 / DIV1_DOWN = 450
 *        post_gain_q1 = (DIV2_UP+DIV2_DOWN) * 10 / DIV2_DOWN            =  32
 *
 *        v_tap_mV = (v_adc_mV * 32/10 + 200) * 450/10
 *                 = v_adc_mV * 144 + 9000
 *
 *      At 16-bit full scale (v_adc = 2500 mV) that is 369 V at the tap,
 *      i.e. 1362 W.  The 1050/1100 W transmit limits therefore sit at
 *      about 90 % of the ADC range - inside it, but with less headroom
 *      than the previous model had.
 *
 *      The capacitor is ideal, so there is no droop to model and no time
 *      constant to wait for: the ADC reading IS the held peak.
 *
 *      IMPLEMENTATION NOTE: the formula above is written in millivolts
 *      because that is how the hardware is described, but the integer
 *      implementation in adc_calibration.c carries the detector node in
 *      TENTHS of a millivolt instead (the "deci" suffix on the local
 *      variables: deci- = 10^-1).  Truncating the detector node to a
 *      whole millivolt would throw away a tenth of the smallest quantity
 *      in the chain - at 100 W that node sits at about 70 mV, so the
 *      truncation would already be 1.4 % of the reading.  The extra digit
 *      costs nothing: both gains are Q1 values, so multiplying by them in
 *      this scale is exact and the only rounding left is the final
 *      division.
 * ===========================================================================*/
#define ADC_DC_Z0_OHM              50L
#define ADC_DC_TURNS               33L    /*!< coupler pickup turns        */
#define ADC_DC_DIV1_UP_OHM         120L   /*!< transformer side -> node    */
#define ADC_DC_DIV1_DOWN_OHM       330L   /*!< node -> ground              */
#define ADC_DC_DIV2_UP_OHM         220000L /*!< detector -> ADC pin         */
#define ADC_DC_DIV2_DOWN_OHM       100000L /*!< ADC pin -> ground           */
#define ADC_DC_VF_MV               200L   /*!< Schottky detector drop      */

/*!< Turns and divider 1 together, x10.  This is the gain from the tap up
 *   to the detector anode, i.e. everything before the diode. */
#define ADC_DC_PRE_GAIN_Q1  ((ADC_DC_TURNS * (ADC_DC_DIV1_UP_OHM + ADC_DC_DIV1_DOWN_OHM) * 10) / ADC_DC_DIV1_DOWN_OHM)
                                                   /*!< 33 * 450/330 = 450 */
/*!< Divider 2 on its own, x10: detector output to ADC pin.  This is the
 *   "D : 1 (V_detector : V_ADC)" of the per-channel override below. */
#define ADC_DC_POST_GAIN_Q1 (((ADC_DC_DIV2_UP_OHM + ADC_DC_DIV2_DOWN_OHM) * 10) / ADC_DC_DIV2_DOWN_OHM)
                                                   /*!< 320/100 = 32            */
/*!< Raw codes below this are inside the rectifier dead band, so the
 *   detector has not started conducting and the reading is noise.  With
 *   the OPA2000 chain this is about 50 W - low enough to be useful. */
#define ADC_DC_DEADBAND_CODE       5000U

/* ===========================================================================
 *  1a. Main-current zero offset - the CC6920BSO-50A resting code
 *
 *  The sensor reads 1650 mV at 0 A, which is 1.65 / 2.5 * 65535 = 43254
 *  codes on a 2.5 V reference.  The calibration measures the ACTUAL
 *  resting code of the unit in front of it and stores it in
 *  g_config.im_zero_offset_adc.
 *
 *  The tolerance band is what makes a measured offset usable: a result
 *  outside it means the sensor, the reference or the wiring is not what
 *  the firmware assumes, and the reading cannot be trusted.  5000 codes
 *  = 190 mV on a 2.5 V reference, i.e. about 7 A - wide enough to
 *  accept a real part and any reasonable offset trim.
 *
 *  That width is exactly why the current protection is built on the
 *  MEASURED point rather than the nominal 1.65 V: both AWD2 and the
 *  COMP2 reference (adc_im_comp2_ref_mV()) add the limit to whatever the
 *  calibration measured, so a sensor resting several amps off nominal
 *  still trips at the current the operator configured.  See the comment
 *  on adc_im_comp2_ref_mV() in adc_calibration.c.
 *
 *  The band is also sized so that the offset plus the widest current
 *  limit usually fits in the 16-bit code space:
 *      43254 + 20761 (30 A) = 64015   < 65535
 *  At the top of the band it does not (43254 + 5000 + 20761 = 69015),
 *  so ma_to_adc_code() saturates at full scale there rather than
 *  wrapping - a wrapped threshold would sit near zero codes and trip
 *  on no current at all.
 * ===========================================================================*/
#define IM_ZERO_EXPECTED          43254U
#define IM_ZERO_TOLERANCE         5000U
#define IM_ZERO_MIN               (IM_ZERO_EXPECTED - IM_ZERO_TOLERANCE)
#define IM_ZERO_MAX               (IM_ZERO_EXPECTED + IM_ZERO_TOLERANCE)

/* ===========================================================================
 *  1b. Main-current sensor models
 *
 *  Two sensor families are fitted to the same board, and they are told
 *  apart by their own resting output - which the firmware samples
 *  during the power-on zero-offset window, so no operator action and no
 *  board-variant flag is needed.
 *
 *  BIDIRECTIONAL (CC6920BSO-50A and relatives)
 *      0 A reads 1.65 V, i.e. raw code 43254 on a 2.5 V reference.
 *      Current flows both ways, so the transfer function is
 *      V = 1650 mV + I * 26.4 mV/A and the zero point has to be
 *      measured per unit.
 *
 *  UNIDIRECTIONAL (a one-way sensor)
 *      0 A reads below 0.3 V, i.e. raw code below 7864.  The transfer
 *      function is V = Vrest + I * 26.4 mV/A: the SAME sensitivity, just
 *      a different resting point and no negative-current range.  So the
 *      current conversion itself does not change - it is always "raw
 *      minus the measured resting point, times the fixed sensitivity" -
 *      and what differs is where the resting point is EXPECTED to be.
 *
 *  The two detection thresholds are deliberately far apart (43254 vs
 *  below 7864) so that a sensor which is neither - a broken one, a
 *  disconnected one, or a node sitting at some rail voltage - cannot be
 *  mistaken for a valid one.
 *
 *  The unidirectional window has only a ceiling, and it runs all the way
 *  down to 0 V: a one-way sensor whose output sits at ground is still a
 *  one-way sensor, and its resting point is measured just like any
 *  other.  Nothing about the current reading or the trip point depends
 *  on where that resting point is - the conversion is always "raw minus
 *  the MEASURED resting point" - so there is no reason to exclude the
 *  low end of the range.
 *
 *  One assumption holds all of this together, and it is worth stating
 *  because the whole design rests on it: the quoted resting levels
 *  (1.65 V and 0.3 V) are the voltages AT THE ADC PIN - i.e. after the
 *  U20 buffer - not at the sensor's own output.  That is what makes a
 *  single sensitivity valid for both families: the buffer is the same
 *  part in both cases, so the same 26.4 mV/A arrives at the pin and the
 *  only thing that changes is where the resting point sits.
 * ===========================================================================*/

/*!< Resting level of the unidirectional sensor, in millivolts: "below
 *   0.3 V at 0 A" as the sensor's datasheet says.  0.3 V on a 2.5 V
 *   reference is raw code 7864. */
#define IM_UNI_REST_MAX_MV           300U
#define IM_UNI_REST_MAX_CODE         ((uint32_t)IM_UNI_REST_MAX_MV * 65535U / 2500U)

/*!< The bidirectional sensor's resting level, in millivolts, and the
 *   code that corresponds to it. */
#define IM_BIDI_REST_MV              1650U
#define IM_BIDI_REST_CODE            ((uint32_t)IM_BIDI_REST_MV * 65535U / 2500U)

/* ===========================================================================
 *  1c.  Per-channel front-end gain  (BOARD property - to be measured)
 *
 *  Every DC-coupled detector output reaches the ADC pin through a
 *  resistive divider, and that ratio is a board property: it can only
 *  be read off the schematic or measured on the bench.  It is NOT the
 *  same for every channel, which is exactly why the shared nominal
 *  constant above can only be a placeholder.
 *
 *  The value is the factor that scales the detector's output up to the
 *  ADC pin, times ten - the "D : 1 (V_detector : V_ADC)" of that
 *  channel's divider:
 *
 *      v_det_mV = v_adc_mV * post_gain_q1 / 10
 *      v_pre_mV = v_det_mV + ADC_DC_VF_MV
 *      v_rf_mV  = v_pre_mV * ADC_DC_PRE_GAIN_Q1 / 10
 *      P_mW     = v_rf_mV^2 / 100000
 *
 *  For a board divider of D : 1 (V_detector : V_ADC) the value to use
 *  is post_gain_q1 = 10 * D.  The nominal value is ADC_DC_POST_GAIN_Q1
 *  (32 = 3.2, from the 220k/100k pair) and is what the four
 *  directional couplers use until they are bench-calibrated.
 *
 *  0 means "not measured yet".  Then:
 *    - the READING falls back to the nominal curve, so the operator
 *      still sees a plausible number on the panel, and
 *    - the channel's analog watchdog threshold is parked at full
 *      scale, so it cannot trip on a curve nobody has verified.
 *
 *  Fill the number in and the threshold comes up by itself: it is
 *  computed from the very same function the reading uses, so the
 *  hardware trip point and the value on the screen can never disagree.
 * ===========================================================================*/
#define ADC_DC_POST_GAIN_RF_IN_Q1  0U   /*!< D5/R40 peak detector -> ADC1_INP4 */
#define ADC_DC_POST_GAIN_REV1_Q1   0U   /*!< coupler 1 reverse     -> ADC1_INP6  */


/* ===========================================================================
 *  2. ADC1 conversion prototypes
 * ===========================================================================*/
int32_t adc1_raw_to_rf_in_mW       (uint16_t raw);  /*!< Rank 0  INP4  (PC4)  */
int32_t adc1_raw_to_im_mA          (uint16_t raw);  /*!< Rank 1  INP5  (PB1)  */
int32_t adc1_raw_to_rev1_mW        (uint16_t raw);  /*!< Rank 2  INP6  (PF12) */
int32_t adc1_raw_to_fwd1_mW        (uint16_t raw);  /*!< Rank 3  INP8  (PC5)  */
int32_t adc1_raw_to_rev2_mW        (uint16_t raw);  /*!< Rank 4  INP9  (PB0)  */
int32_t adc1_raw_to_vbias_sense_mV (uint16_t raw);  /*!< Rank 5  INP14 (PA2)  */
int32_t adc1_raw_to_fwd2_mW        (uint16_t raw);  /*!< Rank 6  INP15 (PA3)  */
int32_t adc1_raw_to_vm_mV          (uint16_t raw);  /*!< Rank 7  INP16 (PA0)  */
uint16_t adc1_raw_to_key_adc_code   (uint16_t raw);  /*!< Rank 8  INP17 (PA1)  */

/* ===========================================================================
 *  2a. Directional-coupler power, both directions
 *
 *  The forward direction (raw -> mW) is what the display and the
 *  protection limits use.  The reverse direction (mW -> raw) is what the
 *  analog watchdogs need: a limit expressed in watts has to become a
 *  threshold in ADC code space.
 *
 *  Both directions go through ONE implementation, so a threshold written
 *  into a watchdog register and the number shown on the panel can never
 *  disagree about where the limit is.  The inverse is therefore defined
 *  as "the smallest code whose reading reaches the requested power",
 *  which is what makes the watchdog trip at exactly the power the
 *  operator sees - not at some nearby code that rounds to the same watt.
 *
 *  Both use the nominal front end (the four directional couplers) and no
 *  bench trim, so they describe the board rather than a particular
 *  calibrated coupler.  For a trim-aware threshold use the per-channel
 *  path in adc_calibration.c, which is what boot_apply_thresholds() calls.
 *
 *  WHICH CHANNELS NEED A REVERSE DIRECTION
 *  -----------------------------------------
 *  A conversion normally needs the forward direction only, because the
 *  reading is what the firmware consumes.  The exception is a channel with
 *  an ANALOG WATCHDOG behind it: that threshold register holds an ADC
 *  CODE, not an engineering unit, so a limit configured in watts has to
 *  be converted back into code space before it can be programmed.  On this
 *  board:
 *
 *      AWD1  RF input power      -> adc_dc_mW_to_raw()
 *      AWD2  main current       -> ma_to_adc_code()
 *      AWD3  coupler-1 reverse  -> adc_dc_mW_to_raw()
 *
 *  Every other channel - NTC, PA bias, main voltage, the IF envelopes, the
 *  key resistor - is compared in SOFTWARE, in engineering units, against
 *  the forward reading, so it needs the forward direction only.  The rule
 *  for a new channel: write the inverse if and only if there is a
 *  hardware threshold to program.
 *
 *  The reverse is not a hot path either: it runs when a threshold is
 *  programmed - at power-on, and on every band change or protection-limit
 *  edit (boot_apply_thresholds()).
 * ===========================================================================*/

/*!< Nominal coupler chain: ADC code -> forward power in mW.  Same curve as
 *   adc1_raw_to_fwd1_mW() and friends.  Reads 0 for a code below
 *   ADC_DC_DEADBAND_CODE, where the detector has not started conducting
 *   and the reading would be noise. */
int32_t  adc_dc_raw_to_mW (uint16_t raw);

/*!< Nominal coupler chain: forward power in mW -> ADC code.  Returns the
 *   smallest code whose reading is at least \p mW, i.e. the code at which
 *   the limit is actually reached.  The rounding is to the safe side: the
 *   returned code reads at or just above \p mW, never below it, so a
 *   watchdog programmed from it trips late rather than early.
 *
 *   Saturates at ADC_FULL_SCALE for a power the chain cannot represent
 *   (full scale is about 1362 W), so an unreachable limit parks the
 *   watchdog wide open rather than wrapping to a small code. */
uint16_t adc_dc_mW_to_raw (uint32_t mW);

/* ===========================================================================
 *  3. ADC2 conversion prototypes
 * ===========================================================================*/
int16_t adc2_raw_to_if_pa_env_mV    (uint16_t raw);  /*!< Rank 0 INP0  (PC2_C) */
int16_t adc2_raw_to_if_radio_env_mV (uint16_t raw);  /*!< Rank 1 INP1  (PC3_C) */

int16_t adc2_inj_raw_to_core_ntc_tenth_c(uint16_t raw);   /*!< InjRank 1 INP2  (PF13) */
int16_t adc2_inj_raw_to_aux_ntc_tenth_c (uint16_t raw);   /*!< InjRank 2 INP6  (PF14) */
int16_t adc2_inj_raw_to_mcu_internal_tenth_c(uint16_t raw); /*!< InjRank 3 TEMPSENSOR */

/* ===========================================================================
 *  4. Main current chip (CC6920BSO-50A) zero-offset calibration
 *
 *    The state machine lives in user_adc.c.  Persistent zero-offset code
 *    is stored in config_t (g_config.im_zero_offset_adc,
 *    g_config.im_calibrated, see opa2000.h).
 *
 *    This is a MEASUREMENT step, not a safety interlock: it produces the
 *    number that lets the current be reported in amps, and it lets the
 *    AWD2 threshold be fitted to the measured zero.  It does NOT gate the
 *    transmit path - the amplifier may be keyed as soon as the power-on
 *    silence is over, whether or not this calibration ever succeeds.
 *
 *    The RF-permission gate that protects PWR_ON_SIG (PG7) and
 *    RF_BIAS_ON (PG2) is the volatile global g_allow_rf defined in
 *    opa2000.c.  Set true by user_adc.c once the power-on silence is
 *    over, cleared by protection.c on any P0 fault and restored when the
 *    operator dismisses it.
 * ===========================================================================*/

/* (no additional prototypes required - g_allow_rf / g_config live in
 * opa2000.h) */

/* ===========================================================================
 *  5. ADC watchdog threshold setters (called from band/config update)
 * ===========================================================================*/
void adc_awd_set_rf_in_mW(uint32_t max_mW);    /*!< AWD1, RF input       */
void adc_awd_set_im_mA   (uint16_t max_mA);    /*!< AWD2, main current   */
void adc_awd_set_rev1_mW (uint32_t max_mW);    /*!< AWD3, reflected power 1 */

/* ===========================================================================
 *  5a. Is the main-current protection trustworthy?
 *
 *  The main-current protection - AWD2 (the hardware watchdog on CH5) and
 *  COMP2 (the comparator on the same node) - is expressed RELATIVE to the
 *  CC6920's zero offset: both the watchdog threshold and the comparator
 *  reference are "zero offset plus the current limit".  That only works
 *  if the zero offset is known.
 *
 *  So this predicate is the gate for the whole current-protection chain:
 *  it is true only after a zero-offset calibration has completed AND the
 *  offset it produced is inside the plausible band FOR THE DETECTED
 *  SENSOR.  While it is false, both AWD2 and COMP2 are parked where they
 *  cannot trip - a main overcurrent that is not detected is better than
 *  one that is detected at the wrong current.
 *
 *  Note what this does NOT gate: the amplifier may still be keyed, and
 *  every other protection (RF input, reflected power, SWR, PA bias,
 *  over-temperature) stays fully armed.  The zero-offset calibration is a
 *  measurement step; only the current protection depends on its result.
 * ===========================================================================*/
bool adc_im_protection_valid(void);

/* ===========================================================================
 *  5b. Which main-current sensor is fitted?
 *
 *  Told apart by the sensor's own resting output, measured during the
 *  power-on zero-offset window:
 *
 *      around 43254 (1.65 V)  ->  IM_SENSOR_BIDIRECTIONAL
 *      below 7864   (0.30 V)  ->  IM_SENSOR_UNIDIRECTIONAL
 *      anything else          ->  IM_SENSOR_UNKNOWN
 *
 *  "Anything else" is a real case, not a theoretical one: a disconnected
 *  sensor, a broken one, or a node stuck at a rail voltage all read
 *  something that matches neither signature.  Those must not be
 *  mistaken for a working sensor, so they stay UNKNOWN and the current
 *  protection stays parked.
 *
 *  @param raw  the average of the zero-offset sampling window
 *  @return one of the IM_SENSOR_* values
 * ===========================================================================*/
uint8_t adc_im_detect_sensor(uint16_t raw);

/*!< The sensitivity of the fitted sensor, in TENTHS of a mV per amp
 *   (so the value carries one decimal place).  Both families use the same
 *   26.4 mV/A slope (the CC6920BSO-50A transfer function the whole
 *   current path is built around), so this is a single constant today -
 *   but it lives behind an accessor so a third family with a different
 *   slope only has to change here, and so that the AWD2 threshold, the
 *   comparator reference and the current reading cannot disagree about
 *   it.
 *
 *   The Q1 scaling is what keeps it exact: 26.4 mV/A is not a whole
 *   number of millivolts, and rounding it to 26 would put the
 *   comparator's trip point 1.6 % away from the watchdog's - the
 *   backstop would fire at a different current than the limit it is
 *   supposed to be backing up. */
uint32_t adc_im_mv_per_amp_q1(void);

/*!< The measured resting point of the fitted sensor, in millivolts. */
uint32_t adc_im_zero_mV(void);

/*!< The comparator reference voltage for the main-current limit.
 *
 *   For the bidirectional sensor this is the operator's stored value
 *   (the GUI lets them trim it on the bench, and it is expressed
 *   against a 1.65 V baseline).
 *
 *   For the unidirectional sensor the stored value would trip at nearly
 *   twice the intended current, so the reference is derived from the
 *   MEASURED zero point and the configured limit instead:
 *
 *       ref = zero_mV + limit_mA * mV_per_A / 1000
 *
 * @param configured_mV  the stored reference (g_config.comp2_ref_mV)
 * @param limit_mA       the configured current limit
 */
uint32_t adc_im_comp2_ref_mV(uint32_t configured_mV, uint32_t limit_mA);

/* ===========================================================================
 *  6. Convenience: convert & format helper
 * ===========================================================================*/
/*! Convert ADC code (16-bit) to fixed-point millivolt in q4.4 format
 *  for fast printf-style formatting.  1 LSB = 1/16 mV.
 *  Example: 2500 mV -> 40000 q4.4 -> display "2.500 V".
 */
static inline int32_t adc_raw_to_mV_q4_4(uint16_t raw) {
    /* raw * 2500 / 65535 scaled to q4.4 = raw * 16 / 655.35 */
    return ((int32_t)raw * 16) / 655;
}

#ifdef __cplusplus
}
#endif

#endif /* ADC_CALIBRATION_H */
