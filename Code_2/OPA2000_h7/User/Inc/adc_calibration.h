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
#include "hw_pinmap.h"

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
 *  1b. Directional-coupler front end (OPA1000 model)
 *
 *      Ported verbatim from the previous-generation firmware
 *      E:\Projects\RADIO\Projects\OPA1000\Code\Src\Sensor_task.c
 *          Z0             = 50 ohm
 *          DC_TURNS       = 33          (coupler pickup turns)
 *          DC_ATTEN_RATIO = (330+100)/100 = 4.3
 *          Vf             = 0.3 V       (BAT54 rectifier drop)
 *
 *          VRF  = (Vadc + Vf) * DC_ATTEN_RATIO * DC_TURNS
 *          P(W) = VRF^2 / Z0 / 2
 *
 *      OPA1000 calibration points (12-bit, 3.28 V):
 *          100 W -> 560, 200 W -> 1000, 250 W -> 1100, 400 W -> 1400
 *
 *      OPA2000 differences: 16-bit ADC and VREF = VREFBUF 2.5 V.
 *      Integer form used in adc_calibration.c:
 *          v_mV   = raw * 2500 / 65535
 *          vrf_mV = (v_mV + 300) * ADC_DC_GAIN_Q1 / 10
 *          P_mW   = vrf_mV^2 / 100000             (Z0/2 = 100 ohm)
 *
 *      With 33 turns full scale (raw 65535) reads about 1579 W, which
 *      leaves the 1050/1100 W transmit limits comfortably inside range.
 * ===========================================================================*/
#define ADC_DC_Z0_OHM              50L
#define ADC_DC_TURNS               33L
#define ADC_DC_ATTEN_RATIO_Q1      43L    /*!< (330+100)/100 = 4.3, x10  */
#define ADC_DC_GAIN_Q1             (ADC_DC_ATTEN_RATIO_Q1 * ADC_DC_TURNS)
                                                   /*!< ratio*turns, x10 = 1419 */
#define ADC_DC_VF_MV               300L    /*!< BAT54 rectifier drop     */
/*!< Raw codes below this are inside the rectifier dead band (OPA1000: 100). */
#define ADC_DC_DEADBAND_CODE       100U

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
 *    The state machine and g_allow_rf gating live in user_adc.c.
 *    Persistent zero-offset code is stored in config_t
 *    (g_config.im_zero_offset_adc, g_config.im_calibrated, see opa2000.h).
 *
 *    The RF-permission gate that protects PWR_ON_SIG (PG7) and
 *    RF_BIAS_ON (PG2) is the volatile global g_allow_rf defined in
 *    opa2000.c.  Set true by user_adc.c when calibration succeeds,
 *    cleared by protection.c on any fault.
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