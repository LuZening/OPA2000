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

/* Main supply voltage sense: R110 divider + U20-A buffer.  Nominal mapping:
 * ADC full scale == 50 V. */
#define VM_NOMINAL_FULL_SCALE_MV      50000L

/* NTC front end: 3V3 -> R35 (pull-up) -> NTC -> GND, tap to ADC. */
#define NTC_SUPPLY_MV                 3300L
#define NTC_R0_OHMS                   10000L     /*!< NTC @ 25 C */
#define NTC_PULL_OHMS                 10000L     /*!< R35        */
#define NTC_BETA                      3435L
#define NTC_T0_K                      29815L      /*!< 29.815 * 1000 */
#define NTC_INV_T0_Q30                3601348    /*!< round(2^30 / 298.15 K) */

/* Sanity clamp for the temperature outputs (0.1 C). */
#define TEMP_MIN_TENTH_C              (-400)
/* 0.1 C = 273.15 K, rounded up by half a 0.1 C step. */
#define KELVIN_TO_TENTH_C             2732

/*============================================================================
 *  Internal helpers
 *==========================================================================*/

/* ln(2) in Q16.16. */
#define LN2_Q16   45426

/**
 * @brief floor(sqrt(n)) for a 64-bit n, by binary search.
 *
 * Only used to invert the coupler power curve for the AWD thresholds, so
 * it runs at task rate, never in an ISR.
 */
static uint32_t isqrt_u64(uint64_t n)
{
    uint64_t lo = 0U, hi = 1U << 22, mid;   /* (2^22)^2 = 2^44 > any P*1e5 */

    while (lo < hi) {
        mid = lo + ((hi - lo + 1U) / 2U);
        if (mid <= (n / mid)) lo = mid;
        else                   hi = mid - 1U;
    }
    return (uint32_t)lo;
}

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
    z7 = (int32_t)(((int64_t)z5 * z2) >> 16);

    return 2 * (z + z3 / 3 + z5 / 5 + z7 / 7) + k * LN2_Q16;
}

/**
 * @brief NTC divider -> temperature using the Beta equation.
 * @param raw ADC code on the NTC tap
 * @return temperature in 0.1 C, clamped to [TEMP_MIN, TEMP_MAX]
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

    r_ratio_q16 = (v_mv << 16) / (NTC_SUPPLY_MV - v_mv);
    if (r_ratio_q16 == 0u) return TEMP_MIN_TENTH_C;      /* NTC open */

    /* R_ntc/R0 = (R_ntc/R35) * (R35/R0); with R35 == R0 the two cancel. */
    ln_q16 = ln_q16_fixed(r_ratio_q16);

    ln_over_b_q30 = (int32_t)(((int64_t)ln_q16 * 16384) / NTC_BETA);
    inv_q30 = NTC_INV_T0_Q30 + ln_over_b_q30;

    t_q30 = ((int64_t)1 << 60) / inv_q30;                /* Kelvin, Q30 */
    tenth = (int32_t)((t_q30 * 10) >> 30) - KELVIN_TO_TENTH_C;

    if (tenth < TEMP_MIN_TENTH_C) tenth = TEMP_MIN_TENTH_C;
    return (int16_t)tenth;
}

/* OPA1000 directional-coupler model, integer form (see adc_calibration.h
 * section 1b for the derivation and the OPA1000 calibration points):
 *   vrf_mV = (adc_mV + Vf) * GAIN_Q1 / 10
 *   P_mW   = vrf_mV^2 / 100000                        (Z0 / 2 = 100 ohm)  */
static int32_t dc_raw_to_mW(uint16_t raw)
{
    uint32_t vrf_mV;
    int64_t  p_mW;

    if (raw < ADC_DC_DEADBAND_CODE) return 0;

    vrf_mV = ((uint32_t)ADC_RAW_TO_MV(raw) + ADC_DC_VF_MV)
             * ADC_DC_GAIN_Q1 / 10U;

    p_mW = ((int64_t)vrf_mV * vrf_mV) / 100000;
    return (int32_t)p_mW;
}

/* Inverse of dc_raw_to_mW(): milliwatts -> ADC code, saturating at full
 * scale.  Used to turn the configured protection limits into AWD
 * thresholds, which live in ADC code space. */
static uint32_t dc_mW_to_raw(uint32_t mw)
{
    // TODO: 由用户根据电路结构来修改
    uint32_t vrf_mV, v_mV, raw;

    if (mw == 0U) return 0U;

    /* vrf_mV = sqrt(mw * 100000) */
    vrf_mV = (uint32_t)isqrt_u64((uint64_t)mw * 100000U);
    if (vrf_mV <= ADC_DC_VF_MV) return 0U;

    v_mV = vrf_mV * 10U / ADC_DC_GAIN_Q1;
    if (v_mV <= ADC_DC_VF_MV) return 0U;
    v_mV -= ADC_DC_VF_MV;

    raw = (v_mV * ADC_FULL_SCALE) / ADC_VREF_MV;
    return (raw > ADC_FULL_SCALE) ? ADC_FULL_SCALE : raw;
}

/*============================================================================
 *  ADC1 conversions
 *==========================================================================*/

int32_t adc1_raw_to_rf_in_mW(uint16_t raw)
{
    /* The RF input detector (D5/R40 peak detector) has no coupler, so it
     * does not follow the OPA1000 DC model.  Same curve for now; the real
     * slope belongs in the RF-input calibration once the bench numbers
     * are available. */
    return dc_raw_to_mW(raw);
}

int32_t adc1_raw_to_fwd1_mW(uint16_t raw) { return dc_raw_to_mW(raw); }
int32_t adc1_raw_to_fwd2_mW(uint16_t raw) { return dc_raw_to_mW(raw); }
int32_t adc1_raw_to_rev1_mW(uint16_t raw) { return dc_raw_to_mW(raw); }
int32_t adc1_raw_to_rev2_mW(uint16_t raw) { return dc_raw_to_mW(raw); }

int32_t adc1_raw_to_im_mA(uint16_t raw)
{
    int64_t ma;

    /* CC6920BSO-50A: 0 A = 1.65 V, 26.4 mV/A.
     * I_mA = delta_code * 2500 / 65535 * 1000 / 26.4
     *       = delta_code * 25000000 / 17301240                        */
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

static uint32_t ma_to_adc_code(uint32_t ma)
{
    /* inverse of I_mA = delta_code * 25000000 / 17301240 */
    uint32_t delta = (uint32_t)(((uint64_t)ma * 17301240u + 12500000u) / 25000000u);
    uint32_t code  = (uint32_t)g_config.im_zero_offset_adc + delta;
    return (code > ADC_FULL_SCALE) ? ADC_FULL_SCALE : code;
}

void adc_awd_set_rf_in_mW(uint32_t max_mW)
{
    // TODO: 由用户根据输入探测分压电路结构来计算
//    ADC1->HTR1 = dc_mW_to_raw(max_mW);
    ADC1->LTR1 = 0u;
}

void adc_awd_set_im_mA(uint16_t max_mA)
{
    ADC1->HTR2 = ma_to_adc_code(max_mA);
    ADC1->LTR2 = 0u;
}

/*! AWD3 watches coupler 1 reverse power (PA -> LPF side). */
void adc_awd_set_rev1_mW(uint32_t max_mW)
{
    // TODO: 由用户根据Coupler1电路结构来计算
//    ADC1->HTR3 = dc_mW_to_raw(max_mW);
    ADC1->LTR3 = 0u;
}
