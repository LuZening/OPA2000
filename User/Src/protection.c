/**
 * @file protection.c
 * @brief Protection system implementation.
 *
 * @details The atomic shutdown sequence (called from any ISR or task):
 *   1. RELAY1_OFF + RELAY2_OFF     (disconnect PA from antenna)
 *   2. PWR_ON_DISABLE              (cut high-voltage supply to PA)
 *   3. RF_BIAS_DISABLE             (cut PA bias)
 *   4. RF_IN_PROTECT_RAISE         (raise PB13 hardware latch line)
 *   5. g_alert_active = cause, g_alert_flag = true
 *   6. g_allow_rf = false          (block new PTT)
 *
 * This sequence is identical regardless of trigger source (ISR or task)
 * so all paths converge to one safe state.
 *
 * The task side runs protection_check() on the 100 ms tick to cover
 * conditions that are too slow for analog watchdogs (SWR, NTC, FWD
 * software limits, VBIAS sanity).
 */

#include "protection.h"
#include "stm32h7xx_hal.h"

/*============================================================================
 *  State (volatile for ISR/task sharing)
 *============================================================================*/

static volatile bool     s_protection_active = false;
static volatile AlertType s_last_fault        = ALERT_NONE;

/*============================================================================
 *  Atomic shutdown (ISR-safe, no FreeRTOS API)
 *============================================================================*/

void protection_trigger_locked(AlertType cause)
{
    /* Prevent re-entry: only first trigger wins, until cleared. */
    if (s_protection_active) {
        return;
    }
    s_protection_active = true;
    s_last_fault        = cause;

    /* 1. drop both RELAYs immediately */
    RELAY1_OFF();
    RELAY2_OFF();
    /* 2. cut high voltage */
    PWR_ON_DISABLE();
    /* 3. cut PA bias */
    RF_BIAS_DISABLE();
    /* 4. raise hardware fault latch (PB13 -> D7 -> RF_IN_PROTECT_MIXED) */
    RF_IN_PROTECT_RAISE();

    /* 5. publish to cross-task state (volatiles already declared
     *    volatile in opa2000.c; reads on task side are atomic on M7). */
    g_alert_active = cause;
    g_alert_flag   = true;
    /* 6. block new PTT until IM is re-calibrated or fault cleared */
    g_allow_rf     = false;
    g_ptt_active   = false;
    g_power_state  = POWER_STATE_OFF;
}

/*============================================================================
 *  ISR callbacks (wire to HAL via user-supplied trampolines)
 *============================================================================*/

void protection_hardpwr_err_isr(void)
{
    /* EXTI rising edge on PB12 from PSU fault monitor. */
    if (HAL_GPIO_ReadPin(HARDPWR_ERR_GPIO_Port, HARDPWR_ERR_Pin)
        == GPIO_PIN_SET) {
        protection_trigger_locked(ALERT_HARDWARE_FAULT);
    }
}

void protection_comp1_isr(void)
{
    /* COMP1 = RF input overdrive / over-power (PB2 / ADC1_INP4). */
    protection_trigger_locked(ALERT_OVERDRIVE);
}

void protection_comp2_isr(void)
{
    /* COMP2 = main current over-current (PE9 / ADC1_INP5). */
    protection_trigger_locked(ALERT_CURRENT_HIGH);
}

void protection_awd1_cb(void)
{
    /* AWD1 -> CH4 RF input power over band-dependent threshold.
     * ioc default AWD1HighThreshold = 32767 -> set to band threshold by
     * adc_awd_set_rf_in_mW() before TX starts. */
    protection_trigger_locked(ALERT_RFIN_OVER);
}

void protection_awd2_cb(void)
{
    /* AWD2 -> CH5 main current > 30 A. */
    protection_trigger_locked(ALERT_CURRENT_HIGH);
}

void protection_awd3_cb(void)
{
    /* AWD3 -> CH6 reflected 1 > 800 W hardware last-line. */
    protection_trigger_locked(ALERT_REFLECT_POWER_HIGH);
}

/*============================================================================
 *  Task-side software protection (100 ms tick)
 *============================================================================*/

void protection_check(void)
{
    /* Don't re-trigger if already latched. */
    if (s_protection_active) {
        return;
    }

    /* Only run power-over-current tests in TX state. */
    if (g_power_state != POWER_STATE_TX) {
        return;
    }

    /* REV2 > 400 W (always) */
    if (g_adc_data.rev2_mW > 400000) {
        protection_trigger_locked(ALERT_REFLECT_POWER_HIGH);
        return;
    }
    /* REV2 > 200 W and SWR > 2.0 */
    if (g_adc_data.rev2_mW > 200000 && g_adc_data.swr_x100 > 200) {
        protection_trigger_locked(ALERT_SWR_HIGH);
        return;
    }
    /* FWD1 > 1100 W */
    if (g_adc_data.fwd1_mW > 1100000) {
        protection_trigger_locked(ALERT_HARDWARE_FAULT);
        return;
    }
    /* FWD2 > 1050 W */
    if (g_adc_data.fwd2_mW > 1050000) {
        protection_trigger_locked(ALERT_HARDWARE_FAULT);
        return;
    }
    /* NTC > 90 C */
    if (g_adc_data.ntc_core_tenth_c > 900) {
        protection_trigger_locked(ALERT_NTC_OVERHEAT);
        return;
    }
    /* VBIAS > config threshold (mV) */
    if (g_adc_data.vbias_mV > g_config.band_vbias_max_mV) {
        protection_trigger_locked(ALERT_VBIAS_FAULT);
        return;
    }
}

/*============================================================================
 *  Ack / clear (called from GUI or PC command after user ack)
 *============================================================================*/

void protection_clear(void)
{
    s_protection_active = false;
    s_last_fault        = ALERT_NONE;
    g_alert_active      = ALERT_NONE;
    g_alert_flag        = false;
    RF_IN_PROTECT_CLEAR();
    /* Note: g_allow_rf stays false until user re-runs the IM zero-offset
     * calibration (adc_im_zero_offset_tick()).  That is intentional -
     * the operator must explicitly re-arm the RF path. */
}

bool protection_is_active(void) { return s_protection_active; }

/*============================================================================
 *  GUI helpers
 *============================================================================*/

const char *protection_get_fault_string(AlertType a)
{
    switch (a) {
        case ALERT_SWR_HIGH:            return "SWR Too High";
        case ALERT_REFLECT_POWER_HIGH:  return "Reflected Power High";
        case ALERT_CURRENT_HIGH:         return "Main Current Too High";
        case ALERT_HARDWARE_FAULT:      return "Hardware Fault";
        case ALERT_OVERDRIVE:           return "RF Input Overdrive";
        case ALERT_FREQ_MISMATCH:       return "Frequency Mismatch";
        case ALERT_NTC_OVERHEAT:        return "Core Temperature High";
        case ALERT_VBIAS_FAULT:         return "PA Bias Fault";
        default:                        return "Unknown Fault";
    }
}