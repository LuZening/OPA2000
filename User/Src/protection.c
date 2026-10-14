/**
 * @file protection.c
 * @brief Protection system implementation.
 *
 * @details Two families of source, one shutdown path.
 *
 * Digital - a discrete fault line on an EXTI pin, handled inside the EXTI
 * ISR.  HARDERR_PWR (PB12) comes from the power board and is ACTIVE HIGH.
 * The handler re-reads the pin before acting so a glitch edge cannot shut
 * the amplifier down.  protection_digital_isr() is the generic entry for
 * the RF deck lines still to be added.
 *
 * Analog - a conversion result against a limit.  Where the analog watchdog
 * owns the channel the comparison is in hardware and the AWD interrupt
 * drives the shutdown:
 *     AWD2 -> CH5  IMain   30 A
 *     AWD3 -> CH6  REV1    800 W
 * The channels with no watchdog behind them are compared in interrupt
 * context right after the fresh results are published:
 *     ADC1 DMA complete   REV2 400 W, FWD1 1100 W, FWD2 1050 W
 *     ADC2 injected done  NTC  90 C
 *
 * Shutdown sequence (identical from every source, ISR-safe):
 *   1. RELAY1_OFF + RELAY2_OFF + PTT_OUT low + ptt_state = IDLE
 *   2. PWR_ON_DISABLE       (cut high-voltage supply to PA)
 *   3. RF_BIAS_DISABLE      (cut PA bias)
 *   4. RF_IN_PROTECT_RAISE  (tell the RF board to protect, PB13)
 *   5. buzzer_start()       (3-tone warning, loops until cleared)
 *   6. g_alert_active / g_alert_flag, g_allow_rf = false, g_tx_active = false
 *
 * The latch holds until protection_clear(), which the GUI agent calls on
 * popup dismiss and the Networking agent calls on a CAT / SPI6 command.
 *
 * protection_check() runs the same comparisons on the 100 ms task tick as
 * a backstop for anything the fast paths miss.
 */

#include "protection.h"
#include "power_control.h"
#include "buzzer.h"
#include "adc_calibration.h"
#include "stm32h7xx_hal.h"

/*============================================================================
 *  Protection limits
 *
 *  Milliwatts for the coupler channels, milliamps for IMain, tenths of a
 *  degree for the NTC.  The two channels with a hardware watchdog behind
 *  them (IMain, REV1) get their threshold in ADC code from adc_awd_set_*().
 *==========================================================================*/

#define PROTECT_REV2_MAX_MW        400000L   /*!< coupler 2 reverse, 400 W */
#define PROTECT_REV2_SWR_MW        200000L   /*!< 200 W ...                  */
#define PROTECT_SWR_MAX_X100       200       /*!< ... above SWR 2.0 trips   */
#define PROTECT_FWD1_MAX_MW       1100000L   /*!< coupler 1 forward, 1100 W */
#define PROTECT_FWD2_MAX_MW       1050000L   /*!< coupler 2 forward, 1050 W */
#define PROTECT_NTC_MAX_TENTH_C      900     /*!< core NTC, 90.0 C          */

/*============================================================================
 *  State (volatile for ISR/task sharing)
 *============================================================================*/

static volatile bool     s_protection_active = false;
static volatile AlertType s_last_fault        = ALERT_NONE;

/*============================================================================
 *  Initialisation
 *==========================================================================*/

void protection_init(void)
{
    s_protection_active = false;
    s_last_fault        = ALERT_NONE;
    g_alert_active      = ALERT_NONE;
    g_alert_flag        = false;
    /* Idle state: no fault latched, so the RF board must be told "no fault".
     * Doing this once here also means a protection_clear() before any fault
     * cannot leave PB13 in whatever state the pin came up in. */
    RF_IN_PROTECT_CLEAR();
    buzzer_stop();
}

/*============================================================================
 *  Atomic shutdown (ISR-safe, no FreeRTOS API)
 *==========================================================================*/

void protection_trigger_locked(AlertType cause)
{
    /* Prevent re-entry: only first trigger wins, until cleared. */
    if (s_protection_active) {
        return;
    }
    s_protection_active = true;
    s_last_fault        = cause;

    /* 1. drop both RELAYs and the keying feedback immediately */
    RELAY1_OFF();
    RELAY2_OFF();
    power_ptt_out_set(false);
    ptt_state = PTT_STATE_IDLE;
    /* 2. cut high voltage */
    PWR_ON_DISABLE();
    /* 3. cut PA bias */
    RF_BIAS_DISABLE();
    /* 4. raise hardware fault latch (PB13 -> D7 -> RF_IN_PROTECT_MIXED) so the
     *    RF board runs its own protection and latches the hardware bus */
    RF_IN_PROTECT_RAISE();

    /* 5. start the 3-tone warning; it loops until protection_clear().
     *    Done here rather than in the task so the alarm is audible even if
     *    the fault happened while the scheduler was busy. */
    buzzer_start();

    /* 6. publish to cross-task state (volatiles already declared
     *    volatile in opa2000.c; reads on task side are atomic on M7). */
    g_alert_active = cause;
    g_alert_flag   = true;
    /* 7. block new PTT until IM is re-calibrated or fault cleared */
    g_allow_rf     = false;
    g_tx_active    = false;
    /* NOTE: g_psu_on is deliberately left alone.  A fault drops the
     * relays and the bias, but the front panel still reads "ready" so the
     * operator is told to power-cycle rather than silently powered off. */
}

/*============================================================================
 *  Digital sources - EXTI interrupt context
 *==========================================================================*/

void protection_digital_isr(AlertType cause)
{
    protection_trigger_locked(cause);
}

void protection_hardpwr_err_isr(void)
{
    /* HARDERR_PWR (PB12) is ACTIVE HIGH: only act while the line really is
     * asserted, so a spurious edge cannot shut the amplifier down. */
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

/*============================================================================
 *  Analog sources with a hardware watchdog - ADC IRQ context
 *==========================================================================*/

void protection_awd1_cb(void)
{
    /* AWD1 -> CH4 RF input power over band-dependent threshold.
     * ioc default AWD1HighThreshold = 32767 -> set to band threshold by
     * adc_awd_set_rf_in_mW() before TX starts. */
    protection_trigger_locked(ALERT_OVERDRIVE);
}

void protection_awd2_cb(void)
{
    /* AWD2 -> CH5 main current > 30 A. */
    protection_trigger_locked(ALERT_CURRENT_HIGH);
}

void protection_awd3_cb(void)
{
    /* AWD3 -> CH6 coupler 1 reverse > 800 W hardware last-line. */
    protection_trigger_locked(ALERT_REFLECT_POWER_HIGH);
}

/*============================================================================
 *  Analog sources without a watchdog - compared in interrupt context
 *
 *  These channels have no analog watchdog behind them, so the comparison
 *  happens in the DMA / injected-completion interrupt, right after the
 *  fresh conversion results have been published to g_adc_data.
 *==========================================================================*/

void protection_check_adc1_dma(void)
{
    /* Already latched, or nothing is keyed: no RF to protect. */
    if (s_protection_active || !g_tx_active) {
        return;
    }

    /* REV2 > 400 W (coupler 2 reverse, the one the operator sees) */
    if (g_adc_data.rev2_mW > PROTECT_REV2_MAX_MW) {
        protection_trigger_locked(ALERT_REFLECT_POWER_HIGH);
        return;
    }
    /* REV2 > 200 W and SWR > 2.0 */
    if (g_adc_data.rev2_mW > PROTECT_REV2_SWR_MW &&
        g_adc_data.swr_x100 > PROTECT_SWR_MAX_X100) {
        protection_trigger_locked(ALERT_SWR_HIGH);
        return;
    }
    /* FWD1 > 1100 W (coupler 1 forward) */
    if (g_adc_data.fwd1_mW > PROTECT_FWD1_MAX_MW) {
        protection_trigger_locked(ALERT_REFLECT_POWER_HIGH);
        return;
    }
    /* FWD2 > 1050 W (coupler 2 forward) */
    if (g_adc_data.fwd2_mW > PROTECT_FWD2_MAX_MW) {
        protection_trigger_locked(ALERT_REFLECT_POWER_HIGH);
    }
}

void protection_check_ntc(void)
{
    if (s_protection_active) {
        return;
    }
    /* Over-temperature latches whether or not we are keyed: the PA is still
     * dissipating and the condition will not clear on its own. */
    if (g_adc_data.ntc_core_tenth_c > PROTECT_NTC_MAX_TENTH_C) {
        protection_trigger_locked(ALERT_NTC_OVERHEAT);
    }
}

/*============================================================================
 *  Task-side software protection (100 ms tick) - backstop
 *==========================================================================*/

void protection_check(void)
{
    /* Don't re-trigger if already latched. */
    if (s_protection_active) {
        return;
    }

    /* Over-temperature applies whether or not we are keyed. */
    if (g_adc_data.ntc_core_tenth_c > PROTECT_NTC_MAX_TENTH_C) {
        protection_trigger_locked(ALERT_NTC_OVERHEAT);
        return;
    }

    /* The coupler limits only matter while the amplifier passes RF. */
    if (!g_tx_active) {
        return;
    }

    protection_check_adc1_dma();

    /* VBIAS out of range with the bias engaged. */
    if (g_adc_data.vbias_mV > g_config.band_vbias_max_mV) {
        protection_trigger_locked(ALERT_VBIAS_FAULT);
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
    buzzer_stop();
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