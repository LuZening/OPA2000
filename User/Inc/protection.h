/**
 * @file protection.h
 * @brief Protection system header.
 *
 * @details Two layers:
 *   1. Hardware-triggered (ISR context, main prio 0, no FreeRTOS API):
 *        - ADC1 AWD1/AWD2/AWD3 on CH4/CH5/CH6
 *        - COMP1/COMP2
 *        - EXTI12 (HARDPWR_ERR on PB12)
 *      Each handler calls protection_trigger_locked() to atomically:
 *        - release RELAY1 + RELAY2
 *        - drop PWR_ON_SIG (PG7)
 *        - raise RF_IN_PROTECT_CPU_SIG (PB13) -> hardware latch
 *        - clear g_allow_rf, set g_alert_flag, set g_alert_active
 *   2. Software-triggered (Control task, low prio):
 *        - REV2 > 400 W  (always)
 *        - REV2 > 200 W and SWR > 2.0
 *        - FWD1 > 1100 W
 *        - FWD2 > 1050 W
 *        - NTC > 90 C
 *        - VBIAS > config threshold
 *        - ADC value sanity (e.g. IM out of range)
 *
 * The task-side checks run in protection_check() called from the Control
 * task on the 100 ms tick.
 */

#ifndef PROTECTION_H
#define PROTECTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "opa2000.h"
#include "hw_pinmap.h"

/*============================================================================
 *  ISR-side: lock-free direct GPIO shutdown
 *==========================================================================*/

/* Bring the protection subsystem to its power-on state: latch cleared, alert
 * flags down, RF_IN_PROTECT low, buzzer silent.  Call once from main() before
 * the scheduler starts (before any interrupt can reach a handler). */
void protection_init(void);

/* Atomic hardware shutdown.  Called from:
 *   - ADC1 AWD1/2/3 callbacks  (from ADC1_2_IRQHandler, main prio 0)
 *   - COMP1/2 callbacks         (from COMP_IRQHandler, main prio 0)
 *   - EXTI15_10 IRQHandler      (HARDPWR_ERR, main prio 0)
 *   - User ad-hoc triggers from protection.c
 * MUST NOT call any FreeRTOS API. */
void protection_trigger_locked(AlertType cause);

/*============================================================================
 *  Protection sources
 *
 *  Digital  : a discrete fault line, brought in on an EXTI pin, handled
 *             inside the EXTI ISR.  The line is ACTIVE HIGH - the handler
 *             re-reads the pin before acting so a spurious edge cannot
 *             shut the amplifier down.
 *               HARDERR_PWR (PB12)  power board alarm
 *               (spare)             future RF deck fault lines
 *  Analog   : a conversion result compared against a limit.  Where the
 *             channel is wired to an analog watchdog the comparison is in
 *             hardware and fires the AWD interrupt; the remaining channels
 *             are compared in the DMA / injected-completion interrupt.
 *               AWD2  CH5   IMain   30 A
 *               AWD3  CH6   REV1    800 W
 *               DMA          REV2    400 W
 *               DMA          FWD1    1100 W
 *               DMA          FWD2    1050 W
 *               ADC2 inj     NTC     90 C
 *============================================================================*/

/* Digital fault lines.  Generic so RF deck signals can be added without
 * touching the shutdown path.  Interrupt context, no FreeRTOS API. */
void protection_hardpwr_err_isr(void);   /* PB12, ACTIVE HIGH         */
void protection_digital_isr(AlertType cause);   /* future EXTI fault lines */
void protection_comp1_isr(void);          /* COMP1 RF input overdrive   */
void protection_comp2_isr(void);          /* COMP2 main over-current     */

/* AWD callbacks are wired into HAL_ADC_LevelOutOfWindowCallback. */
void protection_awd1_cb(void);   /* CH4 RF input  */
void protection_awd2_cb(void);   /* CH5 IMain     */
void protection_awd3_cb(void);   /* CH6 REV1      */

/*============================================================================
 *  Fast analog checks - interrupt context, no FreeRTOS API
 *
 *  Called from the ADC1 DMA-completion interrupt, right after the fresh
 *  conversion results have been published to g_adc_data.  Covers only the
 *  channels that have no analog watchdog behind them.
 *============================================================================*/
void protection_check_adc1_dma(void);

/* Called from the ADC2 injected-completion interrupt, where the core NTC
 * result lands. */
void protection_check_ntc(void);

/*============================================================================
 *  Task-side checks (Control 100 ms tick) - backstop and slow conditions
 *============================================================================*/

/* Run all software protection checks.  Each check may call
 * protection_trigger_locked() directly via the same code path as ISR. */
void protection_check(void);

/* Clear latched fault after the user acknowledges the alert and the
 * underlying condition has been resolved. */
void protection_clear(void);

/* Has any fault been latched?  Read-only accessor for GUI. */
bool protection_is_active(void);

/* Fault description string for GUI display. */
const char *protection_get_fault_string(AlertType a);

#ifdef __cplusplus
}
#endif

#endif /* PROTECTION_H */