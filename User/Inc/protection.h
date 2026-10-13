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
 *============================================================================*/

/* Atomic hardware shutdown.  Called from:
 *   - ADC1_AWD1/2/3 callbacks  (from ADC1_2_IRQHandler, main prio 0)
 *   - COMP1/2 callbacks         (from COMP_IRQHandler, main prio 0)
 *   - EXTI15_10 IRQHandler      (HARDPWR_ERR, main prio 0)
 *   - User ad-hoc triggers from protection.c
 * MUST NOT call any FreeRTOS API. */
void protection_trigger_locked(AlertType cause);

/* Hardware fault line state machine: latched until cleared. */
void protection_hardpwr_err_isr(void);   /* PB12 EXTI rising edge   */
void protection_comp1_isr(void);          /* COMP1 over-current/drv */
void protection_comp2_isr(void);          /* COMP2 over-current     */

/* AWD callbacks are wired into HAL_ADC_LevelOutOfWindowCallback. */
void protection_awd1_cb(void);   /* CH4 RF input */
void protection_awd2_cb(void);   /* CH5 main current */
void protection_awd3_cb(void);   /* CH6 reflected 1 */

/*============================================================================
 *  Task-side checks (Control 100 ms tick)
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