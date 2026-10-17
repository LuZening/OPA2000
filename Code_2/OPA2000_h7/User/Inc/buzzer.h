/**
 * @file buzzer.h
 * @brief 3-tone alert buzzer driven by a state machine in TIM7 ISR.
 *
 * @details Per the firmware design spec:
 *   - State 0: play note 1 (high, ~2.5 kHz), 300 ms
 *   - State 1: play note 2 (mid, ~1.5 kHz),  300 ms
 *   - State 2: play note 3 (low, ~0.8 kHz),  300 ms
 *   - State 3: silence 3 s, then loop back to 0
 *
 * The state counter is incremented by the same 100 us tick that drives
 * freq_detect_isr() (TIM7 update).  When the user clears the alert
 * the state machine stops immediately.
 *
 * No DMA is used (the spec allows this - the interval is 300 ms / 3 s,
 * plenty of time to rewrite TIM15 ARR from the ISR).
 */

#ifndef BUZZER_H
#define BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "opa2000.h"
#include "hw_pinmap.h"

/* HAL timer handle (defined in main.c) */
extern TIM_HandleTypeDef htim15;

/*============================================================================
 *  Public API
 *============================================================================*/

/* Initialise TIM15 CH2 in PWM mode for the buzzer. */
void buzzer_init(void);
/* Start the 3-tone warning loop, capped at 10 s.  Idempotent.
 * Kept as the default for P0 alerts and for callers that don't care. */
void buzzer_start(void);
/* Start the 3-tone loop with a custom deadline, in ms.  Values above
 * 10 000 are clamped to 10 000 - the design rule is "no fault keeps the
 * buzzer ringing for more than 10 s, even undismissed". */
void buzzer_start_with_timeout(uint32_t max_ms);
/* Start the P1 (mild) alert pattern: single mid-tone for 1 s.  Under the
 * 10 s global cap. */
void buzzer_start_p1(void);
/* Stop and silence the buzzer. */
void buzzer_stop(void);
/* Per-tick state machine advance.  Called from the TIM7 update ISR. */
void buzzer_tick(void);
/* Is the buzzer currently active? */
bool buzzer_is_active(void);

/*============================================================================
 *  Test hooks (host unit tests only; no firmware caller)
 *
 *  The 10 s cap is wall-clock based, so a host test cannot wait it out.
 *  These expose the private deadline so a test can move it instead of
 *  sleeping - which a plain "extern volatile uint32_t s_deadline_ms"
 *  could not do, since a static symbol does not link across
 *  translation units.
 *==========================================================================*/
#if defined(HOST_TEST_NATIVE)
/*!< Current deadline in ms (0 = no deadline armed). */
uint32_t buzzer_test_get_deadline(void);
/*!< Overwrite the deadline, e.g. to push it into the past. */
void     buzzer_test_set_deadline(uint32_t deadline_ms);
/*!< Push the deadline one millisecond into the past. */
void     buzzer_test_expire_deadline(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* BUZZER_H */