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
/* Start the 3-tone warning loop.  Idempotent. */
void buzzer_start(void);
/* Stop and silence the buzzer. */
void buzzer_stop(void);
/* Per-tick state machine advance.  Called from the TIM7 update ISR. */
void buzzer_tick(void);
/* Is the buzzer currently active? */
bool buzzer_is_active(void);

#ifdef __cplusplus
}
#endif

#endif /* BUZZER_H */