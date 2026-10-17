/**
 * @file fan_tach.h
 * @brief Fan / pump TACH (RPM) detection by 0.5 ms polling.
 *
 * @details Per the firmware design spec:
 *   - TIM7 update fires every 500 us (i.e. 2 kHz sample rate).
 *   - In the ISR we read PD4 (FAN1_SENSEX) and PD5 (PUMP_SENSEX) as
 *     ordinary GPIO inputs.
 *   - An integer-only state machine detects falling edges and counts
 *     them (no FPU instructions).
 *   - Task side: every 100 ms convert the pulse count to RPM using
 *     F_CPU = 2000 Hz * 60 / poles, where poles = 2 (typical PC fan).
 *
 * The state machine preserves the previous GPIO level so that a
 * falling edge triggers exactly one count even if the ISR samples the
 * same low value twice.
 */

#ifndef FAN_TACH_H
#define FAN_TACH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "opa2000.h"
#include "hw_pinmap.h"

/* HAL timer handle (defined in main.c) */
extern TIM_HandleTypeDef htim7;

/*============================================================================
 *  Public API
 *============================================================================*/

/* Configure TIM7 for 500 us tick + initialise state. */
void fan_tach_init(void);
/* Start the 500 us polling ISR. */
void fan_tach_start(void);
/* Stop the polling. */
void fan_tach_stop(void);
/* TIM7 update ISR - read both TACH pins and update pulse counters. */
void fan_tach_tick(void);
/* Get the latest RPM readings (or 0 if no signal). */
uint32_t fan_tach_get_fan_rpm(void);
uint32_t fan_tach_get_pump_rpm(void);
/* Task-side periodic computation (called every 100 ms from the Control
 * task).  Snapshots the pulse counter and computes RPM. */
void fan_tach_calculate_rpm(void);

#ifdef __cplusplus
}
#endif

#endif /* FAN_TACH_H */