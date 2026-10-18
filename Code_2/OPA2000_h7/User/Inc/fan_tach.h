/**
 * @file fan_tach.h
 * @brief Fan / pump TACH (RPM) detection by edge counting in the TIM7 ISR.
 *
 * @details
 *   - TIM7 is owned by freq_detect.c, which programs it for a 100 us
 *     update tick and starts it.  fan_tach only *rides along* on that
 *     interrupt (fan_tach_tick() is called from TIM7_IRQHandler), so it
 *     must never reprogram PSC/ARR - doing so would break frequency
 *     detection.  100 us is far faster than any fan tach needs: the
 *     fastest plausible FG signal here is ~200 Hz (12000 rpm x 2 PPR),
 *     so a 10 kHz sample rate gives 50 samples per pulse.
 *   - In the ISR we read PD4 (FAN1_SENSEX) and PD5 (PUMP_SENSEX) as
 *     ordinary GPIO inputs and count falling edges.  Integer only, no
 *     FPU instructions.
 *   - Task side: fan_tach_calculate_rpm() snapshots the counters and
 *     converts them to RPM against the *measured* elapsed time, not a
 *     hardcoded window, so the reading stays correct if the caller
 *     period changes.
 *
 * The state machine preserves the previous GPIO level so that a falling
 * edge triggers exactly one count even if the ISR samples the same low
 * value twice.
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
 *  Compile-time options
 *==========================================================================*/

/* Pulses per revolution, per channel.
 *
 * FAN_POLES = 2 is the de-facto standard for 4-wire PC / axial fans: the
 * Hall sensor sees one magnet on a two-pole magnetic circuit, hence the
 * familiar "RPM = FG_Hz * 30" rule.
 *
 * PUMP_POLES is separate because a coolant pump is NOT a PC fan.  Its
 * tach is often a reed switch or a single-pole Hall sensor - those
 * commonly give 1 pulse per revolution, and some industrial pumps give 4.
 * Change this to your pump's datasheet figure: getting it wrong scales
 * every reported pump rpm by (assumed / actual), and nothing else in the
 * firmware will tell you.  FAN_POLES / PUMP_POLES live here rather than
 * in fan_tach.c so this file is the single place to edit them. */
#define FAN_POLES               2U
#define PUMP_POLES              2U

/* FAN2 currently shares its PWM duty with FAN1 and its TACH pin with the
 * pump (PD5), so there is physically nothing to measure separately: the
 * two fans are fed identically and therefore turn at the same speed.
 * With this at 1, fan_tach_get_fan2_rpm() reports the FAN1 reading,
 * which is what the hardware can actually support.  Set it to 0 once
 * FAN2 gets a PWM channel and a TACH pin of its own. */
#define FAN2_TACH_MIRRORS_FAN1   1

/*============================================================================
 *  Public API
 *==========================================================================*/

/* Reset the counters, the edge state and the last RPM. */
void fan_tach_init(void);
/* TIM7 update ISR - read both TACH pins and update pulse counters. */
void fan_tach_tick(void);
/* Get the latest RPM readings (or 0 if no signal). */
uint32_t fan_tach_get_fan_rpm(void);
uint32_t fan_tach_get_fan2_rpm(void);
uint32_t fan_tach_get_pump_rpm(void);
/* Task-side periodic computation (called from the monitor task's slow
 * tick).  Snapshots the pulse counters and computes RPM. */
void fan_tach_calculate_rpm(void);

#ifdef __cplusplus
}
#endif

#endif /* FAN_TACH_H */
