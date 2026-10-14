/**
 * @file power_control.h
 * @brief Power / Relay / Band / Fan / Buzzer control header.
 *
 * @details This header is the Control agent's primary interface to the
 *          hardware outputs that put the amplifier into / out of TX.
 *
 *          Layering:
 *            - All GPIO definitions live in User/Inc/hw_pinmap.h
 *            - power_control.c uses macros from hw_pinmap.h
 *              (LPF_ACTIVE(), RELAY1_ON, PSU_ON_PWR_ON, ...)
 *            - The PTT state machine runs *without* any FreeRTOS API
 *              calls in the ISR (HAL_GPIO_EXTI_Callback).  The task
 *              side reads the volatile flags only.
 *
 *          Fan / pump TACH detection runs in TIM7 0.5 ms tick; see
 *          fan_tach.c.
 *
 *          Buzzer 3-tone warning is implemented as a state machine
 *          driven by TIM7 ISR (no DMA); see buzzer.c (todo).
 *
 * @version 1.0
 * @date    2026-10
 * @author  Control Agent (wt-control branch)
 */

#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "opa2000.h"
#include "hw_pinmap.h"

/*============================================================================
 *  PTT state machine - written from EXTI5 ISR, read from task
 *============================================================================*/
typedef enum {
    PTT_STATE_IDLE = 0,
    PTT_STATE_TX_STARTING,    /* RELAY2 closed, waiting to close RELAY1 */
    PTT_STATE_TX_RUNNING,     /* both RELAY1+2 closed, transmitting */
    PTT_STATE_TX_STOPPING,    /* RELAY1 open, waiting to open RELAY2 */
} PTTStateType;

extern volatile PTTStateType ptt_state;
extern volatile int          ptt_event_flag;  /*!< -1=none, 0=release, 1=press */

/*============================================================================
 *  Fan / pump control
 *============================================================================*/

/* Initialise fan PWM (TIM3_CH3) and pump PWM (TIM4_CH4) at 10 kHz. */
void power_init_fan_pwm(void);
/* Set fan duty 0..100 % (smooth start/stop allowed). */
void power_set_fan_speed(uint8_t speed_percent);
/* Set pump duty 0..100 %. */
void power_set_pump_speed(uint8_t speed_percent);
/* Update fan based on temperature + hysteresis (50/80/100 %). */
void power_update_fan(int16_t temperature_tenth_c);

/*============================================================================
 *  Power / PTT sequence
 *============================================================================*/

/* All outputs to off / default state. */
void power_init_outputs(void);
/* PSU + RF_BIAS on.  Safe even if calibration not done yet. */
void power_psu_on(void);
/* PSU + RF_BIAS off.  RELEAYS must be released first. */
void power_psu_off(void);
/* Both RELAY1 + RELAY2 on (TX). */
void power_relays_tx_on(void);
/* Both RELAY1 + RELAY2 off. */
void power_relays_tx_off(void);

/*============================================================================
 *  Front-panel power (task context; called from gui.c / key.c / app_tasks.c)
 *==========================================================================*/

/* Bring every output and the fan/pump PWM to their power-on state.  Call
 * once before the scheduler starts. */
void power_init(void);

/* Operator "ready": main supply, HV and PA bias engaged, PA not keyed. */
void power_on(void);
/* Drop back to ready from transmit (supply stays engaged). */
void power_standby(void);
/* Full shutdown: relays open, supply / HV / bias off. */
void power_off(void);

/*============================================================================
 *  Transmit request from a task source
 *
 *  Same interlocks as the PTT EXTI path: refused while the CC6920
 *  zero-offset is unfinished (g_allow_rf) or protection is latched
 *  (g_alert_flag).  Used for CAT / serial / front-panel initiated TX.
 *==========================================================================*/
void power_tx_enable(void);
void power_tx_disable(void);

/* Live level of PTT_INX, for task-side polling when an EXTI edge may have
 * been missed.  True means "transmit requested" (pin low, active low). */
bool power_get_ptt_state(void);

/* Follow the radio: pick the LPF band that frequency_hz falls into.
 * Out-of-band frequencies leave the current band alone. */
void power_update_band(uint32_t frequency_hz);

/*============================================================================
 *  Band switching
 *============================================================================*/

/* All LPF outputs LOW (inactive); pull high only the selected band.
 * Default on invalid band: BAND_15M_10M. */
void switch_band(BandType band);

/*============================================================================
 *  LCD backlight (simple GPIO control; PWM dimming is in TIM4_CH3 task)
 *============================================================================*/
void power_set_lcd_backlight(bool enable);

/*============================================================================
 *  PTT ISR / task glue
 *============================================================================*/

/* Called from EXTI5 ISR for BOTH edges of PTT_INX (falling = press,
 * rising = release; the pin is active low).  Runs no FreeRTOS APIs, reads
 * the live pin level rather than trusting the edge, and is therefore safe
 * to call for a duplicate edge. */
void power_control_ptt_isr(void);

/* Release-safety fallback, called from the fast TIM7 tick.  Reconciles the
 * state machine against the live PTT_INX level so a release edge that never
 * arrives still unkeys the amplifier within one tick.  Level based and
 * idempotent; interrupt-safe (no FreeRTOS API, no FPU). */
void power_ptt_poll(void);

/* Returns then clears the PTT event flag. */
int  power_get_ptt_flag(void);
void power_clear_ptt_flag(void);

/* Drive the PTT_OUT keying feedback pin (PC15, active high).  Normally
 * called by the PTT state machine; protection calls it directly so the
 * feedback to the radio drops in the same instant the relays do. */
void power_ptt_out_set(bool asserted);

/*============================================================================
 *  RELAY delay timer (TIM5 one-shot 15 ms)
 *============================================================================*/

/* Called from TIM5 update ISR when the relay-delay expires. */
void power_relay_timer_callback(void);

#ifdef __cplusplus
}
#endif

#endif /* POWER_CONTROL_H */