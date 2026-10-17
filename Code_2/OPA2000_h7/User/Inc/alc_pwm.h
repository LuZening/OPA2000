/**
 * @file alc_pwm.h
 * @brief 200 kHz ALC PWM via TIM2 OC + dual DMA -> PG0 BSRR.
 *
 * @details TIM2_CH1 and TIM2_CH2 are configured in Output Compare
 *          No-Output mode, each driving a DMA channel.  When the
 *          counter matches CCR1, DMA writes (1<<0) to PG0 BSRR
 *          (set bit).  When the counter matches CCR2, DMA writes
 *          (1<<16) to PG0 BSRR (reset bit).  Together they produce a
 *          fixed-frequency, variable-duty PWM on PG0 without any CPU
 *          intervention after start.
 *
 *          Frequency = TIM2 clock / (ARR + 1)
 *          Duty      = (CCR2 - CCR1) / (ARR + 1)
 *
 *          Per the firmware design spec the frequency should be
 *          ~200 kHz.  TIM2 clock = 200 MHz, so ARR = 999.
 */

#ifndef ALC_PWM_H
#define ALC_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "opa2000.h"
#include "hw_pinmap.h"

/* HAL timer + DMA handles (defined in main.c) */
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_tim2_ch1;
extern DMA_HandleTypeDef hdma_tim2_ch2;

/* BSRR payload words - cached so DMA doesn't need a 32-bit aligned
 * word in DTCMRAM.  Volatile so the DMA controller's view is
 * authoritative. */
extern volatile uint32_t alc_bsrr_set;
extern volatile uint32_t alc_bsrr_reset;

/*============================================================================
 *  Public API
 *============================================================================*/

/* Configure TIM2 CH1/CH2 in OC mode + enable the two DMA streams.
 * Does NOT start PWM output yet. */
void alc_pwm_init(void);
/* Start the ALC PWM (DMA + TIM2 start). */
void alc_pwm_start(void);
/* Stop the ALC PWM. */
void alc_pwm_stop(void);

/* Reconfigure ARR (frequency) and CCR1/CCR2 (duty cycle).  Can be
 * called from task context. */
void alc_pwm_set_period(uint16_t arr);
void alc_pwm_set_pulse(uint16_t rising_ccr, uint16_t falling_ccr);

#ifdef __cplusplus
}
#endif

#endif /* ALC_PWM_H */