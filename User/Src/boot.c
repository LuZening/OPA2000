/**
 * @file    boot.c
 * @brief   Boot-time sequencing helpers (pre-kernel, single-threaded).
 *
 * @details See boot.h for the full boot-order contract.  Everything in
 *          this file runs from main() before osKernelStart(), so no RTOS
 *          primitive is touched here.
 */

#include "boot.h"

#include "opa2000.h"
#include "adc_calibration.h"
#include "config.h"
#include "hw_pinmap.h"
#include "stm32h7xx_hal.h"
#include "main.h"

#ifndef LVGL_SIM
extern DAC_HandleTypeDef hdac1;
extern TIM_HandleTypeDef htim4;
#endif

/*============================================================================
 *  COMP threshold scaling
 *============================================================================
 *  COMP1 / COMP2 compare an external analog input against a reference
 *  produced by DAC1 CH1 / CH2 (Core/Src/comp.c selects
 *  COMP_INPUT_MINUS_DAC1_CH1 / _CH2 - the DAC1 outputs are the only DAC
 *  sources a comparator can reference on this part).
 *
 *  DAC1 is 12-bit and runs off VDDA (3.3 V), NOT off the ADC's VREFBUF
 *  2.5 V rail, so:
 *
 *      dac_code = ref_mV * DAC_FULL_SCALE / DAC_VDDA_MV
 *
 *  The reference VOLTAGE itself lives in g_config (comp1_ref_mV /
 *  comp2_ref_mV) and is adjustable by the operator.  No analog gain is
 *  baked in here on purpose - the gain in front of each input is a board
 *  property, so inventing one would silently produce a threshold that
 *  does not correspond to the advertised wattage or amp rating.
 *==========================================================================*/

/* Millivolts -> 12-bit DAC code, clamped to the DAC's full scale. */
static uint16_t boot_mv_to_dac_code(uint32_t mv)
{
    if (mv >= DAC_VDDA_MV) {
        return (uint16_t)DAC_FULL_SCALE;
    }
    return (uint16_t)((mv * DAC_FULL_SCALE) / DAC_VDDA_MV);
}

/*============================================================================
 *  Public API
 *============================================================================*/

void boot_gpio_safe_state(void)
{
#ifndef LVGL_SIM
    /* --- 1. RF path must stay de-energised while we boot ---------- */
    RF_BIAS_DISABLE();          /* PG2 low  - PA bias off          */
    PWR_ON_DISABLE();           /* PG7 low  - HV supply off        */
    PSU_ON_PWR_OFF();           /* PG5 low  - PSU latch released   */
    RELAY1_OFF();               /* PG4 low  - antenna disconnected */
    RELAY2_OFF();               /* PG3 low  - PA disconnected      */
    PTT_OUT_OFF();              /* PC15 low - keying feedback off  */

    /* --- 2. Keep the unit powered (soft-off is an active-low line) - */
    HAL_GPIO_WritePin(REAR_RESERVED1_GPIO_Port, REAR_RESERVED1_Pin,
                       GPIO_PIN_SET);

    /* --- 3. Blank the screen until the GUI has something to show ---- */
    /* BL_PWM is TIM4 CH3 (PD14).  MX_TIM4_Init() already ran, so the
     * timer is clocked; we only need to force the compare register to
     * 0 (0 % duty) and make sure the PWM output is running.  Doing it
     * here - rather than relying on the reset value - means the panel
     * never flashes a half-initialised frame. */
    if (htim4.Instance != NULL) {
        __HAL_TIM_SET_COUNTER(&htim4, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0U);
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    }
#else
    /* PC simulator has no GPIO to poke. */
#endif
}

void boot_apply_comp_thresholds(void)
{
#ifndef LVGL_SIM
    /* Re-read from g_config on every call so an operator edit in the
     * Settings menu takes effect without a reboot.  The GUI sliders
     * already clamp to *_REF_MV_MAX, but clamp again here: this is the
     * one place where a bad stored value could actually pin a comparator
     * output at its rail. */
    uint32_t ref1 = g_config.comp1_ref_mV;
    uint32_t ref2 = g_config.comp2_ref_mV;

    if (ref1 > COMP1_REF_MV_MAX) { ref1 = COMP1_REF_MV_MAX; }
    if (ref2 > COMP2_REF_MV_MAX) { ref2 = COMP2_REF_MV_MAX; }

    /* Data first.  Errata 2.8.1 wants a write to a data register before
     * the MODE field is programmed; MX_DAC1_Init() already established
     * that ordering, so here we only load new values. */
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
                     boot_mv_to_dac_code(ref1));
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R,
                     boot_mv_to_dac_code(ref2));

    /* Idempotent: DAC_CR_CENx stays set across a re-apply and
     * HAL_DAC_Start() has nothing to undo on a second call. */
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
#endif
}

void boot_apply_thresholds(int band)
{
#ifndef LVGL_SIM
    /* --- Analog watchdogs -------------------------------------------
     * AWD1 = RF input, AWD2 = main current, AWD3 = coupler-1 reverse.
     * The band-specific RF-in limit comes from band_rf_in_max_w[]; the
     * other two are global. */
    uint16_t rf_in_index = 0U;
    switch (band) {
        case BAND_160M:    rf_in_index = 0U; break;
        case BAND_80M:     rf_in_index = 1U; break;
        case BAND_40M_30M: rf_in_index = 2U; break;
        case BAND_20M_17M: rf_in_index = 3U; break;
        case BAND_15M_10M: rf_in_index = 4U; break;
        default:           rf_in_index = 4U; break;   /* safest = 15-10 m */
    }

    adc_awd_set_rf_in_mW((uint32_t)g_config.band_rf_in_max_w[rf_in_index] * 1000U);
    adc_awd_set_im_mA(g_config.band_im_max_mA);
    adc_awd_set_rev1_mW((uint32_t)g_config.band_rev1_max_w * 1000U);
#endif

    /* Comparator references.  Split out so the Settings screen can
     * re-apply just the COMP thresholds after an edit without also
     * re-indexing the watchdogs, which are band-dependent. */
    boot_apply_comp_thresholds();
}

void boot_backlight_on(void)
{
#ifndef LVGL_SIM
    /* Handled through the same PWM plumbing as the fan/pump so the
     * panel comes up at a known, calibrated level rather than whatever
     * the GPIO default happened to be.
     *
     * 90 %, not 100 %: the touch calibration screen is often the very
     * first thing the operator sees, and it asks them to press targets
     * and hold steady.  Full brightness on a panel that has just woken
     * is harsh and, on some LCDs, raises the panel temperature enough to
     * make the calibration feel worse.  One named constant so the level
     * can be retuned in one place. */
    power_set_backlight_duty(BOOT_BACKLIGHT_PCT);
#endif
}
