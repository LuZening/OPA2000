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

/*----------------------------------------------------------------------------
 *  Power-on stabilisation - hold the watchdogs wide open
 *---------------------------------------------------------------------------
 *
 *  Called from main() right after the peripherals come up.  Right now the
 *  rail has not had time to charge, the D5/R40 peak detector output is
 *  still crawling out of its reset floor, and the CC6920 output sits
 *  wherever the bias network happens to land.  Programming the operator's
 *  real thresholds at that moment is how the amplifier comes up already
 *  alarming.
 *
 *  So the first write is deliberately useless as a limit:
 *
 *    AWD1/AWD2/AWD3  high threshold  = 16-bit full scale (65535)
 *    COMP1/COMP2     DAC reference   = DAC_VDDA_MV, i.e. the top of the
 *                                       12-bit DAC (4095, the pin sits at
 *                                       the VDDA rail)
 *
 *  A watchdog with its threshold at full scale cannot be crossed, and a
 *  comparator whose reference is at the rail cannot trip, so nothing
 *  latches while the analog side settles.
 *
 *  This also CLOSES the apply gate (s_thresholds_armed below): while the
 *  settling window is open, boot_apply_thresholds() is a no-op.  That is not
 *  belt-and-braces, it is load-bearing - power_init() runs switch_band()
 *  from the Power task within a few milliseconds of the scheduler starting,
 *  i.e. deep inside the settling window, and switch_band() ends in
 *  boot_apply_thresholds().  Without the gate that one call would push the
 *  operator's real limits into the hardware and undo everything this
 *  function just did.
 *
 *  The real limits are pushed by boot_arm_thresholds() once
 *  adc_stabilisation_tick() has confirmed the readings are stable.
 *--------------------------------------------------------------------------*/

/*!< False from reset until boot_arm_thresholds() runs.  boot_apply_thresholds()
 *   tests this and returns without touching the hardware while it is false. */
static bool s_thresholds_armed = false;

void boot_hold_thresholds_open(void)
{
    s_thresholds_armed = false;

#ifndef LVGL_SIM
    /* DAC1 CH1 / CH2 straight to the rail.  boot_mv_to_dac_code() clamps
     * anything at or above DAC_VDDA_MV to DAC_FULL_SCALE, which is exactly
     * what we want, so pass VDDA itself rather than 0xFFFFFFFF. */
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
                     boot_mv_to_dac_code(DAC_VDDA_MV));
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R,
                     boot_mv_to_dac_code(DAC_VDDA_MV));
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

    /* ADC watchdogs.  ma_to_adc_code() clamps to ADC_FULL_SCALE, and the
     * two DC-coupled paths currently return 0 for everything anyway, so
     * the exact argument does not matter yet - what matters is that this
     * is a deliberate full-scale write and not a config-derived one. */
    adc_awd_set_rf_in_mW(UINT32_MAX);
    adc_awd_set_im_mA(UINT16_MAX);
    adc_awd_set_rev1_mW(UINT32_MAX);

    /* Hold the external fault latch cleared from here on.  MX_GPIO_Init()
     * left this open-drain pin at its reset value (released), which is
     * exactly what lets a power-up transient latch a fault - so pull it
     * low now, before the kernel even starts.  adc_stabilisation_tick()
     * keeps re-asserting it every tick and releases it when armed. */
    CLEAR_FAULTX_OC_LOW();
#endif
}

void boot_gpio_safe_state(void)
{
#ifndef LVGL_SIM
    /* --- 1. RF path must stay de-energised while we boot ---------- */
    RF_BIAS_DISABLE();          /* PG2 low  - PA bias off          */
    PWR_ON_DISABLE();           /* PG7 low  - HV supply off        */
    RELAY1_OFF();               /* PG4 low  - antenna disconnected */
    RELAY2_OFF();               /* PG3 low  - PA disconnected      */
    PTT_OUT_OFF();              /* PC15 low - keying feedback off  */


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

/*============================================================================
 *  Which variable each COMP channel watches
 *---------------------------------------------------------------------------
 *
 *  Both comparators put the measured quantity on the non-inverting input and
 *  the operator-programmable threshold on the inverting input, so the output
 *  goes high the moment the measured value rises above the reference.  The
 *  output is non-inverted and drives an IRQ (COMP_IRQn) as well as an
 *  external fault latch through D1 / D6.
 *
 *  COMP1 - RF input overdrive
 *      positive  PB2  (COMP1_INP) - same net as ADC1_INP4 / PC4, i.e. RF1
 *                after the D5/R40 peak detector
 *      negative  DAC1_OUT1 - threshold, g_config.comp1_ref_mV
 *      output    PE12 (COMP1_OUT) - via D1
 *    The mV-to-watt conversion for this detector is NOT calibrated yet, so
 *    comp1_ref_mV is only a raw voltage and this channel acts as a coarse
 *    backstop.  See User/TODO-COMP1标定待办.txt.
 *
 *  COMP2 - main current overcurrent
 *      positive  PE9  (COMP2_INP) - same net as ADC1_INP5 / PB1, i.e. the
 *                CC6920BSO-50A current sensor buffered by U20-B (GS8552-SR)
 *      negative  DAC1_OUT2 - threshold, g_config.comp2_ref_mV
 *      output    PE8  (COMP2_OUT) - via D6
 *    This one has a published transfer function (V = 1650 mV + I * 26.4), so
 *    comp2_ref_mV maps to a real amp rating - the Settings screen shows the
 *    derived amps next to the millivolts for exactly that reason.
 *
 *  Both INP pins are analog-only: PB2 and PE9 are shared with no ADC channel
 *  of their own, which is why the software side reads the same two quantities
 *  through ADC1 ranks 1 and 2 instead.
 *==========================================================================*/

void boot_apply_comp_thresholds(void)
{
    /* Same settling gate as boot_apply_thresholds(): while the window is
     * open both comparator references must sit at the VDDA rail, so a
     * Settings edit that happens to land in that first second cannot push a
     * real reference into the DAC and trip the comparator on noise. */
    if (!s_thresholds_armed) {
        return;
    }

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
    /* Power-on settling window still open: the watchdogs are parked wide and
     * must stay that way.  Several unrelated callers reach this function -
     * switch_band() (power_init(), the GUI band buttons, freq_detect()) and
     * the protection Settings screen - and the first of those fires from the
     * Power task milliseconds after the scheduler starts, long before the
     * analog side has settled.  Honouring it here would undo
     * boot_hold_thresholds_open() and bring the false alarm straight back.
     *
     * Dropping the request is safe: the settling window always ends in
     * boot_arm_thresholds(), which reads the same g_config and the then
     * current band, so nothing is lost - the only difference is WHEN it
     * lands. */
    if (!s_thresholds_armed) {
        return;
    }

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

void boot_arm_thresholds(int band)
{
    /* Open the gate first, then apply: boot_apply_thresholds() checks the
     * flag, so the other order would silently drop this very call. */
    s_thresholds_armed = true;
    boot_apply_thresholds(band);
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

    /* Bring the panel out of standby on the same handshake.  Without
     * this the backlight is on but the LTDC source/gate drivers stay
     * blanked - the operator sees a uniformly lit white (or black)
     * panel with no UI.  See hw_pinmap.h for the polarity note. */
    TFT_DISP_ENABLE();
#endif
}

void boot_panel_off(void)
{
#ifndef LVGL_SIM
    /* Mirror of boot_backlight_on(), called on power_soft_off() so the
     * panel darkens in step with the supply rather than glowing after
     * the rail has dropped.  Order matters: blank the source drivers
     * (TFT_DISP) BEFORE killing the backlight PWM, so the panel does
     * not flash a frozen frame at full brightness during the collapse. */
    TFT_DISP_DISABLE();
#endif
}
