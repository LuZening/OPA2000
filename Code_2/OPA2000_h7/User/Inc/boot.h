/**
 * @file    boot.h
 * @brief   Boot-time sequencing helpers (pre-kernel, single-threaded).
 *
 * @details Everything here runs in main() BEFORE osKernelStart(), i.e.
 *          single-threaded with no RTOS primitives in play.  That is by
 *          design: the config mutex and the GUI mutex may not exist yet.
 *
 *  Boot order (see main() USER CODE 2 and CLAUDE.md):
 *    0. Hardware reset state: RF_BIAS (PG2) = LOW, PWR_ON (PG7) = LOW,
 *       REAR_RESERVED1 (PF4) = HIGH.  Nothing transmits.
 *    1. boot_gpio_safe_state()  - pin the three safety pins explicitly
 *                                and force the backlight PWM to 0 %.
 *    2. config_init()           - load or rebuild the config.
 *    3. touch_init()            - probe GT911 / NS2009.
 *    4. boot_hold_thresholds_open() - park every watchdog at full scale
 *                                (AWD1/2/3 high threshold = 65535, both
 *                                COMP DAC references = VDDA rail) and
 *                                drive CLEAR_FAULTX_OC low.
 *    5. osKernelStart()         - tasks take over; the backlight comes
 *                                up once the GUI has painted a frame.
 *
 *  Why step 4 must happen before the kernel starts: the COMP interrupts
 *  are already armed by MX_COMP1/2_Init().  If the DAC references are
 *  still 0 then any noise on the compared analog inputs would trip the
 *  comparator and latch a protection flag before the operator ever sees
 *  the screen.  Parking everything wide open costs microseconds and
 *  cannot be crossed.
 *
 *  The operator's REAL limits are pushed much later - from the ADC task's
 *  adc_stabilisation_tick(), after a 1 s settling window plus a successful
 *  IM zero-offset calibration.  See user_adc.c.
 */
#ifndef BOOT_H
#define BOOT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Program the watchdogs wide open for the power-on settling period.
 *
 *  Called from main() once the peripherals are up but before the analog
 *  side has settled.  Every high threshold goes to full scale:
 *
 *    - AWD1/AWD2/AWD3 high threshold = 16-bit full scale (65535)
 *    - DAC1 CH1 / CH2 -> COMP1/COMP2 = the top of the DAC, so the reference
 *      pin sits at the VDDA rail
 *
 *  A watchdog whose threshold is at full scale cannot be crossed and a
 *  comparator referenced to the rail cannot trip, so nothing latches while
 *  the rail charges and the detector outputs settle.
 *
 *  The real limits replace these in boot_apply_thresholds(), which the ADC
 *  task calls once the IM zero-offset calibration has passed - see
 *  app_adc_task() and adc_stabilisation_tick().
 *
 *  Idempotent; safe to call more than once.
 */
void boot_hold_thresholds_open(void);

/**
 * @brief Pin the three safety pins and blank the display.
 *
 *  - RF_BIAS_ON  (PG2)  LOW   - PA bias disabled
 *  - PWR_ON      (PG7)  LOW   - high-voltage supply off
 *  - REAR_RESERVED1 (PF4) HIGH - keep the main rail alive (soft-off idle)
 *  - BL_PWM      (PD14) 0 %   - screen stays black until the GUI is up
 *
 *  Safe to call more than once; it is idempotent.
 */
void boot_gpio_safe_state(void);

/**
 * @brief Push the protection limits from g_config into the hardware.
 *
 *  Programs:
 *    - AWD1 (ADC1 watchdog 1) on RF input        <- band_rf_in_max_w
 *    - AWD2 (ADC1 watchdog 2) on main current   <- band_im_max_mA
 *    - AWD3 (ADC1 watchdog 3) on coupler-1 rev <- band_rev1_max_w
 *    - DAC1 CH1 -> COMP1 inverting input        <- comp1_ref_mV
 *    - DAC1 CH2 -> COMP2 inverting input        <- comp2_ref_mV
 *
 *  @param band  band whose band_rf_in_max_w[] entry should be used;
 *               pass the current band so the RF-in limit tracks the
 *               operator's band choice.
 *
 *  @note SILENT NO-OP until boot_arm_thresholds() has run.  Between
 *        boot_hold_thresholds_open() and the end of the power-on settling
 *        window every watchdog is parked at full scale, and this function
 *        refuses to touch the hardware.  That gate is load-bearing, not
 *        defensive: power_init() -> switch_band() -> this function runs
 *        from the Power task a few milliseconds after the scheduler
 *        starts, i.e. inside the window.  Dropping the request loses
 *        nothing, because boot_arm_thresholds() re-reads the same
 *        g_config when the window closes.
 */
void boot_apply_thresholds(int band);

/**
 * @brief Close the settling gate and push the real limits in one step.
 *
 *  Called exactly once, by adc_stabilisation_tick() on the tick where the
 *  IM zero-offset calibration passes.  From here on boot_apply_thresholds()
 *  behaves normally, so later band changes and Settings edits take effect
 *  immediately again.
 *
 *  @param band  the current band, as for boot_apply_thresholds().
 */
void boot_arm_thresholds(int band);

/**
 * @brief Re-program just the two COMP DAC references from g_config.
 *
 *  Split out of boot_apply_thresholds() because the watchdogs are indexed
 *  by band while the comparator references are not.  The Settings screen
 *  calls this after the operator edits comp1_ref_mV / comp2_ref_mV so the
 *  new threshold takes hold without a reboot, and so the band-dependent
 *  watchdog re-indexing stays out of that path.
 *
 *  Both values are clamped to COMP1_REF_MV_MAX / COMP2_REF_MV_MAX here as
 *  well as in the GUI: this is the last place that can stop a corrupted
 *  config from parking a comparator output at its rail.
 */
void boot_apply_comp_thresholds(void);

/**
 * @brief Turn the backlight on.  Called by the GUI task once it has
 *        rendered a few frames, so the operator never sees a black
 *        screen with no explanation.
 *
 *  Also called directly when the touch calibration screen is entered,
 *  because that screen is frequently the first thing shown after a
 *  factory reset and must be readable for the operator to proceed.
 */
void boot_backlight_on(void);

/*!< Panel off: drives TFT_DISP low to put the LTDC panel into standby.
 *   Called from power_soft_off() so the panel darkens in step with the
 *   supply, and ordered BEFORE the backlight PWM is killed so the LCD
 *   does not flash a frozen frame at full brightness during collapse.
 *   No-op on the LVGL simulator. */
void boot_panel_off(void);

/*!< Backlight duty used for every "panel is awake" path.  90 % rather
 *   than 100 %: the calibration screen is often the first thing the
 *   operator sees, and full brightness on a panel that has just woken is
 *   harsh and warms the LCD. */
#define BOOT_BACKLIGHT_PCT      100U

/**
 * @brief Drive BL_PWM to a duty cycle in percent (0..100).
 *  @note   Used by boot_gpio_safe_state() for the 0 % blank and by
 *          boot_backlight_on() for the run level.  Defined in
 *          power_control.c next to the rest of the PWM plumbing.
 */
void power_set_backlight_duty(uint8_t percent);

#ifdef __cplusplus
}
#endif

#endif /* BOOT_H */
