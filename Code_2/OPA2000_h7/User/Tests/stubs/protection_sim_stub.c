/**
 * @file    protection_sim_stub.c
 * @brief   LVGL_SIM stand-in for protection_clear() so gui.c links
 *          cleanly when the firmware protection.c is not built.
 *
 * @details protection.c depends on the STM32 HAL, the AWD callbacks
 *          and a real ADC backend, none of which are available in the
 *          LVGL PC simulator.  gui.c only references protection_clear()
 *          to acknowledge an alert; for the simulator we just clear
 *          the global flag.
 *
 *          The simulator's own alert path runs through comm_sim_stub.c
 *          (which swallows comm_post_alert_evt) and through the GUI's
 *          own LVGL_SIM synthesised reads (gui_update_main_values).
 */

#include "opa2000.h"
#include <stdint.h>

void protection_clear(void)
{
    g_alert_flag   = false;
    g_alert_active = ALERT_NONE;
    g_alert_severity = ALERT_SEV_NONE;
}

/* The main screen's debug overlay lists the per-source trigger history.
 * The real bitmask lives in protection.c, which is not part of this build,
 * so the simulator reports "nothing has fired" - every row 0.  That is the
 * honest answer for a machine with no protection hardware behind it, and it
 * is also what keeps the overlay's layout honest: the rows are laid out and
 * measured with the same widths either way, so a simulator screenshot is
 * still a valid check that the text is NOT being truncated.
 *
 * uint32_t, not unsigned int: protection.h declares it uint32_t so the
 * firmware and this stand-in agree on the width the 32-bit bitmask needs.
 * MSVC would mangle a mismatched declaration as a different symbol, which
 * is exactly the LNK2001 this stub exists to prevent. */
uint32_t protection_source_fired_mask(void)
{
    return 0U;
}

/* ---- adc_calibration.c stand-ins -----------------------------------
 *
 * gui.c's protection-settings screen shows what the COMP2 reference voltage
 * means in amps, and since the derivation change that baseline is the
 * MEASURED zero point rather than a hardcoded 1.65 V.  adc_calibration.c
 * pulls in the STM32 HAL, so it is not part of this build either.
 *
 * These return the nominal values a correctly fitted CC6920 would sit at,
 * which keeps the simulator screen showing the same amps the firmware would
 * for a healthy unit. */
uint32_t adc_im_zero_mV(void)
{
    return 1650U;
}

uint32_t adc_im_mv_per_amp_q1(void)
{
    return 264U;   /* 26.4 mV/A */
}