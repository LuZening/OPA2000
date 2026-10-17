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

void protection_clear(void)
{
    g_alert_flag   = false;
    g_alert_active = ALERT_NONE;
    g_alert_severity = ALERT_SEV_NONE;
}