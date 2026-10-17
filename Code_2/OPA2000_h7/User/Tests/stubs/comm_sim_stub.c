/**
 * @file    comm_sim_stub.c
 * @brief   LVGL_SIM stand-in for the comm_post_alert_evt() symbol that
 *          protection.c links against.
 *
 * @details comm.c itself is not built into the LVGL simulator (it
 *          pulls in the LPUART1 HAL + USB CDC, neither of which is
 *          available on a PC).  But protection.c publishes alert
 *          transitions through comm_post_alert_evt() so the GUI can
 *          push them out via the PC CAT link, and the symbol must
 *          resolve at link time.
 *
 *          We swallow the call; the simulator has no real CAT link to
 *          notify.
 */

#include "opa2000.h"
#include <stdbool.h>

void comm_post_alert_evt(AlertType cause, AlertSeverity sev, bool on)
{
    (void)cause;
    (void)sev;
    (void)on;
}