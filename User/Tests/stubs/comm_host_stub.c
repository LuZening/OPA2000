/**
 * @file    comm_host_stub.c
 * @brief   Host-build stand-in for the parts of comm.c that protection.c
 *          calls into.
 *
 * @details comm.c itself is not linked into the host test binary: it
 *          pulls in LPUART1 HAL, USB CDC and the FreeRTOS semaphore API,
 *          none of which have a host stub.  But protection.c pushes an
 *          "alert raised / cleared" event into the LPUART1 comm ring
 *          (see comm_post_alert_evt), so the symbol has to resolve.
 *
 *          This stub swallows the event and records it, which lets
 *          test_protection.c assert that a trigger and a dismiss each
 *          published exactly one event - the behaviour that matters for
 *          the CAT link - without standing up the whole UART stack.
 *
 *          Not part of the firmware image.
 */

#include "opa2000.h"
#include <stdbool.h>

/*!< Number of events the firmware published since the last reset. */
unsigned int comm_stub_event_count = 0U;
/*!< Severity of the most recent event. */
volatile AlertSeverity comm_stub_last_severity = ALERT_SEV_NONE;
/*!< true if the most recent event was a raise. */
volatile bool comm_stub_last_on = false;

void comm_post_alert_evt(AlertType cause, AlertSeverity sev, bool on)
{
    (void)cause;
    comm_stub_event_count++;
    comm_stub_last_severity = sev;
    comm_stub_last_on      = on;
}
