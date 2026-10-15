/**
 * @file gui.h
 * @brief LVGL GUI Header
 */
#ifndef GUI_H
#define GUI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"
#include "opa2000.h"

/* Font declarations (needed for both normal build and LVGL simulator) */
LV_FONT_DECLARE(MyCJK_12);
LV_FONT_DECLARE(MyCJK_16);

/*============================================================================
 *  Concurrency rules (firmware, NOT the PC simulator)
 *============================================================================
 *  LVGL is not thread-safe.  Every entry point into the GUI in this header
 *  (gui_init / gui_tick / gui_show_alert / gui_dismiss_alert / gui_toggle_* /
 *  gui_process_* / gui_update_* / gui_set_language / etc.) is wrapped by
 *  gui_lock()/gui_unlock() so that two FreeRTOS tasks (the GUI task + the
 *  protect task + the comm task + the monitor task) cannot mutate widgets
 *  at the same time.
 *
 *  ISR / interrupt context:  NO gui_*() function may be called from an ISR.
 *  An ISR that needs to inform the GUI must set a flag (e.g. g_alert_flag)
 *  and let the GUI task pick it up in its next tick.  Calling
 *  gui_lock() from ISR would deadlock on osMutexAcquire.
 *
 *  The PC simulator build (LVGL_SIM) has no FreeRTOS, so the lock/unlock
 *  helpers compile to no-ops.  The "no ISR calls" rule still applies to
 *  the firmware but is unenforced there.
 *============================================================================*/
#ifndef LVGL_SIM
#include "cmsis_os.h"
extern osMutexId_t gui_mutex;
#endif

/** Lock the GUI resource (no-op in LVGL_SIM).  Must NOT be called from ISR. */
void gui_lock(void);

/** Unlock the GUI resource (no-op in LVGL_SIM). */
void gui_unlock(void);

/*============================================================================
 *  SIM-only diagnostics
 *  Exposed so the multithreaded stress test in LVGL.Simulator.cpp can
 *  assert that every gui_lock() is matched by a gui_unlock().  Never call
 *  from firmware code; under LVGL_SIM the symbol is unconditionally
 *  available because the simulator's stress-test C file needs it.
 *============================================================================*/
#ifdef LVGL_SIM
uint32_t gui_sim_outstanding_locks(void);
#endif

/*============================================================================
 * Function Prototypes
 *============================================================================*/

/**
 * @brief Initialize LVGL GUI
 */
void gui_init(void);

/**
 * @brief Start GUI task
 */
void gui_start(void);

/**
 * @brief GUI tick task (call every 5-10ms)
 */
void gui_tick(void);

/**
 * @brief Update GUI display values
 * @note Call periodically from GUI task
 */
void gui_update_display(void);

/**
 * @brief Show alert dialog
 * @param alert_type Type of alert
 */
void gui_show_alert(AlertType alert_type);

/**
 * @brief Dismiss alert dialog
 */
void gui_dismiss_alert(void);

/**
 * @brief Check if alert is shown
 * @return true if alert dialog is active
 */
bool gui_is_alert_active(void);

/**
 * @brief Get translated string
 * @param str_id String ID
 * @return Translated string pointer
 */
const char* gui_get_string(uint16_t str_id);

/**
 * @brief Set language
 * @param lang Language type
 */
void gui_set_language(LanguageType lang);

/**
 * @brief Show calibration screen
 */
void gui_show_calibration(void);

/**
 * @brief Process calibration touch input (call from main loop)
 */
void gui_process_calibration(void);

/**
 * @brief Show coupler calibration screen
 * @param coupler_id 1 or 2
 */
void gui_show_coupler_cal(uint8_t coupler_id);

/**
 * @brief Process coupler calibration ADC update (call from main loop)
 */
void gui_process_coupler_cal(void);

/**
 * @brief Update sensor inspect values (call from main loop)
 */
void gui_update_sensor_inspect(void);

/**
 * @brief Update calibration screen
 * @param current_point Current calibration point index
 */
void gui_update_calibration(uint8_t current_point);

/**
 * @brief Power button pressed
 */
void gui_power_button_pressed(void);

/**
 * @brief Get current screen
 * @return Current screen ID
 */
uint8_t gui_get_current_screen(void);

/**
 * @brief Toggle menu visibility
 */
void gui_toggle_menu(void);

/**
 * @brief Toggle band selection popup
 */
void gui_toggle_band_popup(void);

/**
 * @brief Focus previous group object
 */
void gui_focus_prev(void);

/**
 * @brief Focus next group object
 */
void gui_focus_next(void);

/**
 * @brief Simulate Enter key press
 */
void gui_enter_pressed(void);

#ifdef __cplusplus
}
#endif

#endif /* GUI_H */