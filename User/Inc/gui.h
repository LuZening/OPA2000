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
LV_FONT_DECLARE(MyCJK_16);

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