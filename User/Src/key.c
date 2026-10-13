/**
 * @file key.c
 * @brief Button Input Processing Implementation
 * @note Supports 6 buttons via PCA9539 I/O expander with debounce and short/long press detection
 */

#include "key.h"
#include "pca9539.h"
#include <string.h>

/*============================================================================
 * Private Variables
 *============================================================================*/

static bool key_initialized = false;
static KeyCallback key_callback = NULL;

/* Key state array - one entry per key */
static KeyInfoType key_info[6];

/* Raw button state from PCA9539 */
static uint8_t key_raw_state = 0;
static uint8_t key_last_raw_state = 0;

/* Timing */
static uint32_t key_last_scan_tick = 0;
#define KEY_SCAN_PERIOD_MS      10      /* Scan every 10ms */

/*============================================================================
 * Private Function Prototypes
 *============================================================================*/
static void key_process_single(uint8_t key, bool pressed);
static void key_notify_callback(uint8_t key, KeyEventType event);

/*============================================================================
 * Public Functions
 *============================================================================*/

/**
 * @brief Initialize key processing
 */
void key_init(void)
{
    if (key_initialized) {
        return;
    }

    /* Initialize key info */
    memset(key_info, 0, sizeof(key_info));

    key_initialized = true;
    key_last_scan_tick = HAL_GetTick();
}

/**
 * @brief Process key scan (call periodically from task)
 */
void key_process(void)
{
    uint32_t current_tick;
    uint8_t i;
    bool pressed;

    if (!key_initialized) {
        return;
    }

    /* Rate limiting - scan every 10ms */
    current_tick = HAL_GetTick();
    if ((current_tick - key_last_scan_tick) < KEY_SCAN_PERIOD_MS) {
        return;
    }
    key_last_scan_tick = current_tick;

    /* Read current raw state from PCA9539 */
    uint8_t port_data;
    if (pca9539_read_input(PCA9539_PORT0, &port_data)) {
        /* Buttons are active low (pulled up), invert */
        key_raw_state = (uint8_t)(~port_data) & 0x3F;  /* Only use bits 0-5 */
    } else {
        /* I2C error - keep last state */
        return;
    }

    /* Check each key */
    for (i = 0; i < 6; i++) {
        pressed = (key_raw_state & (1 << i)) ? true : false;

        /* Detect edge */
        bool last_pressed = (key_last_raw_state & (1 << i)) ? true : false;

        if (pressed != last_pressed) {
            /* State changed */
            if (pressed) {
                /* Key pressed - start debounce */
                key_info[i].state = KEY_STATE_DEBOUNCING;
                key_info[i].press_start_tick = current_tick;
                key_info[i].longpress_sent = false;
                key_info[i].repeat_sent = false;
            } else {
                /* Key released */
                if (key_info[i].state == KEY_STATE_PRESSED) {
                    /* Short press completed */
                    key_notify_callback(i, KEY_EVENT_SHORTPRESS);
                } else if (key_info[i].state == KEY_STATE_LONGPRESS) {
                    /* Long press released */
                    key_notify_callback(i, KEY_EVENT_RELEASED);
                }
                key_info[i].state = KEY_STATE_IDLE;
            }
        }

        /* Process based on current state */
        key_process_single(i, pressed);
    }

    /* Update last raw state for next cycle */
    key_last_raw_state = key_raw_state;
}

/**
 * @brief Process single key state machine
 */
static void key_process_single(uint8_t key, bool pressed)
{
    KeyInfoType *ki = &key_info[key];
    uint32_t current_tick = HAL_GetTick();
    uint32_t press_duration;

    switch (ki->state) {
        case KEY_STATE_IDLE:
            /* Nothing to do */
            break;

        case KEY_STATE_DEBOUNCING:
            /* Wait debounce time */
            if ((current_tick - ki->press_start_tick) >= KEY_DEBOUNCE_MS) {
                /* Debounce complete - confirm still pressed */
                if (pressed) {
                    ki->state = KEY_STATE_PRESSED;
                } else {
                    /* Was a glitch - reset */
                    ki->state = KEY_STATE_IDLE;
                }
            }
            break;

        case KEY_STATE_PRESSED:
            /* Check for long press */
            press_duration = current_tick - ki->press_start_tick;
            if (press_duration >= KEY_LONGPRESS_MS) {
                ki->state = KEY_STATE_LONGPRESS;
                ki->longpress_sent = true;
                key_notify_callback(key, KEY_EVENT_LONGPRESS_START);
            }
            break;

        case KEY_STATE_LONGPRESS:
            /* Handle repeat */
            press_duration = current_tick - ki->press_start_tick;
            if (!ki->repeat_sent && (press_duration >= (KEY_LONGPRESS_MS + KEY_REPEAT_MS))) {
                ki->repeat_sent = true;
                key_notify_callback(key, KEY_EVENT_LONGPRESS_HOLD);
            } else if (ki->repeat_sent) {
                /* Reset repeat after another interval */
                uint32_t repeat_count = (press_duration - KEY_LONGPRESS_MS) / KEY_REPEAT_MS;
                if ((press_duration - KEY_LONGPRESS_MS) >= ((repeat_count + 1) * KEY_REPEAT_MS)) {
                    ki->repeat_sent = false;  /* Allow next repeat */
                }
            }
            break;

        default:
            ki->state = KEY_STATE_IDLE;
            break;
    }
}

/**
 * @brief Notify callback of key event
 */
static void key_notify_callback(uint8_t key, KeyEventType event)
{
    if (key_callback != NULL) {
        key_callback(key, event);
    }

    /* Also call individual key action handlers */
    switch (key) {
        case KEY_POWER:
            key_power_action(event);
            break;
        case KEY_MENU:
            key_menu_action(event);
            break;
        case KEY_BAND:
            key_band_action(event);
            break;
        case KEY_LEFT:
            key_left_action(event);
            break;
        case KEY_RIGHT:
            key_right_action(event);
            break;
        case KEY_ENTER:
            key_enter_action(event);
            break;
        default:
            break;
    }
}

/**
 * @brief Register key event callback
 */
void key_register_callback(KeyCallback callback)
{
    key_callback = callback;
}

/**
 * @brief Get key current pressed state
 */
bool key_is_pressed(uint8_t key)
{
    if (key > 5) {
        return false;
    }
    return (key_raw_state & (1 << key)) ? true : false;
}

/**
 * @brief Force clear all key states
 */
void key_reset(void)
{
    memset(key_info, 0, sizeof(key_info));
    key_last_raw_state = 0;
    key_raw_state = 0;
}

/*============================================================================
 * Key Action Implementations
 *============================================================================*/

/**
 * @brief Power key action handler
 * @note Short press: Toggle IDLE/STANDBY mode
 *       Long press: Enter sleep mode
 */
void key_power_action(KeyEventType event)
{
    extern void power_standby(void);
    extern void power_on(void);
    extern void power_off(void);
    extern volatile PowerStateType g_power_state;

    switch (event) {
        case KEY_EVENT_SHORTPRESS:
            {
                PowerStateType state = g_power_state;
                if (state == POWER_STATE_STANDBY) {
                    /* Already in standby, power off */
                    power_off();
                } else if (state == POWER_STATE_OFF) {
                    /* Power on to standby */
                    power_on();
                } else {
                    /* In TX state, go to standby */
                    power_standby();
                }
            }
            break;

        case KEY_EVENT_LONGPRESS_START:
            /* Long press - enter sleep mode */
            key_enter_sleep();
            break;

        default:
            break;
    }
}

/**
 * @brief Menu key action handler
 * @note Short press: Enter/exit GUI menu
 */
void key_menu_action(KeyEventType event)
{
    extern void gui_toggle_menu(void);

    if (event == KEY_EVENT_SHORTPRESS) {
        gui_toggle_menu();
    }
}

/**
 * @brief Band key action handler
 * @note Short press: Show/hide band selection popup
 */
void key_band_action(KeyEventType event)
{
    extern void gui_toggle_band_popup(void);

    if (event == KEY_EVENT_SHORTPRESS) {
        gui_toggle_band_popup();
    }
}

/**
 * @brief Left key action handler
 * @note Short press: Focus previous group object
 */
void key_left_action(KeyEventType event)
{
    extern void gui_focus_prev(void);

    if (event == KEY_EVENT_SHORTPRESS) {
        gui_focus_prev();
    }
}

/**
 * @brief Right key action handler
 * @note Short press: Focus next group object
 */
void key_right_action(KeyEventType event)
{
    extern void gui_focus_next(void);

    if (event == KEY_EVENT_SHORTPRESS) {
        gui_focus_next();
    }
}

/**
 * @brief Enter key action handler
 * @note Short press: Simulate Enter key press
 */
void key_enter_action(KeyEventType event)
{
    extern void gui_enter_pressed(void);

    if (event == KEY_EVENT_SHORTPRESS) {
        gui_enter_pressed();
    }
}

/**
 * @brief Enter sleep mode (placeholder)
 * @note To be implemented - suspend operations and enter low power mode
 */
void key_enter_sleep(void)
{
    /* TODO: Implement sleep mode
     * - Save current state to EEPROM
     * - Disable unnecessary peripherals
     * - Enter stop mode or standby mode
     * - Wakeup on power key press
     */
}