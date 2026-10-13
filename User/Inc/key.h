/**
 * @file key.h
 * @brief Button Input Processing Header
 * @note Supports 6 buttons via PCA9539 I/O expander with debounce and short/long press detection
 */

#ifndef KEY_H
#define KEY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "opa2000.h"

/*============================================================================
 * Button Definitions (connected to PCA9539 Port 0)
 *============================================================================*/
#define KEY_POWER       0       /* P00 - Power key */
#define KEY_MENU        1       /* P01 - Menu key */
#define KEY_BAND        2       /* P02 - Band key */
#define KEY_LEFT        3       /* P03 - Left arrow key */
#define KEY_RIGHT       4       /* P04 - Right arrow key */
#define KEY_ENTER       5       /* P05 - Enter/Confirm key */

/*============================================================================
 * Button Timing Configuration
 *============================================================================*/
#define KEY_DEBOUNCE_MS         20      /* Debounce time: 20ms (must be < 50ms) */
#define KEY_LONGPRESS_MS        800     /* Long press threshold: 800ms */
#define KEY_REPEAT_MS           200     /* Repeat interval for held keys */

/*============================================================================
 * Button Event Types
 *============================================================================*/
typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_SHORTPRESS,              /* Short press detected */
    KEY_EVENT_LONGPRESS_START,         /* Long press started (first time) */
    KEY_EVENT_LONGPRESS_HOLD,          /* Long press held (repeated) */
    KEY_EVENT_RELEASED                 /* Key released */
} KeyEventType;

/*============================================================================
 * Button State
 *============================================================================*/
typedef enum {
    KEY_STATE_IDLE = 0,
    KEY_STATE_DEBOUNCING,
    KEY_STATE_PRESSED,
    KEY_STATE_LONGPRESS
} KeyStateType;

/*============================================================================
 * Button Information Structure
 *============================================================================*/
typedef struct {
    KeyStateType state;                /* Current button state */
    uint32_t press_start_tick;         /* Tick when button was pressed */
    bool longpress_sent;               /* Long press event already sent */
    bool repeat_sent;                  /* Repeat event already sent */
} KeyInfoType;

/*============================================================================
 * Key Event Callback Function Type
 *============================================================================*/
typedef void (*KeyCallback)(uint8_t key, KeyEventType event);

/*============================================================================
 * Function Prototypes
 *============================================================================*/

/**
 * @brief Initialize key processing
 * @note Must be called after PCA9539 initialization
 */
void key_init(void);

/**
 * @brief Process key scan (call periodically from task)
 * @note Non-blocking, call every 10ms
 */
void key_process(void);

/**
 * @brief Register key event callback
 * @param callback Function to call on key events
 */
void key_register_callback(KeyCallback callback);

/**
 * @brief Get key current pressed state
 * @param key Key ID (KEY_POWER to KEY_ENTER)
 * @return true if key is currently pressed
 */
bool key_is_pressed(uint8_t key);

/**
 * @brief Force clear all key states (for wake from sleep)
 */
void key_reset(void);

/*============================================================================
 * Key Action Functions (to be implemented by user)
 *============================================================================*/

/**
 * @brief Handle power key action
 * @param event Key event type
 */
void key_power_action(KeyEventType event);

/**
 * @brief Handle menu key action
 * @param event Key event type
 */
void key_menu_action(KeyEventType event);

/**
 * @brief Handle band key action
 * @param event Key event type
 */
void key_band_action(KeyEventType event);

/**
 * @brief Handle left key action
 * @param event Key event type
 */
void key_left_action(KeyEventType event);

/**
 * @brief Handle right key action
 * @param event Key event type
 */
void key_right_action(KeyEventType event);

/**
 * @brief Handle enter key action
 * @param event Key event type
 */
void key_enter_action(KeyEventType event);

/**
 * @brief Enter sleep mode (placeholder)
 * @note To be implemented
 */
void key_enter_sleep(void);

#ifdef __cplusplus
}
#endif

#endif /* KEY_H */