/**
 * @file key.h
 * @brief Button Input Processing Header
 * @note 6 buttons: KEY1 on PC13 (PWR_WKUP4, keeps the wake-up path),
 *       KEY2..KEY6 on a resistor ladder read by ADC1_INP17 (PA1).
 */

#ifndef KEY_H
#define KEY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "opa2000.h"
#include "hw_pinmap.h"

/*============================================================================
 * Button Definitions
 *============================================================================*/
#define KEY_POWER       0   /* PC13 KEY1_WKUP  - dedicated GPIO (wake-up)   */
#define KEY_MENU        1   /* ADC ladder     - menu                      */
#define KEY_BAND        2   /* ADC ladder     - band select               */
#define KEY_LEFT        3   /* ADC ladder     - left                      */
#define KEY_RIGHT       4   /* ADC ladder     - right                     */
#define KEY_ENTER       5   /* ADC ladder     - enter / confirm           */
#define KEY_COUNT       6

#define KEY_NONE        0xFFU   /*!< no ladder key recognised */

/*============================================================================
 * Resistor ladder decoding (ADC1_INP17, PA1)
 *
 * Each key pulls the node to its own divider tap, so a pressed key shows up
 * as a distinct ADC code.  KEY_NONE when nothing is pressed is whatever the
 * pull-up alone produces.
 *
 * Thresholds are the *midpoints* between neighbouring taps, so a tap only
 * has to be within half a bucket of its nominal value to be recognised.
 * They are deliberately wide: the ladder is resistively loaded by a
 * C91 filter and the code drifts with temperature, so precision here buys
 * nothing.  Recalibrate from measured codes if the ladder is respun.
 *============================================================================*/
#define KEY_ADC_IDLE_MIN     62000U  /*!< above this: pull-up, nothing pressed */
#define KEY_ADC_TAP_MENU     55000U  /*!< >= this: KEY_MENU                   */
#define KEY_ADC_TAP_BAND     47000U  /*!< >= this: KEY_BAND                   */
#define KEY_ADC_TAP_LEFT     38000U  /*!< >= this: KEY_LEFT                   */
#define KEY_ADC_TAP_RIGHT    26000U  /*!< >= this: KEY_RIGHT                  */
#define KEY_ADC_TAP_ENTER    10000U  /*!< >= this: KEY_ENTER                  */

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
    KEY_EVENT_SHORTPRESS,              /*!< Short press detected */
    KEY_EVENT_LONGPRESS_START,         /*!< Long press started (first time) */
    KEY_EVENT_LONGPRESS_HOLD,          /*!< Long press held (repeated) */
    KEY_EVENT_RELEASED                 /*!< Key released */
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
    KeyStateType state;                /*!< Current button state */
    uint32_t press_start_tick;         /*!< Tick when button was pressed */
    bool longpress_sent;               /*!< Long press event already sent */
    bool repeat_sent;                  /*!< Repeat event already sent */
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
 */
void key_init(void);

/**
 * @brief Process key scan (call periodically from task)
 * @note Non-blocking, call every 10ms
 */
void key_process(void);

/**
 * @brief Decode an ADC1_INP17 code into a key id
 * @param code Raw 16-bit ADC code from g_adc_data.key_adc
 * @return KEY_POWER..KEY_ENTER, or KEY_NONE
 */
uint8_t key_decode_adc(uint16_t code);

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
 * Key Action Functions
 *============================================================================*/

/**
 * @brief Handle power key action
 */
void key_power_action(KeyEventType event);

/**
 * @brief Handle menu key action
 */
void key_menu_action(KeyEventType event);

/**
 * @brief Handle band key action
 */
void key_band_action(KeyEventType event);

/**
 * @brief Handle left key action
 */
void key_left_action(KeyEventType event);

/**
 * @brief Handle right key action
 */
void key_right_action(KeyEventType event);

/**
 * @brief Handle enter key action
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