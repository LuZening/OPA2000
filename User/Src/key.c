/**
 * @file key.c
 * @brief Button Input Processing Implementation
 * @note 6 buttons: KEY1 on PC13 (PWR_WKUP4) keeps a real GPIO so the
 *       wake-up path survives; KEY2..KEY6 share one ADC1_INP17 (PA1) node
 *       through a resistor ladder, so only one of them can be read per scan.
 *       The old PCA9539 I2C expander is gone from the hardware.
 */

#include "key.h"
#include "gui.h"
#include "power_control.h"
#include <string.h>

/*============================================================================
 * Private Variables
 *============================================================================*/

static bool key_initialized = false;
static KeyCallback key_callback = NULL;

/* Key state array - one entry per key */
static KeyInfoType key_info[KEY_COUNT];

/* Which key is down right now, KEY_NONE if none. */
static uint8_t key_active = KEY_NONE;

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

    memset(key_info, 0, sizeof(key_info));

    key_active      = KEY_NONE;
    key_initialized = true;
    key_last_scan_tick = HAL_GetTick();
}

/**
 * @brief Decode an ADC1_INP17 code into a key id
 *
 * Descending ladder: the highest tap below the idle level is the key that was
 * pressed.  Anything above KEY_ADC_IDLE_MIN is the pull-up with no key down.
 */
uint8_t key_decode_adc(uint16_t code)
{
    if (code >= KEY_ADC_IDLE_MIN) {
        return KEY_NONE;
    }
    if (code >= KEY_ADC_TAP_MENU) {
        return KEY_MENU;
    }
    if (code >= KEY_ADC_TAP_BAND) {
        return KEY_BAND;
    }
    if (code >= KEY_ADC_TAP_LEFT) {
        return KEY_LEFT;
    }
    if (code >= KEY_ADC_TAP_RIGHT) {
        return KEY_RIGHT;
    }
    if (code >= KEY_ADC_TAP_ENTER) {
        return KEY_ENTER;
    }
    /* Below the lowest tap: shorted or floating node, not a key. */
    return KEY_NONE;
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

    /* KEY1 is a plain GPIO, active low (pull-up).  It is deliberately not
     * derived from the ADC: it doubles as the PWR_WKUP4 wake source. */
    pressed = (HAL_GPIO_ReadPin(KEY1_WKUP_GPIO_Port, KEY1_WKUP_Pin) == GPIO_PIN_RESET);

    if (pressed) {
        /* Power key wins over the ladder: it can wake the unit from standby,
         * and a simultaneous ladder reading is almost certainly noise. */
        key_active = KEY_POWER;
    } else {
        key_active = key_decode_adc(g_adc_data.key_adc);
    }

    for (i = 0; i < KEY_COUNT; i++) {
        bool now_pressed = (key_active == i);

        /* Detect edge */
        bool last_pressed = (key_info[i].state != KEY_STATE_IDLE) &&
                            (key_info[i].state != KEY_STATE_DEBOUNCING);

        if (now_pressed && !last_pressed) {
            key_info[i].state = KEY_STATE_DEBOUNCING;
            key_info[i].press_start_tick = current_tick;
            key_info[i].longpress_sent = false;
            key_info[i].repeat_sent = false;
        } else if (!now_pressed && last_pressed) {
            /* Key released */
            if (key_info[i].state == KEY_STATE_PRESSED) {
                key_notify_callback(i, KEY_EVENT_SHORTPRESS);
            } else if (key_info[i].state == KEY_STATE_LONGPRESS) {
                key_notify_callback(i, KEY_EVENT_RELEASED);
            }
            key_info[i].state = KEY_STATE_IDLE;
        }

        /* Process based on current state */
        key_process_single(i, now_pressed);
    }
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
    if (key >= KEY_COUNT) {
        return false;
    }
    return (key_active == key);
}

/**
 * @brief Force clear all key states
 */
void key_reset(void)
{
    memset(key_info, 0, sizeof(key_info));
    key_active = KEY_NONE;
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
    switch (event) {
        case KEY_EVENT_SHORTPRESS:
            /* Same three-state cycle as the GUI banner button, and the same
             * semantics: it toggles the HV rail (PWR_ON + RF_BIAS), not the
             * main supply latch and not the relays. */
            if (g_tx_active) {
                /* Transmitting, drop back to ready */
                power_standby();
            } else if (g_hv_on) {
                /* High voltage already up, take it down */
                power_hv_disable();
            } else {
                /* High voltage down, bring it up to standby */
                power_hv_enable();
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