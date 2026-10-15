/**
 * @file touch.c
 * @brief Touch Screen Driver Implementation
 *
 * @details Both controllers sit on the SAME bus - I2C2 (PF0/SDA,
 *          PF1/SCL) - and are told apart by their 7-bit I2C address:
 *
 *   1. NS2009  RTOUCH, resistive, 7-bit 0b1001000 = 0x48
 *   2. GT911   CTOUCH, capacitive, 7-bit 0b0010100 = 0x14
 *
 *          The first one to ACK (within its timeout) wins.  Probing is
 *          single-threaded - it runs in main() before osKernelStart(),
 *          so there is no I2C contention and no need for a mutex.
 *
 *          The capacitive probe is tried FIRST because the GT911 is the
 *          production panel and the NS2009 is the alternative stocking
 *          option; on a board that has both fitted the capacitive one is
 *          the one wired to the FPC connector.
 *
 *          The legacy XPT2046/TSC2046 SPI path was removed; SPI1 is no
 *          longer used by the touch driver.
 */

#include "touch.h"
#include "config.h"
#include "stm32h7xx_hal.h"
#include "main.h"
#include "ns2009.h"

/*============================================================================
 * Private Variables
 *============================================================================*/

/* Current touch type */
static TouchType detected_touch_type = TOUCH_NONE;

/* True when the resistive panel is an NS2009 on I2C2.  Only one resistive
 * controller is supported now - the legacy XPT2046/TSC2046 SPI path was
 * dropped, so the resistive backend is always NS2009. */
static bool s_resistive_is_ns2009 = false;

/* Public view of the detected controller, so the GUI can skip drawing
 * capacitive-only affordances.  touch_init() is the only writer. */
volatile TouchType g_touch_type = TOUCH_NONE;

/* Current touch position */
static TouchPointType current_touch = {0, 0, false};

/* Interrupt flag for capacitive touch */
static volatile uint8_t touch_interrupt_flag = 0;

/* Calibration data */
static bool calibrating = false;
static uint8_t cal_point_index = 0;
static struct {
    uint16_t raw_x;
    uint16_t raw_y;
    uint16_t screen_x;
    uint16_t screen_y;
} cal_data[CAL_POINT_COUNT];

/* Screen dimensions - must match the LTDC layer in Core/Src/ltdc.c
 * (ImageWidth 480 / ImageHeight 272, RGB565).  Used to clamp the
 * calibrated output in touch_apply_calibration(). */
#define SCREEN_WIDTH    480
#define SCREEN_HEIGHT   272

/* Note: the calibration target positions live in gui.c (cal_targets[])
 * because the red-dot widgets are an LVGL concept; touch.c only
 * receives the screen coordinate from the GUI when a point is
 * accepted.  There is deliberately no second copy here. */

/* GT911 (CTOUCH) 7-bit I2C address 0b0010100 = 0x14.  HAL wants the
 * address shifted left by one so the R/W bit lands in bit 0.  Note this is
 * NOT the 0x5D / 0x14 pair quoted in some GT911 datasheets for the
 * alternative address-selection strap: on this board the controller
 * answers at 0x14. */
#define GT911_I2C_ADDRESS     (0x14U << 1)   /* 0x28, HAL expects 8-bit */

/* Both controllers are on I2C2 (PF0/SDA, PF1/SCL), configured by
 * MX_I2C2_Init().  I2C3 exists in the .ioc (PA8/PC9, with DMA) but has
 * nothing on it - see touch.c's header comment. */
extern I2C_HandleTypeDef hi2c2;

/*============================================================================
 * Private Functions
 *============================================================================*/

/**
 * @brief Read touch from NS2009 over I2C2.
 *
 * Returns true if the chip reported a press; false if the bus errored
 * or the read was below the press threshold.
 */
static bool touch_ns2009_read(uint16_t *x, uint16_t *y, bool *pressed)
{
    uint16_t raw_x, raw_y;
    bool ok = ns2009_read(&raw_x, &raw_y);
    if (!ok) {
        *pressed = false;
        return false;
    }
    *x = raw_x;
    *y = raw_y;
    *pressed = true;
    return true;
}

/**
 * @brief Initialize the NS2009 driver.  Nothing to do: the chip needs
 * no configuration beyond power-on.  Probe is what decided we exist.
 */
static void touch_ns2009_init(void) { /* no-op */ }

/**
 * @brief Read capacitive touch position via I2C
 */
static bool touch_capacitive_read_raw(uint16_t *x, uint16_t *y, bool *pressed)
{
    /* GT911 touch controller, on I2C2 alongside the NS2009. */
    /* Status register: 0x814E */
    /* Points: 0x8150 - up to 5 points */

    uint8_t status;
    uint8_t point_data[7];
    HAL_StatusTypeDef ret;

    /* Read status */
    ret = HAL_I2C_Mem_Read(&hi2c2, GT911_I2C_ADDRESS,
                           0x814E, I2C_MEMADD_SIZE_16BIT,
                           &status, 1, 100);

    if (ret != HAL_OK) {
        return false;
    }

    /* Check if touch detected (bit 0) */
    *pressed = (status & 0x80) != 0;
    if (!*pressed) {
        /* Clear buffer */
        status = 0;
        HAL_I2C_Mem_Write(&hi2c2, GT911_I2C_ADDRESS,
                         0x814E, I2C_MEMADD_SIZE_16BIT,
                         &status, 1, 100);
        return false;
    }

    /* Read first touch point */
    ret = HAL_I2C_Mem_Read(&hi2c2, GT911_I2C_ADDRESS,
                           0x8150, I2C_MEMADD_SIZE_16BIT,
                           point_data, 7, 100);

    if (ret != HAL_OK) {
        return false;
    }

    /* Extract coordinates */
    *x = ((uint16_t)point_data[1] << 8) | point_data[2];
    *y = ((uint16_t)point_data[3] << 8) | point_data[4];

    /* Clear status */
    status = 0;
    HAL_I2C_Mem_Write(&hi2c2, GT911_I2C_ADDRESS,
                     0x814E, I2C_MEMADD_SIZE_16BIT,
                     &status, 1, 100);

    return true;
}

/*============================================================================
 * Public Functions
 *============================================================================*/

/**
 * @brief Initialize touch screen
 */
void touch_init(void)
{
    detected_touch_type = touch_detect_type();
    g_touch_type = detected_touch_type;

    if (detected_touch_type == TOUCH_RESISTIVE) {
        s_resistive_is_ns2009 = true;
        touch_ns2009_init();
    } else if (detected_touch_type == TOUCH_CAPACITIVE) {
        s_resistive_is_ns2009 = false;
        touch_capacitive_init();
    }
}

/**
 * @brief Detect touch screen type
 *
 * Both controllers hang off I2C2 and are distinguished purely by address:
 *
 *   1. GT911 on I2C2, 7-bit 0x14 (capacitive, production panel).
 *   2. NS2009 on I2C2, 7-bit 0x48 (resistive, RTOUCH).
 *
 * Capacitive is probed first because that is the production panel; the
 * NS2009 is the alternative stocking option.  Neither I2C bus is shared
 * with anything else, so a NACK means "not fitted" rather than "bus busy".
 *
 * The 100 ms per-transaction budget is enforced by
 * NS2009_I2C_TIMEOUT_MS (100) and the GT911 read below (50 ms).  A chip
 * that does not ACK within that is treated as absent.
 */
TouchType touch_detect_type(void)
{
    /* Step 1: GT911 over I2C2 (capacitive). */
    {
        uint8_t test_byte = 0;
        if (HAL_I2C_Mem_Read(&hi2c2, GT911_I2C_ADDRESS,
                             0x0000, I2C_MEMADD_SIZE_16BIT,
                             &test_byte, 1, 50) == HAL_OK) {
            if (test_byte == '9' || test_byte == '1') {
                return TOUCH_CAPACITIVE;
            }
        }
    }

    /* Step 2: NS2009 over I2C2 (resistive, RTOUCH). */
    if (ns2009_probe()) {
        return TOUCH_RESISTIVE;
    }

    /* Neither controller answered.  Fall back to resistive so the
     * GUI at least comes up; the read path will simply never report a
     * press.  This keeps a board with a broken I2C bus from bricking
     * the whole unit. */
    return TOUCH_RESISTIVE;
}

/**
 * @brief Get touch position
 */
bool touch_get_position(TouchPointType *point)
{
    if (point == NULL) return false;

    point->x = current_touch.x;
    point->y = current_touch.y;
    point->pressed = current_touch.pressed;

    return current_touch.pressed;
}

/**
 * @brief Process touch data
 */
void touch_process(void)
{
    bool pressed;
    uint16_t x, y;

    if (detected_touch_type == TOUCH_RESISTIVE) {
        if (s_resistive_is_ns2009) {
            touch_ns2009_read(&x, &y, &pressed);
        }
        if (pressed && g_config.touch.calibrated) {
            /* Apply calibration.  NS2009 reports the panel raw
             * coordinates 0..4095, so the gain/offset transform from
             * the 5-point calibration maps that to screen pixels. */
            touch_apply_calibration(x, y, &x, &y);
        }
        current_touch.x = x;
        current_touch.y = y;
        current_touch.pressed = pressed;
    } else if (detected_touch_type == TOUCH_CAPACITIVE) {
        if (touch_capacitive_read(&x, &y, &pressed)) {
            /* Capacitive touch usually doesn't need calibration */
            current_touch.x = x;
            current_touch.y = y;
            current_touch.pressed = pressed;
        } else {
            current_touch.pressed = false;
        }
    }
}

/**
 * @brief Start calibration
 */
void touch_calibrate_start(void)
{
    calibrating = true;
    cal_point_index = 0;

    /* Reset calibration data */
    memset(cal_data, 0, sizeof(cal_data));
}

/**
 * @brief Add calibration point
 */
bool touch_calibrate_add_point(uint16_t screen_x, uint16_t screen_y)
{
    uint16_t raw_x, raw_y;
    bool pressed;

    if (!calibrating || cal_point_index >= CAL_POINT_COUNT) {
        return false;
    }

    /* Wait for touch.  Only the resistive (NS2009) path has a
     * calibration screen; the capacitive panel needs none. */
    bool got = touch_ns2009_read(&raw_x, &raw_y, &pressed);
    if (!got || !pressed) {
        return false;
    }

    /* Store point */
    cal_data[cal_point_index].raw_x = raw_x;
    cal_data[cal_point_index].raw_y = raw_y;
    cal_data[cal_point_index].screen_x = screen_x;
    cal_data[cal_point_index].screen_y = screen_y;

    cal_point_index++;

    return (cal_point_index >= CAL_POINT_COUNT);
}

/**
 * @brief Complete calibration
 */
bool touch_calibrate_complete(void)
{
    float x1, y1;

    if (cal_point_index < CAL_POINT_COUNT) {
        calibrating = false;
        return false;
    }

    /* Linear regression for X: screen_x = raw_x * slope + offset */
    /* Using least squares: slope = (N*sum(xy) - sum(x)*sum(y)) / (N*sum(x^2) - sum(x)^2) */
    float sum_raw_x = 0, sum_screen_x = 0;
    float sum_raw_x2 = 0;
    float sum_raw_x_screen_x = 0;

    for (int i = 0; i < CAL_POINT_COUNT; i++) {
        sum_raw_x += cal_data[i].raw_x;
        sum_screen_x += cal_data[i].screen_x;
        sum_raw_x2 += (float)cal_data[i].raw_x * cal_data[i].raw_x;
        sum_raw_x_screen_x += (float)cal_data[i].raw_x * cal_data[i].screen_x;
    }

    x1 = (CAL_POINT_COUNT * sum_raw_x2 - sum_raw_x * sum_raw_x);
    if (fabsf(x1) < 0.001f) {
        calibrating = false;
        return false;
    }

    g_config.touch.x_gain = (CAL_POINT_COUNT * sum_raw_x_screen_x - sum_raw_x * sum_screen_x) / x1;
    g_config.touch.x_offset = (sum_screen_x - g_config.touch.x_gain * sum_raw_x) / CAL_POINT_COUNT;

    /* Linear regression for Y */
    float sum_raw_y = 0, sum_screen_y = 0;
    float sum_raw_y2 = 0;
    float sum_raw_y_screen_y = 0;

    for (int i = 0; i < CAL_POINT_COUNT; i++) {
        sum_raw_y += cal_data[i].raw_y;
        sum_screen_y += cal_data[i].screen_y;
        sum_raw_y2 += (float)cal_data[i].raw_y * cal_data[i].raw_y;
        sum_raw_y_screen_y += (float)cal_data[i].raw_y * cal_data[i].screen_y;
    }

    y1 = (CAL_POINT_COUNT * sum_raw_y2 - sum_raw_y * sum_raw_y);
    if (fabsf(y1) < 0.001f) {
        calibrating = false;
        return false;
    }

    g_config.touch.y_gain = (CAL_POINT_COUNT * sum_raw_y_screen_y - sum_raw_y * sum_screen_y) / y1;
    g_config.touch.y_offset = (sum_screen_y - g_config.touch.y_gain * sum_raw_y) / CAL_POINT_COUNT;

    /* Check center point residual */
    /* Center point is at index 4 */
    float predicted_x = cal_data[4].raw_x * g_config.touch.x_gain + g_config.touch.x_offset;
    float predicted_y = cal_data[4].raw_y * g_config.touch.y_gain + g_config.touch.y_offset;

    float residual_x = fabsf(predicted_x - cal_data[4].screen_x) / cal_data[4].screen_x;
    float residual_y = fabsf(predicted_y - cal_data[4].screen_y) / cal_data[4].screen_y;

    if (residual_x > 0.1f || residual_y > 0.1f) {
        /* Calibration failed - residual too high */
        calibrating = false;
        return false;
    }

    /* Calibration successful */
    g_config.touch.calibrated = 1U;
    calibrating = false;

    /* Save to config */
    config_save();

    return true;
}

/**
 * @brief Check if touch is pressed
 */
bool touch_is_pressed(void)
{
    return current_touch.pressed;
}

/**
 * @brief Apply calibration
 */
void touch_apply_calibration(uint16_t raw_x, uint16_t raw_y,
                             uint16_t *cal_x, uint16_t *cal_y)
{
    if (!g_config.touch.calibrated) {
        *cal_x = raw_x;
        *cal_y = raw_y;
        return;
    }

    *cal_x = (uint16_t)(raw_x * g_config.touch.x_gain + g_config.touch.x_offset);
    *cal_y = (uint16_t)(raw_y * g_config.touch.y_gain + g_config.touch.y_offset);

    /* Clamp to screen bounds */
    if (*cal_x > SCREEN_WIDTH) *cal_x = SCREEN_WIDTH;
    if (*cal_y > SCREEN_HEIGHT) *cal_y = SCREEN_HEIGHT;
}

/*============================================================================
 * Touch Type Specific Functions
 *============================================================================*/

/**
 * @brief Initialize capacitive touch
 */
void touch_capacitive_init(void)
{
    /* GT911 power-on reset.  CTouch_RSTX is PF3 per the .ioc (CTouch_RSTX);
     * this used to be a hardcoded PD10, which is LTDC_B3 - an LCD data
     * line - so the "reset" was pulling the panel bus down instead of the
     * touch controller. */
    HAL_GPIO_WritePin(CTouch_RSTX_GPIO_Port, CTouch_RSTX_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(CTouch_RSTX_GPIO_Port, CTouch_RSTX_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
}

/**
 * @brief Read capacitive touch
 */
bool touch_capacitive_read(uint16_t *x, uint16_t *y, bool *pressed)
{
    return touch_capacitive_read_raw(x, y, pressed);
}

/**
 * @brief Touch interrupt ISR handler
 * @note Called from HAL_GPIO_EXTI_Callback
 *       For capacitive touch (GT911), this indicates touch event
 *       For resistive touch, this may not be used
 */
void touch_int_isr(void)
{
    /* Set flag to indicate touch interrupt occurred */
    /* The GUI task will process this in touch_process() */
    touch_interrupt_flag = 1;
}