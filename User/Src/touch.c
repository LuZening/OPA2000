/**
 * @file touch.c
 * @brief Touch Screen Driver Implementation
 */

#include "touch.h"
#include "config.h"
#include "stm32h7xx_hal.h"
#include "main.h"

/*============================================================================
 * Private Variables
 *============================================================================*/

/* Current touch type */
static TouchType detected_touch_type = TOUCH_NONE;

/* Public view of the detected controller, so the GUI can skip drawing
 * capacitive-only affordances.  touch_init() is the only writer. */
volatile TouchType g_touch_type = TOUCH_NONE;

/* Current touch position */
static TouchPointType current_touch = {0, 0, false};

/* Interrupt flag for capacitive touch */
static volatile uint8_t touch_interrupt_flag = 0;

/* Resistive touch SPI buffer */
static uint8_t resistive_tx_buf[TOUCH_SPI_BUFFER_SIZE];
static uint8_t resistive_rx_buf[TOUCH_SPI_BUFFER_SIZE];

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
 * (ImageWidth 480 / ImageHeight 272, RGB565). */
#define SCREEN_WIDTH    480
#define SCREEN_HEIGHT   272

/* Calibration screen positions (10% from edges + center) */
static const CalPointType cal_points[CAL_POINT_COUNT] = {
    {SCREEN_WIDTH / 10, SCREEN_HEIGHT / 10},           /* Top-left */
    {SCREEN_WIDTH * 9 / 10, SCREEN_HEIGHT / 10},       /* Top-right */
    {SCREEN_WIDTH / 10, SCREEN_HEIGHT * 9 / 10},       /* Bottom-left */
    {SCREEN_WIDTH * 9 / 10, SCREEN_HEIGHT * 9 / 10},   /* Bottom-right */
    {SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2}              /* Center */
};

/* GT911 I2C address */
#define GT911_I2C_ADDRESS     0xBA    /* 0x5D << 1 */

/*============================================================================
 * Private Functions
 *============================================================================*/

/**
 * @brief Read resistive touch position via SPI
 */
static bool touch_resistive_read_raw(uint16_t *x, uint16_t *y, bool *pressed)
{
    /* TCS2046 commands */
    #define TCS_CMD_START     0x80
    #define TCS_CMD_X_READ    0x50  /* 0xA0 | 0x10 */
    #define TCS_CMD_Y_READ    0x70  /* 0xA0 | 0x30 */

    uint8_t cmd;

    /* Check pen interrupt - active low means pressed */
    /* Assuming GPIO is configured for touch interrupt */
    *pressed = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11) == GPIO_PIN_RESET);

    if (!*pressed) {
        *x = 0;
        *y = 0;
        return false;
    }

    /* Read X position */
    cmd = TCS_CMD_X_READ;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);  /* CS low */
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
    HAL_SPI_Receive(&hspi1, resistive_rx_buf, 2, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);    /* CS high */

    *x = ((resistive_rx_buf[0] << 8) | resistive_rx_buf[1]) >> 4;

    /* Read Y position */
    cmd = TCS_CMD_Y_READ;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
    HAL_SPI_Receive(&hspi1, resistive_rx_buf, 2, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

    *y = ((resistive_rx_buf[0] << 8) | resistive_rx_buf[1]) >> 4;

    return true;
}

/**
 * @brief Read capacitive touch position via I2C
 */
static bool touch_capacitive_read_raw(uint16_t *x, uint16_t *y, bool *pressed)
{
    /* GT911 touch controller */
    /* Status register: 0x814E */
    /* Points: 0x8150 - up to 5 points */

    uint8_t status;
    uint8_t point_data[7];
    HAL_StatusTypeDef ret;

    /* Read status */
    ret = HAL_I2C_Mem_Read(&hi2c3, GT911_I2C_ADDRESS,
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
        HAL_I2C_Mem_Write(&hi2c3, GT911_I2C_ADDRESS,
                         0x814E, I2C_MEMADD_SIZE_16BIT,
                         &status, 1, 100);
        return false;
    }

    /* Read first touch point */
    ret = HAL_I2C_Mem_Read(&hi2c3, GT911_I2C_ADDRESS,
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
    HAL_I2C_Mem_Write(&hi2c3, GT911_I2C_ADDRESS,
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
        touch_resistive_init();
    } else if (detected_touch_type == TOUCH_CAPACITIVE) {
        touch_capacitive_init();
    }
}

/**
 * @brief Detect touch screen type
 */
TouchType touch_detect_type(void)
{
    /* Try to read from GT911 - if successful, it's capacitive */
    uint8_t test_byte = 0;

    /* Try I2C read from GT911 */
    if (HAL_I2C_Mem_Read(&hi2c3, GT911_I2C_ADDRESS,
                         0x0000, I2C_MEMADD_SIZE_16BIT,
                         &test_byte, 1, 50) == HAL_OK) {
        /* Check for GT911 product ID "911" */
        if (test_byte == '9' || test_byte == '1') {
            return TOUCH_CAPACITIVE;
        }
    }

    /* Default to resistive */
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
        if (touch_resistive_read(&x, &y, &pressed)) {
            if (pressed && g_config.touch.calibrated) {
                /* Apply calibration */
                touch_apply_calibration(x, y, &x, &y);
            }
            current_touch.x = x;
            current_touch.y = y;
            current_touch.pressed = pressed;
        } else {
            current_touch.pressed = false;
        }
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

    /* Wait for touch */
    if (!touch_resistive_read_raw(&raw_x, &raw_y, &pressed) || !pressed) {
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
 * @brief Initialize resistive touch
 */
void touch_resistive_init(void)
{
    /* Chip select pin should already be configured */
    /* Additional initialization if needed */
}

/**
 * @brief Read resistive touch
 */
bool touch_resistive_read(uint16_t *x, uint16_t *y, bool *pressed)
{
    return touch_resistive_read_raw(x, y, pressed);
}

/**
 * @brief Initialize capacitive touch
 */
void touch_capacitive_init(void)
{
    /* GT911 initialization sequence */
    /* This typically includes configuring interrupt and reset pins */

    /* Reset GT911 */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);  /* Reset pin */
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
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