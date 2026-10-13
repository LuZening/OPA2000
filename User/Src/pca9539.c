/**
 * @file pca9539.c
 * @brief PCA9539 16-bit I/O Expander Driver Implementation
 */

#include "pca9539.h"
#include "i2c.h"
#include <string.h>

/*============================================================================
 * Private Variables
 *============================================================================*/

static bool pca9539_initialized = false;

/* Button debounce state */
#define DEBOUNCE_TIME_MS    50

typedef struct {
    uint8_t last_state;
    uint32_t last_change_time;
} Pca9539Debounce;

static Pca9539Debounce debounce_state[2];  /* For both ports */

/* Current button states */
static Pca9539ButtonState button_state;
static Pca9539ButtonState button_pressed;  /* Edge detection */
static uint32_t last_process_time = 0;

/*============================================================================
 * Private Functions
 *============================================================================*/

/**
 * @brief Get I2C handle
 */
static I2C_HandleTypeDef* pca9539_get_i2c_handle(void)
{
    return &hi2c3;  /* I2C3 is used for external I2C devices */
}

/*============================================================================
 * Public Functions
 *============================================================================*/

/**
 * @brief Initialize PCA9539 I/O expander
 */
bool pca9539_init(void)
{
    I2C_HandleTypeDef *hi2c = pca9539_get_i2c_handle();

    /* Verify PCA9539 is present */
    uint8_t test_data;
    if (!pca9539_read_reg(PCA9539_REG_INPUT_PORT0, &test_data)) {
        pca9539_initialized = false;
        return false;
    }

    /* Configure port 0 as input (buttons) - 0xFF = all inputs */
    if (!pca9539_set_direction(PCA9539_PORT0, 0xFF)) {
        return false;
    }

    /* Configure port 1 as input (reserved for future use) */
    if (!pca9539_set_direction(PCA9539_PORT1, 0xFF)) {
        return false;
    }

    /* Initialize button states */
    memset(&button_state, 0, sizeof(button_state));
    memset(&button_pressed, 0, sizeof(button_pressed));
    memset(&debounce_state, 0, sizeof(debounce_state));

    pca9539_initialized = true;
    return true;
}

/**
 * @brief Write to PCA9539 register
 */
bool pca9539_write_reg(uint8_t reg, uint8_t data)
{
    I2C_HandleTypeDef *hi2c = pca9539_get_i2c_handle();
    uint8_t tx_data[2] = {reg, data};

    return (HAL_I2C_Master_Transmit(hi2c, PCA9539_I2C_ADDRESS,
                                    tx_data, 2, PCA9539_I2C_TIMEOUT) == HAL_OK);
}

/**
 * @brief Read from PCA9539 register
 */
bool pca9539_read_reg(uint8_t reg, uint8_t *data)
{
    I2C_HandleTypeDef *hi2c = pca9539_get_i2c_handle();

    if (HAL_I2C_Master_Transmit(hi2c, PCA9539_I2C_ADDRESS,
                                &reg, 1, PCA9539_I2C_TIMEOUT) != HAL_OK) {
        return false;
    }

    return (HAL_I2C_Master_Receive(hi2c, PCA9539_I2C_ADDRESS,
                                   data, 1, PCA9539_I2C_TIMEOUT) == HAL_OK);
}

/**
 * @brief Set port direction
 */
bool pca9539_set_direction(uint8_t port, uint8_t direction)
{
    uint8_t reg = (port == PCA9539_PORT0) ?
                  PCA9539_REG_CONFIG_PORT0 : PCA9539_REG_CONFIG_PORT1;
    return pca9539_write_reg(reg, direction);
}

/**
 * @brief Write output port
 */
bool pca9539_write_output(uint8_t port, uint8_t data)
{
    uint8_t reg = (port == PCA9539_PORT0) ?
                  PCA9539_REG_OUTPUT_PORT0 : PCA9539_REG_OUTPUT_PORT1;
    return pca9539_write_reg(reg, data);
}

/**
 * @brief Read input port
 */
bool pca9539_read_input(uint8_t port, uint8_t *data)
{
    uint8_t reg = (port == PCA9539_PORT0) ?
                  PCA9539_REG_INPUT_PORT0 : PCA9539_REG_INPUT_PORT1;
    return pca9539_read_reg(reg, data);
}

/**
 * @brief Get all button states
 */
bool pca9539_get_buttons(Pca9539ButtonState *state)
{
    if (state == NULL || !pca9539_initialized) {
        return false;
    }

    *state = button_state;
    return true;
}

/**
 * @brief Check if specific button is pressed
 */
bool pca9539_is_button_pressed(uint8_t pin)
{
    if (pin > 5 || !pca9539_initialized) {
        return false;
    }

    /* Map pin to button state */
    switch (pin) {
        case PCA9539_PIN_BAND_UP:    return button_state.band_up;
        case PCA9539_PIN_BAND_DOWN:  return button_state.band_down;
        case PCA9539_PIN_POWER_ON:   return button_state.power_on;
        case PCA9539_PIN_POWER_OFF:  return button_state.power_off;
        case PCA9539_PIN_TUNE:       return button_state.tune;
        case PCA9539_PIN_BYPASS:     return button_state.bypass;
        default:                      return false;
    }
}

/**
 * @brief Process button states (debounce)
 */
void pca9539_process(void)
{
    uint8_t port0_data;
    uint32_t current_time = HAL_GetTick();

    /* Check if enough time has passed for debounce */
    if ((current_time - last_process_time) < DEBOUNCE_TIME_MS) {
        return;
    }

    last_process_time = current_time;

    if (!pca9539_initialized) {
        return;
    }

    /* Read current input state */
    if (!pca9539_read_input(PCA9539_PORT0, &port0_data)) {
        return;
    }

    /* Invert because buttons are active low (pulled up) */
    port0_data = ~port0_data;

    /* Check for state changes (debounce logic) */
    uint8_t changed = port0_data ^ debounce_state[0].last_state;
    if (changed) {
        debounce_state[0].last_state = port0_data;

        /* Update button states (active high logic) */
        button_state.band_up    = (port0_data & (1 << PCA9539_PIN_BAND_UP))    ? true : false;
        button_state.band_down  = (port0_data & (1 << PCA9539_PIN_BAND_DOWN))  ? true : false;
        button_state.power_on   = (port0_data & (1 << PCA9539_PIN_POWER_ON))   ? true : false;
        button_state.power_off  = (port0_data & (1 << PCA9539_PIN_POWER_OFF))  ? true : false;
        button_state.tune       = (port0_data & (1 << PCA9539_PIN_TUNE))       ? true : false;
        button_state.bypass     = (port0_data & (1 << PCA9539_PIN_BYPASS))     ? true : false;
    }
}