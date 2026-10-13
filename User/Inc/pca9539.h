/**
 * @file pca9539.h
 * @brief PCA9539 16-bit I/O Expander Driver Header
 */

#ifndef PCA9539_H
#define PCA9539_H

#ifdef __cplusplus
extern "C" {
#endif

#include "opa2000.h"
#include "i2c.h"

/*============================================================================
 * PCA9539 Configuration
 *============================================================================*/

#define PCA9539_I2C_ADDRESS         0x74    /* PCA9539 I2C address */
#define PCA9539_I2C_TIMEOUT         100     /* I2C timeout in ms */

/* PCA9539 Register Addresses */
#define PCA9539_REG_INPUT_PORT0     0x00    /* Input port 0 */
#define PCA9539_REG_INPUT_PORT1     0x01    /* Input port 1 */
#define PCA9539_REG_OUTPUT_PORT0    0x02    /* Output port 0 */
#define PCA9539_REG_OUTPUT_PORT1    0x03    /* Output port 1 */
#define PCA9539_REG_POLARITY_INV0   0x04    /* Polarity inversion port 0 */
#define PCA9539_REG_POLARITY_INV1   0x05    /* Polarity inversion port 1 */
#define PCA9539_REG_CONFIG_PORT0    0x06    /* Configuration port 0 (1=input, 0=output) */
#define PCA9539_REG_CONFIG_PORT1    0x07    /* Configuration port 1 (1=input, 0=output) */

/* Port definitions */
#define PCA9539_PORT0               0       /* Port 0 (P00-P07) */
#define PCA9539_PORT1               1       /* Port 1 (P10-P17) */

/* Pin definitions for buttons */
#define PCA9539_PIN_BAND_UP         0       /* P00 - Band Up button */
#define PCA9539_PIN_BAND_DOWN       1       /* P01 - Band Down button */
#define PCA9539_PIN_POWER_ON        2       /* P02 - Power On button */
#define PCA9539_PIN_POWER_OFF       3       /* P03 - Power Off button */
#define PCA9539_PIN_TUNE            4       /* P04 - Tune button */
#define PCA9539_PIN_BYPASS          5       /* P05 - Bypass button */

/* Button state structure */
typedef struct {
    bool band_up;
    bool band_down;
    bool power_on;
    bool power_off;
    bool tune;
    bool bypass;
} Pca9539ButtonState;

/*============================================================================
 * Function Prototypes
 *============================================================================*/

/**
 * @brief Initialize PCA9539 I/O expander
 * @return true if initialization successful
 */
bool pca9539_init(void);

/**
 * @brief Write to PCA9539 register
 * @param reg Register address
 * @param data Data to write
 * @return true if successful
 */
bool pca9539_write_reg(uint8_t reg, uint8_t data);

/**
 * @brief Read from PCA9539 register
 * @param reg Register address
 * @param data Pointer to store read data
 * @return true if successful
 */
bool pca9539_read_reg(uint8_t reg, uint8_t *data);

/**
 * @brief Set port direction
 * @param port Port number (0 or 1)
 * @param direction 0xFF = all inputs, 0x00 = all outputs
 * @return true if successful
 */
bool pca9539_set_direction(uint8_t port, uint8_t direction);

/**
 * @brief Write output port
 * @param port Port number (0 or 1)
 * @param data Output data
 * @return true if successful
 */
bool pca9539_write_output(uint8_t port, uint8_t data);

/**
 * @brief Read input port
 * @param port Port number (0 or 1)
 * @param data Pointer to store input data
 * @return true if successful
 */
bool pca9539_read_input(uint8_t port, uint8_t *data);

/**
 * @brief Get all button states
 * @param state Pointer to store button states
 * @return true if successful
 */
bool pca9539_get_buttons(Pca9539ButtonState *state);

/**
 * @brief Check if specific button is pressed
 * @param pin Pin number (0-5 for port 0)
 * @return true if button is pressed
 */
bool pca9539_is_button_pressed(uint8_t pin);

/**
 * @brief Process button states (call periodically)
 * @note Handles button debouncing
 */
void pca9539_process(void);

#ifdef __cplusplus
}
#endif

#endif /* PCA9539_H */