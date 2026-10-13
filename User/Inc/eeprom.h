/**
 * @file eeprom.h
 * @brief AT24C02 EEPROM Driver Header
 */

#ifndef EEPROM_H
#define EEPROM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

/*============================================================================
 * AT24C02 Specifications
 *============================================================================*/

#define AT24C02_ADDRESS        0xA0    /* 1010 A2 A1 A0 = 0xA0 */
#define AT24C02_PAGE_SIZE      8       /* 8 bytes per page */
#define AT24C02_TOTAL_SIZE     256     /* 2K bits = 256 bytes */
#define AT24C02_ADDR_MASK      0xFF    /* Address mask */

/* Timing (ms) */
#define EEPROM_WRITE_CYCLE_TIME    5   /* Write cycle time max 5ms */

/*============================================================================
 * Function Prototypes
 *============================================================================*/

/**
 * @brief Initialize EEPROM I2C
 */
void eeprom_init(void);

/**
 * @brief Write data to EEPROM
 * @param addr Start address (0-255)
 * @param data Data pointer
 * @param len Data length
 * @return true if successful
 */
bool eeprom_write(uint16_t addr, const uint8_t *data, uint16_t len);

/**
 * @brief Read data from EEPROM
 * @param addr Start address (0-255)
 * @param data Data buffer
 * @param len Data length
 * @return true if successful
 */
bool eeprom_read(uint16_t addr, uint8_t *data, uint16_t len);

/**
 * @brief Write single byte to EEPROM
 * @param addr Address (0-255)
 * @param data Byte to write
 * @return true if successful
 */
bool eeprom_write_byte(uint16_t addr, uint8_t data);

/**
 * @brief Read single byte from EEPROM
 * @param addr Address (0-255)
 * @param data Pointer to store read byte
 * @return true if successful
 */
bool eeprom_read_byte(uint16_t addr, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* EEPROM_H */