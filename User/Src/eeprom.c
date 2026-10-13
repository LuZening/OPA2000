/**
 * @file eeprom.c
 * @brief AT24C02 EEPROM Driver Implementation
 */

#include "eeprom.h"
#include "i2c.h"
#include "dma.h"

/*============================================================================
 * Private Variables
 *============================================================================*/

static I2C_HandleTypeDef *hi2c_eeprom = &hi2c3;
static bool eeprom_initialized = false;

/*============================================================================
 * Private Functions
 *============================================================================*/

/**
 * @brief Get device address with page number
 */
static uint16_t eeprom_get_device_address(uint16_t addr)
{
    /* AT24C02 has 256 bytes, 8-byte pages, 32 pages */
    /* A2 A1 A0 pins are tied to GND, so bits are 0 */
    return AT24C02_ADDRESS | ((addr >> 8) << 1);
}

/**
 * @brief Wait for EEPROM write cycle to complete
 */
static HAL_StatusTypeDef eeprom_wait_ready(uint32_t timeout)
{
    HAL_StatusTypeDef status;
    uint32_t tickstart = HAL_GetTick();

    do {
        status = HAL_I2C_IsDeviceReady(hi2c_eeprom, AT24C02_ADDRESS, 1, timeout);
        if (status != HAL_OK) {
            if (HAL_I2C_GetError(hi2c_eeprom) != HAL_I2C_ERROR_AF) {
                return HAL_ERROR;
            }
        } else {
            return HAL_OK;
        }
    } while ((HAL_GetTick() - tickstart) < timeout);

    return HAL_TIMEOUT;
}

/*============================================================================
 * Public Functions
 *============================================================================*/

/**
 * @brief Initialize EEPROM I2C
 */
void eeprom_init(void)
{
    if (eeprom_initialized) {
        return;
    }

    /* MX_I2C3_Init() should have been called before this */
    /* We just need to verify I2C is ready */
    eeprom_initialized = true;
}

/**
 * @brief Write data to EEPROM
 * @note This handles page boundary crossing automatically
 */
bool eeprom_write(uint16_t addr, const uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;
    uint16_t page_remaining;
    uint16_t chunk_size;

    if (!eeprom_initialized || addr >= AT24C02_TOTAL_SIZE) {
        return false;
    }

    /* Limit len to not exceed EEPROM */
    if (addr + len > AT24C02_TOTAL_SIZE) {
        len = AT24C02_TOTAL_SIZE - addr;
    }

    while (len > 0) {
        /* Calculate remaining bytes in current page */
        page_remaining = AT24C02_PAGE_SIZE - (addr % AT24C02_PAGE_SIZE);
        chunk_size = (len < page_remaining) ? len : page_remaining;

        /* Wait for previous write to complete */
        if (eeprom_wait_ready(10) != HAL_OK) {
            return false;
        }

        /* Write page */
        status = HAL_I2C_Mem_Write(hi2c_eeprom,
                                   eeprom_get_device_address(addr),
                                   (uint16_t)(addr & 0xFF),
                                   I2C_MEMADD_SIZE_8BIT,
                                   (uint8_t*)data,
                                   chunk_size,
                                   100);

        if (status != HAL_OK) {
            return false;
        }

        /* Update counters */
        data += chunk_size;
        addr += chunk_size;
        len -= chunk_size;
    }

    return true;
}

/**
 * @brief Read data from EEPROM
 */
bool eeprom_read(uint16_t addr, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;

    if (!eeprom_initialized || addr >= AT24C02_TOTAL_SIZE) {
        return false;
    }

    /* Limit len to not exceed EEPROM */
    if (addr + len > AT24C02_TOTAL_SIZE) {
        len = AT24C02_TOTAL_SIZE - addr;
    }

    /* Sequential read */
    status = HAL_I2C_Mem_Read(hi2c_eeprom,
                              eeprom_get_device_address(addr),
                              (uint16_t)(addr & 0xFF),
                              I2C_MEMADD_SIZE_8BIT,
                              data,
                              len,
                              100);

    return (status == HAL_OK);
}

/**
 * @brief Write single byte to EEPROM
 */
bool eeprom_write_byte(uint16_t addr, uint8_t data)
{
    return eeprom_write(addr, &data, 1);
}

/**
 * @brief Read single byte from EEPROM
 */
bool eeprom_read_byte(uint16_t addr, uint8_t *data)
{
    return eeprom_read(addr, data, 1);
}