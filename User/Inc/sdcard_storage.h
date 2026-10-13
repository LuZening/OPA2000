/**
 * @file sdcard_storage.h
 * @brief SD Card Storage for USB MSC - uses FatFS disk layer
 */

#ifndef SDCARD_STORAGE_H
#define SDCARD_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_msc.h"
#include "ff.h"
#include <stdbool.h>

/*============================================================================
 * Function Prototypes
 *============================================================================*/

/**
 * @brief Initialize SD card storage
 * @param lun Logical unit number (0)
 * @retval 0 if successful, -1 otherwise
 */
int8_t sdcard_storage_init(uint8_t lun);

/**
 * @brief Get SD card capacity
 * @param lun Logical unit number
 * @param block_num Pointer to store block count
 * @param block_size Pointer to store block size
 * @retval 0 if successful
 */
int8_t sdcard_storage_get_capacity(uint8_t lun, uint32_t *block_num, uint16_t *block_size);

/**
 * @brief Check if SD card is ready
 * @param lun Logical unit number
 * @retval 0 if ready, -1 if not ready
 */
int8_t sdcard_storage_is_ready(uint8_t lun);

/**
 * @brief Check if SD card is write protected
 * @param lun Logical unit number
 * @retval 0 if not protected
 */
int8_t sdcard_storage_is_write_protected(uint8_t lun);

/**
 * @brief Read blocks from SD card
 * @param lun Logical unit number
 * @param buff Data buffer
 * @param sector Sector address (LBA)
 * @param count Number of sectors
 * @retval 0 if successful
 */
int8_t sdcard_storage_read(uint8_t lun, uint8_t *buff, uint32_t sector, uint16_t count);

/**
 * @brief Write blocks to SD card
 * @param lun Logical unit number
 * @param buff Data to write
 * @param sector Sector address (LBA)
 * @param count Number of sectors
 * @retval 0 if successful
 */
int8_t sdcard_storage_write(uint8_t lun, const uint8_t *buff, uint32_t sector, uint16_t count);

/**
 * @brief Get max LUN (returns 0 - single SD card)
 * @retval 0
 */
int8_t sdcard_storage_get_max_lun(void);

/**
 * @brief Initialize SD card mutex for shared access
 * Must be called before USB init and FatFS operations
 */
void sdcard_storage_init_mutex(void);

/**
 * @brief Take SD card mutex from ISR context
 * @retval true if mutex taken successfully
 */
bool sdcard_storage_take_mutex_from_isr(void);

/**
 * @brief Give SD card mutex from ISR context
 */
void sdcard_storage_give_mutex_from_isr(void);

/**
 * @brief Take SD card mutex from task context
 * @param timeout_ms Timeout in ms (0 = no wait)
 * @retval true if mutex taken successfully
 */
bool sdcard_storage_take_mutex(uint32_t timeout_ms);

/**
 * @brief Give SD card mutex from task context
 */
void sdcard_storage_give_mutex(void);

#ifdef __cplusplus
}
#endif

#endif /* SDCARD_STORAGE_H */