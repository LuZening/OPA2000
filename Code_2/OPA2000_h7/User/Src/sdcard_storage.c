/**
 * @file sdcard_storage.c
 * @brief SD Card access layer on top of the FatFs disk driver
 */

#include "sdcard_storage.h"
#include "ff.h"
#include "diskio.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>
#include <stdbool.h>

/*============================================================================
 * Private Variables
 *============================================================================*/

/* SD card ready flag */
static bool sdcard_ready = false;

/* Card info (cached) */
static uint32_t sd_block_count = 0;
static uint32_t sd_block_size = 512;

/* Binary semaphore for shared SD card access between MSC ISR and FatFS tasks.
 * Using FreeRTOS semaphore directly because CMSIS-OS2 doesn't provide
 * ISR-safe acquire/release. */
SemaphoreHandle_t sd_card_sem_isr;

/*============================================================================
 * Mutex Functions
 *============================================================================*/

void sdcard_storage_init_mutex(void)
{
  sd_card_sem_isr = xSemaphoreCreateBinary();
  /* Give the semaphore so it starts in "available" state */
  if (sd_card_sem_isr != NULL) {
    xSemaphoreGive(sd_card_sem_isr);
  }
}

bool sdcard_storage_take_mutex(uint32_t timeout_ms)
{
  if (sd_card_sem_isr == NULL) return true;
  return (xSemaphoreTake(sd_card_sem_isr, pdMS_TO_TICKS(timeout_ms)) == pdTRUE);
}

void sdcard_storage_give_mutex(void)
{
  if (sd_card_sem_isr != NULL) {
    xSemaphoreGive(sd_card_sem_isr);
  }
}

bool sdcard_storage_take_mutex_from_isr(void)
{
  if (sd_card_sem_isr == NULL) return true;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  BaseType_t result = xSemaphoreTakeFromISR(sd_card_sem_isr, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  return (result == pdTRUE);
}

void sdcard_storage_give_mutex_from_isr(void)
{
  if (sd_card_sem_isr != NULL) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(sd_card_sem_isr, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

/*============================================================================
 * Public Functions
 *============================================================================*/

/**
 * @brief Initialize SD card storage
 * @param lun Logical unit number (0)
 * @retval 0 if successful, -1 otherwise
 */
int8_t sdcard_storage_init(uint8_t lun)
{
    (void)lun;
    DSTATUS status;

    /* Initialize FatFS disk layer for SD card */
    status = disk_initialize(0);
    if (status & STA_NOINIT) {
        return -1;
    }

    /* Get card capacity via disk_ioctl */
    uint32_t sectors = 0;
    if (disk_ioctl(0, GET_SECTOR_COUNT, &sectors) == RES_OK) {
        sd_block_count = sectors;
    }

    uint16_t sector_size = 0;
    if (disk_ioctl(0, GET_SECTOR_SIZE, &sector_size) == RES_OK) {
        sd_block_size = sector_size;
    }

    sdcard_ready = true;
    return 0;
}

/**
 * @brief Get SD card capacity
 * @param lun Logical unit number
 * @param block_num Pointer to store block count
 * @param block_size Pointer to store block size
 * @retval 0 if successful
 */
int8_t sdcard_storage_get_capacity(uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
    (void)lun;

    if (!sdcard_ready) {
        return -1;
    }

    /* Use cached values */
    *block_num = sd_block_count;
    *block_size = sd_block_size;

    return 0;
}

/**
 * @brief Check if SD card is ready
 * @param lun Logical unit number
 * @retval 0 if ready, -1 if not ready
 */
int8_t sdcard_storage_is_ready(uint8_t lun)
{
    (void)lun;

    if (!sdcard_ready) {
        return -1;
    }

    DSTATUS status = disk_status(0);
    if (status & STA_NOINIT) {
        return -1;
    }

    return 0;
}

/**
 * @brief Check if SD card is write protected
 * @param lun Logical unit number
 * @retval 0 if not protected
 */
int8_t sdcard_storage_is_write_protected(uint8_t lun)
{
    (void)lun;
    return 0;  /* SD cards are not write protected */
}

/**
 * @brief Read blocks from SD card
 * @param lun Logical unit number
 * @param buff Data buffer
 * @param sector Start sector (LBA)
 * @param count Number of sectors
 * @retval 0 if successful
 */
int8_t sdcard_storage_read(uint8_t lun, uint8_t *buff, uint32_t sector, uint16_t count)
{
    (void)lun;

    if (!sdcard_ready) {
        return -1;
    }

    DRESULT res;
    bool taken = sdcard_storage_take_mutex_from_isr();
    if (!taken) {
        return -1;
    }

    res = disk_read(0, buff, sector, count);

    sdcard_storage_give_mutex_from_isr();
    return (res == RES_OK) ? 0 : -1;
}

/**
 * @brief Write blocks to SD card
 * @param lun Logical unit number
 * @param buff Data to write
 * @param sector Start sector (LBA)
 * @param count Number of sectors
 * @retval 0 if successful
 */
int8_t sdcard_storage_write(uint8_t lun, const uint8_t *buff, uint32_t sector, uint16_t count)
{
    (void)lun;

    if (!sdcard_ready) {
        return -1;
    }

    DRESULT res;
    bool taken = sdcard_storage_take_mutex_from_isr();
    if (!taken) {
        return -1;
    }

    res = disk_write(0, buff, sector, count);
    disk_ioctl(0, CTRL_SYNC, NULL);  /* Flush after write */

    sdcard_storage_give_mutex_from_isr();
    return (res == RES_OK) ? 0 : -1;
}

/**
 * @brief Get max LUN (just return 0 - single SD card)
 * @retval 0
 */
int8_t sdcard_storage_get_max_lun(void)
{
    return 0;
}