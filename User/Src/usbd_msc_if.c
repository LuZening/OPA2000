/**
 * @file usbd_msc_if.c
 * @brief Memory management layer for USB MSC - uses sdcard_storage
 */

#include "usbd_msc_if.h"
#include "sdcard_storage.h"

/* Private define ------------------------------------------------------------*/
#define STORAGE_LUN_NBR                  1

/* USB Mass storage Standard Inquiry Data */
const int8_t STORAGE_Inquirydata_HS[] = {
  /* LUN 0 */
  0x00,
  0x80,
  0x02,
  0x02,
  (STANDARD_INQUIRY_DATA_LEN - 5),
  0x00,
  0x00,
  0x00,
  'S', 'T', 'M', '3', '2', ' ', ' ', ' ',
  'S', 'D', ' ', 'C', 'a', 'r', 'd', ' ',
  'M', 'S', 'C', ' ', ' ', ' ', ' ', ' ',
  '1', '.', '0' ,'0'
};

/* Storage interface functions */
int8_t STORAGE_Init_HS(uint8_t lun)
{
  return sdcard_storage_init(lun);
}

int8_t STORAGE_GetCapacity_HS(uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
  return sdcard_storage_get_capacity(lun, block_num, block_size);
}

int8_t STORAGE_IsReady_HS(uint8_t lun)
{
  return sdcard_storage_is_ready(lun);
}

int8_t STORAGE_IsWriteProtected_HS(uint8_t lun)
{
  (void)lun;
  return 0;  /* Not write protected */
}

int8_t STORAGE_Read_HS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  return sdcard_storage_read(lun, buf, blk_addr, blk_len);
}

int8_t STORAGE_Write_HS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  return sdcard_storage_write(lun, buf, blk_addr, blk_len);
}

int8_t STORAGE_GetMaxLun_HS(void)
{
  return sdcard_storage_get_max_lun();
}

/* Storage interface descriptor */
USBD_StorageTypeDef USBD_Storage_Interface_fops_HS = {
    STORAGE_Init_HS,
    STORAGE_GetCapacity_HS,
    STORAGE_IsReady_HS,
    STORAGE_IsWriteProtected_HS,
    STORAGE_Read_HS,
    STORAGE_Write_HS,
    STORAGE_GetMaxLun_HS,
    (int8_t *)STORAGE_Inquirydata_HS
};