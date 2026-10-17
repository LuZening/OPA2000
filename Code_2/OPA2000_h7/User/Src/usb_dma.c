/**
  ******************************************************************************
  * @file    usb_dma.c
  * @brief   USB CDC 的 DMA 收发缓冲定义（放在非缓存的 AXI SRAM 窗口）。
  *
  * 见 usb_dma.h 的说明：USB OTG HS 的内部 DMA 访问不到 DTCM，且缓冲必须
  * 处于非缓存区。本段为 NOLOAD，不需要初值（收发前都会先写入）。
  ******************************************************************************
  */
#include "usb_dma.h"

#define USB_DMA_SECTION  __attribute__((section(".usb_dma_buffer"), aligned(32)))

uint8_t usb_cdc_rx_dma_buffer[APP_RX_DATA_SIZE] USB_DMA_SECTION;
uint8_t usb_cdc_tx_dma_buffer[APP_TX_DATA_SIZE] USB_DMA_SECTION;
