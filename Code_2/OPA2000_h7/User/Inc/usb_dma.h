/**
  ******************************************************************************
  * @file    usb_dma.h
  * @brief   USB CDC 的 DMA 收发缓冲声明。
  *
  * USB OTG HS 位于 D2 域，其内部 DMA 直接读写内存：访问不到 DTCM
  * (0x20000000)，也够不到 D3。同时 USB 写内存后，若 CPU 从可缓存区读同一块
  * 内存会拿到陈旧数据。因此 USB CDC 的收发缓冲统一放在 app.ld 的
  * .usb_dma_buffer（被 .dma_no_cache 收拢到 AXI SRAM 0x24040000 起、
  * 由 MPU 标为不可缓存的窗口里）。定义见 User/Src/usb_dma.c。
  ******************************************************************************
  */
#ifndef USB_DMA_H
#define USB_DMA_H

#include <stdint.h>

#include "usbd_cdc_if.h"   /* APP_RX_DATA_SIZE / APP_TX_DATA_SIZE */

/* 非缓存 AXI SRAM 中的 USB CDC 收发缓冲（替代模板里位于 DTCM 的
 * UserRxBufferHS / UserTxBufferHS）。 */
extern uint8_t usb_cdc_rx_dma_buffer[APP_RX_DATA_SIZE];
extern uint8_t usb_cdc_tx_dma_buffer[APP_TX_DATA_SIZE];

#endif /* USB_DMA_H */
