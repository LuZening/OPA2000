/**
 * @file usbd_composite_desc.h
 * @brief USB Composite Device Descriptor (CDC + MSC)
 */

#ifndef USBD_COMPOSITE_DESC_H
#define USBD_COMPOSITE_DESC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_desc.h"

/*============================================================================
 * Device Descriptors
 *============================================================================*/

#define USB_DEVICE_VID                0x8888
#define USB_DEVICE_PID                0x5052
#define USB_DEVICE_FS_PRODUCT         "OPA2000 CDC+MSC"
#define USB_DEVICE_FS_MANUFACTURER    "BI4WOP"
#define USB_DEVICE_FS_SERIAL          "OPA2000V1.0"

/*============================================================================
 * Configuration Descriptor
 *============================================================================*/

/* CDC Configuration Descriptor (IAD 8 + Control IF 9 + CMD EP 7 + Header 5 + Call Mgmt 5 + ACM 4 + Union 5 + Data IF 9 + Data OUT EP 7 + Data IN EP 7) */
#define CDC_CONFIG_DESC_SIZE          67

/* MSC Configuration Descriptor (Interface 9 + EP OUT 7 + EP IN 7) */
#define MSC_CONFIG_DESC_SIZE          23

/* Total configuration descriptor size: header 9 + CDC 67 + MSC 23 */
#define COMPOSITE_CONFIG_DESC_SIZE    (CDC_CONFIG_DESC_SIZE + MSC_CONFIG_DESC_SIZE + 9)

/* Configuration descriptor array */
extern uint8_t usbd_composite_config_desc[COMPOSITE_CONFIG_DESC_SIZE];

/* Interface numbers */
/* Per USB CDC spec §3.5.2: Communication (Control) IF must have lower number */
#define COMPOSITE_CDC_CMD_IFACE_NUM    0
#define COMPOSITE_CDC_IFACE_NUM        1
#define COMPOSITE_MSC_IFACE_NUM        2

/* Endpoint addresses */
#define COMPOSITE_CDC_CMD_EP           0x81
#define COMPOSITE_CDC_DATA_OUT_EP      0x01
#define COMPOSITE_CDC_DATA_IN_EP       0x82
#define COMPOSITE_MSC_OUT_EP           0x03
#define COMPOSITE_MSC_IN_EP            0x83

/* Endpoint sizes */
#define COMPOSITE_CDC_CMD_EP_SIZE      8
#define COMPOSITE_CDC_DATA_EP_SIZE     64      /* Full-speed */
#define COMPOSITE_MSC_EP_SIZE          64      /* Full-speed */

/*============================================================================
 * Function Prototypes
 *============================================================================*/

/**
 * @brief Get composite device descriptor
 * @param length Pointer to length
 * @return Pointer to descriptor
 */
uint8_t* USBD_Composite_GetDeviceDescriptor(uint16_t *length);

/**
 * @brief Get composite configuration descriptor
 * @param speed Current speed
 * @param length Pointer to length
 * @return Pointer to descriptor
 */
uint8_t* USBD_Composite_GetConfigDescriptor(uint8_t speed, uint16_t *length);

/**
 * @brief Get string descriptor
 * @param index Index
 * @param length Pointer to length
 * @return Pointer to descriptor
 */
uint8_t* USBD_Composite_GetStringDescriptor(uint16_t index, uint16_t *length);

#ifdef __cplusplus
}
#endif

#endif /* USBD_COMPOSITE_DESC_H */
