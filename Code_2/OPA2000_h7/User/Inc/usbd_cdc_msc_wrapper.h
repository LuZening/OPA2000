/**
 * @file usbd_cdc_msc_wrapper.h
 * @brief USB CDC + MSC Composite Device Header
 */

#ifndef __USBD_CDC_MSC_WRAPPER_H
#define __USBD_CDC_MSC_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_def.h"
#include "usbd_cdc.h"
#include "usbd_msc.h"
#include "usbd_composite_desc.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief For Usb device.
  * @{
  */

/** @defgroup USBD_CDC_MSC_WRAPPER USBD_CDC_MSC_WRAPPER
  * @brief USB CDC + MSC Composite Device Wrapper
  * @{
  */

/** @defgroup USBD_CDC_MSC_WRAPPER_Exported_Defines USBD_CDC_MSC_WRAPPER_Exported_Defines
  * @brief Defines.
  * @{
  */

/* USER CODE BEGIN EXPORTED_DEFINES */

/* USER CODE END EXPORTED_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_MSC_WRAPPER_Exported_Types USBD_CDC_MSC_WRAPPER_Exported_Types
  * @brief Types.
  * @{
  */

/* CDC handle reference */
extern USBD_CDC_HandleTypeDef hcdc_comp;

/** Composite handle structure */
typedef struct
{
  USBD_CDC_HandleTypeDef hcdc;    /* CDC Class Data */
  void *hmsc;                      /* MSC Class Data (opaque) */
  uint8_t CDC_ClassId;            /* CDC Class Instance ID */
  uint8_t MSC_ClassId;            /* MSC Class Instance ID */
} USBD_CDC_MSC_HandleTypeDef;

/**
  * @}
  */

/** @defgroup USBD_CDC_MSC_WRAPPER_Exported_Macros USBD_CDC_MSC_WRAPPER_Exported_Macros
  * @brief Macros.
  * @{
  */

/* USER CODE BEGIN EXPORTED_MACROS */

/* USER CODE END EXPORTED_MACROS */

/**
  * @}
  */

/** @defgroup USBD_CDC_MSC_WRAPPER_Exported_Variables USBD_CDC_MSC_WRAPPER_Exported_Variables
  * @brief Public variables.
  * @{
  */

/** Composite class structure */
extern USBD_ClassTypeDef USBD_CDC_MSC;

/** CDC interface callback (from usbd_cdc_if.h) */
extern USBD_CDC_ItfTypeDef USBD_CDC_Interface_fops_HS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_MSC_WRAPPER_Exported_FunctionsPrototype USBD_CDC_MSC_WRAPPER_Exported_FunctionsPrototype
  * @brief Public functions declaration.
  * @{
  */

/* USER CODE BEGIN EXPORTED_FUNCTIONS */

/* USER CODE END EXPORTED_FUNCTIONS */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_MSC_WRAPPER_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/