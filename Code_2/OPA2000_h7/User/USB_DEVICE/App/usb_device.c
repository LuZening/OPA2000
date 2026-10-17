/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_device.c
  * @version        : v1.0_Cube
  * @brief          : This file implements the USB Device (CDC + MSC Composite)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

/* USER CODE BEGIN Includes */
/* MODIFIED by USER for CDC+MSC composite device */
#include "usbd_cdc_msc_wrapper.h"
#include "usbd_msc_if.h"
#include "usbd_composite_desc.h"
#include "sdcard_storage.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDeviceHS;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */

/* Wrapper functions for composite descriptor (match USBD_DescriptorsTypeDef signature) */
static uint8_t* USBD_Composite_DeviceDescWrapper(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    return USBD_Composite_GetDeviceDescriptor(length);
}

static uint8_t* USBD_Composite_LangIDDescWrapper(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = 0;  /* Use default lang ID */
    return NULL;
}

static uint8_t* USBD_Composite_ManufacturerDescWrapper(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    return USBD_Composite_GetStringDescriptor(USBD_IDX_MFC_STR, length);
}

static uint8_t* USBD_Composite_ProductDescWrapper(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    return USBD_Composite_GetStringDescriptor(USBD_IDX_PRODUCT_STR, length);
}

static uint8_t* USBD_Composite_SerialDescWrapper(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    return USBD_Composite_GetStringDescriptor(USBD_IDX_SERIAL_STR, length);
}

static uint8_t* USBD_Composite_ConfigStrWrapper(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = 0;
    return NULL;
}

static uint8_t* USBD_Composite_InterfaceStrWrapper(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = 0;
    return NULL;
}

/* Composite device descriptor set */
static USBD_DescriptorsTypeDef Composite_Desc =
{
    USBD_Composite_DeviceDescWrapper,
    USBD_Composite_LangIDDescWrapper,
    USBD_Composite_ManufacturerDescWrapper,
    USBD_Composite_ProductDescWrapper,
    USBD_Composite_SerialDescWrapper,
    USBD_Composite_ConfigStrWrapper,
    USBD_Composite_InterfaceStrWrapper
};

/* USER CODE END 0 */

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Init USB device Library, add supported class and start the library
  * @retval None
  */
void MX_USB_DEVICE_Init(void)
{
  /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */

  /* USER CODE END USB_DEVICE_Init_PreTreatment */

  /* Init Device Library with composite descriptor (CDC + MSC) */
  if (USBD_Init(&hUsbDeviceHS, &Composite_Desc, DEVICE_HS) != USBD_OK)
  {
    Error_Handler();
  }

  /* Register CDC + MSC Composite class */
  if (USBD_RegisterClass(&hUsbDeviceHS, &USBD_CDC_MSC) != USBD_OK)
  {
    Error_Handler();
  }

  /* Register CDC Interface callback */
  if (USBD_CDC_RegisterInterface(&hUsbDeviceHS, &USBD_Interface_fops_HS) != USBD_OK)
  {
    Error_Handler();
  }

  /* MSC storage interface is registered in USBD_CDC_MSC_Init */

  if (USBD_Start(&hUsbDeviceHS) != USBD_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN USB_DEVICE_Init_PostTreatment */
  HAL_PWREx_EnableUSBVoltageDetector();

  /* Initialize SD card mutex before MSC can access the card */
  sdcard_storage_init_mutex();

  /* USER CODE END USB_DEVICE_Init_PostTreatment */
}

/**
  * @}
  */

/**
  * @}
  */

