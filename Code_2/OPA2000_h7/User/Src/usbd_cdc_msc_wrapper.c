/**
 * @file usbd_cdc_msc_wrapper.c
 * @brief USB CDC + MSC Composite Device Implementation
 */

#include "usbd_cdc_msc_wrapper.h"
#include "usbd_cdc.h"
#include "usbd_msc.h"
#include "usbd_composite_desc.h"
#include "usbd_cdc_if.h"
#include "usbd_msc_if.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_MSC_WRAPPER
  * @{
  */

/** @defgroup USBD_CDC_MSC_WRAPPER_Private_TypesDefinitions USBD_CDC_MSC_WRAPPER_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_MSC_WRAPPER_Private_Defines USBD_CDC_MSC_WRAPPER_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_MSC_WRAPPER_Private_Macros USBD_CDC_MSC_WRAPPER_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_MSC_WRAPPER_Private_FunctionPrototypes USBD_CDC_MSC_WRAPPER_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static uint8_t USBD_CDC_MSC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_CDC_MSC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_CDC_MSC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_CDC_MSC_EP0_TxSent(USBD_HandleTypeDef *pdev);
static uint8_t USBD_CDC_MSC_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_CDC_MSC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_CDC_MSC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t *USBD_CDC_MSC_GetHSConfigDescriptor(uint16_t *length);
static uint8_t *USBD_CDC_MSC_GetFSConfigDescriptor(uint16_t *length);
static uint8_t *USBD_CDC_MSC_GetDeviceQualifierDescriptor(uint16_t *length);

#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
static uint8_t *USBD_CDC_MSC_GetUsrStrDescriptor(USBD_HandleTypeDef *pdev, uint8_t index, uint16_t *length);
#endif

/**
  * @}
  */

/** @defgroup USBD_CDC_MSC_WRAPPER_Private_Variables USBD_CDC_MSC_WRAPPER_Private_Variables
  * @brief Private variables.
  * @{
  */

/* CDC + MSC Composite Class */
USBD_ClassTypeDef USBD_CDC_MSC =
{
  USBD_CDC_MSC_Init,
  USBD_CDC_MSC_DeInit,
  USBD_CDC_MSC_Setup,
  USBD_CDC_MSC_EP0_TxSent,
  USBD_CDC_MSC_EP0_RxReady,
  USBD_CDC_MSC_DataIn,
  USBD_CDC_MSC_DataOut,
  NULL,  /* SOF - not used */
  NULL,  /* ISOUPIncomplete - not used */
  NULL,  /* ISOUPOutIncomplete - not used */
  USBD_CDC_MSC_GetHSConfigDescriptor,
  USBD_CDC_MSC_GetFSConfigDescriptor,
  USBD_CDC_MSC_GetDeviceQualifierDescriptor,
#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
  USBD_CDC_MSC_GetUsrStrDescriptor
#endif
};

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_MSC_WRAPPER_Exported_Variables USBD_CDC_MSC_WRAPPER_Exported_Variables
  * @brief Public variables.
  * @{
  */

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_MSC_WRAPPER_Private_Functions USBD_CDC_MSC_WRAPPER_Private_Functions
  * @brief Private functions.
  * @{
  */

/**
  * @brief  USBD_CDC_MSC_Init
  *         Initialize the CDC and MSC classes
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_CDC_MSC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  uint8_t res = USBD_OK;
  USBD_CDC_MSC_HandleTypeDef *hcomp;

  /* Allocate memory for composite handle */
  hcomp = (USBD_CDC_MSC_HandleTypeDef *)USBD_malloc(sizeof(USBD_CDC_MSC_HandleTypeDef));
  if (hcomp == NULL)
  {
    pdev->pClassData = NULL;
    return USBD_FAIL;
  }

  /* Assign class IDs: CDC gets slot 0, MSC gets slot 1 */
  hcomp->CDC_ClassId = 0;
  hcomp->MSC_ClassId = 1;

  /* Initialize CDC class - use CDC's own slot */
  pdev->classId = hcomp->CDC_ClassId;
  pdev->pClassDataCmsit[hcomp->CDC_ClassId] = NULL;  /* CDC.Init will allocate */
  res = USBD_CDC.Init(pdev, cfgidx);
  if (res != USBD_OK)
  {
    USBD_free(hcomp);
    pdev->pClassData = NULL;
    return res;
  }

  /* Initialize MSC class - use MSC's own slot */
  pdev->classId = hcomp->MSC_ClassId;
  pdev->pClassDataCmsit[hcomp->MSC_ClassId] = NULL;  /* MSC.Init will allocate */
  res = USBD_MSC.Init(pdev, cfgidx);
  if (res != USBD_OK)
  {
    pdev->classId = hcomp->CDC_ClassId;
    USBD_CDC.DeInit(pdev, cfgidx);
    USBD_free(hcomp);
    pdev->pClassData = NULL;
    return res;
  }

  /* Register MSC storage */
  USBD_MSC_RegisterStorage(pdev, &USBD_Storage_Interface_fops_HS);

  /* Store composite handle pointer for later use */
  pdev->pClassData = (void *)hcomp;

  /* Restore classId to 0 (CDC) as default */
  pdev->classId = hcomp->CDC_ClassId;

  return res;
}

/**
  * @brief  USBD_CDC_MSC_DeInit
  *         DeInitialize the CDC and MSC classes
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_CDC_MSC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  uint8_t res = USBD_OK;
  USBD_CDC_MSC_HandleTypeDef *hcomp = (USBD_CDC_MSC_HandleTypeDef *)pdev->pClassData;

  /* DeInitialize CDC using CDC's class slot */
  pdev->classId = hcomp->CDC_ClassId;
  res = USBD_CDC.DeInit(pdev, cfgidx);

  /* DeInitialize MSC using MSC's class slot */
  pdev->classId = hcomp->MSC_ClassId;
  res += USBD_MSC.DeInit(pdev, cfgidx);

  /* Free composite handle memory */
  if (res == USBD_OK)
  {
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
    pdev->pClassDataCmsit[hcomp->CDC_ClassId] = NULL;
    pdev->pClassDataCmsit[hcomp->MSC_ClassId] = NULL;
  }

  return res;
}

/**
  * @brief  USBD_CDC_MSC_Setup
  *         Handle the CDC and MSC specific requests
  * @param  pdev: device instance
  * @param  req: setup request
  * @retval status
  */
static uint8_t USBD_CDC_MSC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_CDC_MSC_HandleTypeDef *hcomp = (USBD_CDC_MSC_HandleTypeDef *)pdev->pClassData;

  /* Route request to appropriate class based on interface */
  switch (req->wIndex)
  {
    case COMPOSITE_CDC_IFACE_NUM:    /* CDC Data Interface */
    case COMPOSITE_CDC_CMD_IFACE_NUM: /* CDC Control Interface */
      pdev->classId = hcomp->CDC_ClassId;
      return USBD_CDC.Setup(pdev, req);

    case COMPOSITE_MSC_IFACE_NUM:    /* MSC Interface */
      pdev->classId = hcomp->MSC_ClassId;
      return USBD_MSC.Setup(pdev, req);

    default:
      return USBD_FAIL;
  }
}

/**
  * @brief  USBD_CDC_MSC_EP0_TxSent
  *         Handle EP0 Tx ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_CDC_MSC_EP0_TxSent(USBD_HandleTypeDef *pdev)
{
  return USBD_CDC.EP0_TxSent(pdev);
}

/**
  * @brief  USBD_CDC_MSC_EP0_RxReady
  *         Handle EP0 Rx ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_CDC_MSC_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  return USBD_CDC.EP0_RxReady(pdev);
}

/**
  * @brief  USBD_CDC_MSC_DataIn
  *         Handle data IN event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_CDC_MSC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_CDC_MSC_HandleTypeDef *hcomp = (USBD_CDC_MSC_HandleTypeDef *)pdev->pClassData;

  /* Route to CDC or MSC based on endpoint */
  if ((epnum & 0x80) == 0x80)  /* IN endpoint */
  {
    if (epnum == (COMPOSITE_CDC_DATA_IN_EP & 0x7F))
    {
      pdev->classId = hcomp->CDC_ClassId;
      return USBD_CDC.DataIn(pdev, epnum);
    }
    else if (epnum == (COMPOSITE_MSC_IN_EP & 0x7F))
    {
      pdev->classId = hcomp->MSC_ClassId;
      return USBD_MSC.DataIn(pdev, epnum);
    }
  }

  return USBD_OK;
}

/**
  * @brief  USBD_CDC_MSC_DataOut
  *         Handle data OUT event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_CDC_MSC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_CDC_MSC_HandleTypeDef *hcomp = (USBD_CDC_MSC_HandleTypeDef *)pdev->pClassData;

  /* Route to CDC or MSC based on endpoint */
  if (epnum == (COMPOSITE_CDC_DATA_OUT_EP & 0x7F))
  {
    pdev->classId = hcomp->CDC_ClassId;
    return USBD_CDC.DataOut(pdev, epnum);
  }
  else if (epnum == (COMPOSITE_MSC_OUT_EP & 0x7F))
  {
    pdev->classId = hcomp->MSC_ClassId;
    return USBD_MSC.DataOut(pdev, epnum);
  }

  return USBD_OK;
}

/**
  * @brief  USBD_CDC_MSC_GetHSConfigDescriptor
  *         Get HS configuration descriptor
  * @param  length: pointer to descriptor length
  * @retval pointer to descriptor
  */
static uint8_t *USBD_CDC_MSC_GetHSConfigDescriptor(uint16_t *length)
{
  *length = sizeof(usbd_composite_config_desc);
  return usbd_composite_config_desc;
}

/**
  * @brief  USBD_CDC_MSC_GetFSConfigDescriptor
  *         Get FS configuration descriptor
  * @param  length: pointer to descriptor length
  * @retval pointer to descriptor
  */
static uint8_t *USBD_CDC_MSC_GetFSConfigDescriptor(uint16_t *length)
{
  *length = sizeof(usbd_composite_config_desc);
  return usbd_composite_config_desc;
}

/**
  * @brief  USBD_CDC_MSC_GetDeviceQualifierDescriptor
  *         Get device qualifier descriptor
  * @param  length: pointer to descriptor length
  * @retval pointer to descriptor
  */
static uint8_t *USBD_CDC_MSC_GetDeviceQualifierDescriptor(uint16_t *length)
{
  /* Device qualifier not implemented for composite device */
  *length = 0;
  return NULL;
}

#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
/**
  * @brief  USBD_CDC_MSC_GetUsrStrDescriptor
  *         Get user string descriptor
  * @param  pdev: device instance
  * @param  index: string index
  * @param  length: pointer to descriptor length
  * @retval pointer to descriptor
  */
static uint8_t *USBD_CDC_MSC_GetUsrStrDescriptor(USBD_HandleTypeDef *pdev, uint8_t index, uint16_t *length)
{
  return USBD_CDC.GetUsrStrDescriptor(pdev, index, length);
}
#endif

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/