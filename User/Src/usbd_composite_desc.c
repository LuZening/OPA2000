/**
 * @file usbd_composite_desc.c
 * @brief USB Composite Device Descriptor Implementation
 */

#include "usbd_composite_desc.h"
#include "usbd_core.h"
#include "usbd_def.h"
#include "usbd_conf.h"

/*============================================================================
 * Descriptor Templates
 *============================================================================*/

/* Device descriptor */
__ALIGN_BEGIN static uint8_t usbd_composite_device_desc[USB_LEN_DEV_DESC] __ALIGN_END = {
    0x12,                       /* bLength */
    USB_DESC_TYPE_DEVICE,       /* bDescriptorType */
    0x00, 0x02,                 /* bcdUSB: 2.00 */
    0xEF,                       /* bDeviceClass: Miscellaneous (Composite with IAD) */
    0x02,                       /* bDeviceSubClass: Common Class */
    0x01,                       /* bDeviceProtocol: Interface Association Descriptor */
    USB_MAX_EP0_SIZE,           /* bMaxPacketSize */
    LOBYTE(USB_DEVICE_VID),     /* idVendor */
    HIBYTE(USB_DEVICE_VID),     /* idVendor */
    LOBYTE(USB_DEVICE_PID),     /* idProduct */
    HIBYTE(USB_DEVICE_PID),     /* idProduct */
    0x00, 0x01,                 /* bcdDevice: 1.00 */
    USBD_IDX_MFC_STR,           /* Index of manufacturer string */
    USBD_IDX_PRODUCT_STR,       /* Index of product string */
    USBD_IDX_SERIAL_STR,        /* Index of serial number string */
    1                           /* bNumConfigurations */
};

/* Configuration descriptor for composite device (CDC + MSC) */
__ALIGN_BEGIN uint8_t usbd_composite_config_desc[COMPOSITE_CONFIG_DESC_SIZE] __ALIGN_END = {
    /* Configuration descriptor */
    0x09,                       /* bLength */
    USB_DESC_TYPE_CONFIGURATION,/* bDescriptorType */
    LOBYTE(COMPOSITE_CONFIG_DESC_SIZE),   /* wTotalLength */
    HIBYTE(COMPOSITE_CONFIG_DESC_SIZE),
    3,                         /* bNumInterfaces: CDC Control + CDC Data + MSC */
    0x01,                       /* bConfigurationValue */
    0x00,                       /* iConfiguration */
    0xC0,                       /* bmAttributes: Self-powered */
    0x32,                       /* bMaxPower: 100mA */

    /* Interface Association Descriptor (IAD) for CDC function */
    0x08,                       /* bLength */
    0x0B,                       /* bDescriptorType: IAD */
    COMPOSITE_CDC_CMD_IFACE_NUM,/* bFirstInterface */
    0x02,                       /* bInterfaceCount: 2 interfaces (Control + Data) */
    0x02,                       /* bFunctionClass: CDC Communication */
    0x02,                       /* bFunctionSubClass: Abstract Control Model */
    0x01,                       /* bFunctionProtocol: AT Commands (V.250) */
    0x00,                       /* iFunction */

    /* Interface 0: CDC Control (Communication class interface, per CDC spec §3.5.2) */
    0x09,                       /* bLength */
    USB_DESC_TYPE_INTERFACE,    /* bDescriptorType */
    COMPOSITE_CDC_CMD_IFACE_NUM, /* bInterfaceNumber */
    0x00,                       /* bAlternateSetting */
    0x01,                       /* bNumEndpoints */
    0x02,                       /* bInterfaceClass: CDC Communication */
    0x02,                       /* bInterfaceSubClass: Abstract Control Model */
    0x01,                       /* bInterfaceProtocol: AT Commands (V.250) */
    0x00,                       /* iInterface */

    /* CDC Command endpoint */
    0x07,                       /* bLength */
    USB_DESC_TYPE_ENDPOINT,     /* bDescriptorType */
    COMPOSITE_CDC_CMD_EP,        /* bEndpointAddress */
    0x03,                       /* bmAttributes: Interrupt */
    LOBYTE(COMPOSITE_CDC_CMD_EP_SIZE),    /* wMaxPacketSize */
    HIBYTE(COMPOSITE_CDC_CMD_EP_SIZE),
    0xFF,                       /* bInterval: 255 (polling interval) */

    /* CDC Header functional descriptor */
    0x05,                       /* bLength */
    0x24,                       /* bDescriptorType: CS_INTERFACE */
    0x00,                       /* bDescriptorSubtype: Header */
    0x10, 0x01,                 /* bcdCDC: 1.10 */

    /* CDC Call Management descriptor */
    0x05,                       /* bLength */
    0x24,                       /* bDescriptorType: CS_INTERFACE */
    0x01,                       /* bDescriptorSubtype: Call Management */
    0x01,                       /* bmCapabilities: call management over Data Interface */
    COMPOSITE_CDC_IFACE_NUM,     /* bDataInterface */

    /* CDC Abstract Control Management descriptor */
    0x04,                       /* bLength */
    0x24,                       /* bDescriptorType: CS_INTERFACE */
    0x02,                       /* bDescriptorSubtype: ACM */
    0x02,                       /* bmCapabilities: support Set_Line_Coding, Get_Line_Coding */

    /* CDC Union descriptor */
    0x05,                       /* bLength */
    0x24,                       /* bDescriptorType: CS_INTERFACE */
    0x06,                       /* bDescriptorSubtype: Union */
    COMPOSITE_CDC_CMD_IFACE_NUM, /* bControlInterface */
    COMPOSITE_CDC_IFACE_NUM,     /* bSubordinateInterface */

    /* Interface 1: CDC Data (Data class interface) */
    0x09,                       /* bLength */
    USB_DESC_TYPE_INTERFACE,    /* bDescriptorType */
    COMPOSITE_CDC_IFACE_NUM,     /* bInterfaceNumber */
    0x00,                       /* bAlternateSetting */
    0x02,                       /* bNumEndpoints */
    0x0A,                       /* bInterfaceClass: CDC Data */
    0x00,                       /* bInterfaceSubClass */
    0x00,                       /* bInterfaceProtocol */
    0x00,                       /* iInterface */

    /* CDC Data OUT endpoint */
    0x07,                       /* bLength */
    USB_DESC_TYPE_ENDPOINT,     /* bDescriptorType */
    COMPOSITE_CDC_DATA_OUT_EP,   /* bEndpointAddress */
    0x02,                       /* bmAttributes: Bulk */
    LOBYTE(COMPOSITE_CDC_DATA_EP_SIZE),   /* wMaxPacketSize */
    HIBYTE(COMPOSITE_CDC_DATA_EP_SIZE),
    0x00,                       /* bInterval */

    /* CDC Data IN endpoint */
    0x07,                       /* bLength */
    USB_DESC_TYPE_ENDPOINT,     /* bDescriptorType */
    COMPOSITE_CDC_DATA_IN_EP,    /* bEndpointAddress */
    0x02,                       /* bmAttributes: Bulk */
    LOBYTE(COMPOSITE_CDC_DATA_EP_SIZE),   /* wMaxPacketSize */
    HIBYTE(COMPOSITE_CDC_DATA_EP_SIZE),
    0x00,                       /* bInterval */

    /* Interface 2: MSC (Mass Storage) */
    0x09,                       /* bLength */
    USB_DESC_TYPE_INTERFACE,    /* bDescriptorType */
    COMPOSITE_MSC_IFACE_NUM,     /* bInterfaceNumber */
    0x00,                       /* bAlternateSetting */
    0x02,                       /* bNumEndpoints */
    0x08,                       /* bInterfaceClass: Mass Storage */
    0x06,                       /* bInterfaceSubClass: SCSI */
    0x50,                       /* bInterfaceProtocol: Bulk-only */
    0x00,                       /* iInterface */

    /* MSC OUT endpoint */
    0x07,                       /* bLength */
    USB_DESC_TYPE_ENDPOINT,     /* bDescriptorType */
    COMPOSITE_MSC_OUT_EP,        /* bEndpointAddress */
    0x02,                       /* bmAttributes: Bulk */
    LOBYTE(COMPOSITE_MSC_EP_SIZE),    /* wMaxPacketSize */
    HIBYTE(COMPOSITE_MSC_EP_SIZE),
    0x00,                       /* bInterval */

    /* MSC IN endpoint */
    0x07,                       /* bLength */
    USB_DESC_TYPE_ENDPOINT,     /* bDescriptorType */
    COMPOSITE_MSC_IN_EP,         /* bEndpointAddress */
    0x02,                       /* bmAttributes: Bulk */
    LOBYTE(COMPOSITE_MSC_EP_SIZE),    /* wMaxPacketSize */
    HIBYTE(COMPOSITE_MSC_EP_SIZE),
    0x00,                       /* bInterval */
};

/* String descriptors */
static uint8_t usbd_composite_string_lang_id[USB_LEN_LANGID_STR_DESC] = {
    USB_LEN_LANGID_STR_DESC,
    USB_DESC_TYPE_STRING,
    LOBYTE(0x0409),     /* LangID: English (US) */
    HIBYTE(0x0409)
};

static uint8_t usbd_composite_string_manufacturer[USBD_MAX_STR_DESC_SIZ];
static uint8_t usbd_composite_string_product[USBD_MAX_STR_DESC_SIZ];
static uint8_t usbd_composite_string_serial[USBD_MAX_STR_DESC_SIZ];

/*============================================================================
 * Private Functions
 *============================================================================*/

static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len);
static void GetSerialNum(void);

/*============================================================================
 * Public Functions
 *============================================================================*/

/**
 * @brief Get device descriptor
 */
uint8_t* USBD_Composite_GetDeviceDescriptor(uint16_t *length)
{
    *length = sizeof(usbd_composite_device_desc);
    return usbd_composite_device_desc;
}

/**
 * @brief Get configuration descriptor
 */
uint8_t* USBD_Composite_GetConfigDescriptor(uint8_t speed, uint16_t *length)
{
    (void)speed;
    *length = sizeof(usbd_composite_config_desc);
    return usbd_composite_config_desc;
}

/**
 * @brief Get string descriptor
 */
uint8_t* USBD_Composite_GetStringDescriptor(uint16_t index, uint16_t *length)
{
    switch (index) {
        case USBD_IDX_LANGID_STR:
            *length = sizeof(usbd_composite_string_lang_id);
            return usbd_composite_string_lang_id;

        case USBD_IDX_MFC_STR:
            USBD_GetString((uint8_t*)USB_DEVICE_FS_MANUFACTURER, usbd_composite_string_manufacturer, length);
            return usbd_composite_string_manufacturer;

        case USBD_IDX_PRODUCT_STR:
            USBD_GetString((uint8_t*)USB_DEVICE_FS_PRODUCT, usbd_composite_string_product, length);
            return usbd_composite_string_product;

        case USBD_IDX_SERIAL_STR:
            GetSerialNum();
            return usbd_composite_string_serial;

        default:
            *length = 0;
            return NULL;
    }
}

/*============================================================================
 * Private Functions
 *============================================================================*/

/**
 * @brief Convert integer to Unicode
 */
static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len)
{
    uint8_t idx = 0;

    for (idx = 0; idx < len; idx++) {
        if (((value >> 28)) < 0xA) {
            pbuf[2 * idx] = (value >> 28) + '0';
        } else {
            pbuf[2 * idx] = (value >> 28) + 'A' - 10;
        }
        value = value << 4;
        pbuf[2 * idx + 1] = 0;
    }
}

/**
 * @brief Generate serial number from MCU unique ID
 */
static void GetSerialNum(void)
{
    uint32_t deviceserial0 = HAL_GetUIDw0();
    uint32_t deviceserial1 = HAL_GetUIDw1();
    uint32_t deviceserial2 = HAL_GetUIDw2();

    uint8_t *pstr = usbd_composite_string_serial;

    pstr[0] = USBD_MAX_STR_DESC_SIZ;
    pstr[1] = USB_DESC_TYPE_STRING;

    IntToUnicode(deviceserial0, &pstr[2], 4);
    IntToUnicode(deviceserial1, &pstr[10], 4);
    IntToUnicode(deviceserial2, &pstr[18], 4);
}
