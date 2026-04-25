#include "usbd_desc.h"
#include "usbd_conf.h"
#include "usbd_ctlreq.h"

/* ST VID / VCP PID — recognized natively on Linux/macOS, needs ST VCP driver on Windows */
#define SP_USB_VID 0x0483U
#define SP_USB_PID 0x5740U

#define SP_LANGID_STRING      0x0409U
#define SP_MANUFACTURER_STR   "STMicroelectronics"
#define SP_PRODUCT_STR        "STM32Pilot CAN Monitor"
#define SP_CONFIGURATION_STR  "STM32Pilot Config"
#define SP_INTERFACE_STR      "STM32Pilot CDC"

static uint8_t *sp_device_descriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *sp_langid_descriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *sp_manufacturer_descriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *sp_product_descriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *sp_serial_descriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *sp_config_descriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *sp_interface_descriptor(USBD_SpeedTypeDef speed, uint16_t *length);

USBD_DescriptorsTypeDef SP_Desc = {
    sp_device_descriptor,
    sp_langid_descriptor,
    sp_manufacturer_descriptor,
    sp_product_descriptor,
    sp_serial_descriptor,
    sp_config_descriptor,
    sp_interface_descriptor,
};

/* bDeviceClass=0x02 (CDC), 2 configurations listed in descriptor */
__ALIGN_BEGIN static uint8_t g_device_desc[USB_LEN_DEV_DESC] __ALIGN_END = {
    0x12, USB_DESC_TYPE_DEVICE,
    0x00, 0x02,             /* bcdUSB 2.00 */
    0x02,                   /* bDeviceClass: CDC */
    0x00,                   /* bDeviceSubClass */
    0x00,                   /* bDeviceProtocol */
    USB_MAX_EP0_SIZE,       /* bMaxPacketSize0 */
    LOBYTE(SP_USB_VID), HIBYTE(SP_USB_VID),
    LOBYTE(SP_USB_PID), HIBYTE(SP_USB_PID),
    0x00, 0x02,             /* bcdDevice 2.00 */
    USBD_IDX_MFC_STR,
    USBD_IDX_PRODUCT_STR,
    USBD_IDX_SERIAL_STR,
    0x01                    /* bNumConfigurations */
};

__ALIGN_BEGIN static uint8_t g_lang_id_desc[USB_LEN_LANGID_STR_DESC] __ALIGN_END = {
    USB_LEN_LANGID_STR_DESC, USB_DESC_TYPE_STRING,
    LOBYTE(SP_LANGID_STRING), HIBYTE(SP_LANGID_STRING)
};

__ALIGN_BEGIN static uint8_t g_str_desc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;

static uint8_t *sp_device_descriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
    (void)speed;
    *length = sizeof(g_device_desc);
    return g_device_desc;
}

static uint8_t *sp_langid_descriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
    (void)speed;
    *length = sizeof(g_lang_id_desc);
    return g_lang_id_desc;
}

static uint8_t *sp_manufacturer_descriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
    (void)speed;
    USBD_GetString((uint8_t *)SP_MANUFACTURER_STR, g_str_desc, length);
    return g_str_desc;
}

static uint8_t *sp_product_descriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
    (void)speed;
    USBD_GetString((uint8_t *)SP_PRODUCT_STR, g_str_desc, length);
    return g_str_desc;
}

static uint8_t *sp_serial_descriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
    (void)speed;
    USBD_GetString((uint8_t *)"000000000", g_str_desc, length);
    return g_str_desc;
}

static uint8_t *sp_config_descriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
    (void)speed;
    USBD_GetString((uint8_t *)SP_CONFIGURATION_STR, g_str_desc, length);
    return g_str_desc;
}

static uint8_t *sp_interface_descriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
    (void)speed;
    USBD_GetString((uint8_t *)SP_INTERFACE_STR, g_str_desc, length);
    return g_str_desc;
}
