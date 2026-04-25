#include "usb_device.h"

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_pcd.h"
#include "usbd_core.h"
#include "usbd_cdc_if.h"
#include "usbd_desc.h"

USBD_HandleTypeDef g_usb_device_fs;
PCD_HandleTypeDef  g_hpcd_usb_fs;

void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd) {
    if (hpcd->Instance == USB) {
        __HAL_RCC_USB_CLK_ENABLE();
        HAL_NVIC_SetPriority(USB_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USB_IRQn);
    }
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd) {
    if (hpcd->Instance == USB) {
        HAL_NVIC_DisableIRQ(USB_IRQn);
        __HAL_RCC_USB_CLK_DISABLE();
    }
}

void USB_IRQHandler(void) {
    HAL_PCD_IRQHandler(&g_hpcd_usb_fs);
}

void usb_device_init(void) {
    USBD_Init(&g_usb_device_fs, &SP_Desc, DEVICE_FS);
    USBD_RegisterClass(&g_usb_device_fs, USBD_CDC_CLASS);
    USBD_CDC_RegisterInterface(&g_usb_device_fs, &g_cdc_fops);
    USBD_Start(&g_usb_device_fs);
}
