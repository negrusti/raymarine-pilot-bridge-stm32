#include "usbd_conf.h"

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_pcd.h"
#include "stm32f0xx_hal_pcd_ex.h"
#include "usbd_core.h"
#include "usbd_cdc.h"

extern PCD_HandleTypeDef g_hpcd_usb_fs;

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev) {
    g_hpcd_usb_fs.Instance = USB;
    g_hpcd_usb_fs.Init.dev_endpoints = 8;
    g_hpcd_usb_fs.Init.ep0_mps = PCD_EP0MPS_64;
    g_hpcd_usb_fs.Init.speed = PCD_SPEED_FULL;
    g_hpcd_usb_fs.Init.phy_itface = PCD_PHY_EMBEDDED;
    g_hpcd_usb_fs.Init.low_power_enable = DISABLE;
    g_hpcd_usb_fs.Init.lpm_enable = DISABLE;
    g_hpcd_usb_fs.Init.battery_charging_enable = DISABLE;

    g_hpcd_usb_fs.pData = pdev;
    pdev->pData = &g_hpcd_usb_fs;

    if (HAL_PCD_Init(&g_hpcd_usb_fs) != HAL_OK) {
        return USBD_FAIL;
    }

    /* PMA layout:
       EP0_OUT  @ 0x18 (64 bytes)
       EP0_IN   @ 0x58 (64 bytes)
       CDC_IN   @ 0xC0 (64 bytes) — bulk data device→host
       CDC_OUT  @ 0x100 (64 bytes) — bulk data host→device
       CDC_CMD  @ 0x140 (8 bytes)  — interrupt notifications device→host */
    HAL_PCDEx_PMAConfig(&g_hpcd_usb_fs, 0x00U,    PCD_SNG_BUF, 0x18U);
    HAL_PCDEx_PMAConfig(&g_hpcd_usb_fs, 0x80U,    PCD_SNG_BUF, 0x58U);
    HAL_PCDEx_PMAConfig(&g_hpcd_usb_fs, CDC_IN_EP,  PCD_SNG_BUF, 0xC0U);
    HAL_PCDEx_PMAConfig(&g_hpcd_usb_fs, CDC_OUT_EP, PCD_SNG_BUF, 0x100U);
    HAL_PCDEx_PMAConfig(&g_hpcd_usb_fs, CDC_CMD_EP, PCD_SNG_BUF, 0x140U);

    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev) {
    return (HAL_PCD_DeInit((PCD_HandleTypeDef *)pdev->pData) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev) {
    return (HAL_PCD_Start((PCD_HandleTypeDef *)pdev->pData) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev) {
    return (HAL_PCD_Stop((PCD_HandleTypeDef *)pdev->pData) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps) {
    return (HAL_PCD_EP_Open((PCD_HandleTypeDef *)pdev->pData, ep_addr, ep_mps, ep_type) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
    return (HAL_PCD_EP_Close((PCD_HandleTypeDef *)pdev->pData, ep_addr) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
    return (HAL_PCD_EP_Flush((PCD_HandleTypeDef *)pdev->pData, ep_addr) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
    return (HAL_PCD_EP_SetStall((PCD_HandleTypeDef *)pdev->pData, ep_addr) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
    return (HAL_PCD_EP_ClrStall((PCD_HandleTypeDef *)pdev->pData, ep_addr) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
    PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef *)pdev->pData;
    if ((ep_addr & 0x80U) != 0U) {
        return hpcd->IN_ep[ep_addr & 0x7FU].is_stall;
    }
    return hpcd->OUT_ep[ep_addr & 0x7FU].is_stall;
}

USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr) {
    return (HAL_PCD_SetAddress((PCD_HandleTypeDef *)pdev->pData, dev_addr) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint16_t size) {
    return (HAL_PCD_EP_Transmit((PCD_HandleTypeDef *)pdev->pData, ep_addr, pbuf, size) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint16_t size) {
    return (HAL_PCD_EP_Receive((PCD_HandleTypeDef *)pdev->pData, ep_addr, pbuf, size) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
    return HAL_PCD_EP_GetRxCount((PCD_HandleTypeDef *)pdev->pData, ep_addr);
}

void USBD_LL_Delay(uint32_t delay) {
    HAL_Delay(delay);
}

void *USBD_static_malloc(uint32_t size) {
    static uint32_t mem[(sizeof(USBD_CDC_HandleTypeDef) / 4U) + 1U];
    (void)size;
    return mem;
}

void USBD_static_free(void *p) {
    (void)p;
}

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd) {
    USBD_LL_SetupStage((USBD_HandleTypeDef *)hpcd->pData, (uint8_t *)hpcd->Setup);
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
    USBD_LL_DataOutStage((USBD_HandleTypeDef *)hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
    USBD_LL_DataInStage((USBD_HandleTypeDef *)hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd) {
    USBD_LL_SOF((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd) {
    USBD_LL_SetSpeed((USBD_HandleTypeDef *)hpcd->pData, USBD_SPEED_FULL);
    USBD_LL_Reset((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd) {
    USBD_LL_Suspend((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd) {
    USBD_LL_Resume((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
    USBD_LL_IsoOUTIncomplete((USBD_HandleTypeDef *)hpcd->pData, epnum);
}

void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
    USBD_LL_IsoINIncomplete((USBD_HandleTypeDef *)hpcd->pData, epnum);
}

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd) {
    USBD_LL_DevConnected((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd) {
    USBD_LL_DevDisconnected((USBD_HandleTypeDef *)hpcd->pData);
}
