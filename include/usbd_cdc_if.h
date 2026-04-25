#pragma once

#include "usbd_cdc.h"
#include <stdint.h>

extern USBD_CDC_ItfTypeDef g_cdc_fops;

uint8_t cdc_get_pending_cmd(void);
uint8_t cdc_transmit(const uint8_t *buf, uint16_t len);
