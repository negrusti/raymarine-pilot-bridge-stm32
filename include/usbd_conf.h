#pragma once

#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define USBD_MAX_NUM_INTERFACES     2U
#define USBD_MAX_NUM_CONFIGURATION  1U
#define USBD_MAX_STR_DESC_SIZ       0x100U
#define USBD_SUPPORT_USER_STRING_DESC 0U
#define USBD_SELF_POWERED           0U
#define USBD_DEBUG_LEVEL            0U
#define DEVICE_FS                   0U

#define USBD_memset memset
#define USBD_memcpy memcpy

void *USBD_static_malloc(uint32_t size);
void  USBD_static_free(void *p);

#define USBD_malloc  USBD_static_malloc
#define USBD_free    USBD_static_free

#define USBD_UsrLog(...) do {} while (0)
#define USBD_ErrLog(...) do {} while (0)
#define USBD_DbgLog(...) do {} while (0)
