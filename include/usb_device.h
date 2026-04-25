#pragma once

#include "usbd_def.h"

extern USBD_HandleTypeDef g_usb_device_fs;

void usb_device_init(void);
