#pragma once

#include <stdint.h>

void can_handler_init(void);
void can_handler_poll(void);
void can_send_pilot_mode(uint8_t mode);  /* mode: 'a'=auto, 's'=standby */
void can_send_keystroke(uint8_t key);   /* key: '+'/'-'=±1°, ']'/'['=±10° */
