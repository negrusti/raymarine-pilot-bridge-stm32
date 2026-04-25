#pragma once

#include <stdint.h>

void board_leds_init(void);
void board_leds_poll(uint32_t now_ms);
void board_leds_pulse_rx(void);
void board_leds_pulse_tx(void);
