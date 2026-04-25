#include "board_leds.h"
#include "can_handler.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_rcc_ex.h"

static void force_flash_vector_remap(void) {
#if defined(SYSCFG_CFGR1_MEM_MODE)
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_SYSCFG_REMAPMEMORY_FLASH();
#endif
}

static void SystemClock_Config(void) {
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    RCC_PeriphCLKInitTypeDef periph_clk = {0};
    RCC_CRSInitTypeDef crs = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    osc.HSI48State = RCC_HSI48_ON;
    osc.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        while (1) {}
    }

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1) != HAL_OK) {
        while (1) {}
    }

    periph_clk.PeriphClockSelection = RCC_PERIPHCLK_USB;
    periph_clk.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
    if (HAL_RCCEx_PeriphCLKConfig(&periph_clk) != HAL_OK) {
        while (1) {}
    }

    __HAL_RCC_CRS_CLK_ENABLE();
    crs.Prescaler = RCC_CRS_SYNC_DIV1;
    crs.Source = RCC_CRS_SYNC_SOURCE_USB;
    crs.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
    crs.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000U, 1000U);
    crs.ErrorLimitValue = 34U;
    crs.HSI48CalibrationValue = 32U;
    HAL_RCCEx_CRSConfig(&crs);
}

int main(void) {
    force_flash_vector_remap();
    HAL_Init();
    SystemClock_Config();
    board_leds_init();
    usb_device_init();
    can_handler_init();

    while (1) {
        uint8_t cmd = cdc_get_pending_cmd();
        if (cmd == 'a') {
            can_send_pilot_mode('a');
            cdc_transmit((uint8_t *)"AUTO\r\n", 6U);
        } else if (cmd == 's') {
            can_send_pilot_mode('s');
            cdc_transmit((uint8_t *)"STBY\r\n", 6U);
        } else if (cmd == 'w') {
            can_send_pilot_mode('w');
            cdc_transmit((uint8_t *)"WIND\r\n", 6U);
        } else if (cmd == 't') {
            can_send_pilot_mode('t');
            cdc_transmit((uint8_t *)"TRACK\r\n", 7U);
        } else if (cmd == '+') {
            can_send_keystroke('+');
            cdc_transmit((uint8_t *)"+1\r\n", 4U);
        } else if (cmd == '-') {
            can_send_keystroke('-');
            cdc_transmit((uint8_t *)"-1\r\n", 4U);
        } else if (cmd == ']') {
            can_send_keystroke(']');
            cdc_transmit((uint8_t *)"+10\r\n", 5U);
        } else if (cmd == '[') {
            can_send_keystroke('[');
            cdc_transmit((uint8_t *)"-10\r\n", 5U);
        }

        can_handler_poll();
        board_leds_poll(HAL_GetTick());
    }
}
