#include "board_leds.h"

#include "stm32f0xx_hal.h"

#define LED_RX_PORT GPIOA
#define LED_RX_PIN  GPIO_PIN_0
#define LED_TX_PORT GPIOA
#define LED_TX_PIN  GPIO_PIN_1

#define LED_PULSE_MS   60U
#define LED_OFF_GAP_MS 60U
#define LED_RX_ON      GPIO_PIN_RESET
#define LED_TX_ON      GPIO_PIN_RESET

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState on_state;
    uint8_t blink_requested;
    uint32_t on_until_ms;
    uint32_t blocked_until_ms;
} led_t;

static led_t g_rx_led;
static led_t g_tx_led;

static GPIO_PinState led_off_state(GPIO_PinState on_state) {
    return (on_state == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
}

static uint8_t time_passed(uint32_t now, uint32_t target) {
    return (int32_t)(now - target) >= 0;
}

static void led_write(const led_t *led, uint8_t active) {
    HAL_GPIO_WritePin(led->port, led->pin,
                      active ? led->on_state : led_off_state(led->on_state));
}

static void led_update(led_t *led, uint32_t now_ms) {
    if (led->blink_requested) {
        led->blink_requested = 0U;
        if (time_passed(now_ms, led->blocked_until_ms)) {
            led->on_until_ms = now_ms + LED_PULSE_MS;
            led->blocked_until_ms = led->on_until_ms + LED_OFF_GAP_MS;
        }
    }
    led_write(led, !time_passed(now_ms, led->on_until_ms));
}

static void led_init_one(led_t *led, GPIO_TypeDef *port, uint16_t pin, GPIO_PinState on_state) {
    led->port = port;
    led->pin = pin;
    led->on_state = on_state;
    led->blink_requested = 0U;
    led->on_until_ms = 0U;
    led->blocked_until_ms = 0U;
    led_write(led, 0U);
}

void board_leds_init(void) {
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* AliExpress CANable clone: PA0=RX (red), PA1=TX (blue), both active-low */
    gpio.Pin = LED_RX_PIN | LED_TX_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);

    led_init_one(&g_rx_led, LED_RX_PORT, LED_RX_PIN, LED_RX_ON);
    led_init_one(&g_tx_led, LED_TX_PORT, LED_TX_PIN, LED_TX_ON);
}

void board_leds_poll(uint32_t now_ms) {
    led_update(&g_rx_led, now_ms);
    led_update(&g_tx_led, now_ms);
}

void board_leds_pulse_rx(void) {
    g_rx_led.blink_requested = 1U;
}

void board_leds_pulse_tx(void) {
    g_tx_led.blink_requested = 1U;
}
