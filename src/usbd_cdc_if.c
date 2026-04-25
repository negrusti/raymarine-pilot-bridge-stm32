#include "usbd_cdc_if.h"

#include <string.h>

#include "usb_device.h"

#define CDC_RX_BUF_SIZE  64U
#define CDC_TX_BUF_SIZE  64U

#define CDC_QUEUE_LEN    16U   /* number of slots */
#define CDC_QUEUE_ITEM   40U   /* max bytes per message */

typedef struct {
    uint8_t data[CDC_QUEUE_ITEM];
    uint8_t len;
} TxItem;

static uint8_t  g_rx_buf[CDC_RX_BUF_SIZE];
static uint8_t  g_tx_buf[CDC_TX_BUF_SIZE];
static volatile uint8_t g_pending_cmd;

static TxItem   g_queue[CDC_QUEUE_LEN];
static uint8_t  g_q_head;   /* next slot to read (send) */
static uint8_t  g_q_tail;   /* next slot to write       */

static int8_t CDC_Init_FS(void) {
    USBD_CDC_SetRxBuffer(&g_usb_device_fs, g_rx_buf);
    USBD_CDC_ReceivePacket(&g_usb_device_fs);
    return USBD_OK;
}

static int8_t CDC_DeInit_FS(void) {
    return USBD_OK;
}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length) {
    (void)pbuf;
    (void)length;
    (void)cmd;
    return USBD_OK;
}

static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *len) {
    uint32_t i;
    for (i = 0U; i < *len; i++) {
        if (pbuf[i] == 'a' || pbuf[i] == 's' ||
            pbuf[i] == 'w' || pbuf[i] == 't' ||
            pbuf[i] == '+' || pbuf[i] == '-' ||
            pbuf[i] == ']' || pbuf[i] == '[') {
            g_pending_cmd = pbuf[i];
        }
    }
    USBD_CDC_SetRxBuffer(&g_usb_device_fs, g_rx_buf);
    USBD_CDC_ReceivePacket(&g_usb_device_fs);
    return USBD_OK;
}

USBD_CDC_ItfTypeDef g_cdc_fops = {
    CDC_Init_FS,
    CDC_DeInit_FS,
    CDC_Control_FS,
    CDC_Receive_FS,
};

uint8_t cdc_get_pending_cmd(void) {
    uint8_t cmd = g_pending_cmd;
    g_pending_cmd = 0U;
    return cmd;
}

/* Try to send the next queued message if TX is free. Call from main loop. */
void cdc_tx_poll(void) {
    if (g_q_head == g_q_tail) return;

    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)g_usb_device_fs.pClassData;
    if (hcdc == NULL || hcdc->TxState != 0U) return;

    TxItem *item = &g_queue[g_q_head];
    memcpy(g_tx_buf, item->data, item->len);
    USBD_CDC_SetTxBuffer(&g_usb_device_fs, g_tx_buf, item->len);
    if (USBD_CDC_TransmitPacket(&g_usb_device_fs) == USBD_OK) {
        g_q_head = (uint8_t)((g_q_head + 1U) % CDC_QUEUE_LEN);
    }
}

uint8_t cdc_transmit(const uint8_t *buf, uint16_t len) {
    if (len == 0U) return USBD_OK;
    if (len > CDC_QUEUE_ITEM) len = CDC_QUEUE_ITEM;

    uint8_t next_tail = (uint8_t)((g_q_tail + 1U) % CDC_QUEUE_LEN);
    if (next_tail == g_q_head) return USBD_BUSY;   /* queue full, drop */

    memcpy(g_queue[g_q_tail].data, buf, len);
    g_queue[g_q_tail].len = (uint8_t)len;
    g_q_tail = next_tail;

    cdc_tx_poll();   /* send immediately if TX is free */
    return USBD_OK;
}
