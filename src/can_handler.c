#include "can_handler.h"

#include <string.h>

#include "board_leds.h"
#include "usbd_cdc_if.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_can.h"

#define CAN_RX_PIN      GPIO_PIN_8
#define CAN_TX_PIN      GPIO_PIN_9
#define CAN_GPIO_PORT   GPIOB
#define CAN_PHY_PIN     GPIO_PIN_13
#define CAN_PHY_PORT    GPIOC

/* 250 kbps @ 48 MHz: prescaler=12, BS1=13TQ, BS2=2TQ */
#define CAN_PRESCALER   12U
#define CAN_BS1         CAN_BS1_13TQ
#define CAN_BS2         CAN_BS2_2TQ

/* NMEA2000 PGN 126720, priority=3, DP=1, PF=0xEF, dst=115 (SeaTalk-NG), src=1 */
#define PILOT_CMD_ID    0x0DEF7301UL

/* Same PGN, priority=7, for keystroke commands */
#define KEYSTROKE_CMD_ID 0x1DEF7301UL

/* PGN 65379 (0xFF63) autopilot status broadcast: priority=7, DP=0, PF=0xFF, PS=0x63, src=115 */
#define PGN65379_ID      0x1CFF6373UL

/* PGN 126720 status from SeaTalk-NG (src=115). Mask ignores destination byte. */
#define STATUS_ID_MASK  0x1FFF00FFUL
#define STATUS_ID_VAL   0x1DEF0073UL

/* PGN 126208 (Group Function) — Axiom sends locked heading commands here. */
#define GRP_FUNC_MASK   0x01FF0000UL
#define GRP_FUNC_VAL    0x01ED0000UL

/* SeaTalk1 datagram 0x84: Compass heading / Autopilot course / Rudder / Mode
   Layout: 84 U6 VW XY 0Z 0M RR SS TT   (9 bytes)
   Offsets within the reassembled PGN 126720 payload (ST1 raw starts at byte 4): */
#define PL_ST1_CMD  4U  /* 0x84 */
#define PL_ST1_ATTR 5U  /* U<<4 | 6  — U = compass heading bits 1:0 + turning dir */
#define PL_VW       6U  /* VW: compass heading mid bits + AP course high bits */
#define PL_XY       7U  /* XY: AP course low byte */
#define PL_Z_MODE   8U  /* 0Z: pilot mode in low nibble Z */
#define PL_M_ALARM  9U  /* 0M: alarms in low nibble M */
#define PL_RR      10U  /* RR: rudder angle (signed, degrees) */
/* PL_SS 11, PL_TT 12 — display flags, not needed here */

#define ST1_CMD_AP  0x84U

/* Fast-packet reassembly */
#define FP_BUF_MAX 32U
typedef struct {
    uint8_t buf[FP_BUF_MAX];
    uint8_t total;
    uint8_t got;
    uint8_t next_frame;
    uint8_t seq;
    uint8_t src;  /* source address that started this reassembly */
    uint8_t dst;  /* destination address from frame 0 */
} FpState;

static FpState g_fp;      /* autopilot status (src=115) */
static FpState g_fp_mfd;  /* MFD group function commands */

/* Keystroke send queue — filled during RX, drained in main loop */
#define KS_QUEUE_LEN 8U
static uint8_t g_ks_queue[KS_QUEUE_LEN];
static uint8_t g_ks_head;
static uint8_t g_ks_tail;

static void ks_enqueue(uint8_t key) {
    uint8_t next = (uint8_t)((g_ks_tail + 1U) % KS_QUEUE_LEN);
    if (next != g_ks_head) {
        g_ks_queue[g_ks_tail] = key;
        g_ks_tail = next;
    }
}

/* Last known state — 0xFF/0xFFFF forces output on first packet */
static uint8_t  g_last_z = 0xFFU;
static uint8_t  g_last_m = 0xFFU;
static uint8_t  g_print_next_crs;
static uint16_t g_locked_crs = 0xFFFFU;  /* AP locked course in degrees, 0xFFFF=unknown */

static CAN_HandleTypeDef g_hcan;
static uint8_t g_fast_seq;

/* ---- helpers ------------------------------------------------------------ */

static void fmt3(char *buf, uint16_t v) {
    buf[0] = (char)('0' + v / 100U);
    buf[1] = (char)('0' + (v % 100U) / 10U);
    buf[2] = (char)('0' + v % 10U);
}

static uint8_t append_str(char *buf, uint8_t pos, const char *s) {
    while (*s) buf[pos++] = *s++;
    return pos;
}

/* ---- GPIO / MSP --------------------------------------------------------- */

static void can_gpio_init(void) {
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(CAN_PHY_PORT, CAN_PHY_PIN, GPIO_PIN_RESET);
    gpio.Pin = CAN_PHY_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CAN_PHY_PORT, &gpio);

    gpio.Pin = CAN_RX_PIN | CAN_TX_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF4_CAN;
    HAL_GPIO_Init(CAN_GPIO_PORT, &gpio);
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == CAN) {
        __HAL_RCC_CAN1_CLK_ENABLE();
        can_gpio_init();
    }
}

/* ---- Init --------------------------------------------------------------- */

void can_handler_init(void) {
    CAN_FilterTypeDef filter = {0};

    g_fast_seq = 0U;

    memset(&g_hcan, 0, sizeof(g_hcan));
    g_hcan.Instance = CAN;
    g_hcan.Init.Prescaler = CAN_PRESCALER;
    g_hcan.Init.Mode = CAN_MODE_NORMAL;
    g_hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    g_hcan.Init.TimeSeg1 = CAN_BS1;
    g_hcan.Init.TimeSeg2 = CAN_BS2;
    g_hcan.Init.TimeTriggeredMode = DISABLE;
    g_hcan.Init.AutoBusOff = DISABLE;
    g_hcan.Init.AutoWakeUp = DISABLE;
    g_hcan.Init.AutoRetransmission = ENABLE;
    g_hcan.Init.ReceiveFifoLocked = DISABLE;
    g_hcan.Init.TransmitFifoPriority = ENABLE;

    if (HAL_CAN_Init(&g_hcan) != HAL_OK) return;

    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0U; filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U; filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(&g_hcan, &filter) != HAL_OK) return;

    HAL_CAN_Start(&g_hcan);
}

/* ---- TX: pilot mode commands -------------------------------------------- */

static void send_raw_frame(uint32_t ext_id, const uint8_t *data, uint8_t dlc) {
    CAN_TxHeaderTypeDef hdr = {0};
    uint32_t mailbox;
    hdr.IDE = CAN_ID_EXT;
    hdr.ExtId = ext_id;
    hdr.RTR = CAN_RTR_DATA;
    hdr.DLC = dlc;
    hdr.TransmitGlobalTime = DISABLE;
    uint32_t tick = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&g_hcan) == 0U) {
        if (HAL_GetTick() - tick > 10U) return;
    }
    HAL_CAN_AddTxMessage(&g_hcan, &hdr, (uint8_t *)data, &mailbox);
}

void can_send_pilot_mode(uint8_t mode) {
    uint8_t seq = (g_fast_seq++ & 0x07U) << 5;
    uint8_t f0[8] = {seq,       0x10, 0x3B, 0x9F, 0xF0, 0x81, 0x86, 0x21};
    uint8_t f1[8] = {seq | 1U, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t f2[8] = {seq | 2U, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    switch (mode) {
        case 'a': f1[1]=0x01U; f1[2]=0xFEU; break;
        case 's': f1[1]=0x02U; f1[2]=0xFDU; break;
        case 'w': f1[1]=0x23U; f1[2]=0xDCU; break;
        case 't': f1[1]=0x03U; f1[2]=0xFCU; f1[3]=0x3CU; f1[4]=0x42U; break;
        default: return;
    }

    send_raw_frame(PILOT_CMD_ID, f0, 8U);
    send_raw_frame(PILOT_CMD_ID, f1, 8U);
    send_raw_frame(PILOT_CMD_ID, f2, 8U);

    board_leds_pulse_tx();
}

void can_send_keystroke(uint8_t key) {
    uint8_t code, inv;
    switch (key) {
        case '+': code = 0x07U; inv = 0xF8U; break;
        case '-': code = 0x05U; inv = 0xFAU; break;
        case ']': code = 0x08U; inv = 0xF7U; break;
        case '[': code = 0x06U; inv = 0xF9U; break;
        default: return;
    }

    uint8_t seq = (g_fast_seq++ & 0x07U) << 5;

    uint8_t f0[8] = {seq,       0x16, 0x3B, 0x9F, 0xF0, 0x81, 0x86, 0x21};
    uint8_t f1[8] = {seq | 1U, code, inv,  0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t f2[8] = {seq | 2U, 0xC1, 0xC2, 0xCD, 0x66, 0x80, 0xD3, 0x42};
    uint8_t f3[8] = {seq | 3U, 0xB1, 0xC8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    send_raw_frame(KEYSTROKE_CMD_ID, f0, 8U);
    send_raw_frame(KEYSTROKE_CMD_ID, f1, 8U);
    send_raw_frame(KEYSTROKE_CMD_ID, f2, 8U);
    send_raw_frame(KEYSTROKE_CMD_ID, f3, 8U);

    g_print_next_crs = 1U;
    board_leds_pulse_tx();
}

/* ---- RX: fast-packet reassembly ----------------------------------------- */

static void fp_feed(FpState *fp, const uint8_t *data) {
    uint8_t idx = data[0] & 0x1FU;
    uint8_t seq = data[0] >> 5;

    if (idx == 0U) {
        uint8_t total = data[1];
        if (total == 0U || total > FP_BUF_MAX) { fp->total = 0U; return; }
        uint8_t copy = (total < 6U) ? total : 6U;
        fp->total = total;
        fp->got = copy;
        fp->seq = seq;
        fp->next_frame = 1U;
        memcpy(fp->buf, &data[2], copy);
    } else {
        if (fp->total == 0U || idx != fp->next_frame || seq != fp->seq) {
            fp->total = 0U;
            return;
        }
        uint8_t remaining = fp->total - fp->got;
        uint8_t copy = (remaining < 7U) ? remaining : 7U;
        memcpy(fp->buf + fp->got, &data[1], copy);
        fp->got += copy;
        fp->next_frame++;
    }
}

/* ---- RX: Group Function (PGN 126208) locked heading decode --------------- */

static void process_grp_func(uint8_t src, uint8_t dst) {
    const uint8_t *p = g_fp_mfd.buf;
    if (g_fp_mfd.total < 17U) return;
    if (p[0] != 0x01U) return;                                    /* Command only */
    if (p[1] != 0x50U || p[2] != 0xFFU || p[3] != 0x00U) return; /* PGN 65360 */

    /* Magnetic heading: payload[15..16] LE uint16, units = 1/10000 rad → degrees */
    uint16_t raw = (uint16_t)p[15] | ((uint16_t)p[16] << 8);
    uint16_t target = (uint16_t)(((uint32_t)raw * 180U + 15708U) / 31416U) % 360U;

    /* Log the commanded heading and current locked course for diagnostics */
    {
        char line[32];
        uint8_t pos = 0;
        line[pos++] = '^'; line[pos++] = ' '; line[pos++] = 'C';
        fmt3(line + pos, target); pos += 3;
        line[pos++] = ' '; line[pos++] = 'L';
        if (g_locked_crs == 0xFFFFU) {
            line[pos++] = '?'; line[pos++] = '?'; line[pos++] = '?';
        } else {
            fmt3(line + pos, g_locked_crs);
        }
        pos += 3;
        line[pos++] = ' '; line[pos++] = 's';
        fmt3(line + pos, src); pos += 3;
        line[pos++] = ' '; line[pos++] = 'd';
        fmt3(line + pos, dst); pos += 3;
        line[pos++] = '\r'; line[pos++] = '\n';
        cdc_transmit((uint8_t *)line, pos);
    }

    /* Translate to relative keystrokes using current AP locked course */
    if (g_locked_crs == 0xFFFFU) return;

    int16_t delta = (int16_t)target - (int16_t)g_locked_crs;
    if (delta > 180)  delta -= 360;
    if (delta < -180) delta += 360;
    if (delta == 0) return;
    /* Sanity guard: ignore implausibly large jumps */
    if (delta > 30 || delta < -30) return;

    uint8_t key10 = (delta > 0) ? ']' : '[';
    uint8_t key1  = (delta > 0) ? '+' : '-';
    if (delta < 0) delta = -delta;

    while (delta >= 10) { ks_enqueue(key10); delta -= 10; }
    while (delta > 0)   { ks_enqueue(key1);  delta--;     }
}

/* ---- RX: ST1 0x84 status decode ----------------------------------------- */

static void process_pilot_status(void) {
    const uint8_t *p = g_fp.buf;

    if (g_fp.total < (PL_RR + 1U)) return;
    if (p[PL_ST1_CMD] != ST1_CMD_AP) return;

    /* Pilot mode: Z = low nibble of PL_Z_MODE byte.
       bit 1 (0x02): Auto engaged (clear = Standby)
       bit 2 (0x04): Vane/Wind mode
       bit 3 (0x08): Track mode                        */
    uint8_t z = p[PL_Z_MODE] & 0x0FU;

    /* Alarms: M = low nibble of PL_M_ALARM byte.
       bit 2 (0x04): Off Course Alarm
       bit 3 (0x08): Wind Shift Alarm                  */
    uint8_t m = p[PL_M_ALARM] & 0x0FU;

    /* Compass heading (degrees):
       H = (U & 0x3)*90 + (VW & 0x3F)*2 + correction
       where U = high nibble of attribute byte          */
    uint8_t u  = (p[PL_ST1_ATTR] >> 4) & 0x0FU;
    uint8_t vw = p[PL_VW];
    uint8_t xy = p[PL_XY];
    uint16_t hdg = (uint16_t)(u & 0x3U) * 90U
                 + (uint16_t)(vw & 0x3FU) * 2U
                 + ((u & 0xCU) ? ((u & 0xCU) == 0xCU ? 2U : 1U) : 0U);

    /* AP course (degrees):
       C = (two high bits of V) * 90 + XY / 2
       V = high nibble of VW byte                      */
    uint16_t crs = (uint16_t)(((vw >> 4) >> 2) & 0x3U) * 90U + (uint16_t)xy / 2U;

    /* Rudder position (signed degrees, positive = starboard) */
    int8_t rudder = (int8_t)p[PL_RR];

    g_locked_crs = crs % 360U;

    /* Broadcast PGN 65379 on every status packet */
    {
        uint16_t mode65379;
        switch (z) {
            case 0x00U: mode65379 = 0U;   break;
            case 0x02U: mode65379 = 64U;  break;
            case 0x06U: mode65379 = 256U; break;
            case 0x0AU: mode65379 = 384U; break;
            default:    mode65379 = 0xFFFFU; break;
        }
        if (mode65379 != 0xFFFFU) {
            uint8_t d65[8] = {
                0x3BU, 0x9FU,
                (uint8_t)(mode65379 & 0xFFU),
                (uint8_t)((mode65379 >> 8) & 0xFFU),
                p[PL_M_ALARM],
                0x00U, 0x07U, 0x00U
            };
            send_raw_frame(PGN65379_ID, d65, 8U);
        }
    }

    /* Report mode change */
    if (z != g_last_z) {
        g_last_z = z;

        char line[40];
        uint8_t pos = 0;

        line[pos++] = '<'; line[pos++] = ' ';

        if      (z & 0x08U) pos = append_str(line, pos, "TRACK");
        else if (z & 0x04U) pos = append_str(line, pos, "WIND");
        else if (z & 0x02U) pos = append_str(line, pos, "AUTO");
        else                 pos = append_str(line, pos, "STANDBY");

        line[pos++] = ' '; line[pos++] = 'H';
        fmt3(line + pos, hdg % 360U); pos += 3;

        if (z & 0x02U) {  /* show AP course only when engaged */
            line[pos++] = ' '; line[pos++] = 'C';
            fmt3(line + pos, crs % 360U); pos += 3;
        }

        line[pos++] = ' '; line[pos++] = 'R';
        if (rudder < 0) {
            line[pos++] = '-';
            fmt3(line + pos, (uint16_t)(-rudder)); pos += 3;
        } else {
            fmt3(line + pos, (uint16_t)rudder); pos += 3;
        }

        line[pos++] = '\r'; line[pos++] = '\n';
        cdc_transmit((uint8_t *)line, pos);
    }

    /* Report alarm changes (print on set only) */
    if (m != g_last_m) {
        uint8_t onset = m & ~g_last_m;
        g_last_m = m;
        if (onset & 0x04U) cdc_transmit((uint8_t *)"< OCA\r\n", 7U);
        if (onset & 0x08U) cdc_transmit((uint8_t *)"< WSA\r\n", 7U);
    }

    /* Print confirmed AP course after a heading adjustment */
    if (g_print_next_crs && (z & 0x02U)) {
        g_print_next_crs = 0U;
        char line[8];
        uint8_t pos = 0;
        line[pos++] = '>'; line[pos++] = ' '; line[pos++] = 'C';
        fmt3(line + pos, crs % 360U); pos += 3;
        line[pos++] = '\r'; line[pos++] = '\n';
        cdc_transmit((uint8_t *)line, pos);
    }
}

/* ---- TX: drain keystroke queue ------------------------------------------ */

void can_handler_tx_poll(void) {
    while (g_ks_head != g_ks_tail) {
        can_send_keystroke(g_ks_queue[g_ks_head]);
        g_ks_head = (uint8_t)((g_ks_head + 1U) % KS_QUEUE_LEN);
    }
}

/* ---- Poll --------------------------------------------------------------- */

void can_handler_poll(void) {
    while (HAL_CAN_GetRxFifoFillLevel(&g_hcan, CAN_RX_FIFO0) > 0U) {
        CAN_RxHeaderTypeDef hdr = {0};
        uint8_t data[8] = {0};

        if (HAL_CAN_GetRxMessage(&g_hcan, CAN_RX_FIFO0, &hdr, data) != HAL_OK) break;

        board_leds_pulse_rx();

        if (hdr.IDE != CAN_ID_EXT || hdr.RTR != CAN_RTR_DATA) continue;

        uint8_t frame_src = (uint8_t)(hdr.ExtId & 0xFFU);

        if ((hdr.ExtId & STATUS_ID_MASK) == STATUS_ID_VAL) {
            fp_feed(&g_fp, data);
            if (g_fp.total > 0U && g_fp.got >= g_fp.total) {
                process_pilot_status();
                g_fp.total = 0U;
            }
        } else if (frame_src != 0x01U && frame_src != 0x73U &&
                   (hdr.ExtId & GRP_FUNC_MASK) == GRP_FUNC_VAL) {
            uint8_t idx = data[0] & 0x1FU;
            if (idx == 0U) {
                /* Frame 0: record who owns this reassembly */
                g_fp_mfd.src = frame_src;
                g_fp_mfd.dst = (uint8_t)((hdr.ExtId >> 8) & 0xFFU);
            } else if (frame_src != g_fp_mfd.src) {
                /* Continuation from wrong source (e.g. AP ack) — discard and reset */
                g_fp_mfd.total = 0U;
                continue;
            }
            fp_feed(&g_fp_mfd, data);
            if (g_fp_mfd.total > 0U && g_fp_mfd.got >= g_fp_mfd.total) {
                process_grp_func(g_fp_mfd.src, g_fp_mfd.dst);
                g_fp_mfd.total = 0U;
            }
        }
    }
}
