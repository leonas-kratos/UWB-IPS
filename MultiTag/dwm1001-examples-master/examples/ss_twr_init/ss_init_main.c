// ============================================================================
// Tag Initiator v2.5
//
// Thay đổi so với v2.4:
//   - DATA frame thêm 1 byte ring_size tại byte 14
//   - Anchor data payload shift sang byte 15+
//   - Bridge đọc ring_size, so với current_ring_size — khác thì resend RING
//
// DATA frame layout v2.5:
//   Byte  0– 9: header
//   Byte 10–11: tag_id       (uint16 LE)
//   Byte 12–13: cycle        (uint16 LE)
//   Byte    14: ring_size    (uint8)   ← MỚI
//   Byte 15+  : NUM_ANCHORS × 6 bytes (anchor_id uint16 + distance int32)
//
// RING frame layout: giữ nguyên v2.4
// ============================================================================

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "nrf.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"

#define MY_TAG_ID           0x5676

#define NUM_ANCHORS         3
#define MAX_TOKEN_WAIT_MS   200
#define RX_BUF_LEN         64

#define ANCHOR_1    0x1001
#define ANCHOR_2    0x1002
#define ANCHOR_3    0x1003

static uint32_t anchor_ids[NUM_ANCHORS] = { ANCHOR_1, ANCHOR_2, ANCHOR_3 };

#define RING_MAX_TAGS   8
#define RING_HDR_LEN    11

static uint16_t ring_order[RING_MAX_TAGS];
static uint8_t  ring_size   = 0;
static uint16_t my_next_tag = 0;
static uint8_t  in_ring     = 0;

static volatile uint8_t g_ring_pending = 0;
static volatile uint8_t g_ring_buf[RING_MAX_TAGS * 2];
static volatile uint8_t g_ring_size    = 0;

#define RANGING_OK       1
#define RANGING_FAIL     0
#define RANGING_ABORTED -1

#define POLL_MSG_LEN            22
#define ALL_MSG_SN_IDX           2
#define ALL_MSG_COMMON_LEN      10
#define POLL_MSG_DEVICE_ID_IDX  10
#define POLL_MSG_DEVICE_ID_LEN   4
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN          4
#define RESP_MSG_DEVICE_ID_IDX  18
#define RESP_MSG_DEVICE_ID_LEN   4
#define POLL_TAG_ID_IDX         14

#define TOKEN_MSG_LEN   16
#define TOKEN_FROM_IDX  10
#define TOKEN_TO_IDX    12
#define TOKEN_CYCLE_IDX 14

// DATA frame v2.5 — thêm ring_size tại byte 14
#define DATA_TAG_IDX        10
#define DATA_CYCLE_IDX      12
#define DATA_RINGSIZE_IDX   14
#define DATA_PAYLOAD_IDX    15
#define DATA_ANCHOR_STRIDE   6
#define DATA_HDR_LEN        15
#define DATA_MSG_LEN        (DATA_HDR_LEN + NUM_ANCHORS * DATA_ANCHOR_STRIDE + 2)

#define RING_SIZE_IDX   10
#define RING_TAGS_IDX   11

static const uint8_t token_hdr_ref[] = { 0x41,0x88,0,0xCA,0xDE,'T','O','K','N',0xE2 };
static const uint8_t data_hdr_ref[]  = { 0x41,0x88,0,0xCA,0xDE,'D','A','T','A',0xE3 };
static const uint8_t ring_hdr_ref[]  = { 0x41,0x88,0,0xCA,0xDE,'R','I','N','G',0xE4 };

static uint8_t tx_poll_msg[POLL_MSG_LEN] = {
    0x41,0x88,0,0xCA,0xDE,'I','O','V','E',0xE0,
    0,0,0,0, 0,0, 0,0,0,0,0,0
};
static uint8_t rx_resp_msg[] = {
    0x41,0x88,0,0xCA,0xDE,'V','E','I','O',0xE1,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0
};
static uint8_t tx_token_msg[TOKEN_MSG_LEN];
static uint8_t tx_data_msg[DATA_MSG_LEN];

static uint8_t  frame_seq_nb      = 0;
static uint8_t  rx_buffer[RX_BUF_LEN];
static uint32_t status_reg        = 0;
static int      measurement_cycle = 0;

typedef struct { uint8_t valid; double distance; } anchor_data_t;
static anchor_data_t anchor_data[NUM_ANCHORS];

#define SPEED_OF_LIGHT 299702547

static void get_ts(uint8_t *f, uint32_t *ts)
{
    *ts = 0;
    for (int i = 0; i < RESP_MSG_TS_LEN; i++) *ts += f[i] << (i * 8);
}
static uint32_t get_device_id(uint8_t *f)
{
    uint32_t id = 0;
    for (int i = 0; i < RESP_MSG_DEVICE_ID_LEN; i++) id += f[i] << (i * 8);
    return id;
}
static void set_device_id(uint8_t *f, uint32_t id)
{
    for (int i = 0; i < POLL_MSG_DEVICE_ID_LEN; i++) f[i] = (id >> (i * 8)) & 0xFF;
}
static inline uint8_t pseudo_rand_jitter(void)
{
    static uint8_t state = 0xA5;
    state ^= frame_seq_nb; state ^= (state << 3); state ^= (state >> 5);
    return state & 0x03;
}

static void handle_ring_frame(const uint8_t *buf, uint32_t flen)
{
    if (flen < (uint32_t)(RING_HDR_LEN + 2)) return;
    uint8_t sz = buf[RING_SIZE_IDX];
    if (sz == 0 || sz > RING_MAX_TAGS) return;
    if (flen < (uint32_t)(RING_HDR_LEN + sz * 2)) return;
    g_ring_size = sz;
    for (int i = 0; i < sz * 2; i++) g_ring_buf[i] = buf[RING_TAGS_IDX + i];
    g_ring_pending = 1;
    printf("[0x%04X] RING received: size=%d\r\n", MY_TAG_ID, sz);
}

static void apply_ring_update(void)
{
    uint8_t sz = g_ring_size;
    for (int i = 0; i < sz; i++)
        ring_order[i] = (uint16_t)g_ring_buf[i*2] | ((uint16_t)g_ring_buf[i*2+1] << 8);
    ring_size = sz;
    my_next_tag = 0; in_ring = 0;
    for (int i = 0; i < sz; i++) {
        if (ring_order[i] == MY_TAG_ID) {
            my_next_tag = ring_order[(i + 1) % sz];
            in_ring = 1; break;
        }
    }
    g_ring_pending = 0;
    printf("[0x%04X] Ring applied: size=%d in_ring=%d next=0x%04X\r\n",
           MY_TAG_ID, ring_size, in_ring, my_next_tag);
    for (int i = 0; i < sz; i++)
        printf("0x%04X%s", ring_order[i], i < sz-1 ? "->" : "\r\n");
}

static int do_ranging(uint32_t anchor_id, uint8_t idx)
{
    int retry_count = 0;
    anchor_data[idx].valid = 0; anchor_data[idx].distance = 0.0;

    while (1) {
        if (g_ring_pending) { dwt_forcetrxoff(); return RANGING_ABORTED; }

        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        set_device_id(&tx_poll_msg[POLL_MSG_DEVICE_ID_IDX], anchor_id);
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
                 (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
            if (g_ring_pending) { dwt_forcetrxoff(); frame_seq_nb++; return RANGING_ABORTED; }
            vTaskDelay(0);
        }
        frame_seq_nb++;

        if (!(status_reg & SYS_STATUS_RXFCG)) {
            dwt_forcetrxoff();
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            dwt_rxreset();
            if (g_ring_pending) return RANGING_ABORTED;
            uint32_t bk = (uint32_t)(retry_count < 4 ? retry_count : 4) + pseudo_rand_jitter();
            if (bk > 0) vTaskDelay(pdMS_TO_TICKS(bk));
            retry_count++; continue;
        }

        uint32_t flen;
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
        flen = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
        if (flen <= RX_BUF_LEN) dwt_readrxdata(rx_buffer, flen, 0);
        rx_buffer[ALL_MSG_SN_IDX] = 0;

        if (memcmp(rx_buffer, ring_hdr_ref, ALL_MSG_COMMON_LEN) == 0) {
            handle_ring_frame(rx_buffer, flen); dwt_forcetrxoff(); return RANGING_ABORTED;
        }
        if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) != 0) {
            dwt_forcetrxoff(); if (g_ring_pending) return RANGING_ABORTED;
            retry_count++; continue;
        }
        if (get_device_id(&rx_buffer[RESP_MSG_DEVICE_ID_IDX]) != anchor_id) {
            dwt_forcetrxoff(); if (g_ring_pending) return RANGING_ABORTED;
            retry_count++; continue;
        }

        uint32_t ptx = dwt_readtxtimestamplo32();
        uint32_t rrx = dwt_readrxtimestamplo32();
        float cor = dwt_readcarrierintegrator() *
                    (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6);
        uint32_t prx, rtx;
        get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &prx);
        get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &rtx);
        double tof = (((int32_t)(rrx-ptx) - (int32_t)(rtx-prx)*(1.f-cor)) / 2.f) * DWT_TIME_UNITS;
        anchor_data[idx].distance = tof * SPEED_OF_LIGHT * 1000.0;
        anchor_data[idx].valid = 1;
        return RANGING_OK;
    }
}

static void data_send(void)
{
    memcpy(tx_data_msg, data_hdr_ref, ALL_MSG_COMMON_LEN);
    tx_data_msg[ALL_MSG_SN_IDX]     = frame_seq_nb++;
    tx_data_msg[DATA_TAG_IDX]       = (uint8_t)(MY_TAG_ID & 0xFF);
    tx_data_msg[DATA_TAG_IDX+1]     = (uint8_t)((MY_TAG_ID >> 8) & 0xFF);
    tx_data_msg[DATA_CYCLE_IDX]     = (uint8_t)(measurement_cycle & 0xFF);
    tx_data_msg[DATA_CYCLE_IDX+1]   = (uint8_t)((measurement_cycle >> 8) & 0xFF);
    tx_data_msg[DATA_RINGSIZE_IDX]  = ring_size;   // ← ring_size tag đang dùng

    for (int i = 0; i < NUM_ANCHORS; i++) {
        uint8_t *p = &tx_data_msg[DATA_PAYLOAD_IDX + i * DATA_ANCHOR_STRIDE];
        p[0] = (uint8_t)(anchor_ids[i] & 0xFF);
        p[1] = (uint8_t)((anchor_ids[i] >> 8) & 0xFF);
        int32_t d_raw = anchor_data[i].valid
                        ? (int32_t)(anchor_data[i].distance * 10.0)
                        : (int32_t)0xFFFFFFFF;
        p[2]=(uint8_t)(d_raw&0xFF); p[3]=(uint8_t)((d_raw>>8)&0xFF);
        p[4]=(uint8_t)((d_raw>>16)&0xFF); p[5]=(uint8_t)((d_raw>>24)&0xFF);
    }

    dwt_forcetrxoff();
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
    dwt_writetxdata(DATA_MSG_LEN, tx_data_msg, 0);
    dwt_writetxfctrl(DATA_MSG_LEN, 0, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {}
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
}

static void token_send(uint16_t to_tag_id)
{
    memcpy(tx_token_msg, token_hdr_ref, ALL_MSG_COMMON_LEN);
    tx_token_msg[ALL_MSG_SN_IDX]    = frame_seq_nb++;
    tx_token_msg[TOKEN_FROM_IDX]    = (uint8_t)(MY_TAG_ID & 0xFF);
    tx_token_msg[TOKEN_FROM_IDX+1]  = (uint8_t)((MY_TAG_ID >> 8) & 0xFF);
    tx_token_msg[TOKEN_TO_IDX]      = (uint8_t)(to_tag_id & 0xFF);
    tx_token_msg[TOKEN_TO_IDX+1]    = (uint8_t)((to_tag_id >> 8) & 0xFF);
    tx_token_msg[TOKEN_CYCLE_IDX]   = (uint8_t)(measurement_cycle & 0xFF);
    tx_token_msg[TOKEN_CYCLE_IDX+1] = (uint8_t)((measurement_cycle >> 8) & 0xFF);

    dwt_forcetrxoff();
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
    dwt_writetxdata(TOKEN_MSG_LEN, tx_token_msg, 0);
    dwt_writetxfctrl(TOKEN_MSG_LEN, 0, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {}
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
}

static void token_wait(void)
{
    dwt_forcetrxoff();
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(MAX_TOKEN_WAIT_MS);

    while (1) {
        if (g_ring_pending) {
            apply_ring_update();
            deadline = xTaskGetTickCount() + pdMS_TO_TICKS(MAX_TOKEN_WAIT_MS);
        }
        if (xTaskGetTickCount() >= deadline) {
            printf("[0x%04X] Token timeout (cycle %d)\r\n", MY_TAG_ID, measurement_cycle);
            dwt_forcetrxoff(); return;
        }
        status_reg = dwt_read32bitreg(SYS_STATUS_ID);
        if (status_reg & SYS_STATUS_RXFCG) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
            uint32_t flen = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
            if (flen <= RX_BUF_LEN) dwt_readrxdata(rx_buffer, flen, 0);
            rx_buffer[ALL_MSG_SN_IDX] = 0;

            if (memcmp(rx_buffer, ring_hdr_ref, ALL_MSG_COMMON_LEN) == 0) {
                handle_ring_frame(rx_buffer, flen);
                apply_ring_update();
                deadline = xTaskGetTickCount() + pdMS_TO_TICKS(MAX_TOKEN_WAIT_MS);
                dwt_rxenable(DWT_START_RX_IMMEDIATE); continue;
            }
            if (memcmp(rx_buffer, token_hdr_ref, ALL_MSG_COMMON_LEN) == 0
                && flen >= TOKEN_MSG_LEN)
            {
                uint16_t to_id = (uint16_t)rx_buffer[TOKEN_TO_IDX]
                               | ((uint16_t)rx_buffer[TOKEN_TO_IDX+1] << 8);
                if (to_id == MY_TAG_ID) { dwt_forcetrxoff(); return; }
            }
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }
        else if (status_reg & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            dwt_rxreset(); dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }
        else { vTaskDelay(0); }
    }
}

int ss_init_run(void)
{
    for (int i = 0; i < NUM_ANCHORS; i++) anchor_data[i].valid = 0;
    for (int i = 0; i < NUM_ANCHORS; i++) {
        int result = do_ranging(anchor_ids[i], i);
        if (result == RANGING_ABORTED) {
            if (g_ring_pending) apply_ring_update();
            printf("[0x%04X] Aborted at anchor %d, restarting\r\n", MY_TAG_ID, i);
            for (int j = 0; j < NUM_ANCHORS; j++) anchor_data[j].valid = 0;
            i = -1; continue;
        }
    }
    measurement_cycle++;
    printf("[0x%04X] cyc=%d rs=%d: %.1f %.1f %.1f\r\n",
           MY_TAG_ID, measurement_cycle, ring_size,
           anchor_data[0].distance, anchor_data[1].distance, anchor_data[2].distance);
    data_send();
    if (!in_ring || ring_size < 2) { vTaskDelay(pdMS_TO_TICKS(10)); return 1; }
    token_send(my_next_tag);
    token_wait();
    return 1;
}

void ss_initiator_task_function(void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    dwt_setleds(DWT_LEDS_ENABLE);
    tx_poll_msg[POLL_TAG_ID_IDX]   = (uint8_t)(MY_TAG_ID & 0xFF);
    tx_poll_msg[POLL_TAG_ID_IDX+1] = (uint8_t)((MY_TAG_ID >> 8) & 0xFF);
    for (int i = 0; i < NUM_ANCHORS; i++) anchor_data[i].valid = 0;

    printf("\r\n========================================\r\n");
    printf("Tag Initiator v2.5 | ID: 0x%04X\r\n", MY_TAG_ID);
    printf("DATA frame: %d bytes (ring_size @ byte 14)\r\n", DATA_MSG_LEN);
    printf("========================================\r\n\r\n");

    for (int i = 0; i < NUM_ANCHORS; i++) {
        int r = do_ranging(anchor_ids[i], i);
        if (r == RANGING_ABORTED) { if (g_ring_pending) apply_ring_update(); break; }
    }
    data_send();

    printf("[0x%04X] Waiting for RING...\r\n", MY_TAG_ID);
    token_wait();

    if (in_ring)
        printf("[0x%04X] In ring, size=%d next=0x%04X\r\n", MY_TAG_ID, ring_size, my_next_tag);
    else
        printf("[0x%04X] Free ranging\r\n", MY_TAG_ID);

    while (1) ss_init_run();
}