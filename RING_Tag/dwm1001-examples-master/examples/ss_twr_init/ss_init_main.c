#include <stdio.h>
#include <string.h>
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "nrf.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"

#define MY_TAG_ID           0x567E

#define MAX_ANCHORS         8
#define MAX_TOKEN_WAIT_MS   100
#define RX_BUF_LEN         64

#define RING_MAX_TAGS       8
#define RING_HDR_LEN        11

/* FIX: timeout wait_for_alist < TAG_TIMEOUT_MS(3000) để tag còn kịp gửi DATA */
#define WAIT_ALIST_TIMEOUT_MS   1500

static uint16_t ring_order[RING_MAX_TAGS];
static uint8_t  ring_size   = 0;
static uint16_t my_next_tag = 0;
static uint8_t  in_ring     = 0;

static volatile uint8_t  g_ring_pending  = 0;
static volatile uint8_t  g_ring_buf[RING_MAX_TAGS * 2];
static volatile uint8_t  g_ring_size     = 0;

static volatile uint8_t  g_alist_pending = 0;
static volatile uint8_t  g_alist_buf[MAX_ANCHORS * 2];
static volatile uint8_t  g_alist_size    = 0;

static uint16_t anchor_ids[MAX_ANCHORS];
static uint8_t  num_anchors = 0;

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

#define DATA_TAG_IDX        10
#define DATA_CYCLE_IDX      12
#define DATA_RINGSIZE_IDX   14
#define DATA_NUMANCHORS_IDX 15
#define DATA_PAYLOAD_IDX    16
#define DATA_ANCHOR_STRIDE   6
#define DATA_HDR_LEN        16
#define DATA_MSG_MAX_LEN    (DATA_HDR_LEN + MAX_ANCHORS * DATA_ANCHOR_STRIDE + 2)

#define RING_SIZE_IDX   10
#define RING_TAGS_IDX   11

#define ALIST_SIZE_IDX  10
#define ALIST_IDS_IDX   11
#define ALIST_HDR_LEN   11

#define ARMV_ID_IDX     10
#define ARMV_MSG_LEN    14

#define MAX_RANGING_RETRY    5
#define RANGING_RETRY_DELAY_MS 5

// Header cho message báo anchor chết
static const uint8_t anch_dead_hdr_ref[] = { 0x41,0x88,0,0xCA,0xDE,'A','D','E','D',0xE8 };

#define ADEAD_ID_IDX    10
#define ADEAD_MSG_LEN   14

static const uint8_t token_hdr_ref[]      = { 0x41,0x88,0,0xCA,0xDE,'T','O','K','N',0xE2 };
static const uint8_t data_hdr_ref[]       = { 0x41,0x88,0,0xCA,0xDE,'D','A','T','A',0xE3 };
static const uint8_t ring_hdr_ref[]       = { 0x41,0x88,0,0xCA,0xDE,'R','I','N','G',0xE4 };
static const uint8_t anch_list_hdr_ref[]  = { 0x41,0x88,0,0xCA,0xDE,'A','L','S','T',0xE6 };
static const uint8_t anch_remove_hdr_ref[]= { 0x41,0x88,0,0xCA,0xDE,'A','R','M','V',0xE7 };

static uint8_t tx_poll_msg[POLL_MSG_LEN] = {
    0x41,0x88,0,0xCA,0xDE,'I','O','V','E',0xE0,
    0,0,0,0, 0,0, 0,0,0,0,0,0
};
static uint8_t rx_resp_msg[] = {
    0x41,0x88,0,0xCA,0xDE,'V','E','I','O',0xE1,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0
};
static uint8_t tx_token_msg[TOKEN_MSG_LEN];
static uint8_t tx_data_msg[DATA_MSG_MAX_LEN];

static uint8_t  frame_seq_nb      = 0;
static uint8_t  rx_buffer[RX_BUF_LEN];
static uint32_t status_reg        = 0;
static int      measurement_cycle = 0;

typedef struct { uint8_t valid; double distance; } anchor_data_t;
static anchor_data_t anchor_data[MAX_ANCHORS];

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
    printf("[0x%04X] RING rx size=%d\r\n", MY_TAG_ID, sz);
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
    printf("[0x%04X] Ring applied size=%d in_ring=%d next=0x%04X\r\n",
           MY_TAG_ID, ring_size, in_ring, my_next_tag);
}

static void handle_alist_frame(const uint8_t *buf, uint32_t flen)
{
    if (flen < (uint32_t)(ALIST_HDR_LEN + 2)) return;
    uint8_t sz = buf[ALIST_SIZE_IDX];
    if (sz == 0 || sz > MAX_ANCHORS) return;
    if (flen < (uint32_t)(ALIST_HDR_LEN + sz * 2)) return;
    g_alist_size = sz;
    for (int i = 0; i < sz * 2; i++) g_alist_buf[i] = buf[ALIST_IDS_IDX + i];
    g_alist_pending = 1;
    printf("[0x%04X] ANCH_LIST rx size=%d\r\n", MY_TAG_ID, sz);
}

static void apply_alist_update(void)
{
    uint8_t sz = g_alist_size;
    uint16_t new_ids[MAX_ANCHORS];
    for (int i = 0; i < sz; i++)
        new_ids[i] = (uint16_t)g_alist_buf[i*2] | ((uint16_t)g_alist_buf[i*2+1] << 8);

    anchor_data_t new_data[MAX_ANCHORS];
    for (int i = 0; i < sz; i++) {
        new_data[i].valid    = 0;
        new_data[i].distance = 0.0;
        for (int j = 0; j < num_anchors; j++) {
            if (anchor_ids[j] == new_ids[i]) {
                new_data[i] = anchor_data[j];
                break;
            }
        }
    }

    for (int i = 0; i < sz; i++) {
        anchor_ids[i]  = new_ids[i];
        anchor_data[i] = new_data[i];
    }
    num_anchors = sz;
    g_alist_pending = 0;
    printf("[0x%04X] Anchors applied num=%d [", MY_TAG_ID, num_anchors);
    for (int i = 0; i < num_anchors; i++)
        printf("0x%04X%s", anchor_ids[i], i < num_anchors-1 ? "," : "");
    printf("]\r\n");
}

static void handle_anch_remove(const uint8_t *buf, uint32_t flen)
{
    if (flen < ARMV_MSG_LEN) return;
    uint16_t remove_id = (uint16_t)buf[ARMV_ID_IDX] | ((uint16_t)buf[ARMV_ID_IDX+1] << 8);
    int found = -1;
    for (int i = 0; i < (int)num_anchors; i++)
        if (anchor_ids[i] == remove_id) { found = i; break; }
    if (found < 0) return;
    for (int i = found; i < (int)num_anchors - 1; i++) {
        anchor_ids[i]  = anchor_ids[i+1];
        anchor_data[i] = anchor_data[i+1];
    }
    num_anchors--;
    printf("[0x%04X] Anchor 0x%04X removed num=%d\r\n", MY_TAG_ID, remove_id, num_anchors);
}

static void anchor_dead_send(uint16_t anchor_id)
{
    uint8_t tx_buf[ADEAD_MSG_LEN];
    memcpy(tx_buf, anch_dead_hdr_ref, ALL_MSG_COMMON_LEN);
    tx_buf[ALL_MSG_SN_IDX]    = frame_seq_nb++;
    tx_buf[ADEAD_ID_IDX]      = (uint8_t)(anchor_id & 0xFF);
    tx_buf[ADEAD_ID_IDX+1]    = (uint8_t)((anchor_id >> 8) & 0xFF);
    tx_buf[ADEAD_ID_IDX+2]    = 0x00;
    tx_buf[ADEAD_ID_IDX+3]    = 0x00;

    dwt_forcetrxoff();
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
    dwt_writetxdata(ADEAD_MSG_LEN, tx_buf, 0);
    dwt_writetxfctrl(ADEAD_MSG_LEN, 0, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {}
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
    printf("[0x%04X] ANCHOR_DEAD broadcast 0x%04X\r\n", MY_TAG_ID, anchor_id);
}

static int do_ranging(uint32_t anchor_id, uint8_t idx)
{
    int retry_count = 0;
    anchor_data[idx].valid = 0;
    anchor_data[idx].distance = 0.0;

    while (retry_count < MAX_RANGING_RETRY) {
        if (g_ring_pending || g_alist_pending) {
            dwt_forcetrxoff();
            return RANGING_ABORTED;
        }

        /* --- Gửi POLL --- */
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        set_device_id(&tx_poll_msg[POLL_MSG_DEVICE_ID_IDX], anchor_id);
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        /* --- Chờ RX --- */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
                 (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
            if (g_ring_pending || g_alist_pending) {
                dwt_forcetrxoff();
                frame_seq_nb++;
                return RANGING_ABORTED;
            }
            vTaskDelay(0);
        }
        frame_seq_nb++;

        /* --- RX lỗi / timeout --- */
        if (!(status_reg & SYS_STATUS_RXFCG)) {
            dwt_forcetrxoff();
            dwt_write32bitreg(SYS_STATUS_ID,
                              SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            dwt_rxreset();
            if (g_ring_pending || g_alist_pending) return RANGING_ABORTED;
            vTaskDelay(pdMS_TO_TICKS(RANGING_RETRY_DELAY_MS));
            retry_count++;
            continue;
        }

        /* --- Đọc frame --- */
        uint32_t flen = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
        if (flen <= RX_BUF_LEN) dwt_readrxdata(rx_buffer, flen, 0);
        rx_buffer[ALL_MSG_SN_IDX] = 0;

        /* --- Kiểm tra frame đặc biệt (ưu tiên cao) --- */
        if (memcmp(rx_buffer, ring_hdr_ref, ALL_MSG_COMMON_LEN) == 0) {
            handle_ring_frame(rx_buffer, flen);
            dwt_forcetrxoff();
            return RANGING_ABORTED;
        }
        if (memcmp(rx_buffer, anch_list_hdr_ref, ALL_MSG_COMMON_LEN) == 0) {
            handle_alist_frame(rx_buffer, flen);
            dwt_forcetrxoff();
            return RANGING_ABORTED;
        }
        if (memcmp(rx_buffer, anch_remove_hdr_ref, ALL_MSG_COMMON_LEN) == 0) {
            handle_anch_remove(rx_buffer, flen);
            dwt_forcetrxoff();
            return RANGING_ABORTED;
        }

        /* --- Kiểm tra RESP đúng header --- */
        if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) != 0) {
            dwt_forcetrxoff();
            if (g_ring_pending || g_alist_pending) return RANGING_ABORTED;
            retry_count++;
            continue;
        }

        /* --- Kiểm tra RESP đúng anchor_id --- */
        if (get_device_id(&rx_buffer[RESP_MSG_DEVICE_ID_IDX]) != anchor_id) {
            dwt_forcetrxoff();
            if (g_ring_pending || g_alist_pending) return RANGING_ABORTED;
            retry_count++;
            continue;
        }

        /* --- Tính khoảng cách --- */
        uint32_t ptx = dwt_readtxtimestamplo32();
        uint32_t rrx = dwt_readrxtimestamplo32();
        float cor = dwt_readcarrierintegrator() *
                    (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6);
        uint32_t prx, rtx;
        get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &prx);
        get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &rtx);
        double tof = (((int32_t)(rrx - ptx) - (int32_t)(rtx - prx) * (1.f - cor)) / 2.f)
                     * DWT_TIME_UNITS;
        anchor_data[idx].distance = tof * SPEED_OF_LIGHT * 1000.0;
        anchor_data[idx].valid    = 1;
        return RANGING_OK;
    }

    /* --- Hết retry: báo anchor chết --- */
    printf("[0x%04X] Anchor 0x%04X no response after %d retries\r\n",
           MY_TAG_ID, (unsigned)anchor_id, MAX_RANGING_RETRY);
    anchor_dead_send((uint16_t)anchor_id);
    return RANGING_FAIL;
}

static void data_send(void)
{
    uint8_t n = num_anchors;
    uint8_t msg_len = DATA_HDR_LEN + n * DATA_ANCHOR_STRIDE + 2;

    memcpy(tx_data_msg, data_hdr_ref, ALL_MSG_COMMON_LEN);
    tx_data_msg[ALL_MSG_SN_IDX]      = frame_seq_nb++;
    tx_data_msg[DATA_TAG_IDX]        = (uint8_t)(MY_TAG_ID & 0xFF);
    tx_data_msg[DATA_TAG_IDX+1]      = (uint8_t)((MY_TAG_ID >> 8) & 0xFF);
    tx_data_msg[DATA_CYCLE_IDX]      = (uint8_t)(measurement_cycle & 0xFF);
    tx_data_msg[DATA_CYCLE_IDX+1]    = (uint8_t)((measurement_cycle >> 8) & 0xFF);
    tx_data_msg[DATA_RINGSIZE_IDX]   = ring_size;
    tx_data_msg[DATA_NUMANCHORS_IDX] = n;

    for (int i = 0; i < n; i++) {
        uint8_t *p = &tx_data_msg[DATA_PAYLOAD_IDX + i * DATA_ANCHOR_STRIDE];
        p[0] = (uint8_t)(anchor_ids[i] & 0xFF);
        p[1] = (uint8_t)((anchor_ids[i] >> 8) & 0xFF);
        int32_t d_raw = anchor_data[i].valid
                        ? (int32_t)(anchor_data[i].distance * 10.0)
                        : (int32_t)0xFFFFFFFF;
        p[2]=(uint8_t)(d_raw&0xFF); p[3]=(uint8_t)((d_raw>>8)&0xFF);
        p[4]=(uint8_t)((d_raw>>16)&0xFF); p[5]=(uint8_t)((d_raw>>24)&0xFF);
    }

    uint8_t crc_idx = DATA_HDR_LEN + n * DATA_ANCHOR_STRIDE;
    tx_data_msg[crc_idx]   = 0x00;
    tx_data_msg[crc_idx+1] = 0x00;

    dwt_forcetrxoff();
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
    dwt_writetxdata(msg_len, tx_data_msg, 0);
    dwt_writetxfctrl(msg_len, 0, 0);
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
        if (g_ring_pending)  { apply_ring_update();  deadline = xTaskGetTickCount() + pdMS_TO_TICKS(MAX_TOKEN_WAIT_MS); }
        if (g_alist_pending) { apply_alist_update(); deadline = xTaskGetTickCount() + pdMS_TO_TICKS(MAX_TOKEN_WAIT_MS); }

        if (xTaskGetTickCount() >= deadline) {
            printf("[0x%04X] Token timeout cycle=%d\r\n", MY_TAG_ID, measurement_cycle);
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
            if (memcmp(rx_buffer, anch_list_hdr_ref, ALL_MSG_COMMON_LEN) == 0) {
                handle_alist_frame(rx_buffer, flen);
                apply_alist_update();
                deadline = xTaskGetTickCount() + pdMS_TO_TICKS(MAX_TOKEN_WAIT_MS);
                dwt_rxenable(DWT_START_RX_IMMEDIATE); continue;
            }
            if (memcmp(rx_buffer, anch_remove_hdr_ref, ALL_MSG_COMMON_LEN) == 0) {
                handle_anch_remove(rx_buffer, flen);
                dwt_rxenable(DWT_START_RX_IMMEDIATE); continue;
            }
            if (memcmp(rx_buffer, token_hdr_ref, ALL_MSG_COMMON_LEN) == 0 && flen >= TOKEN_MSG_LEN) {
                uint16_t to_id = (uint16_t)rx_buffer[TOKEN_TO_IDX]
                               | ((uint16_t)rx_buffer[TOKEN_TO_IDX+1] << 8);
                if (to_id == MY_TAG_ID) { dwt_forcetrxoff(); return; }
            }
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        } else if (status_reg & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            dwt_rxreset();
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        } else {
            vTaskDelay(0);
        }
    }
}

/* FIX BUG 2: thêm timeout WAIT_ALIST_TIMEOUT_MS thay vì treo vô hạn.
 * Nếu hết timeout, return về ss_init_run() để tiếp tục vòng lặp và
 * giữ tag sống (gửi DATA với num_anchors=0) tránh bridge timeout tag. */
static void wait_for_alist(void)
{
    dwt_forcetrxoff();
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    printf("[0x%04X] Waiting ANCH_LIST...\r\n", MY_TAG_ID);

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(WAIT_ALIST_TIMEOUT_MS);
    uint8_t got_alist = 0;

    while (!got_alist) {
        /* FIX: thoát nếu quá thời gian, không treo vô hạn */
        if (xTaskGetTickCount() >= deadline) {
            printf("[0x%04X] wait_for_alist timeout, retrying\r\n", MY_TAG_ID);
            dwt_forcetrxoff();
            return;
        }

        status_reg = dwt_read32bitreg(SYS_STATUS_ID);
        if (status_reg & SYS_STATUS_RXFCG) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
            uint32_t flen = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
            if (flen <= RX_BUF_LEN) dwt_readrxdata(rx_buffer, flen, 0);
            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, anch_list_hdr_ref, ALL_MSG_COMMON_LEN) == 0) {
                handle_alist_frame(rx_buffer, flen);
                apply_alist_update();
                got_alist = 1;
            } else if (memcmp(rx_buffer, ring_hdr_ref, ALL_MSG_COMMON_LEN) == 0) {
                handle_ring_frame(rx_buffer, flen);
                apply_ring_update();
            } else if (memcmp(rx_buffer, anch_remove_hdr_ref, ALL_MSG_COMMON_LEN) == 0) {
                handle_anch_remove(rx_buffer, flen);
            }
            if (!got_alist) dwt_rxenable(DWT_START_RX_IMMEDIATE);
        } else if (status_reg & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            dwt_rxreset();
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        } else {
            vTaskDelay(0);
        }
    }
}

int ss_init_run(void)
{
    if (num_anchors == 0) wait_for_alist();

    /* FIX BUG 2: nếu vẫn không có anchor sau timeout, gửi DATA với na=0
     * để bridge biết tag còn sống và gửi lại ANCH_LIST, tránh bị timeout */
    if (num_anchors == 0) {
        printf("[0x%04X] No anchors, sending empty DATA to stay alive\r\n", MY_TAG_ID);
        measurement_cycle++;
        data_send();
        vTaskDelay(pdMS_TO_TICKS(200));
        return 1;
    }

    for (int i = 0; i < num_anchors; i++) anchor_data[i].valid = 0;
    int i = 0;
    while (i < num_anchors) {
        int result = do_ranging(anchor_ids[i], i);
        if (result == RANGING_ABORTED) {
            int aborted_at = i;
            if (g_ring_pending)  apply_ring_update();
            if (g_alist_pending) apply_alist_update();
            for (int j = aborted_at; j < num_anchors; j++) anchor_data[j].valid = 0;
            i = aborted_at;
            if (num_anchors == 0) {
                /* FIX: không treo, break ra ngoài để xử lý ở cuối */
                break;
            }
            continue;
        }
        i++;
    }

    /* FIX BUG 2: sau khi ranging, nếu num_anchors = 0 thì gửi DATA rỗng
     * và return về vòng lặp chính, KHÔNG gọi wait_for_alist() block mãi */
    if (num_anchors == 0) {
        printf("[0x%04X] All anchors gone, sending empty DATA\r\n", MY_TAG_ID);
        measurement_cycle++;
        data_send();
        vTaskDelay(pdMS_TO_TICKS(200));
        return 1;
    }

    measurement_cycle++;
    printf("[0x%04X] cyc=%d rs=%d na=%d:", MY_TAG_ID, measurement_cycle, ring_size, num_anchors);
    for (int i = 0; i < num_anchors; i++) {
        if (anchor_data[i].valid) printf(" %.1f", anchor_data[i].distance);
        else printf(" -1");
    }
    printf("\r\n");
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

    printf("Tag Initiator v3.2 | ID: 0x%04X\r\n", MY_TAG_ID);

    while (1) ss_init_run();
}