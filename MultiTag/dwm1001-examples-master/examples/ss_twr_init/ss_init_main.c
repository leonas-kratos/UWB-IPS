// ============================================================================
// Token Passing v2.3 — Tách DATA frame và TOKEN frame
//
// Thay đổi so với v2.2:
//   - DATA frame (20 bytes): chứa distances, bridge nghe
//   - TOKEN frame (16 bytes): giữ nguyên v2.0, chỉ pass token
//   - Không nhúng distance vào token → token nhỏ gọn như v2.0
//
// Frame layout:
//   DATA frame:
//     Byte  0– 9: header (0x41 0x88 SN 0xCA 0xDE 'D' 'A' 'T' 'A' 0xE3)
//     Byte 10–11: tag_id (uint16 LE)
//     Byte 12–13: cycle  (uint16 LE)
//     Byte 14+  : NUM_ANCHORS × 6 bytes (anchor_id uint16 + distance int32)
//
//   TOKEN frame: giữ nguyên v2.0 (16 bytes)
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

// ============================================================================
// CONFIG
// ============================================================================
#define MY_TAG_INDEX       5
#define NUM_TAGS           3
#define MAX_TOKEN_WAIT_MS  200

// ============================================================================
// DEVICE IDs
// ============================================================================
static const uint16_t TAG_IDS[] = { 0x5678, 0x5679, 0x567A, 0x567B, 0x567C };

#define MY_INITIATOR_DEVICE_ID  TAG_IDS[MY_TAG_INDEX]
#define NEXT_TAG_ID             TAG_IDS[(MY_TAG_INDEX + 1) % NUM_TAGS]

#define ANCHOR_1    0x1001
#define ANCHOR_2    0x1002
#define ANCHOR_3    0x1003
#define NUM_ANCHORS 3

static uint32_t anchor_ids[NUM_ANCHORS] = { ANCHOR_1, ANCHOR_2, ANCHOR_3 };

// ============================================================================
// DATA
// ============================================================================
typedef struct {
    dwt_rxdiag_t diagnostics;
    uint8_t      valid;
    double       distance;  // mm
} anchor_data_t;

static anchor_data_t anchor_data[NUM_ANCHORS];

// ============================================================================
// FRAME LAYOUT
// ============================================================================
#define POLL_MSG_LEN            22
#define POLL_TAG_ID_IDX         14
#define ALL_MSG_COMMON_LEN      10
#define ALL_MSG_SN_IDX           2
#define POLL_MSG_DEVICE_ID_IDX  10
#define POLL_MSG_DEVICE_ID_LEN   4
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN          4
#define RESP_MSG_DEVICE_ID_IDX  18
#define RESP_MSG_DEVICE_ID_LEN   4
#define RX_BUF_LEN              64

// TOKEN frame — giữ nguyên v2.0, 16 bytes
#define TOKEN_MSG_LEN       16
#define TOKEN_FROM_IDX      10
#define TOKEN_TO_IDX        12
#define TOKEN_CYCLE_IDX     14
#define TOKEN_SEND_REPEAT    1
#define TOKEN_SEND_GAP_MS    1

// DATA frame — frame mới chứa distances
#define DATA_HDR_LEN        14   // 10 header + 2 tag_id + 2 cycle
#define DATA_TAG_IDX        10
#define DATA_CYCLE_IDX      12
#define DATA_PAYLOAD_IDX    14
#define DATA_ANCHOR_STRIDE   6   // 2 anchor_id + 4 distance
#define DATA_MSG_LEN        (DATA_HDR_LEN + NUM_ANCHORS * DATA_ANCHOR_STRIDE) + 2

static uint8_t tx_poll_msg[POLL_MSG_LEN] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'I', 'O', 'V', 'E', 0xE0,
    0, 0, 0, 0,
    0, 0,
    0, 0, 0, 0, 0, 0
};
static uint8_t rx_resp_msg[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'I', 'O', 0xE1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
static const uint8_t token_hdr_ref[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'O', 'K', 'N', 0xE2
};
static const uint8_t data_hdr_ref[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'D', 'A', 'T', 'A', 0xE3
};

static uint8_t tx_token_msg[TOKEN_MSG_LEN];
static uint8_t tx_data_msg[DATA_MSG_LEN];

static uint8_t  frame_seq_nb      = 0;
static uint8_t  rx_buffer[RX_BUF_LEN];
static uint32_t status_reg        = 0;
static int      measurement_cycle = 0;

#define SPEED_OF_LIGHT 299702547

// ============================================================================
// JITTER
// ============================================================================
static inline uint8_t pseudo_rand_jitter(void)
{
    static uint8_t state = 0xA5;
    state ^= frame_seq_nb;
    state ^= (state << 3);
    state ^= (state >> 5);
    return state & 0x03;
}

// ============================================================================
// MSG HELPERS
// ============================================================================
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

// ============================================================================
// DATA SEND — TX distances cho bridge, không liên quan token ring
// ============================================================================
static void data_send(void)
{
    // Build header
    memcpy(tx_data_msg, data_hdr_ref, ALL_MSG_COMMON_LEN);
    tx_data_msg[ALL_MSG_SN_IDX]  = frame_seq_nb++;
    tx_data_msg[DATA_TAG_IDX]    = (uint8_t)(MY_INITIATOR_DEVICE_ID & 0xFF);
    tx_data_msg[DATA_TAG_IDX+1]  = (uint8_t)((MY_INITIATOR_DEVICE_ID >> 8) & 0xFF);
    tx_data_msg[DATA_CYCLE_IDX]  = (uint8_t)(measurement_cycle & 0xFF);
    tx_data_msg[DATA_CYCLE_IDX+1]= (uint8_t)((measurement_cycle >> 8) & 0xFF);

    // Encode distances
    for (int i = 0; i < NUM_ANCHORS; i++) {
        uint8_t *p = &tx_data_msg[DATA_PAYLOAD_IDX + i * DATA_ANCHOR_STRIDE];

        p[0] = (uint8_t)(anchor_ids[i] & 0xFF);
        p[1] = (uint8_t)((anchor_ids[i] >> 8) & 0xFF);

        int32_t d_raw;
        if (anchor_data[i].valid)
            d_raw = (int32_t)(anchor_data[i].distance * 10.0);
        else
            d_raw = (int32_t)0xFFFFFFFF;

        p[2] = (uint8_t)( d_raw        & 0xFF);
        p[3] = (uint8_t)((d_raw >>  8) & 0xFF);
        p[4] = (uint8_t)((d_raw >> 16) & 0xFF);
        p[5] = (uint8_t)((d_raw >> 24) & 0xFF);
    }

    // TX — không cần response
    dwt_forcetrxoff();
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
    dwt_writetxdata(DATA_MSG_LEN, tx_data_msg, 0);
    dwt_writetxfctrl(DATA_MSG_LEN, 0, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {}
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
}

// ============================================================================
// TOKEN SEND — giữ nguyên v2.0, 16 bytes
// ============================================================================
static void token_send(uint16_t to_tag_id)
{
    memcpy(tx_token_msg, token_hdr_ref, ALL_MSG_COMMON_LEN);
    tx_token_msg[TOKEN_FROM_IDX]    = (uint8_t)(MY_INITIATOR_DEVICE_ID & 0xFF);
    tx_token_msg[TOKEN_FROM_IDX+1]  = (uint8_t)((MY_INITIATOR_DEVICE_ID >> 8) & 0xFF);
    tx_token_msg[TOKEN_TO_IDX]      = (uint8_t)(to_tag_id & 0xFF);
    tx_token_msg[TOKEN_TO_IDX+1]    = (uint8_t)((to_tag_id >> 8) & 0xFF);
    tx_token_msg[TOKEN_CYCLE_IDX]   = (uint8_t)(measurement_cycle & 0xFF);
    tx_token_msg[TOKEN_CYCLE_IDX+1] = (uint8_t)((measurement_cycle >> 8) & 0xFF);

    for (int i = 0; i < TOKEN_SEND_REPEAT; i++) {
        tx_token_msg[ALL_MSG_SN_IDX] = frame_seq_nb++;

        dwt_forcetrxoff();
        dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
        dwt_writetxdata(TOKEN_MSG_LEN, tx_token_msg, 0);
        dwt_writetxfctrl(TOKEN_MSG_LEN, 0, 0);
        dwt_starttx(DWT_START_TX_IMMEDIATE);
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {}
        dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);

        if (TOKEN_SEND_REPEAT > 1)
            vTaskDelay(pdMS_TO_TICKS(TOKEN_SEND_GAP_MS));
    }
}

// ============================================================================
// TOKEN WAIT — giữ nguyên v2.0
// ============================================================================
static void token_wait(void)
{
    dwt_forcetrxoff();
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(MAX_TOKEN_WAIT_MS);

    while (1) {
        if (xTaskGetTickCount() >= deadline) {
            printf("[0x%04X] Token timeout — self-recover (cycle %d)\r\n",
                   MY_INITIATOR_DEVICE_ID, measurement_cycle);
            dwt_forcetrxoff();
            return;
        }

        status_reg = dwt_read32bitreg(SYS_STATUS_ID);

        if (status_reg & SYS_STATUS_RXFCG) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
            uint32_t flen = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
            if (flen <= RX_BUF_LEN) dwt_readrxdata(rx_buffer, flen, 0);
            rx_buffer[ALL_MSG_SN_IDX] = 0;

            if (memcmp(rx_buffer, token_hdr_ref, ALL_MSG_COMMON_LEN) == 0
                && flen >= TOKEN_MSG_LEN)
            {
                uint16_t to_id = (uint16_t)rx_buffer[TOKEN_TO_IDX]
                | ((uint16_t)rx_buffer[TOKEN_TO_IDX+1] << 8);
                if (to_id == MY_INITIATOR_DEVICE_ID) {
                    dwt_forcetrxoff();
                    return;
                }
            }
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }
        else if (status_reg & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
            dwt_write32bitreg(SYS_STATUS_ID,
                              SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            dwt_rxreset();
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }
        else {
            vTaskDelay(0);
        }
    }
}

// ============================================================================
// RANGING — giữ nguyên v2.0
// ============================================================================
static int do_ranging(uint32_t anchor_id, uint8_t idx)
{
    int retry_count = 0;

    anchor_data[idx].valid    = 0;
    anchor_data[idx].distance = 0.0;

    while (1) {
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        set_device_id(&tx_poll_msg[POLL_MSG_DEVICE_ID_IDX], anchor_id);

        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
            (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
            vTaskDelay(0);
            }
            frame_seq_nb++;

        if (!(status_reg & SYS_STATUS_RXFCG)) {
            dwt_forcetrxoff();
            dwt_write32bitreg(SYS_STATUS_ID,
                              SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            dwt_rxreset();
            uint32_t backoff_ms = (uint32_t)(retry_count < 4 ? retry_count : 4)
            + pseudo_rand_jitter();
            if (backoff_ms > 0) vTaskDelay(pdMS_TO_TICKS(backoff_ms));
            retry_count++;
            continue;
        }

        uint32_t flen;
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
        flen = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
        if (flen <= RX_BUF_LEN) dwt_readrxdata(rx_buffer, flen, 0);
        rx_buffer[ALL_MSG_SN_IDX] = 0;

        if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) != 0) {
            dwt_forcetrxoff();
            uint32_t backoff_ms = pseudo_rand_jitter();
            if (backoff_ms > 0) vTaskDelay(pdMS_TO_TICKS(backoff_ms));
            retry_count++;
            continue;
        }

        dwt_readdiagnostics(&anchor_data[idx].diagnostics);

        if (get_device_id(&rx_buffer[RESP_MSG_DEVICE_ID_IDX]) != anchor_id) {
            dwt_forcetrxoff();
            uint32_t backoff_ms = pseudo_rand_jitter();
            if (backoff_ms > 0) vTaskDelay(pdMS_TO_TICKS(backoff_ms));
            retry_count++;
            continue;
        }

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
        return 1;
    }
}

// ============================================================================
// MAIN LOOP
// ============================================================================
int ss_init_run(void)
{
    for (int i = 0; i < NUM_ANCHORS; i++) anchor_data[i].valid = 0;

    for (int i = 0; i < NUM_ANCHORS; i++)
        do_ranging(anchor_ids[i], i);

    measurement_cycle++;

    // In local để debug
    printf("[0x%04X] cyc=%d: %.1f %.1f %.1f\r\n",
           MY_INITIATOR_DEVICE_ID, measurement_cycle,
           anchor_data[0].distance,
           anchor_data[1].distance,
           anchor_data[2].distance);

    // Gửi DATA frame cho bridge
    data_send();

    // Gửi TOKEN frame cho tag tiếp theo
    token_send(NEXT_TAG_ID);
    token_wait();

    return 1;
}

// ============================================================================
// TASK ENTRY
// ============================================================================
void ss_initiator_task_function(void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);

    dwt_setleds(DWT_LEDS_ENABLE);

    tx_poll_msg[POLL_TAG_ID_IDX]   = (uint8_t)(MY_INITIATOR_DEVICE_ID & 0xFF);
    tx_poll_msg[POLL_TAG_ID_IDX+1] = (uint8_t)((MY_INITIATOR_DEVICE_ID >> 8) & 0xFF);

    printf("\r\n========================================\r\n");
    printf("Token Passing v2.3\r\n");
    printf("Tag 0x%04X | index %d / %d tags\r\n",
           MY_INITIATOR_DEVICE_ID, MY_TAG_INDEX, NUM_TAGS);
    printf("Next: 0x%04X | Anchors: %d\r\n", NEXT_TAG_ID, NUM_ANCHORS);
    printf("DATA frame: %d bytes | TOKEN frame: %d bytes\r\n",
           DATA_MSG_LEN, TOKEN_MSG_LEN);
    printf("========================================\r\n\r\n");

    for (int i = 0; i < NUM_ANCHORS; i++) anchor_data[i].valid = 0;

    if (MY_TAG_INDEX == 0) {
        printf("[0x%04X] Master — starting.\r\n", MY_INITIATOR_DEVICE_ID);
    } else {
        printf("[0x%04X] Waiting for token...\r\n", MY_INITIATOR_DEVICE_ID);
        token_wait();
        printf("[0x%04X] Got token.\r\n", MY_INITIATOR_DEVICE_ID);
    }

    while (1) ss_init_run();
}
