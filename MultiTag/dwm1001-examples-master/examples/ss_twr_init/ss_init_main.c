// ============================================================================
// Tag Initiator v2.4 — RING interrupt priority
//
// Thay đổi so với v2.3:
//   - RING frame từ bridge có priority cao nhất
//   - do_ranging() check g_ring_pending tại 3 checkpoint:
//       1. Trước khi TX poll
//       2. Trong wait-RX loop
//       3. Sau lỗi RX, trước backoff
//   - Khi ranging nhận được RING frame trực tiếp → abort ngay
//   - ss_init_run() restart ranging từ anchor 0 khi bị abort
//   - Tag gửi DATA ngay khi boot (alive signal cho bridge)
//   - Solo tag (ring_size < 2): free ranging, không dùng token
//
// RING frame layout (nhận từ bridge):
//   Byte  0– 9: header (0x41 0x88 SN 0xCA 0xDE 'R' 'I' 'N' 'G' 0xE4)
//   Byte 10   : ring_size (uint8)
//   Byte 11+  : tag_id list (ring_size × uint16 LE), thứ tự vòng ring
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
// CONFIG — chỉnh MY_TAG_ID cho mỗi node
// ============================================================================
#define MY_TAG_ID           0x567A      // ← thay đổi cho mỗi tag

#define NUM_ANCHORS         3
#define MAX_TOKEN_WAIT_MS   200
#define RX_BUF_LEN         64

#define ANCHOR_1    0x1001
#define ANCHOR_2    0x1002
#define ANCHOR_3    0x1003

static uint32_t anchor_ids[NUM_ANCHORS] = { ANCHOR_1, ANCHOR_2, ANCHOR_3 };

// ============================================================================
// RING STATE — cập nhật từ RING frame của bridge
// ============================================================================
#define RING_MAX_TAGS   8
#define RING_HDR_LEN    11   // 10 common + 1 byte size

static uint16_t ring_order[RING_MAX_TAGS];
static uint8_t  ring_size     = 0;
static uint16_t my_next_tag   = 0;
static uint8_t  in_ring       = 0;

// ── Volatile flag: set bất cứ khi nào nhận RING frame ──────────────────────
// do_ranging() check flag này tại mỗi checkpoint để thoát sớm
static volatile uint8_t  g_ring_pending = 0;
static volatile uint8_t  g_ring_buf[RING_MAX_TAGS * 2];
static volatile uint8_t  g_ring_size    = 0;

// ============================================================================
// RETURN CODES cho do_ranging()
// ============================================================================
#define RANGING_OK       1
#define RANGING_FAIL     0
#define RANGING_ABORTED -1   // bị RING frame interrupt

// ============================================================================
// FRAME DEFS
// ============================================================================
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
#define DATA_PAYLOAD_IDX    14
#define DATA_ANCHOR_STRIDE   6
#define DATA_HDR_LEN        14
#define DATA_MSG_LEN        (DATA_HDR_LEN + NUM_ANCHORS * DATA_ANCHOR_STRIDE + 2)

static const uint8_t token_hdr_ref[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'O', 'K', 'N', 0xE2
};
static const uint8_t data_hdr_ref[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'D', 'A', 'T', 'A', 0xE3
};
static const uint8_t ring_hdr_ref[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'R', 'I', 'N', 'G', 0xE4
};

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
static uint8_t tx_token_msg[TOKEN_MSG_LEN];
static uint8_t tx_data_msg[DATA_MSG_LEN];

static uint8_t  frame_seq_nb      = 0;
static uint8_t  rx_buffer[RX_BUF_LEN];
static uint32_t status_reg        = 0;
static int      measurement_cycle = 0;

typedef struct {
    uint8_t  valid;
    double   distance;   // mm
} anchor_data_t;

static anchor_data_t anchor_data[NUM_ANCHORS];

#define SPEED_OF_LIGHT 299702547

// ============================================================================
// HELPERS
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
    for (int i = 0; i < POLL_MSG_DEVICE_ID_LEN; i++)
        f[i] = (id >> (i * 8)) & 0xFF;
}

static inline uint8_t pseudo_rand_jitter(void)
{
    static uint8_t state = 0xA5;
    state ^= frame_seq_nb;
    state ^= (state << 3);
    state ^= (state >> 5);
    return state & 0x03;
}

// ============================================================================
// RING FRAME HANDLER
// Gọi khi nhận được frame có header RING, từ bất kỳ đâu (ranging/token_wait)
// Lưu vào volatile buffer, set flag — main loop apply sau
// ============================================================================
static void handle_ring_frame(const uint8_t *buf, uint32_t flen)
{
    if (flen < (uint32_t)(RING_HDR_LEN + 2)) return;
    uint8_t sz = buf[10];
    if (sz == 0 || sz > RING_MAX_TAGS) return;
    printf("[RING raw] flen=%lu: ", flen);
    for (uint32_t i = 0; i < flen && i < 20; i++)
        printf("%02X ", buf[i]);
    printf("\r\n");
    if (flen < (uint32_t)(RING_HDR_LEN + sz * 2)) return;

    g_ring_size = sz;
    for (int i = 0; i < sz * 2; i++)
        g_ring_buf[i] = buf[RING_HDR_LEN + i];

    // Set flag sau cùng — đảm bảo buffer đã ghi xong
    g_ring_pending = 1;
}

// ============================================================================
// APPLY RING UPDATE
// Gọi từ main loop — tính lại my_next_tag từ ring mới, xóa flag
// ============================================================================
static void apply_ring_update(void)
{
    uint8_t sz = g_ring_size;

    for (int i = 0; i < sz; i++)
        ring_order[i] = (uint16_t)g_ring_buf[i * 2]
                      | ((uint16_t)g_ring_buf[i * 2 + 1] << 8);
    ring_size = sz;

    my_next_tag = 0;
    in_ring     = 0;
    for (int i = 0; i < sz; i++) {
        if (ring_order[i] == MY_TAG_ID) {
            my_next_tag = ring_order[(i + 1) % sz];
            in_ring     = 1;
            break;
        }
    }

    // Xóa flag sau khi đã apply xong
    g_ring_pending = 0;

    printf("[0x%04X] Ring applied: size=%d in_ring=%d next=0x%04X\r\n",
           MY_TAG_ID, ring_size, in_ring, my_next_tag);
    printf("[0x%04X] Order: ", MY_TAG_ID);
    for (int i = 0; i < sz; i++)
        printf("0x%04X%s", ring_order[i], i < sz - 1 ? "->" : "\r\n");
}

// ============================================================================
// DO RANGING — v2.4
// Check g_ring_pending tại 4 checkpoint để abort kịp thời
// Return: RANGING_OK | RANGING_FAIL | RANGING_ABORTED
// ============================================================================
static int do_ranging(uint32_t anchor_id, uint8_t idx)
{
    int retry_count = 0;

    anchor_data[idx].valid    = 0;
    anchor_data[idx].distance = 0.0;

    while (1) {
        // ── CHECKPOINT 1: trước khi TX poll ──────────────────────────────
        if (g_ring_pending) {
            dwt_forcetrxoff();
            return RANGING_ABORTED;
        }

        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        set_device_id(&tx_poll_msg[POLL_MSG_DEVICE_ID_IDX], anchor_id);

        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        // ── WAIT LOOP: chờ response từ anchor ────────────────────────────
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
                 (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
        {
            // ── CHECKPOINT 2: trong wait loop ────────────────────────────
            if (g_ring_pending) {
                dwt_forcetrxoff();
                frame_seq_nb++;
                return RANGING_ABORTED;
            }
            vTaskDelay(0);
        }
        frame_seq_nb++;

        // RX error / timeout
        if (!(status_reg & SYS_STATUS_RXFCG)) {
            dwt_forcetrxoff();
            dwt_write32bitreg(SYS_STATUS_ID,
                              SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            dwt_rxreset();

            // ── CHECKPOINT 3: sau lỗi, trước backoff ─────────────────────
            if (g_ring_pending) return RANGING_ABORTED;

            uint32_t backoff_ms = (uint32_t)(retry_count < 4 ? retry_count : 4)
                                + pseudo_rand_jitter();
            if (backoff_ms > 0) vTaskDelay(pdMS_TO_TICKS(backoff_ms));
            retry_count++;
            continue;
        }

        // Đọc frame nhận được
        uint32_t flen;
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
        flen = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
        if (flen <= RX_BUF_LEN) dwt_readrxdata(rx_buffer, flen, 0);
        rx_buffer[ALL_MSG_SN_IDX] = 0;

        // ── CHECKPOINT 4: nhận được RING frame trực tiếp ─────────────────
        // Bridge gửi trong lúc tag đang ranging → abort ngay lập tức
        if (memcmp(rx_buffer, ring_hdr_ref, ALL_MSG_COMMON_LEN) == 0) {
            handle_ring_frame(rx_buffer, flen);
            dwt_forcetrxoff();
            return RANGING_ABORTED;
        }

        // Không phải response mong đợi
        if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) != 0) {
            dwt_forcetrxoff();
            if (g_ring_pending) return RANGING_ABORTED;
            retry_count++;
            continue;
        }

        // Không phải từ anchor đúng
        if (get_device_id(&rx_buffer[RESP_MSG_DEVICE_ID_IDX]) != anchor_id) {
            dwt_forcetrxoff();
            if (g_ring_pending) return RANGING_ABORTED;
            retry_count++;
            continue;
        }

        // ── Tính distance ─────────────────────────────────────────────────
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
}

// ============================================================================
// DATA SEND — gửi distances cho bridge
// ============================================================================
static void data_send(void)
{
    memcpy(tx_data_msg, data_hdr_ref, ALL_MSG_COMMON_LEN);
    tx_data_msg[ALL_MSG_SN_IDX]    = frame_seq_nb++;
    tx_data_msg[DATA_TAG_IDX]      = (uint8_t)(MY_TAG_ID & 0xFF);
    tx_data_msg[DATA_TAG_IDX + 1]  = (uint8_t)((MY_TAG_ID >> 8) & 0xFF);
    tx_data_msg[DATA_CYCLE_IDX]    = (uint8_t)(measurement_cycle & 0xFF);
    tx_data_msg[DATA_CYCLE_IDX + 1]= (uint8_t)((measurement_cycle >> 8) & 0xFF);

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

    dwt_forcetrxoff();
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
    dwt_writetxdata(DATA_MSG_LEN, tx_data_msg, 0);
    dwt_writetxfctrl(DATA_MSG_LEN, 0, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {}
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
}

// ============================================================================
// TOKEN SEND
// ============================================================================
static void token_send(uint16_t to_tag_id)
{
    memcpy(tx_token_msg, token_hdr_ref, ALL_MSG_COMMON_LEN);
    tx_token_msg[ALL_MSG_SN_IDX]     = frame_seq_nb++;
    tx_token_msg[TOKEN_FROM_IDX]     = (uint8_t)(MY_TAG_ID & 0xFF);
    tx_token_msg[TOKEN_FROM_IDX + 1] = (uint8_t)((MY_TAG_ID >> 8) & 0xFF);
    tx_token_msg[TOKEN_TO_IDX]       = (uint8_t)(to_tag_id & 0xFF);
    tx_token_msg[TOKEN_TO_IDX + 1]   = (uint8_t)((to_tag_id >> 8) & 0xFF);
    tx_token_msg[TOKEN_CYCLE_IDX]    = (uint8_t)(measurement_cycle & 0xFF);
    tx_token_msg[TOKEN_CYCLE_IDX + 1]= (uint8_t)((measurement_cycle >> 8) & 0xFF);

    dwt_forcetrxoff();
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
    dwt_writetxdata(TOKEN_MSG_LEN, tx_token_msg, 0);
    dwt_writetxfctrl(TOKEN_MSG_LEN, 0, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {}
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
}

// ============================================================================
// TOKEN WAIT — v2.4
// Xử lý cả RING frame trong khi đợi token
// RING frame → apply ngay, reset deadline, tiếp tục đợi token
// ============================================================================
static void token_wait(void)
{
    dwt_forcetrxoff();
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(MAX_TOKEN_WAIT_MS);

    while (1) {
        // Nếu RING pending từ ranging lần trước → apply ngay
        if (g_ring_pending) {
            apply_ring_update();
            deadline = xTaskGetTickCount() + pdMS_TO_TICKS(MAX_TOKEN_WAIT_MS);
        }

        if (xTaskGetTickCount() >= deadline) {
            printf("[0x%04X] Token timeout — self-recover (cycle %d)\r\n",
                   MY_TAG_ID, measurement_cycle);
            dwt_forcetrxoff();
            return;
        }

        status_reg = dwt_read32bitreg(SYS_STATUS_ID);

        if (status_reg & SYS_STATUS_RXFCG) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
            uint32_t flen = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
            if (flen <= RX_BUF_LEN) dwt_readrxdata(rx_buffer, flen, 0);
            rx_buffer[ALL_MSG_SN_IDX] = 0;

            // RING frame — priority cao nhất, apply ngay
            if (memcmp(rx_buffer, ring_hdr_ref, ALL_MSG_COMMON_LEN) == 0) {
                handle_ring_frame(rx_buffer, flen);
                apply_ring_update();
                // Reset deadline sau khi nhận ring mới
                deadline = xTaskGetTickCount() + pdMS_TO_TICKS(MAX_TOKEN_WAIT_MS);
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                continue;
            }

            // TOKEN frame — kiểm tra có phải cho mình không
            if (memcmp(rx_buffer, token_hdr_ref, ALL_MSG_COMMON_LEN) == 0
                && flen >= TOKEN_MSG_LEN)
            {
                uint16_t to_id = (uint16_t)rx_buffer[TOKEN_TO_IDX]
                               | ((uint16_t)rx_buffer[TOKEN_TO_IDX + 1] << 8);
                if (to_id == MY_TAG_ID) {
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
// MAIN LOOP
// ============================================================================
int ss_init_run(void)
{
    uint8_t ring_was_aborted = 0;

    // Reset anchor data đầu mỗi cycle
    for (int i = 0; i < NUM_ANCHORS; i++) anchor_data[i].valid = 0;

    // Ranging tất cả anchors — restart từ đầu nếu bị RING interrupt
    for (int i = 0; i < NUM_ANCHORS; i++) {
        int result = do_ranging(anchor_ids[i], i);

        if (result == RANGING_ABORTED) {
            // Apply ring update (nếu chưa apply — g_ring_pending vẫn set)
            if (g_ring_pending) apply_ring_update();
            ring_was_aborted = 1;

            printf("[0x%04X] Ranging aborted at anchor %d, ring applied, restarting\r\n",
                   MY_TAG_ID, i);

            // Restart ranging từ anchor 0
            for (int j = 0; j < NUM_ANCHORS; j++) anchor_data[j].valid = 0;
            i = -1;  // for loop sẽ i++ → i = 0
            ring_was_aborted = 0;  // chỉ cần report 1 lần
            continue;
        }
        // RANGING_OK hoặc RANGING_FAIL — tiếp tục anchor tiếp theo
    }

    measurement_cycle++;

    // Debug local
    printf("[0x%04X] cyc=%d in_ring=%d: %.1f %.1f %.1f\r\n",
           MY_TAG_ID, measurement_cycle, in_ring,
           anchor_data[0].distance,
           anchor_data[1].distance,
           anchor_data[2].distance);

    // Gửi DATA cho bridge (alive signal + distances)
    data_send();

    // Nếu không trong ring → free ranging, không dùng token
    if (!in_ring || ring_size < 2) {
        // Vẫn lắng nghe RING frame qua token_wait với timeout ngắn
        // (dùng lại token_wait vì nó đã xử lý RING frame)
        vTaskDelay(pdMS_TO_TICKS(10));
        return 1;
    }

    // Trong ring → pass token và đợi token tiếp theo
    token_send(my_next_tag);
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

    // Setup poll message tag ID
    tx_poll_msg[POLL_TAG_ID_IDX]     = (uint8_t)(MY_TAG_ID & 0xFF);
    tx_poll_msg[POLL_TAG_ID_IDX + 1] = (uint8_t)((MY_TAG_ID >> 8) & 0xFF);

    for (int i = 0; i < NUM_ANCHORS; i++) anchor_data[i].valid = 0;

    printf("\r\n========================================\r\n");
    printf("Tag Initiator v2.4\r\n");
    printf("Tag ID: 0x%04X | Anchors: %d\r\n", MY_TAG_ID, NUM_ANCHORS);
    printf("RING interrupt: enabled\r\n");
    printf("========================================\r\n\r\n");

    // ── Boot sequence ────────────────────────────────────────────────────────
    // Bước 1: Ranging ngay để có data
    printf("[0x%04X] Boot ranging...\r\n", MY_TAG_ID);
    for (int i = 0; i < NUM_ANCHORS; i++) {
        int result = do_ranging(anchor_ids[i], i);
        if (result == RANGING_ABORTED) {
            if (g_ring_pending) apply_ring_update();
            break;
        }
    }

    // Bước 2: Gửi DATA ngay — alive signal cho bridge
    printf("[0x%04X] Sending alive signal to bridge...\r\n", MY_TAG_ID);
    data_send();

    // Bước 3: Đợi RING từ bridge (hoặc vào free ranging nếu không có)
    printf("[0x%04X] Waiting for RING frame...\r\n", MY_TAG_ID);
    token_wait();  // xử lý RING trong này, timeout tự thoát

    if (in_ring)
        printf("[0x%04X] Joined ring, next=0x%04X\r\n", MY_TAG_ID, my_next_tag);
    else
        printf("[0x%04X] No ring, free ranging\r\n", MY_TAG_ID);

    // ── Main loop ────────────────────────────────────────────────────────────
    while (1) ss_init_run();
}