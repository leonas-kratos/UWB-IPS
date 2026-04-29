// ============================================================================
// Bridge Node v4.0
//
// Thay đổi so với v3.0:
//
//  [RHYTHM-1] Phát hiện ring vỡ nhịp bằng cách so sánh tag_id của DATA frame
//             liên tiếp với thứ tự ring đã biết (current_ring[]).
//             Nếu tag gửi DATA không đúng thứ tự → tăng disorder_count.
//             Sai 2 lần liên tiếp (DISORDER_THRESHOLD) → trigger rebuild & resend RING.
//
//  [RHYTHM-2] Phân biệt 2 loại rebuild:
//             - Membership change (tag mới/tag chết) → rebuild + gửi RING
//             - Rhythm disorder (thứ tự sai, membership không đổi) → chỉ resend RING hiện tại
//             Không gộp chung → tránh thay đổi ring order không cần thiết.
//
//  [RHYTHM-3] expected_next_idx: bridge theo dõi index của tag tiếp theo
//             được kỳ vọng gửi DATA trong current_ring[]. Reset về 0 khi
//             nhận đúng, tăng lỗi khi sai.
//
//  [FIX] Giữ lại tất cả fix từ v3.0:
//        - Frame queue tách RX/process
//        - Periodic timeout check
//        - Cooldown chống storm
// ============================================================================

#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "nrf.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"

// ============================================================================
// CONFIG
// ============================================================================
#define NUM_ANCHORS             3
#define MAX_TAGS                8
#define RX_BUF_LEN              80
#define FRAME_QUEUE_LEN         8

#define TAG_TIMEOUT_MS          1500    // ms không nhận DATA → tag dead
#define RING_COOLDOWN_MS        10     // cooldown giữa 2 lần gửi RING

// [RHYTHM-1] Sai thứ tự liên tiếp bao nhiêu lần thì resend RING
#define DISORDER_THRESHOLD      2

// ============================================================================
// FRAME HEADER REFS
// ============================================================================
static const uint8_t data_hdr_ref[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'D', 'A', 'T', 'A', 0xE3
};
static const uint8_t ring_hdr_ref[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'R', 'I', 'N', 'G', 0xE4
};

// ============================================================================
// FRAME INDICES
// ============================================================================
#define ALL_MSG_SN_IDX          2
#define ALL_MSG_COMMON_LEN      10
#define DATA_TAG_IDX            10
#define DATA_CYCLE_IDX          12
#define DATA_PAYLOAD_IDX        14
#define DATA_ANCHOR_STRIDE      6
#define DATA_HDR_LEN            14
#define DATA_MSG_MIN_LEN        (DATA_HDR_LEN + NUM_ANCHORS * DATA_ANCHOR_STRIDE)

#define RING_SIZE_IDX           10      // byte 10: ring_size
#define RING_TAGS_IDX           11      // byte 11+: tag_ids
#define RING_HDR_LEN            11      // 10 common + 1 byte size
#define RING_MAX_TAGS           8
#define RING_MSG_MAX_LEN        (RING_HDR_LEN + RING_MAX_TAGS * 2 + 2)

// ============================================================================
// ALIVE TABLE
// ============================================================================
typedef struct {
    uint16_t tag_id;
    uint32_t last_seen_tick;
    uint8_t  in_ring;
} tag_entry_t;

static tag_entry_t alive_table[MAX_TAGS];
static uint8_t     alive_count = 0;

// ============================================================================
// RING STATE
// ============================================================================
static uint16_t current_ring[RING_MAX_TAGS];
static uint8_t  current_ring_size = 0;

// [RHYTHM-3] Theo dõi thứ tự: index trong current_ring[] của tag tiếp theo
// được kỳ vọng gửi DATA
static int8_t   expected_next_idx  = -1;    // -1 = chưa học được thứ tự
static uint8_t  disorder_count     = 0;     // số lần sai thứ tự liên tiếp

// ============================================================================
// FRAME QUEUE
// ============================================================================
typedef struct {
    uint8_t  buf[RX_BUF_LEN];
    uint32_t len;
    uint8_t  valid;
} frame_slot_t;

static frame_slot_t frame_queue[FRAME_QUEUE_LEN];
static uint8_t q_write = 0;
static uint8_t q_read  = 0;

// ============================================================================
// STATE
// ============================================================================
static uint8_t  frame_seq_nb      = 0;
static uint32_t last_ring_tx_tick = 0;

// Stats
static uint32_t total_rx          = 0;
static uint32_t valid_rx          = 0;
static uint32_t invalid_rx        = 0;
static uint32_t dropped           = 0;
static uint32_t ring_tx_count     = 0;
static uint32_t disorder_triggers = 0;

// ============================================================================
// HELPERS
// ============================================================================
static uint16_t decode_u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int32_t decode_i32_le(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0]
                   | ((uint32_t)p[1] << 8)
                   | ((uint32_t)p[2] << 16)
                   | ((uint32_t)p[3] << 24));
}

// Tìm index của tag_id trong current_ring[], trả về -1 nếu không có
static int ring_index_of(uint16_t tag_id)
{
    for (int i = 0; i < current_ring_size; i++)
        if (current_ring[i] == tag_id) return i;
    return -1;
}

// ============================================================================
// RING TX — build và broadcast RING frame, về lại RX ngay
// ============================================================================
static void ring_send(void)
{
    uint32_t now = xTaskGetTickCount();

    // Cooldown check
    if ((now - last_ring_tx_tick) < pdMS_TO_TICKS(RING_COOLDOWN_MS)) {
        printf("[Bridge] RING TX skipped (cooldown)\r\n");
        return;
    }

    uint8_t n = current_ring_size;
    uint8_t tx_buf[RING_MSG_MAX_LEN];
    uint8_t payload_len = RING_HDR_LEN + n * 2;
    uint8_t msg_len     = payload_len + 2;  // +2 FCS

    memcpy(tx_buf, ring_hdr_ref, ALL_MSG_COMMON_LEN);
    tx_buf[ALL_MSG_SN_IDX]  = frame_seq_nb++;
    tx_buf[RING_SIZE_IDX]   = n;
    for (int i = 0; i < n; i++) {
        tx_buf[RING_TAGS_IDX + i * 2]     = (uint8_t)(current_ring[i] & 0xFF);
        tx_buf[RING_TAGS_IDX + i * 2 + 1] = (uint8_t)((current_ring[i] >> 8) & 0xFF);
    }
    tx_buf[payload_len]     = 0x00;
    tx_buf[payload_len + 1] = 0x00;

    dwt_forcetrxoff();
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
    dwt_writetxdata(msg_len, tx_buf, 0);
    dwt_writetxfctrl(msg_len, 0, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {}
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

    last_ring_tx_tick = xTaskGetTickCount();
    ring_tx_count++;

    printf("[Bridge] RING TX #%lu: size=%d [", (unsigned long)ring_tx_count, n);
    for (int i = 0; i < n; i++)
        printf("0x%04X%s", current_ring[i], i < n - 1 ? "→" : "");
    printf("]\r\n");

    // Reset rhythm tracking sau mỗi lần gửi RING
    // Tags sẽ sắp xếp lại, thứ tự sẽ khác → không thể dùng expected_next_idx cũ
    expected_next_idx = -1;
    disorder_count    = 0;

    // Về lại RX ngay
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

// ============================================================================
// RING REBUILD — tính lại ring từ alive table, gửi nếu membership thay đổi
// [RHYTHM-2] Chỉ thay đổi current_ring khi membership thực sự thay đổi
// ============================================================================
static void rebuild_ring_if_needed(void)
{
    uint32_t now = xTaskGetTickCount();

    // Thu thập các tag còn alive
    uint16_t new_ring[RING_MAX_TAGS];
    uint8_t  new_size = 0;

    for (int i = 0; i < alive_count; i++) {
        uint32_t age = now - alive_table[i].last_seen_tick;
        if (age < pdMS_TO_TICKS(TAG_TIMEOUT_MS)) {
            if (new_size < RING_MAX_TAGS)
                new_ring[new_size++] = alive_table[i].tag_id;
        }
    }

    // So sánh membership với ring hiện tại (không quan tâm thứ tự ở đây)
    uint8_t membership_changed = 0;
    if (new_size != current_ring_size) {
        membership_changed = 1;
    } else {
        // Kiểm tra từng tag trong new_ring có trong current_ring không
        for (int i = 0; i < new_size; i++) {
            if (ring_index_of(new_ring[i]) < 0) {
                membership_changed = 1;
                break;
            }
        }
    }

    if (!membership_changed) return;

    // Cập nhật in_ring flag
    for (int i = 0; i < alive_count; i++) {
        alive_table[i].in_ring = 0;
        for (int j = 0; j < new_size; j++) {
            if (new_ring[j] == alive_table[i].tag_id) {
                alive_table[i].in_ring = 1;
                break;
            }
        }
    }

    // Log thay đổi
    if (new_size < current_ring_size) {
        // Tìm tag bị xóa
        for (int i = 0; i < current_ring_size; i++) {
            int found = 0;
            for (int j = 0; j < new_size; j++) {
                if (new_ring[j] == current_ring[i]) { found = 1; break; }
            }
            if (!found)
                printf("[Bridge] Tag 0x%04X timed out — removed from ring\r\n",
                       current_ring[i]);
        }
    } else if (new_size > current_ring_size) {
        for (int j = 0; j < new_size; j++) {
            if (ring_index_of(new_ring[j]) < 0)
                printf("[Bridge] Tag 0x%04X added to ring\r\n", new_ring[j]);
        }
    }

    memcpy(current_ring, new_ring, new_size * sizeof(uint16_t));
    current_ring_size = new_size;

    if (new_size >= 2) {
        printf("[Bridge] Membership changed → sending RING\r\n");
        ring_send();
    } else {
        last_ring_tx_tick = now;
        expected_next_idx = -1;
        disorder_count    = 0;
        if (new_size == 1)
        {
            printf("[Bridge] 1 tag alive (0x%04X) — free ranging, no RING\r\n",
                   new_ring[0]);
        }
        else
            printf("[Bridge] No tags alive\r\n");
    }
}

// ============================================================================
// RHYTHM CHECK — [RHYTHM-1][RHYTHM-2][RHYTHM-3]
// Gọi sau mỗi DATA frame hợp lệ để kiểm tra thứ tự gửi DATA có đúng ring không
//
// Logic:
//   - Nếu ring size < 2 → không check (không có ring)
//   - Nếu expected_next_idx == -1 → học tag đầu tiên làm baseline
//   - Nếu tag_id == current_ring[expected_next_idx] → đúng thứ tự, reset disorder
//   - Nếu sai → tăng disorder_count
//   - disorder_count >= DISORDER_THRESHOLD → resend RING hiện tại (không rebuild)
// ============================================================================
static void check_rhythm(uint16_t tag_id)
{
    // Không check khi ring chưa đủ tag
    if (current_ring_size < 2) {
        expected_next_idx = -1;
        disorder_count    = 0;
        return;
    }

    int tag_pos = ring_index_of(tag_id);
    if (tag_pos < 0) {
        // Tag này không có trong ring — bỏ qua, rebuild sẽ handle
        return;
    }

    if (expected_next_idx < 0) {
        // Lần đầu nhận DATA sau RING → học baseline
        expected_next_idx = (tag_pos + 1) % current_ring_size;
        disorder_count    = 0;
        printf("[Bridge] Rhythm baseline: first=0x%04X, next expected=0x%04X\r\n",
               tag_id, current_ring[expected_next_idx]);
        return;
    }

    if (tag_pos == expected_next_idx) {
        // Đúng thứ tự → advance, reset disorder
        disorder_count    = 0;
        expected_next_idx = (expected_next_idx + 1) % current_ring_size;
    } else {
        // Sai thứ tự
        disorder_count++;
        printf("[Bridge] Rhythm disorder #%d: expected 0x%04X got 0x%04X\r\n",
               disorder_count,
               current_ring[expected_next_idx],
               tag_id);

        if (disorder_count >= DISORDER_THRESHOLD) {
            disorder_triggers++;
            printf("[Bridge] Disorder threshold reached (%d) — resending RING #%lu\r\n",
                   DISORDER_THRESHOLD, (unsigned long)disorder_triggers);
            // [RHYTHM-2] Chỉ resend RING hiện tại, KHÔNG rebuild membership
            // ring_send() sẽ tự reset expected_next_idx và disorder_count
            ring_send();
        } else {
            // Chưa đủ ngưỡng — update expected sang tag vừa nhận + 1
            // để tránh treo nếu ring đang recover dở
            expected_next_idx = (tag_pos + 1) % current_ring_size;
        }
    }
}

// ============================================================================
// ON DATA FRAME — cập nhật alive table, check rhythm, check membership
// ============================================================================
static void process_data_frame(const uint8_t *buf, uint32_t flen)
{
    if (flen < (uint32_t)DATA_MSG_MIN_LEN) {
        invalid_rx++;
        return;
    }

    uint16_t tag_id = decode_u16_le(&buf[DATA_TAG_IDX]);
    uint16_t cycle  = decode_u16_le(&buf[DATA_CYCLE_IDX]);
    uint32_t now    = xTaskGetTickCount();

    valid_rx++;

    // --- Cập nhật alive table ---
    int found = -1;
    for (int i = 0; i < alive_count; i++) {
        if (alive_table[i].tag_id == tag_id) { found = i; break; }
    }
    if (found < 0 && alive_count < MAX_TAGS) {
        found = alive_count++;
        alive_table[found].tag_id  = tag_id;
        alive_table[found].in_ring = 0;
        printf("[Bridge] New tag detected: 0x%04X\r\n", tag_id);
    }
    if (found >= 0) alive_table[found].last_seen_tick = now;

    // --- In data ra UART ---
    printf("0x%04X", tag_id);
    for (int i = 0; i < NUM_ANCHORS; i++) {
        const uint8_t *p = &buf[DATA_PAYLOAD_IDX + i * DATA_ANCHOR_STRIDE];
        int32_t d_raw = decode_i32_le(p + 2);
        if (d_raw == (int32_t)0xFFFFFFFF)
            printf(",-1");
        else
            printf(",%.0f", d_raw / 10.0);
    }
    printf("\r\n");

    // --- [RHYTHM-1] Check thứ tự trước ---
    check_rhythm(tag_id);

    // --- Check membership (có tag mới không, cooldown bên trong) ---
    rebuild_ring_if_needed();
}

// ============================================================================
// BRIDGE RX LOOP
// ============================================================================
static void bridge_rx_loop(void)
{
    dwt_forcetrxoff();
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    TickType_t last_timeout_check = xTaskGetTickCount();

    while (1) {
        uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID);

        // ── Phase 1: đọc frame vào queue NGAY ────────────────────────────
        if (status_reg & SYS_STATUS_RXFCG) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

            uint32_t flen = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;

            if (flen > 0 && flen <= RX_BUF_LEN) {
                frame_slot_t *slot = &frame_queue[q_write];
                if (slot->valid) {
                    dropped++;
                } else {
                    memset(slot->buf, 0, RX_BUF_LEN);
                    dwt_readrxdata(slot->buf, flen, 0);
                    slot->len   = flen;
                    slot->valid = 1;
                    q_write = (q_write + 1) % FRAME_QUEUE_LEN;
                }
            } else {
                invalid_rx++;
            }

            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            total_rx++;
        }
        else if (status_reg & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
            dwt_write32bitreg(SYS_STATUS_ID,
                              SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            dwt_rxreset();
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }

        // ── Phase 2: process 1 frame từ queue ────────────────────────────
        frame_slot_t *rslot = &frame_queue[q_read];
        if (rslot->valid) {
            uint8_t saved_sn = rslot->buf[ALL_MSG_SN_IDX];
            rslot->buf[ALL_MSG_SN_IDX] = 0;

            if (memcmp(rslot->buf, data_hdr_ref, ALL_MSG_COMMON_LEN) == 0)
                process_data_frame(rslot->buf, rslot->len);
            // Token, POLL, RESP → bỏ qua yên lặng

            rslot->buf[ALL_MSG_SN_IDX] = saved_sn;
            rslot->valid = 0;
            q_read = (q_read + 1) % FRAME_QUEUE_LEN;
        }

        // ── Phase 3: periodic timeout check (mỗi RING_COOLDOWN_MS) ───────
        uint32_t now = xTaskGetTickCount();
        if ((now - last_timeout_check) >= pdMS_TO_TICKS(RING_COOLDOWN_MS)) {
            last_timeout_check = now;
            rebuild_ring_if_needed();
        }

        vTaskDelay(0);
    }
}

// ============================================================================
// TASK ENTRY
// ============================================================================
void bridge_task_function(void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);

    dwt_setleds(DWT_LEDS_ENABLE);

    memset(frame_queue,  0, sizeof(frame_queue));
    memset(alive_table,  0, sizeof(alive_table));
    memset(current_ring, 0, sizeof(current_ring));

    expected_next_idx = -1;
    disorder_count    = 0;

    printf("\r\n========================================\r\n");
    printf("Bridge Node v4.0\r\n");
    printf("RX-only | TX only for RING frames\r\n");
    printf("Anchors: %d | Max tags: %d\r\n", NUM_ANCHORS, MAX_TAGS);
    printf("Tag timeout: %dms | Cooldown: %dms\r\n", TAG_TIMEOUT_MS, RING_COOLDOWN_MS);
    printf("Disorder threshold: %d consecutive errors\r\n", DISORDER_THRESHOLD);
    printf("========================================\r\n\r\n");

    bridge_rx_loop();
}