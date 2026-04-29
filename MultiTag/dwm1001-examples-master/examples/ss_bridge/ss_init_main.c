// ============================================================================
// Bridge Node v4.1
//
// Thay đổi so với v4.0:
//
//  [SYNC-1] Đọc ring_size từ DATA frame (byte 14).
//           Lưu tag_ring_size vào alive_table cho từng tag.
//
//  [SYNC-2] Sau mỗi rebuild/resend, nếu bất kỳ tag nào báo ring_size khác
//           current_ring_size → resend RING mỗi RING_RESYNC_MS (10ms)
//           cho đến khi tất cả tag báo đúng.
//
//  [FIX] DATA frame offset v2.5: DATA_RINGSIZE_IDX=14, DATA_PAYLOAD_IDX=15
// ============================================================================

#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "nrf.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"

#define NUM_ANCHORS             3
#define MAX_TAGS                8
#define RX_BUF_LEN              80
#define FRAME_QUEUE_LEN         8

#define TAG_TIMEOUT_MS          3000
#define RING_COOLDOWN_MS        10
#define RING_RESYNC_MS          50      // interval resend khi có tag chưa sync
#define DISORDER_THRESHOLD      50

static const uint8_t data_hdr_ref[] = { 0x41,0x88,0,0xCA,0xDE,'D','A','T','A',0xE3 };
static const uint8_t ring_hdr_ref[] = { 0x41,0x88,0,0xCA,0xDE,'R','I','N','G',0xE4 };

#define ALL_MSG_SN_IDX          2
#define ALL_MSG_COMMON_LEN      10

// DATA frame v2.5
#define DATA_TAG_IDX            10
#define DATA_CYCLE_IDX          12
#define DATA_RINGSIZE_IDX       14      // ← ring_size từ tag (1 byte)
#define DATA_PAYLOAD_IDX        15
#define DATA_ANCHOR_STRIDE      6
#define DATA_HDR_LEN            15
#define DATA_MSG_MIN_LEN        (DATA_HDR_LEN + NUM_ANCHORS * DATA_ANCHOR_STRIDE)

// RING frame — giữ nguyên v4.0
#define RING_SIZE_IDX           10
#define RING_TAGS_IDX           11
#define RING_HDR_LEN            11
#define RING_MAX_TAGS           8
#define RING_MSG_MAX_LEN        (RING_HDR_LEN + RING_MAX_TAGS * 2 + 2)

// ============================================================================
// ALIVE TABLE
// ============================================================================
typedef struct {
    uint16_t tag_id;
    uint32_t last_seen_tick;
    uint8_t  tag_ring_size;     // [SYNC-1] ring_size tag đang dùng
    uint8_t  in_ring;
} tag_entry_t;

static tag_entry_t alive_table[MAX_TAGS];
static uint8_t     alive_count = 0;

// ============================================================================
// RING STATE
// ============================================================================
static uint16_t current_ring[RING_MAX_TAGS];
static uint8_t  current_ring_size = 0;

static int8_t  expected_next_idx = -1;
static uint8_t disorder_count   = 0;

// [SYNC-2] Resync state
static uint8_t  resync_active    = 0;
static uint32_t last_resync_tick = 0;

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

static uint8_t  frame_seq_nb      = 0;
static uint32_t last_ring_tx_tick = 0;
static uint32_t total_rx          = 0;
static uint32_t ring_tx_count     = 0;

// ============================================================================
// HELPERS
// ============================================================================
static uint16_t decode_u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static int32_t decode_i32_le(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1]<<8)
                   | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24));
}
static int ring_index_of(uint16_t tag_id)
{
    for (int i = 0; i < current_ring_size; i++)
        if (current_ring[i] == tag_id) return i;
    return -1;
}

// ============================================================================
// [SYNC-2] Kiểm tra tất cả tag trong ring đã dùng đúng ring_size chưa
// ============================================================================
static uint8_t all_tags_synced(void)
{
    if (current_ring_size < 2) return 1;
    for (int i = 0; i < alive_count; i++) {
        if (!alive_table[i].in_ring) continue;
        if (alive_table[i].tag_ring_size != current_ring_size) return 0;
    }
    return 1;
}

// ============================================================================
// RING TX
// ============================================================================
static void ring_send(void)
{
    uint32_t now = xTaskGetTickCount();
    if ((now - last_ring_tx_tick) < pdMS_TO_TICKS(RING_COOLDOWN_MS)) return;

    uint8_t n = current_ring_size;
    uint8_t tx_buf[RING_MSG_MAX_LEN];
    uint8_t payload_len = RING_HDR_LEN + n * 2;
    uint8_t msg_len     = payload_len + 2;

    memcpy(tx_buf, ring_hdr_ref, ALL_MSG_COMMON_LEN);
    tx_buf[ALL_MSG_SN_IDX] = frame_seq_nb++;
    tx_buf[RING_SIZE_IDX]  = n;
    for (int i = 0; i < n; i++) {
        tx_buf[RING_TAGS_IDX + i*2]   = (uint8_t)(current_ring[i] & 0xFF);
        tx_buf[RING_TAGS_IDX + i*2+1] = (uint8_t)((current_ring[i] >> 8) & 0xFF);
    }
    tx_buf[payload_len] = 0x00; tx_buf[payload_len+1] = 0x00;

    dwt_forcetrxoff();
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
    dwt_writetxdata(msg_len, tx_buf, 0);
    dwt_writetxfctrl(msg_len, 0, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {}
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

    last_ring_tx_tick = xTaskGetTickCount();
    ring_tx_count++;

    printf("[Bridge] RING TX #%lu: size=%d [",
           (unsigned long)ring_tx_count, n);
    for (int i = 0; i < n; i++)
        printf("0x%04X%s", current_ring[i], i < n-1 ? "->" : "");
    printf("]\r\n");

    expected_next_idx = -1;
    disorder_count    = 0;

    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

// ============================================================================
// RING REBUILD
// ============================================================================
static void rebuild_ring_if_needed(void)
{
    uint32_t now = xTaskGetTickCount();
    uint16_t new_ring[RING_MAX_TAGS];
    uint8_t  new_size = 0;

    for (int i = 0; i < alive_count; i++) {
        if ((now - alive_table[i].last_seen_tick) < pdMS_TO_TICKS(TAG_TIMEOUT_MS))
            if (new_size < RING_MAX_TAGS) new_ring[new_size++] = alive_table[i].tag_id;
    }

    uint8_t changed = (new_size != current_ring_size);
    if (!changed) {
        for (int i = 0; i < new_size; i++)
            if (ring_index_of(new_ring[i]) < 0) { changed = 1; break; }
    }
    if (!changed) return;

    for (int i = 0; i < alive_count; i++) {
        alive_table[i].in_ring = 0;
        for (int j = 0; j < new_size; j++)
            if (new_ring[j] == alive_table[i].tag_id) { alive_table[i].in_ring = 1; break; }
    }

    if (new_size < current_ring_size) {
        for (int i = 0; i < current_ring_size; i++) {
            int found = 0;
            for (int j = 0; j < new_size; j++) if (new_ring[j] == current_ring[i]) { found=1; break; }
            if (!found) printf("[Bridge] Tag 0x%04X removed\r\n", current_ring[i]);
        }
    } else {
        for (int j = 0; j < new_size; j++)
            if (ring_index_of(new_ring[j]) < 0) printf("[Bridge] Tag 0x%04X added\r\n", new_ring[j]);
    }

    memcpy(current_ring, new_ring, new_size * sizeof(uint16_t));
    current_ring_size = new_size;

    if (new_size >= 1) {
        printf("[Bridge] Membership changed → sending RING size=%d\r\n", new_size);
        resync_active    = 1;
        last_resync_tick = now;
        ring_send();
    } else {
        resync_active     = 0;
        expected_next_idx = -1;
        disorder_count    = 0;
        printf("[Bridge] %s\r\n", new_size == 1 ? "1 tag — free ranging" : "No tags");
    }
}

// ============================================================================
// RHYTHM CHECK — giữ nguyên v4.0
// ============================================================================
static void check_rhythm(uint16_t tag_id)
{
    if (current_ring_size < 2) { expected_next_idx = -1; disorder_count = 0; return; }
    int tag_pos = ring_index_of(tag_id);
    if (tag_pos < 0) return;

    if (expected_next_idx < 0) {
        expected_next_idx = (tag_pos + 1) % current_ring_size;
        disorder_count = 0; return;
    }
    if (tag_pos == expected_next_idx) {
        disorder_count = 0;
        expected_next_idx = (expected_next_idx + 1) % current_ring_size;
    } else {
        disorder_count++;
        printf("[Bridge] Disorder #%d: expected 0x%04X got 0x%04X\r\n",
               disorder_count, current_ring[expected_next_idx], tag_id);
        if (disorder_count >= DISORDER_THRESHOLD) {
            printf("[Bridge] Disorder → resending RING\r\n");
            resync_active    = 1;
            last_resync_tick = xTaskGetTickCount();
            ring_send();
        } else {
            expected_next_idx = (tag_pos + 1) % current_ring_size;
        }
    }
}

// ============================================================================
// PROCESS DATA FRAME — [SYNC-1] đọc ring_size từ byte 14
// ============================================================================
static void process_data_frame(const uint8_t *buf, uint32_t flen)
{
    if (flen < (uint32_t)DATA_MSG_MIN_LEN) return;

    uint16_t tag_id       = decode_u16_le(&buf[DATA_TAG_IDX]);
    uint8_t  tag_rs       = buf[DATA_RINGSIZE_IDX];   // [SYNC-1]
    uint32_t now          = xTaskGetTickCount();

    // Cập nhật alive table
    int found = -1;
    for (int i = 0; i < alive_count; i++)
        if (alive_table[i].tag_id == tag_id) { found = i; break; }
    if (found < 0 && alive_count < MAX_TAGS) {
        found = alive_count++;
        alive_table[found].tag_id        = tag_id;
        alive_table[found].in_ring       = 0;
        alive_table[found].tag_ring_size = 0;
        printf("[Bridge] New tag: 0x%04X\r\n", tag_id);
    }
    if (found >= 0) {
        alive_table[found].last_seen_tick = now;
        alive_table[found].tag_ring_size  = tag_rs;   // [SYNC-1]
    }

    // In data
    printf("0x%04X", tag_id);
    for (int i = 0; i < NUM_ANCHORS; i++) {
        const uint8_t *p = &buf[DATA_PAYLOAD_IDX + i * DATA_ANCHOR_STRIDE];
        int32_t d_raw = decode_i32_le(p + 2);
        printf(",%s", d_raw == (int32_t)0xFFFFFFFF ? "-1" : "");
        if (d_raw != (int32_t)0xFFFFFFFF) printf("%.0f", d_raw / 10.0);
    }
    printf("\r\n");

    // [SYNC-2] Tag vừa báo ring_size → kiểm tra còn ai lệch không
    if (resync_active && tag_rs == current_ring_size) {
        if (all_tags_synced()) {
            resync_active = 0;
            printf("[Bridge] All tags synced to ring_size=%d\r\n", current_ring_size);
        }
    }

    check_rhythm(tag_id);
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

        if (status_reg & SYS_STATUS_RXFCG) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
            uint32_t flen = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
            if (flen > 0 && flen <= RX_BUF_LEN) {
                frame_slot_t *slot = &frame_queue[q_write];
                if (!slot->valid) {
                    memset(slot->buf, 0, RX_BUF_LEN);
                    dwt_readrxdata(slot->buf, flen, 0);
                    slot->len = flen; slot->valid = 1;
                    q_write = (q_write + 1) % FRAME_QUEUE_LEN;
                }
            }
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            total_rx++;
        }
        else if (status_reg & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            dwt_rxreset(); dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }

        frame_slot_t *rslot = &frame_queue[q_read];
        if (rslot->valid) {
            uint8_t saved = rslot->buf[ALL_MSG_SN_IDX];
            rslot->buf[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rslot->buf, data_hdr_ref, ALL_MSG_COMMON_LEN) == 0)
                process_data_frame(rslot->buf, rslot->len);
            rslot->buf[ALL_MSG_SN_IDX] = saved;
            rslot->valid = 0;
            q_read = (q_read + 1) % FRAME_QUEUE_LEN;
        }

        uint32_t now = xTaskGetTickCount();

        // Periodic membership check
        if ((now - last_timeout_check) >= pdMS_TO_TICKS(RING_COOLDOWN_MS)) {
            last_timeout_check = now;
            rebuild_ring_if_needed();
        }

        // [SYNC-2] Resync loop: resend RING mỗi RING_RESYNC_MS cho đến khi sync
        if (resync_active && current_ring_size >= 2) {
            if ((now - last_resync_tick) >= pdMS_TO_TICKS(RING_RESYNC_MS)) {
                last_resync_tick = now;
                if (!all_tags_synced()) {
                    // Log tag nào chưa sync
                    for (int i = 0; i < alive_count; i++) {
                        if (alive_table[i].in_ring &&
                            alive_table[i].tag_ring_size != current_ring_size)
                            printf("[Bridge] Tag 0x%04X still on rs=%d (need %d), resending...\r\n",
                                   alive_table[i].tag_id,
                                   alive_table[i].tag_ring_size,
                                   current_ring_size);
                    }
                    ring_send();
                } else {
                    resync_active = 0;
                    printf("[Bridge] All synced to ring_size=%d\r\n", current_ring_size);
                }
            }
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
    expected_next_idx = -1; disorder_count = 0; resync_active = 0;

    printf("\r\n========================================\r\n");
    printf("Bridge Node v4.1\r\n");
    printf("Tag timeout: %dms | Resync: %dms | Disorder threshold: %d\r\n",
           TAG_TIMEOUT_MS, RING_RESYNC_MS, DISORDER_THRESHOLD);
    printf("DATA frame: ring_size @ byte 14\r\n");
    printf("========================================\r\n\r\n");

    bridge_rx_loop();
}