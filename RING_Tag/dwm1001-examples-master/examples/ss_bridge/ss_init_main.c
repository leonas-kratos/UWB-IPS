#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "nrf.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"

#define MAX_TAGS                10
#define MAX_ANCHORS             10
#define RX_BUF_LEN              80
#define FRAME_QUEUE_LEN         8

#define TAG_TIMEOUT_MS          1000
#define RING_COOLDOWN_MS        10
#define RING_RESYNC_MS          50
#define ANCH_RESYNC_MS          50
#define DISORDER_THRESHOLD      50
#define ANCHOR_DEAD_CYCLES      2

static const uint8_t data_hdr_ref[]      = { 0x41,0x88,0,0xCA,0xDE,'D','A','T','A',0xE3 };
static const uint8_t ring_hdr_ref[]      = { 0x41,0x88,0,0xCA,0xDE,'R','I','N','G',0xE4 };
static const uint8_t anch_hello_hdr_ref[]= { 0x41,0x88,0,0xCA,0xDE,'A','H','L','O',0xE5 };
static const uint8_t anch_list_hdr_ref[] = { 0x41,0x88,0,0xCA,0xDE,'A','L','S','T',0xE6 };
static const uint8_t anch_remove_hdr_ref[]={ 0x41,0x88,0,0xCA,0xDE,'A','R','M','V',0xE7 };

#define ALL_MSG_SN_IDX          2
#define ALL_MSG_COMMON_LEN      10

#define DATA_TAG_IDX            10
#define DATA_CYCLE_IDX          12
#define DATA_RINGSIZE_IDX       14
#define DATA_NUMANCHORS_IDX     15
#define DATA_PAYLOAD_IDX        16
#define DATA_ANCHOR_STRIDE      6
#define DATA_HDR_LEN            16

#define RING_SIZE_IDX           10
#define RING_TAGS_IDX           11
#define RING_HDR_LEN            11
#define RING_MAX_TAGS           8
#define RING_MSG_MAX_LEN        (RING_HDR_LEN + RING_MAX_TAGS * 2 + 2)

#define AHELLO_ID_IDX           10
#define AHELLO_MSG_LEN          14

#define ALIST_SIZE_IDX          10
#define ALIST_IDS_IDX           11
#define ALIST_HDR_LEN           11
#define ALIST_MSG_MAX_LEN       (ALIST_HDR_LEN + MAX_ANCHORS * 2 + 2)

#define ARMV_ID_IDX             10
#define ARMV_MSG_LEN            14

typedef struct {
    uint16_t tag_id;
    uint32_t last_seen_tick;
    uint8_t  tag_ring_size;
    uint8_t  tag_num_anchors;
    uint8_t  in_ring;
} tag_entry_t;

typedef struct {
    uint16_t anchor_id;
    uint8_t  active;
    uint8_t  dead_cycle_count[MAX_TAGS];
} anchor_entry_t;

static tag_entry_t    alive_table[MAX_TAGS];
static uint8_t        alive_count = 0;

static anchor_entry_t anchor_table[MAX_ANCHORS];
static uint8_t        anchor_count = 0;
static uint16_t       current_anchor_list[MAX_ANCHORS];
static uint8_t        current_num_anchors = 0;

static uint16_t current_ring[RING_MAX_TAGS];
static uint8_t  current_ring_size = 0;

static int8_t  expected_next_idx = -1;
static uint8_t disorder_count    = 0;

static uint8_t  ring_resync_active    = 0;
static uint32_t last_ring_resync_tick = 0;
static uint8_t  anch_resync_active    = 0;
static uint32_t last_anch_resync_tick = 0;

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
static uint32_t last_alist_tx_tick= 0;
static uint32_t total_rx          = 0;
static uint32_t ring_tx_count     = 0;
static uint32_t alist_tx_count    = 0;

static const uint8_t anch_dead_hdr_ref[] = { 0x41,0x88,0,0xCA,0xDE,'A','D','E','D',0xE8 };
#define ADEAD_ID_IDX    10
#define ADEAD_MSG_LEN   14

static uint8_t anchor_dead_reported[MAX_ANCHORS] = {0};  // đếm số tag báo dead

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

static int anchor_index_of(uint16_t anchor_id)
{
    for (int i = 0; i < anchor_count; i++)
        if (anchor_table[i].anchor_id == anchor_id) return i;
    return -1;
}

static int tag_index_of(uint16_t tag_id)
{
    for (int i = 0; i < alive_count; i++)
        if (alive_table[i].tag_id == tag_id) return i;
    return -1;
}

static uint8_t all_tags_ring_synced(void)
{
    if (current_ring_size < 2) return 1;
    for (int i = 0; i < alive_count; i++) {
        if (!alive_table[i].in_ring) continue;
        if (alive_table[i].tag_ring_size != current_ring_size) return 0;
    }
    return 1;
}

static uint8_t all_tags_anch_synced(void)
{
    for (int i = 0; i < alive_count; i++) {
        if (alive_table[i].tag_num_anchors != current_num_anchors) return 0;
    }
    return 1;
}

static void do_tx(uint8_t *buf, uint8_t len)
{
    dwt_forcetrxoff();
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
    dwt_writetxdata(len, buf, 0);
    dwt_writetxfctrl(len, 0, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {}
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

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
    tx_buf[payload_len]   = 0x00;
    tx_buf[payload_len+1] = 0x00;

    do_tx(tx_buf, msg_len);
    last_ring_tx_tick = xTaskGetTickCount();
    ring_tx_count++;

    printf("[Bridge] RING TX #%lu size=%d [", (unsigned long)ring_tx_count, n);
    for (int i = 0; i < n; i++)
        printf("0x%04X%s", current_ring[i], i < n-1 ? "->" : "");
    printf("]\r\n");

    expected_next_idx = -1;
    disorder_count    = 0;
}

static void rebuild_anchor_list(void)
{
    uint8_t n = 0;
    for (int i = 0; i < anchor_count; i++)
        if (anchor_table[i].active) current_anchor_list[n++] = anchor_table[i].anchor_id;

    for (int i = 0; i < (int)n - 1; i++)
        for (int j = i+1; j < n; j++)
            if (current_anchor_list[j] < current_anchor_list[i]) {
                uint16_t tmp = current_anchor_list[i];
                current_anchor_list[i] = current_anchor_list[j];
                current_anchor_list[j] = tmp;
            }
    current_num_anchors = n;
}

static void anch_list_send(void)
{
    uint32_t now = xTaskGetTickCount();
    if ((now - last_alist_tx_tick) < pdMS_TO_TICKS(RING_COOLDOWN_MS)) return;

    uint8_t n = current_num_anchors;
    uint8_t tx_buf[ALIST_MSG_MAX_LEN];
    uint8_t payload_len = ALIST_HDR_LEN + n * 2;
    uint8_t msg_len     = payload_len + 2;

    memcpy(tx_buf, anch_list_hdr_ref, ALL_MSG_COMMON_LEN);
    tx_buf[ALL_MSG_SN_IDX]  = frame_seq_nb++;
    tx_buf[ALIST_SIZE_IDX]  = n;
    for (int i = 0; i < n; i++) {
        tx_buf[ALIST_IDS_IDX + i*2]   = (uint8_t)(current_anchor_list[i] & 0xFF);
        tx_buf[ALIST_IDS_IDX + i*2+1] = (uint8_t)((current_anchor_list[i] >> 8) & 0xFF);
    }
    tx_buf[payload_len]   = 0x00;
    tx_buf[payload_len+1] = 0x00;

    do_tx(tx_buf, msg_len);
    last_alist_tx_tick = xTaskGetTickCount();
    alist_tx_count++;

    printf("[Bridge] ANCH_LIST TX #%lu size=%d [", (unsigned long)alist_tx_count, n);
    for (int i = 0; i < n; i++)
        printf("0x%04X%s", current_anchor_list[i], i < n-1 ? "," : "");
    printf("]\r\n");
}

static void anch_remove_send(uint16_t anchor_id)
{
    uint8_t tx_buf[ARMV_MSG_LEN];
    memcpy(tx_buf, anch_remove_hdr_ref, ALL_MSG_COMMON_LEN);
    tx_buf[ALL_MSG_SN_IDX]    = frame_seq_nb++;
    tx_buf[ARMV_ID_IDX]       = (uint8_t)(anchor_id & 0xFF);
    tx_buf[ARMV_ID_IDX+1]     = (uint8_t)((anchor_id >> 8) & 0xFF);
    tx_buf[ARMV_ID_IDX+2]     = 0x00;
    tx_buf[ARMV_ID_IDX+3]     = 0x00;

    do_tx(tx_buf, ARMV_MSG_LEN);
    printf("[Bridge] ANCH_REMOVE 0x%04X\r\n", anchor_id);
}

static void process_anch_hello(const uint8_t *buf, uint32_t flen)
{
    if (flen < AHELLO_MSG_LEN) return;
    uint16_t anchor_id = decode_u16_le(&buf[AHELLO_ID_IDX]);

    int idx = anchor_index_of(anchor_id);

    if (idx >= 0) {
        if (anchor_table[idx].active) return;
        anchor_table[idx].active = 1;
        memset(anchor_table[idx].dead_cycle_count, 0,
               sizeof(anchor_table[idx].dead_cycle_count));
        anchor_dead_reported[idx] = 0;
        printf("[Bridge] Anchor 0x%04X re-joined\r\n", anchor_id);
    } else {
        if (anchor_count >= MAX_ANCHORS) return;
        idx = anchor_count++;
        anchor_table[idx].anchor_id = anchor_id;
        anchor_table[idx].active    = 1;
        memset(anchor_table[idx].dead_cycle_count, 0,
               sizeof(anchor_table[idx].dead_cycle_count));
        anchor_dead_reported[idx] = 0;
        printf("[Bridge] New anchor: 0x%04X\r\n", anchor_id);
    }

    rebuild_anchor_list();
    anch_list_send();
    anch_resync_active    = 1;
    last_anch_resync_tick = xTaskGetTickCount();
}

static void check_anchor_health(const uint8_t *buf, uint32_t flen, uint16_t tag_id, int tag_idx)
{
    uint8_t  na  = buf[DATA_NUMANCHORS_IDX];
    uint32_t now = xTaskGetTickCount();

    for (int i = 0; i < na; i++) {
        const uint8_t *p = &buf[DATA_PAYLOAD_IDX + i * DATA_ANCHOR_STRIDE];
        uint16_t anchor_id = decode_u16_le(p);
        int32_t  d_raw     = decode_i32_le(p + 2);
        int      aidx      = anchor_index_of(anchor_id);
        if (aidx < 0 || !anchor_table[aidx].active) continue;

        if (d_raw == (int32_t)0xFFFFFFFF)
            anchor_table[aidx].dead_cycle_count[tag_idx]++;
        else
            anchor_table[aidx].dead_cycle_count[tag_idx] = 0;

        uint8_t all_dead = 1;
        for (int t = 0; t < alive_count; t++) {
            if (anchor_table[aidx].dead_cycle_count[t] < ANCHOR_DEAD_CYCLES) {
                all_dead = 0; break;
            }
        }
        if (all_dead && alive_count > 0) {
            printf("[Bridge] Anchor 0x%04X dead — removing\r\n", anchor_id);
            anch_remove_send(anchor_id);
            anchor_table[aidx].active = 0;
            anchor_dead_reported[aidx] = 0;
            memset(anchor_table[aidx].dead_cycle_count, 0,
                   sizeof(anchor_table[aidx].dead_cycle_count));
            rebuild_anchor_list();
            anch_list_send();
            anch_resync_active    = 1;
            last_anch_resync_tick = now;
        }
    }
}

static void process_anch_dead(const uint8_t *buf, uint32_t flen)
{
    if (flen < ADEAD_MSG_LEN) return;
    uint16_t anchor_id = decode_u16_le(&buf[ADEAD_ID_IDX]);
    int aidx = anchor_index_of(anchor_id);
    if (aidx < 0 || !anchor_table[aidx].active) return;

    anchor_dead_reported[aidx]++;
    printf("[Bridge] ANCHOR_DEAD report for 0x%04X count=%d\r\n",
           anchor_id, anchor_dead_reported[aidx]);

    if (anchor_dead_reported[aidx] >= 1) {
        printf("[Bridge] Removing anchor 0x%04X by tag report\r\n", anchor_id);
        anch_remove_send(anchor_id);
        anchor_table[aidx].active = 0;
        anchor_dead_reported[aidx] = 0;
        memset(anchor_table[aidx].dead_cycle_count, 0,
               sizeof(anchor_table[aidx].dead_cycle_count));
        rebuild_anchor_list();
        anch_list_send();
        anch_resync_active    = 1;
        last_anch_resync_tick = xTaskGetTickCount();
    }
}

static void ring_send_wrapper(void)
{
    ring_resync_active    = 1;
    last_ring_resync_tick = xTaskGetTickCount();
    ring_send();
}

static void rebuild_ring_if_needed(void)
{
    uint32_t now = xTaskGetTickCount();
    for (int i = 0; i < alive_count; i++) {
    if ((now - alive_table[i].last_seen_tick) >= pdMS_TO_TICKS(TAG_TIMEOUT_MS)) {

            printf("[Bridge] Tag 0x%04X FULLY REMOVED\r\n", alive_table[i].tag_id);

            // shift left
            for (int j = i; j < alive_count - 1; j++) {
                alive_table[j] = alive_table[j + 1];
            }

            alive_count--;
            i--; // rất quan trọng
        }
    }
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
        printf("[Bridge] Membership changed → RING size=%d\r\n", new_size);
        ring_send_wrapper();
    } else {
        ring_resync_active = 0;
        expected_next_idx  = -1;
        disorder_count     = 0;
    }
}

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
            ring_send_wrapper();
        } else {
            expected_next_idx = (tag_pos + 1) % current_ring_size;
        }
    }
}

static void process_data_frame(const uint8_t *buf, uint32_t flen)
{
    uint8_t  na = buf[DATA_NUMANCHORS_IDX];
    uint32_t data_min_len = (uint32_t)(DATA_HDR_LEN + na * DATA_ANCHOR_STRIDE);
    if (flen < data_min_len) return;

    uint16_t tag_id  = decode_u16_le(&buf[DATA_TAG_IDX]);
    uint8_t  tag_rs  = buf[DATA_RINGSIZE_IDX];
    uint32_t now     = xTaskGetTickCount();

    int found = tag_index_of(tag_id);
    if (found < 0 && alive_count < MAX_TAGS) {
        found = alive_count++;
        alive_table[found].tag_id        = tag_id;
        alive_table[found].in_ring       = 0;
        alive_table[found].tag_ring_size = 0;
        alive_table[found].tag_num_anchors = 0;
        printf("[Bridge] New tag: 0x%04X\r\n", tag_id);
    }
    if (found >= 0) {
        alive_table[found].last_seen_tick  = now;
        alive_table[found].tag_ring_size   = tag_rs;
        alive_table[found].tag_num_anchors = na;
    }

    printf("tag=0x%04X  rs=%2d  na=%2d", tag_id, tag_rs, na);

    for (int i = 0; i < na; i++) {
        const uint8_t *p = &buf[DATA_PAYLOAD_IDX + i * DATA_ANCHOR_STRIDE];
        uint16_t anchor_id = decode_u16_le(p);
        int32_t d_raw = decode_i32_le(p + 2);

        if (d_raw == (int32_t)0xFFFFFFFF)
            printf(" | %04X : %4s", anchor_id, "-1");
        else
            printf(" | %04X : %4.0f", anchor_id, d_raw / 10.0);
    }

    printf("\r\n");

    if (found >= 0) check_anchor_health(buf, flen, tag_id, found);

    if (ring_resync_active && tag_rs == current_ring_size && all_tags_ring_synced()) {
        ring_resync_active = 0;
        printf("[Bridge] All tags ring-synced to size=%d\r\n", current_ring_size);
    }

    if (na == 0 && current_num_anchors > 0) {
        printf("[Bridge] Tag 0x%04X has na=0, sending ANCH_LIST now\r\n", tag_id);
        anch_list_send();
        anch_resync_active    = 1;
        last_anch_resync_tick = xTaskGetTickCount();
    } else if (anch_resync_active && na == current_num_anchors && all_tags_anch_synced()) {
        anch_resync_active = 0;
        printf("[Bridge] All tags anch-synced to num=%d\r\n", current_num_anchors);
    }

    check_rhythm(tag_id);
    rebuild_ring_if_needed();
}

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
        } else if (status_reg & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            dwt_rxreset();
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }

        frame_slot_t *rslot = &frame_queue[q_read];
        if (rslot->valid) {
            uint8_t saved = rslot->buf[ALL_MSG_SN_IDX];
            rslot->buf[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rslot->buf, data_hdr_ref, ALL_MSG_COMMON_LEN) == 0)
                process_data_frame(rslot->buf, rslot->len);
            else if (memcmp(rslot->buf, anch_hello_hdr_ref, ALL_MSG_COMMON_LEN) == 0)
                process_anch_hello(rslot->buf, rslot->len);
            else if (memcmp(rslot->buf, anch_dead_hdr_ref, ALL_MSG_COMMON_LEN) == 0)
                process_anch_dead(rslot->buf, rslot->len);
            rslot->buf[ALL_MSG_SN_IDX] = saved;
            rslot->valid = 0;
            q_read = (q_read + 1) % FRAME_QUEUE_LEN;
        }

        uint32_t now = xTaskGetTickCount();

        if ((now - last_timeout_check) >= pdMS_TO_TICKS(RING_COOLDOWN_MS)) {
            last_timeout_check = now;
            rebuild_ring_if_needed();
        }

        if (ring_resync_active && current_ring_size >= 2) {
            if ((now - last_ring_resync_tick) >= pdMS_TO_TICKS(RING_RESYNC_MS)) {
                last_ring_resync_tick = now;
                if (!all_tags_ring_synced()) {
                    for (int i = 0; i < alive_count; i++) {
                        if (alive_table[i].in_ring &&
                            alive_table[i].tag_ring_size != current_ring_size)
                            printf("[Bridge] Tag 0x%04X ring rs=%d need %d, resending\r\n",
                                   alive_table[i].tag_id,
                                   alive_table[i].tag_ring_size,
                                   current_ring_size);
                    }
                    ring_send();
                } else {
                    ring_resync_active = 0;
                    printf("[Bridge] All ring-synced size=%d\r\n", current_ring_size);
                }
            }
        }

        if (anch_resync_active && current_num_anchors > 0) {
            if ((now - last_anch_resync_tick) >= pdMS_TO_TICKS(ANCH_RESYNC_MS)) {
                last_anch_resync_tick = now;
                if (!all_tags_anch_synced()) {
                    for (int i = 0; i < alive_count; i++) {
                        if (alive_table[i].tag_num_anchors != current_num_anchors)
                            printf("[Bridge] Tag 0x%04X na=%d need %d, resending ANCH_LIST\r\n",
                                   alive_table[i].tag_id,
                                   alive_table[i].tag_num_anchors,
                                   current_num_anchors);
                    }
                    anch_list_send();
                } else {
                    anch_resync_active = 0;
                    printf("[Bridge] All anch-synced num=%d\r\n", current_num_anchors);
                }
            }
        }

        vTaskDelay(0);
    }
}

void bridge_task_function(void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    dwt_setleds(DWT_LEDS_ENABLE);
    memset(frame_queue,   0, sizeof(frame_queue));
    memset(alive_table,   0, sizeof(alive_table));
    memset(anchor_table,  0, sizeof(anchor_table));
    memset(current_ring,  0, sizeof(current_ring));
    expected_next_idx = -1;
    disorder_count    = 0;
    ring_resync_active = 0;
    anch_resync_active = 0;

    printf("Bridge Node v5.0\r\n");

    bridge_rx_loop();
}