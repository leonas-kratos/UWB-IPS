// ============================================================================
// Bridge Node v2.1 — Data frame sniffer
// Chỉ parse DATA frame (header 'DATA' 0xE3), bỏ qua token và ranging frames
// ============================================================================

#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "nrf.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"

#define NUM_TAGS        5
#define NUM_ANCHORS     3
#define RX_BUF_LEN      80
#define FRAME_QUEUE_LEN 8

static const uint16_t TAG_IDS[]    = { 0x5678, 0x5679, 0x567A, 0x567B, 0x0000 };
static const uint32_t ANCHOR_IDS[] = { 0x1001, 0x1002, 0x1003, 0x1004 };

#define ALL_MSG_SN_IDX      2
#define ALL_MSG_COMMON_LEN  10
#define DATA_TAG_IDX        10
#define DATA_CYCLE_IDX      12
#define DATA_PAYLOAD_IDX    14
#define DATA_ANCHOR_STRIDE   6
#define DATA_HDR_LEN        14
#define DATA_MSG_LEN        (DATA_HDR_LEN + NUM_ANCHORS * DATA_ANCHOR_STRIDE)

static const uint8_t data_hdr_ref[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'D', 'A', 'T', 'A', 0xE3
};

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

static uint32_t total_rx   = 0;
static uint32_t valid_rx   = 0;
static uint32_t invalid_rx = 0;
static uint32_t dropped    = 0;

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
static int is_known_tag(uint16_t id)
{
    for (int i = 0; i < NUM_TAGS; i++)
        if (TAG_IDS[i] == id) return 1;
    return 0;
}

// ============================================================================
// PROCESS DATA FRAME
// ============================================================================
static void process_data_frame(const uint8_t *buf, uint32_t flen)
{
    if (flen < (uint32_t)DATA_MSG_LEN) {
        invalid_rx++;
        return;
    }

    uint16_t tag_id = decode_u16_le(&buf[DATA_TAG_IDX]);
    uint16_t cycle  = decode_u16_le(&buf[DATA_CYCLE_IDX]);

    if (!is_known_tag(tag_id)) {
        invalid_rx++;
        return;
    }

    valid_rx++;

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
}

// ============================================================================
// BRIDGE RX LOOP
// ============================================================================
static void bridge_rx_loop(void)
{
    dwt_forcetrxoff();
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    while (1) {
        uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID);

        // Phase 1: đọc frame vào queue NGAY
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

            // Re-enable RX NGAY
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            total_rx++;
        }
        else if (status_reg & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
            dwt_write32bitreg(SYS_STATUS_ID,
                              SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            dwt_rxreset();
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }

        // Phase 2: process 1 frame từ queue
        frame_slot_t *rslot = &frame_queue[q_read];
        if (rslot->valid) {
            uint8_t saved_sn = rslot->buf[ALL_MSG_SN_IDX];
            rslot->buf[ALL_MSG_SN_IDX] = 0;

            if (memcmp(rslot->buf, data_hdr_ref, ALL_MSG_COMMON_LEN) == 0) {
                process_data_frame(rslot->buf, rslot->len);
            }
            // Token và ranging frames bỏ qua yên lặng

            rslot->buf[ALL_MSG_SN_IDX] = saved_sn;
            rslot->valid = 0;
            q_read = (q_read + 1) % FRAME_QUEUE_LEN;
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
    memset(frame_queue, 0, sizeof(frame_queue));

    printf("\r\n========================================\r\n");
    printf("Bridge Node v2.1\r\n");
    printf("Listening for DATA frames (0xE3)\r\n");
    printf("Tags: %d | Anchors: %d\r\n", NUM_TAGS, NUM_ANCHORS);
    printf("DATA frame: %d bytes expected\r\n", DATA_MSG_LEN);
    printf("========================================\r\n\r\n");

    bridge_rx_loop();
}
