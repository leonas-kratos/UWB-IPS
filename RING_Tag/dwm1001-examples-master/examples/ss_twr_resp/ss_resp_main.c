#include "sdk_config.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"
#include "SEGGER_RTT.h"

#define MY_DEVICE_ID        0x1009
#define HELLO_RETRY_MS      5
#define RNG_DELAY_MS        1

#define ALL_MSG_SN_IDX                  2
#define ALL_MSG_COMMON_LEN              10
#define POLL_MSG_TARGET_DEVICE_ID_IDX   10
#define POLL_MSG_TARGET_DEVICE_ID_LEN   4
#define RESP_MSG_POLL_RX_TS_IDX         10
#define RESP_MSG_RESP_TX_TS_IDX         14
#define RESP_MSG_TS_LEN                 4
#define RESP_MSG_DEVICE_ID_IDX          18
#define RESP_MSG_DEVICE_ID_LEN          4

#define AHELLO_ID_IDX       10
#define AHELLO_MSG_LEN      14

#define ARMV_ID_IDX         10
#define ARMV_MSG_LEN        14

#define UUS_TO_DWT_TIME             65536
#define POLL_RX_TO_RESP_TX_DLY_UUS  1000

#define RX_BUF_LEN      24
#define RX_BUFFER_LEN   24

static const uint8_t anch_hello_hdr[]  = { 0x41,0x88,0,0xCA,0xDE,'A','H','L','O',0xE5 };
static const uint8_t anch_remove_hdr[] = { 0x41,0x88,0,0xCA,0xDE,'A','R','M','V',0xE7 };

static uint8_t rx_poll_msg[] = {
    0x41,0x88,0,0xCA,0xDE,'I','O','V','E',0xE0,
    0,0,0,0,0,0
};
static uint8_t tx_resp_msg[] = {
    0x41,0x88,0,0xCA,0xDE,'V','E','I','O',0xE1,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0
};
static uint8_t tx_hello_msg[AHELLO_MSG_LEN];

static uint8_t  frame_seq_nb = 0;
static uint8_t  rx_buffer[RX_BUF_LEN];
static uint32_t status_reg   = 0;
static uint8_t  removed      = 0;

typedef signed long long   int64;
typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;

static uint64 get_rx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64  ts = 0;
    dwt_readrxtimestamp(ts_tab);
    for (int i = 4; i >= 0; i--) { ts <<= 8; ts |= ts_tab[i]; }
    return ts;
}

static void resp_msg_set_ts(uint8_t *ts_field, const uint64 ts)
{
    for (int i = 0; i < RESP_MSG_TS_LEN; i++)
        ts_field[i] = (ts >> (i * 8)) & 0xFF;
}

static void resp_msg_set_device_id(uint8_t *field, const uint32_t device_id)
{
    for (int i = 0; i < RESP_MSG_DEVICE_ID_LEN; i++)
        field[i] = (device_id >> (i * 8)) & 0xFF;
}

static uint32_t poll_msg_get_target_device_id(uint8_t *field)
{
    uint32_t id = 0;
    for (int i = 0; i < POLL_MSG_TARGET_DEVICE_ID_LEN; i++)
        id += field[i] << (i * 8);
    return id;
}

static void hello_send(void)
{
    memcpy(tx_hello_msg, anch_hello_hdr, ALL_MSG_COMMON_LEN);
    tx_hello_msg[ALL_MSG_SN_IDX]  = frame_seq_nb++;
    tx_hello_msg[AHELLO_ID_IDX]   = (uint8_t)(MY_DEVICE_ID & 0xFF);
    tx_hello_msg[AHELLO_ID_IDX+1] = (uint8_t)((MY_DEVICE_ID >> 8) & 0xFF);
    tx_hello_msg[AHELLO_ID_IDX+2] = 0x00;
    tx_hello_msg[AHELLO_ID_IDX+3] = 0x00;

    dwt_forcetrxoff();
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
    dwt_writetxdata(AHELLO_MSG_LEN, tx_hello_msg, 0);
    dwt_writetxfctrl(AHELLO_MSG_LEN, 0, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {}
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
    SEGGER_RTT_printf(0, "Anchor 0x%04X HELLO sent\n", MY_DEVICE_ID);
}

static int wait_for_poll(void)
{
    dwt_forcetrxoff();
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(HELLO_RETRY_MS);

    while (xTaskGetTickCount() < deadline) {
        status_reg = dwt_read32bitreg(SYS_STATUS_ID);

        if (status_reg & SYS_STATUS_RXFCG) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
            uint32_t flen = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
            if (flen <= RX_BUFFER_LEN) dwt_readrxdata(rx_buffer, flen, 0);
            rx_buffer[ALL_MSG_SN_IDX] = 0;

            if (flen >= 10 && rx_buffer[9] == 0xE2) {
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                return 1;
            }

            if (memcmp(rx_buffer, anch_remove_hdr, ALL_MSG_COMMON_LEN) == 0
                && flen >= ARMV_MSG_LEN) {
                uint16_t rid = (uint16_t)rx_buffer[ARMV_ID_IDX]
                             | ((uint16_t)rx_buffer[ARMV_ID_IDX+1] << 8);
                if (rid == MY_DEVICE_ID) {
                    SEGGER_RTT_printf(0, "Anchor 0x%04X removed by bridge\n", MY_DEVICE_ID);
                    removed = 1;
                    return 0;
                }
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                return 1;
            }

            if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0) {
                uint32_t target = poll_msg_get_target_device_id(
                                      &rx_buffer[POLL_MSG_TARGET_DEVICE_ID_IDX]);
                if (target == MY_DEVICE_ID) return 1;
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
    return 0;
}

static uint32_t last_poll_tick = 0;

int ss_resp_run(void)
{
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(HELLO_RETRY_MS);
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
        if (xTaskGetTickCount() >= deadline) {
            dwt_forcetrxoff();
            return 0;
        }
        vTaskDelay(0);
    }

    if (status_reg & SYS_STATUS_RXFCG) {
        uint32_t frame_len;
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
        if (frame_len <= RX_BUFFER_LEN) dwt_readrxdata(rx_buffer, frame_len, 0);
        rx_buffer[ALL_MSG_SN_IDX] = 0;

        if (frame_len >= 10 && rx_buffer[9] == 0xE2) return 1;

        if (memcmp(rx_buffer, anch_remove_hdr, ALL_MSG_COMMON_LEN) == 0
            && frame_len >= ARMV_MSG_LEN) {
            uint16_t rid = (uint16_t)rx_buffer[ARMV_ID_IDX]
                         | ((uint16_t)rx_buffer[ARMV_ID_IDX+1] << 8);
            if (rid == MY_DEVICE_ID) {
                SEGGER_RTT_printf(0, "Anchor 0x%04X removed by bridge\n", MY_DEVICE_ID);
                removed = 1;
            }
            return 1;
        }

        if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0) {
            uint32_t target = poll_msg_get_target_device_id(
                                  &rx_buffer[POLL_MSG_TARGET_DEVICE_ID_IDX]);
            if (target == MY_DEVICE_ID) {
                int ret;
                poll_rx_ts = get_rx_timestamp_u64();
                uint32_t resp_tx_time =
                    (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(resp_tx_time);
                resp_tx_ts = (((uint64)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

                resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
                resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);
                resp_msg_set_device_id(&tx_resp_msg[RESP_MSG_DEVICE_ID_IDX], MY_DEVICE_ID);
                tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
                dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);
                ret = dwt_starttx(DWT_START_TX_DELAYED);
                if (ret == DWT_SUCCESS) {
                    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {}
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
                    frame_seq_nb++;
                } else {
                    dwt_rxreset();
                }
            }
        }
    } else {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        dwt_rxreset();
    }
    return 1;
}

dwt_rxdiag_t diag;

void ss_responder_task_function(void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    dwt_setleds(DWT_LEDS_ENABLE);
    SEGGER_RTT_printf(0, "Anchor v2.1 | ID: 0x%04X\n", MY_DEVICE_ID);

    while (1) {
        if (removed) {
            removed = 0;
            SEGGER_RTT_printf(0, "Anchor 0x%04X re-joining\n", MY_DEVICE_ID);
        }

        hello_send();

        while (!removed) {
            int r = ss_resp_run();
            if (r == 0) {
                hello_send();
            }
            if (r > 0) {
                dwt_readdiagnostics(&diag);
                SEGGER_RTT_printf(0, "%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                    diag.maxNoise, diag.stdNoise,
                    diag.firstPathAmp1, diag.firstPathAmp2, diag.firstPathAmp3,
                    diag.maxGrowthCIR, diag.rxPreamCount, diag.firstPath);
            }
        }
    }
}