
// ============================================================================
// RESPONDER CODE (ss_resp_main.c) - Only responds to matching device ID
// ============================================================================

#include "sdk_config.h" 
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"
#include "SEGGER_RTT.h"  

/* Inter-ranging delay period, in milliseconds. See NOTE 1*/
#define RNG_DELAY_MS 1

/* Device ID for this responder - change this value for each device */
#define MY_DEVICE_ID 0x1003

/* Frames used in the ranging process. See NOTE 2,3 below. */
// MODIFIED: Poll message now includes target device ID
static uint8 rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'I', 'O', 'V', 'E', 0xE0, 0, 0, 0, 0, 0, 0};
static uint8 tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'I', 'O', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10

/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2
#define POLL_MSG_TARGET_DEVICE_ID_IDX 10  // Target device ID location in poll message
#define POLL_MSG_TARGET_DEVICE_ID_LEN 4   // Target device ID length
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
#define RESP_MSG_DEVICE_ID_IDX 18
#define RESP_MSG_DEVICE_ID_LEN 4

/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received response message.
* Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 24
#define RX_BUFFER_LEN 24
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
* 1 uus = 512 / 499.2 s and 1 s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

// Not enough time to write the data so TX timeout extended for nRF operation.
#define POLL_RX_TO_RESP_TX_DLY_UUS  1000

/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 300

/* Timestamps of frames transmission/reception.
* As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef signed long long int64;
typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;

/* Declaration of static functions. */
static uint64 get_rx_timestamp_u64(void);
static void resp_msg_set_ts(uint8 *ts_field, const uint64 ts);
static void resp_msg_set_device_id(uint8 *device_id_field, const uint32 device_id);
static uint32 poll_msg_get_target_device_id(uint8 *device_id_field);  // Function to extract target device ID from poll

int ss_resp_run(void)
{
  /* Activate reception immediately. */
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  /* Poll for reception of a frame or error/timeout. See NOTE 5 below. */
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
  { vTaskDelay(0); };

  if (status_reg & SYS_STATUS_RXFCG)
  {
    uint32 frame_len;

    /* Clear good RX frame event in the DW1000 status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

    /* A frame has been received, read it into the local buffer. */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
    if (frame_len <= RX_BUFFER_LEN)
    {
      dwt_readrxdata(rx_buffer, frame_len, 0);
    }

    /* Check that the frame is a poll sent by "SS TWR initiator" example.
    * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
    rx_buffer[ALL_MSG_SN_IDX] = 0;

    /* Bỏ qua TOKEN frame giữa các tag (function code 0xE2) */
    if (frame_len >= 10 && rx_buffer[9] == 0xE2)
    {
      SEGGER_RTT_printf(0, "Token frame received, ignoring.\n");
      return(1);
    }

    if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
    {
      uint32 target_device_id;
      
      /* Extract target device ID from poll message */
      target_device_id = poll_msg_get_target_device_id(&rx_buffer[POLL_MSG_TARGET_DEVICE_ID_IDX]);
      
      SEGGER_RTT_printf(0, "Received poll for Device ID: 0x%04X\n", target_device_id);
      
      /* Check if this poll is intended for this device */
      if (target_device_id == MY_DEVICE_ID)
      {
        uint32 resp_tx_time;
        int ret;

        SEGGER_RTT_printf(0, "Device ID matches! Processing poll and sending response.\n");

        /* Retrieve poll reception timestamp. */
        poll_rx_ts = get_rx_timestamp_u64();

        /* Compute final message transmission time. See NOTE 7 below. */
        resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
        dwt_setdelayedtrxtime(resp_tx_time);

        /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
        resp_tx_ts = (((uint64)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

        /* Write all timestamps and device ID in the final message. See NOTE 8 below. */
        resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
        resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);
        resp_msg_set_device_id(&tx_resp_msg[RESP_MSG_DEVICE_ID_IDX], MY_DEVICE_ID);

        /* Write and send the response message. See NOTE 9 below. */
        tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. See Note 5 below.*/
        dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
        ret = dwt_starttx(DWT_START_TX_DELAYED);

        /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. */
        if (ret == DWT_SUCCESS)
        {
          /* Poll DW1000 until TX frame sent event set. See NOTE 5 below. */
          while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
          {};

          /* Clear TXFRS event. */
          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

          /* Increment frame sequence number after transmission of the poll message (modulo 256). */
          frame_seq_nb++;
          
          SEGGER_RTT_printf(0, "Response sent successfully.\n");
        }
        else
        {
          SEGGER_RTT_printf(0, "Failed to send response. TX error.\n");
          /* Reset RX to properly reinitialise LDE operation. */
          dwt_rxreset();
        }
      }
      else
      {
        /* Poll is not for this device - ignore it and continue listening */
        SEGGER_RTT_printf(0, "Device ID does not match (my ID: 0x%04X). Ignoring poll.\n", MY_DEVICE_ID);
        /* No response sent - just continue to next reception cycle */
      }
    }
  }
  else
  {
    /* Clear RX error events in the DW1000 status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

    /* Reset RX to properly reinitialise LDE operation. */
    dwt_rxreset();
  }

  return(1);		
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn get_rx_timestamp_u64()
*
* @brief Get the RX time-stamp in a 64-bit variable.
*        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
*
* @param  none
*
* @return  64-bit value of the read time-stamp.
*/
static uint64 get_rx_timestamp_u64(void)
{
  uint8 ts_tab[5];
  uint64 ts = 0;
  int i;
  dwt_readrxtimestamp(ts_tab);
  for (i = 4; i >= 0; i--)
  {
    ts <<= 8;
    ts |= ts_tab[i];
  }
  return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn resp_msg_set_ts()
*
* @brief Fill a given timestamp field in the response message with the given value. In the timestamp fields of the
*        response message, the least significant byte is at the lower address.
*
* @param  ts_field  pointer on the first byte of the timestamp field to fill
*         ts  timestamp value
*
* @return none
*/
static void resp_msg_set_ts(uint8 *ts_field, const uint64 ts)
{
  int i;
  for (i = 0; i < RESP_MSG_TS_LEN; i++)
  {
    ts_field[i] = (ts >> (i * 8)) & 0xFF;
  }
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn resp_msg_set_device_id()
*
* @brief Fill the device ID field in the response message with the given value. In the device ID field of the
*        response message, the least significant byte is at the lower address.
*
* @param  device_id_field  pointer on the first byte of the device ID field to fill
*         device_id  device ID value
*
* @return none
*/
static void resp_msg_set_device_id(uint8 *device_id_field, const uint32 device_id)
{
  int i;
  for (i = 0; i < RESP_MSG_DEVICE_ID_LEN; i++)
  {
    device_id_field[i] = (device_id >> (i * 8)) & 0xFF;
  }
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn poll_msg_get_target_device_id()
*
* @brief Read the target device ID value from the poll message. In the device ID field of the poll message, 
*        the least significant byte is at the lower address.
*
* @param  device_id_field  pointer on the first byte of the device ID field to get
*
* @return target device ID value
*/
static uint32 poll_msg_get_target_device_id(uint8 *device_id_field)
{
  int i;
  uint32 device_id = 0;
  for (i = 0; i < POLL_MSG_TARGET_DEVICE_ID_LEN; i++)
  {
    device_id += device_id_field[i] << (i * 8);
  }
  return device_id;
}

/**@brief SS TWR Responder task entry function.
*
* @param[in] pvParameter   Pointer that will be used as the parameter for the task.
*/
dwt_rxdiag_t diag;
void printDiag(dwt_rxdiag_t diag)
{
  dwt_readdiagnostics(&diag);
  SEGGER_RTT_printf(0 ,"%d, %d, %d, %d, %d, %d, %d, %d\r\n", diag.maxNoise, 
                                                           diag.stdNoise, 
                                                           diag.firstPathAmp1, 
                                                           diag.firstPathAmp2,
                                                           diag.firstPathAmp3,
                                                           diag.maxGrowthCIR,
                                                           diag.rxPreamCount,
                                                           diag.firstPath);
}

void ss_responder_task_function (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);

  dwt_setleds(DWT_LEDS_ENABLE);

  SEGGER_RTT_printf(0, "Responder started. My Device ID: 0x%04X\n", MY_DEVICE_ID);

  while (true)
  {
    ss_resp_run();
    printDiag(diag);
    // KHÔNG vTaskDelay cố định — anchor phải luôn sẵn sàng nhận poll ngay lập tức.
    // ss_resp_run() đã có vTaskDelay(0) bên trong vòng chờ để yield cho FreeRTOS.
  }
}
