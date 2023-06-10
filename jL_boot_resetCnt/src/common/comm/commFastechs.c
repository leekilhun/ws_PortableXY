/*
 * commFastechs.c
 *
 *  Created on: 2022. 5. 17.
 *      Author: gns2.lee
 */

#include "commFastechs.h"
#include "util.h"
#include "uart.h"


#ifdef _USE_HW_CMD_FASTECHS


static void crc_update(uint32_t* crc_in, uint8_t data);
static uint8_t   comm_ch = 0;
static uint32_t  comm_baud = 0;
static bool  is_init = false;


#ifdef _USE_HW_RTOS
static osMutexId mutex_lock;
#define AP_FM_LOCK_BEGIN osMutexWait(mutex_lock, osWaitForever)
#define AP_FM_LOCK_END   osMutexRelease(mutex_lock)
#else
#define AP_FM_LOCK_BEGIN
#define AP_FM_LOCK_END
#endif



#define CMD_STX1                    0xAA
#define CMD_STX2                    0xCC
#define CMD_ETX1                    0xAA
#define CMD_ETX2                    0xEE
#define STUFFING_BYTE               0xAA

#define DEF_FASTECH_COMM_SUCCESS            0

#define CMD_FASTECH_STATE_WAIT_STX          0
#define CMD_FASTECH_STATE_WAIT_ID           1
#define CMD_FASTECH_STATE_WAIT_SYNC         2
#define CMD_FASTECH_STATE_WAIT_TYPE         3
#define CMD_FASTECH_STATE_WAIT_RESPONSE     4
#define CMD_FASTECH_STATE_SKIP_STUFFING     5
#define CMD_FASTECH_STATE_WAIT_DATA         6
#define CMD_FASTECH_STATE_WAIT_CHECKSUM     7
#define CMD_FASTECH_STATE_WAIT_ETX          8




void commFm_Init()
{
#ifdef _USE_HW_RTOS
  osMutexDef(mutex_lock);
  mutex_lock = osMutexCreate (osMutex(mutex_lock));
#endif


  is_init = true;

}


bool commFm_Open(uint8_t ch, uint32_t baud)
{
  comm_ch = ch;
  comm_baud = baud;
  return uartOpen(ch, baud);
}


bool commFm_Recovery()
{
  return uartOpen(comm_ch, comm_baud);
}


bool commFm_Close(fastechs_t *p_cmd)
{
  return false;
}

#define CRC_POLY 0xA001

void crc_update(uint32_t* crc_in, uint8_t data)
{
  uint16_t crc = (uint16_t)*crc_in;
  uint8_t i;
  crc ^= data;

  /* Loop through all 8 data bits */
  for (i = 0; i <= 7; i++)
  {
    if (crc & 0x0001)
    {
      crc >>= 1;
      crc ^= CRC_POLY;
    }
    else
      crc >>= 1;
  }
  *crc_in = crc;
}


bool commFm_ReceivePacket(fastechs_t *p_cmd)
{
  bool ret = false;
  if (!is_init)
    return ret;

  uint8_t data_cnt = 0;
  uint8_t data_arr[FASTECHS_CMD_MAX_DATA_LENGTH + 8] = {0,};

  while(uartAvailable(comm_ch))
  {
    data_arr[data_cnt++] = uartRead(comm_ch);
  }

  if (data_cnt == 0)
  {
    return false;
  }

  if (millis()-p_cmd->pre_time >= 100)
  {
    p_cmd->state = CMD_FASTECH_STATE_WAIT_STX;
  }
  p_cmd->pre_time = millis();

  uint8_t rx_data;
  for (uint8_t i = 0 ; i < data_cnt ; i ++ )
  {
    rx_data = data_arr[i];

    switch(p_cmd->state)
    {
      case CMD_FASTECH_STATE_WAIT_STX:
      {
        if (rx_data == CMD_STX1)
        {
          p_cmd->wait_next = true;
          p_cmd->data_len = 0;
          memset(&p_cmd->rx_packet.buffer[0],0,FASTECHS_CMD_MAX_DATA_LENGTH+8);
          p_cmd->rx_packet.buffer[p_cmd->data_len++] = rx_data;
          break;
        }
        if (p_cmd->wait_next)
        {
          p_cmd->wait_next = false;
          if (rx_data == CMD_STX2)
          {
            p_cmd->state = CMD_FASTECH_STATE_WAIT_ID;
            p_cmd->rx_packet.buffer[p_cmd->data_len++] = rx_data;
            p_cmd->rx_packet.check_sum = 0xffff;
          }
          else
          {
            p_cmd->state = CMD_FASTECH_STATE_WAIT_STX;
          }
        }
      }
      break;
      case CMD_FASTECH_STATE_WAIT_ID:
      {
        p_cmd->rx_packet.mt_id = rx_data;
        crc_update(&p_cmd->rx_packet.check_sum, rx_data);
        p_cmd->rx_packet.buffer[p_cmd->data_len++] = rx_data;
        p_cmd->state = CMD_FASTECH_STATE_WAIT_SYNC;
      }
      break;
      case CMD_FASTECH_STATE_WAIT_SYNC:
      {
        p_cmd->rx_packet.sync_no = rx_data;
        p_cmd->rx_packet.buffer[p_cmd->data_len++] = rx_data;
        crc_update(&p_cmd->rx_packet.check_sum, rx_data);
        p_cmd->state = CMD_FASTECH_STATE_WAIT_TYPE;
      }
      break;
      case CMD_FASTECH_STATE_WAIT_TYPE:
      {
        p_cmd->rx_packet.type = rx_data;
        crc_update(&p_cmd->rx_packet.check_sum, rx_data);
        p_cmd->rx_packet.buffer[p_cmd->data_len++] = rx_data;

        switch(p_cmd->rx_packet.type)
        {
          case DEF_FASTECH_COMM_TYPE_GET_ALARM_INFO:
            p_cmd->rx_packet.length = 1;
            break;
          case DEF_FASTECH_COMM_TYPE_GET_RAM_PARAM: //GET_RAM_PARAM
          case DEF_FASTECH_COMM_TYPE_GET_AXIS_STATUS: //GET_AXIS_STATUS
            p_cmd->rx_packet.length = 4;
            break;
          case DEF_FASTECH_COMM_TYPE_GET_MOTION_STATUS: //GET_MOTION_STATUS
            p_cmd->rx_packet.length = 20;
            break;
          case DEF_FASTECH_COMM_TYPE_GET_ALL_STATUS: //GET_ALL_STATUS
            p_cmd->rx_packet.length = 33;
            break;
          default:
            p_cmd->rx_packet.length = 0;
            break;
        }
        p_cmd->state = CMD_FASTECH_STATE_WAIT_RESPONSE;
      }
      break;
      case CMD_FASTECH_STATE_WAIT_RESPONSE:
      {
        p_cmd->rx_packet.response = rx_data;
        crc_update(&p_cmd->rx_packet.check_sum, rx_data);
        p_cmd->rx_packet.buffer[p_cmd->data_len++] = rx_data;

        // TODO:: error response
        if (p_cmd->rx_packet.response != DEF_FASTECH_COMM_SUCCESS)
        {
          p_cmd->error++;
          p_cmd->state = CMD_FASTECH_STATE_WAIT_STX;
          return false;
        }

        if (p_cmd->rx_packet.length > 0)
        {
          p_cmd->index = 0;
          p_cmd->skip_stuffing = false;
          p_cmd->state = CMD_FASTECH_STATE_WAIT_DATA;
          p_cmd->rx_packet.data = &p_cmd->rx_packet.buffer[p_cmd->data_len];
        }
        else
        {
          p_cmd->state = CMD_FASTECH_STATE_WAIT_CHECKSUM;
        }
      }
      break;
      case CMD_FASTECH_STATE_SKIP_STUFFING:
      {
        p_cmd->rx_packet.data[p_cmd->index++] = rx_data;
        p_cmd->rx_packet.buffer[p_cmd->data_len++] = rx_data;

        crc_update(&p_cmd->rx_packet.check_sum, rx_data);
        p_cmd->state = CMD_FASTECH_STATE_WAIT_DATA;
      }
      break;
      case CMD_FASTECH_STATE_WAIT_DATA:
      {
        // 입력할 데이터 넣기
        if (p_cmd->index <= p_cmd->rx_packet.length )
        {
          if (rx_data == STUFFING_BYTE)
          {
            p_cmd->rx_packet.buffer[p_cmd->data_len++] = rx_data;
            p_cmd->state = CMD_FASTECH_STATE_SKIP_STUFFING;
          }
          else
          {
            p_cmd->rx_packet.data[p_cmd->index++] = rx_data;
            p_cmd->rx_packet.buffer[p_cmd->data_len++] = rx_data;

            p_cmd->wait_next = false;
            crc_update(&p_cmd->rx_packet.check_sum, rx_data);
          }

          if (p_cmd->index == p_cmd->rx_packet.length)
          {
            p_cmd->wait_next = false;
            p_cmd->state = CMD_FASTECH_STATE_WAIT_CHECKSUM;
          }
        }
        else
        {
          p_cmd->index = 0;
          p_cmd->state = CMD_FASTECH_STATE_WAIT_STX;
        }

      }
      break;
      case CMD_FASTECH_STATE_WAIT_CHECKSUM:
      {
        if (p_cmd->wait_next)
        {
          p_cmd->wait_next = false;
          p_cmd->rx_packet.buffer[p_cmd->data_len++] = rx_data;
          p_cmd->rx_packet.check_sum_recv |= rx_data<<8;
          p_cmd->state = CMD_FASTECH_STATE_WAIT_ETX;
        }
        else
        {
          p_cmd->wait_next = true;
          p_cmd->rx_packet.buffer[p_cmd->data_len++] = rx_data;
          p_cmd->rx_packet.check_sum_recv = rx_data;
        }
      }
      break;
      case CMD_FASTECH_STATE_WAIT_ETX:
      {
        if (rx_data == CMD_ETX1)
        {
          p_cmd->rx_packet.buffer[p_cmd->data_len++] = rx_data;
          p_cmd->wait_next = true;
          break;
        }
        if (p_cmd->wait_next)
        {
          p_cmd->wait_next = false;
          if (rx_data == CMD_ETX2)
          {
            p_cmd->rx_packet.buffer[p_cmd->data_len++] = rx_data;
            if (p_cmd->rx_packet.check_sum == p_cmd->rx_packet.check_sum_recv)
            {
              //check_pass_ms = millis() - pre_ms ;
              p_cmd->resp_time =  millis() - p_cmd->pre_time;
              ret = true;
            }
          }
        }

        p_cmd->state = CMD_FASTECH_STATE_WAIT_STX;
      }
      break;
    }//switch

  }//for

  return ret;
}


uint8_t commFm_SendCmd(fm_id_t* p_id, uint8_t* p_data, uint8_t length)
{
  AP_FM_LOCK_BEGIN;

  uint8_t packet[FASTECHS_CMD_MAX_DATA_LENGTH] = { 0, };
  uint8_t cmd = p_data[0];
  uint8_t send_sync_no = 0;
  uint32_t crc = 0xffff;
  uint8_t index = 0;
  packet[index++] = CMD_STX1;
  packet[index++] = CMD_STX2;

  packet[index] = p_id->mt_id;
  crc_update(&crc, packet[index++]);

  send_sync_no = ++(p_id->sync_no) % 0xff;
  packet[index] = send_sync_no;
  crc_update(&crc, packet[index++]);

  packet[index] = cmd;
  crc_update(&crc, packet[index++]);
  if (length > 1)
  {
    for (uint8_t i = 1; i < length; i++)
    {
      packet[index] = p_data[i]; // data
      crc_update(&crc, packet[index++]);
    }
  }
  packet[index++] = crc & 0xff;
  packet[index++] = (crc >> 8) & 0xff;

  packet[index++] = CMD_ETX1;
  packet[index++] = CMD_ETX2;

  uartWrite(comm_ch, &packet[0], index);

  AP_FM_LOCK_END;
  return send_sync_no;
}

#endif

