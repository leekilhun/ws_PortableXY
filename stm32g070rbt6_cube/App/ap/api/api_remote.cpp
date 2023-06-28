/*
 * api_remote.cpp
 *
 *  Created on: 2023. 6. 10.
 *      Author: gns2l
 */



#include "ap.hpp"
#include "api_remote.hpp"


errno_t api_remote::SendData(RCTRL::TX_TYPE type, uint8_t obj_id )
{
  uint16_t crc = 0xffff;
  uint16_t length = 0;
  enum :uint8_t {
    idx_sof, idx_type, idx_objid, idx_length_l,idx_length_h,
    idx_data
  };
  std::array<uint8_t, RCTRL::CMD_MAX_PACKET_LENGTH> value = { };
  using tx_t = RCTRL::TX_TYPE;

  value[idx_sof] = RCTRL::CMD_STX;
  value[idx_type] = type;
  value[idx_objid] = obj_id;
  switch (type)
  {
    case tx_t::TX_MCU_DATA:
      length = sizeof(*(m_cfg.ptr_mcu_data));
      std::memcpy(&value[idx_data],(uint8_t*)m_cfg.ptr_mcu_data, length);
      value[idx_length_l] = (uint8_t)(length >> 0);
      value[idx_length_h] = (uint8_t)(length >> 8);
      break;
    default:
      break;
  }
  //end of switch

  /*

   | SOF  | Type |obj_id| Data Length |Data          |   Checksum   | EOF  |
   | :--- |:-----|:---- | :---------- |:-------------|:-------------| :--  |
   | 0x4A |1 byte|1 byte| 2 byte(L+H) |Data 0～Data n|2 byte(crc 16)| 0x4C |

   */

  uint8_t idx = 1;
  for (uint8_t i = 1; i < (length + idx_data); i++)
  {
    idx++;
    UTL::crc16_modbus_update(&crc,value[i]);
  }
  value[idx++] = (uint8_t)(crc >> 0);
  value[idx++] = (uint8_t)(crc >> 8);
  value[idx++] = RCTRL::CMD_ETX;

  /*
  std::string s{};
  for (uint8_t i = 0; i < value.size(); i++)
  {
    //s += std::to_string(static_cast<int>(m_receiveData.data[i])) + " ";
    char hex[5];
    std::snprintf(hex, sizeof(hex), "%02X", value[i]);
    s += hex;
    s += " ";
  }
  LOG_PRINT("packet! [%s]",s.c_str());
  */
  return m_cfg.ptr_comm->SendCmd(value.data(), (uint32_t)value.size());
}



void api_remote::ThreadJob()
{
  doRunStep();
#ifndef _USE_HW_RTOS
  m_cfg.ptr_comm->ReceiveProcess();
#endif
}


void api_remote::doRunStep()
{
  using tx_t = RCTRL::TX_TYPE;

  enum : uint8_t {
    STEP_INIT,STEP_TODO,STEP_TIMEOUT,STEP_RESET_COMM_ALARM,
    STEP_STATE_UPDATE,STEP_STATE_UPDATE_START,STEP_STATE_UPDATE_WAIT,STEP_STATE_UPDATE_END,
  };
  constexpr uint8_t step_retry_max = 3;
  constexpr uint32_t step_wait_delay = 50;


  switch(m_step.GetStep())
  {
    case STEP_INIT:
    {
      m_step.SetStep(STEP_TODO);
    }
    break;

    /*######################################################
      to do
    ######################################################*/
    case STEP_TODO:
    {
      if(m_step.msgQ.Available())
      {
        LOG_PRINT("STEP_TODO m_step.msgQ.Available()");
        volatile uint8_t step{};
        m_step.msgQ.Get((uint8_t*)&step);
        m_step.SetStep(step);
      }
    }
    break;
    /*######################################################
      timeout
    ######################################################*/
    case STEP_TIMEOUT:
    {
      LOG_PRINT("STEP_TIMEOUT recovery result[%d]",  m_cfg.ptr_comm->Recovery());

      m_step.SetStep(STEP_TODO);
    }
    break;
    /*######################################################
      STEP_STATE_UPDATE
    ######################################################*/
    case STEP_STATE_UPDATE:
    {
      m_step.wait_resp = false;
      m_step.wait_step = 0;
      m_step.retry_cnt = 0;

      m_step.SetStep(STEP_STATE_UPDATE_START);
    }
    break;

    case STEP_STATE_UPDATE_START:
    {
      if (SendData(tx_t::TX_MCU_DATA) == ERROR_SUCCESS)
        m_step.SetStep(STEP_STATE_UPDATE_WAIT);
      else
      {
        LOG_PRINT("STEP_STATE_UPDATE_START send data failed!");
        m_step.SetStep(STEP_TIMEOUT);
      }
    }
    break;

    case STEP_STATE_UPDATE_WAIT:
    {
      if(m_step.LessThan(step_wait_delay))
        break;

      if (m_cfg.ptr_comm->IsAvailableComm())
        m_step.SetStep(STEP_STATE_UPDATE_END);
      else
      {
        if (m_step.retry_cnt++ < step_retry_max)
        {
          m_step.SetStep(STEP_STATE_UPDATE_WAIT); // timer reset
          break;
        }
        m_step.SetStep(STEP_TIMEOUT);
        LOG_PRINT("STEP_TIMEOUT retry[%d]",  m_step.retry_cnt);
      }
    }
    break;

    case STEP_STATE_UPDATE_END:
    {

      m_step.SetStep(STEP_TODO);
    }
    break;

    default:
      break;
  }
  // end of switch


}



void api_remote::ProcessCmd(RCTRL::uart_remote::packet_st& data)
{
  this->m_receiveData = data;
  using cmd_t = RCTRL::CMD_TYPE;
  enum : uint8_t {
      STEP_INIT,STEP_TODO,STEP_TIMEOUT,STEP_RESET_COMM_ALARM,
      STEP_STATE_UPDATE,STEP_STATE_UPDATE_START,STEP_STATE_UPDATE_WAIT,STEP_STATE_UPDATE_END,
    };


  cmd_t rx_cmd = (cmd_t)m_receiveData.type;
  LOG_PRINT("ProcessCmd cmd type[%d]",  rx_cmd);
  switch (rx_cmd) {
    case cmd_t::CMD_READ_MCU_DATA:
      m_step.msgQ.Put(STEP_STATE_UPDATE);
      break;

    case cmd_t::CMD_OK_RESPONSE:
      break;
    default:
      break;
  }


}
