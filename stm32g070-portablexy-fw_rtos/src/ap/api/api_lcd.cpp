/*
 * api_lcd.cpp
 *
 *  Created on: 2023. 6. 10.
 *      Author: gns2l
 */




#include "ap.hpp"
#include "api_lcd.hpp"

     

errno_t api_lcd::SendData(NXLCD::TX_TYPE type, uint8_t obj_id )
{
 

  return m_cfg.ptr_comm->SendNextionData(type, nullptr, 0 );
}



void api_lcd::ThreadJob()
{
  doRunStep();
#ifndef _USE_HW_RTOS
  m_cfg.ptr_comm->ReceiveProcess();
#endif
}


void api_lcd::doRunStep()
{
  using tx_t = NXLCD::TX_TYPE;

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




void api_lcd::ProcessCmd(NXLCD::uart_nextion::packet_st& data)
{
  this->m_receiveData = data;
  using cmd_t = NXLCD::CMD_TYPE;
  //using tx_t = NXLCD::TX_TYPE;
  enum : uint8_t
  {
    STEP_INIT,
    STEP_TODO,
    STEP_TIMEOUT,
    STEP_RESET_COMM_ALARM,
    STEP_STATE_UPDATE,
    STEP_STATE_UPDATE_START,
    STEP_STATE_UPDATE_WAIT,
    STEP_STATE_UPDATE_END,
  };

  /*

   */

  cmd_t rx_cmd = (cmd_t)m_receiveData.type;
  LOG_PRINT("ProcessCmd cmd type[%d]", rx_cmd);
  switch (rx_cmd)
  {
  case cmd_t::CMD_READ_MCU_DATA:
      m_step.msgQ.Put(STEP_STATE_UPDATE);
      break;

  case cmd_t::CMD_READ_FIRM_INFO:
      break;
  case cmd_t::CMD_CTRL_MOT_ORIGIN:
      break;
  case cmd_t::CMD_CTRL_MOT_ONOFF:
      break;
  case cmd_t::CMD_CTRL_MOT_MOVE:
      break;
  case cmd_t::CMD_CTRL_MOT_STOP:
      break;
  case cmd_t::CMD_CTRL_MOT_JOG:
      break;
  case cmd_t::CMD_CTRL_MOT_LIMIT:
      break;
  case cmd_t::CMD_CTRL_MOT_ZEROSET:
      break;
  case cmd_t::CMD_CTRL_MOT_RELMOVE:
      break;
  case cmd_t::CMD_CTRL_MOT_CLEAR_ALARM:
      break;
  case cmd_t::CMD_CTRL_MOT_CHANGE_VEL:
      break;
  case cmd_t::CMD_CTRL_MOT_MOVE_VEL:
      break;
  case cmd_t::CMD_CTRL_MOT_RELMOVE_VEL:
      break;
  case cmd_t::CMD_CTRL_MOT_VEL_JOG:
      break;
  case cmd_t::CMD_CTRL_BKCMD:
      break;
  case cmd_t::CMD_CTRL_REQ_BEEP:
      break;

  default:
      break;
  }
  // end of switch
}