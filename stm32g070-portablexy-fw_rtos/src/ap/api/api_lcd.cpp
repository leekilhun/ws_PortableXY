/*
 * api_lcd.cpp
 *
 *  Created on: 2023. 6. 10.
 *      Author: gns2l
 */




#include "ap.hpp"
#include "api_lcd.hpp"

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

errno_t api_lcd::SendData(NXLCD::TX_TYPE type, uint8_t obj_id )
{
  uint16_t length = 0;
  enum :uint8_t {
    idx_type,  
    idx_length_l,idx_length_h,
    idx_data
  };
  std::array<uint8_t, NXLCD::CMD_MAX_PACKET_LENGTH> value = { };
  using tx_t = NXLCD::TX_TYPE;
  value[idx_type] = type;
  switch (type)
  {
    case tx_t::TX_MCU_DATA:
      length = sizeof(*(m_cfg.ptr_mcu_data));
      value[idx_length_l] = (uint8_t)(length >> 0);
      value[idx_length_h] = (uint8_t)(length >> 8);
      std::memcpy(&value[idx_data],(uint8_t*)m_cfg.ptr_mcu_data, length);
      break;    

    case tx_t::TX_OK_RESPONSE:
    default:
      break;
  }
  //end of switch

  return (m_cfg.ptr_comm->SendNextionData(type, value.data(), length + 1 )? ERROR_SUCCESS : ERROR_FAIL);
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

  constexpr uint8_t step_retry_max = 3;
  constexpr uint32_t step_wait_delay = 100;


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
      LOG_PRINT("STEP_TIMEOUT recovery result[%d]", m_cfg.ptr_comm->Recovery());
      m_cfg.ptr_comm->SendNextionData(tx_t::TX_LCD_END_REPARSEMODE, nullptr);
      m_step.wait_resp = false; // reset
      m_modeReparse = false;
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
      //if (m_modeReparse == false)
      {
        m_modeReparse = true;
        uint8_t set_reparse = 1;
        LOG_PRINT("STEP_STATE_UPDATE SendNextionData");
        m_cfg.ptr_comm->SendNextionData(tx_t::TX_LCD_START_REPARSEMODE, &set_reparse);
      }

      m_step.SetStep(STEP_STATE_UPDATE_START);
    }
    break;

    case STEP_STATE_UPDATE_START:
    {
      if(m_step.LessThan(step_wait_delay))
        break;

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
      m_cfg.ptr_comm->SendNextionData(tx_t::TX_LCD_END_REPARSEMODE, nullptr);
      m_modeReparse = false;
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


  /*

   */

  cmd_t rx_cmd = (cmd_t)m_receiveData.type;
  LOG_PRINT("ProcessCmd cmd type[%d], resp_ms[%d]", rx_cmd, m_receiveData.resp_ms);
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
  case cmd_t::CMD_RET_CURR_PAGE_NO:
      LcdUpdate();
      break;
  case cmd_t::CMD_CTRL_REQ_BEEP:
      //if (m_cfg.ptr_mcu_reg->option_reg.use_beep)
      //  buzzerBeep(1, 2);
      break;

  default:
      break;
  }
  // end of switch
}



// mcu 상태 정보를 보낸다
void api_lcd::LcdUpdate(){
	using PAGE = NXLCD::uart_nextion::page_e;

	if (m_cfg.ptr_mcu_reg->state_reg.auto_ready
			 || m_cfg.ptr_auto->IsModeAuto())
	{
		if (m_currPage !=PAGE::MAIN)
		{
			//ChangePage(PAGE::MAIN);
		}
	}
  else if (m_cfg.ptr_mcu_reg->state_reg.alarm_status)
  {
  	//if (m_currPage != PAGE::ALARM)
  		//ChangePage(PAGE::ALARM);
  }

  switch (m_currPage)
  {
  case PAGE::INIT:
    __attribute__((fallthrough));
  case PAGE::SETUP:
    m_step.msgQ.Put(STEP_STATE_UPDATE);
    break;

  case PAGE::MAIN:
    __attribute__((fallthrough));
  case PAGE::MANUAL:
    __attribute__((fallthrough));
  case PAGE::TASK:
    break;

  case PAGE::TEACH:
    break;

  case PAGE::IO:
    break;

  case PAGE::LOG:
    break;

  case PAGE::SELTTEST:
    break;

  case PAGE::ALARM:
    break;

  default:
    break;
  }
}




