/*
 * ap.hpp
 *
 *  Created on: Apr 23, 2023
 *      Author: gns2l
 */

#ifndef INCLUDE_AP_HPP_
#define INCLUDE_AP_HPP_

/*

#include "utils.h"
#include "uart.h"
#include "led.h"
*/


#include "ap_def.h"

#include "def_obj.hpp"

/* ap object*/
#include "ap_utils.hpp"


/* ap object */
/* register -> communication -> basic unit(engine) -> combined unit(engine) -> control */
//register + io_manager
#include "ap_reg.hpp"
#include "ap_dat.hpp"
#include "ap_io.hpp"

//basic
#include "uart_moons.hpp"
#include "uart_nextion.hpp"
#include "uart_remote.hpp"

#include "enOp.hpp"
#include "enCyl.hpp"
#include "enVac.hpp"
#include "enMotor_moons.hpp"



struct prc_step_t
{
  volatile uint8_t curr_step{};
  volatile uint8_t pre_step{};
  volatile uint8_t wait_step{};
  volatile uint32_t prev_ms{};
  volatile uint32_t elap_ms{};
  volatile uint8_t retry_cnt{};
  UTL::_que<uint8_t> msgQ;
  volatile bool wait_resp{};//true - wait step complete, false - completed step

  inline void SetStep(uint8_t step){
  	elap_ms = millis() - prev_ms;
    prev_ms = millis();
  	pre_step = curr_step;
    curr_step = step;
  }

  inline uint8_t GetStep() const{
    return curr_step;
  }

  inline uint32_t ElapTime() const {
  	return elap_ms;
  }

  inline bool LessThan(uint32_t msec){
  	elap_ms = millis() - prev_ms;
  	if (elap_ms < msec)
  		return true;
  	else
  		return false;
  }

  inline bool MoreThan(uint32_t msec){
  		return !LessThan(msec);
  }


  inline bool Available() const {
  	return !wait_resp;
  }


  inline bool IsInStep (uint8_t step) {
  	if (msgQ.m_Head)
  	{
  		if (msgQ.m_Buff[msgQ.m_Head - 1] != step
  				|| msgQ.m_Buff[msgQ.m_Head - 1] != step + 1
					|| msgQ.m_Buff[msgQ.m_Head - 1] != step + 2)
  		{
  			return true;
  		}
  	}
  	return false;
  }

} ;
//control

#include "cnMotors.hpp"
#include "cnAuto.hpp"
#include "cnTasks.hpp"

//machine test
#include "machine_test.hpp"

//api
#include "api_lcd.hpp"
#include "api_remote.hpp"



void  apInit(void);
void  apMain(void);

#endif /* INCLUDE_AP_HPP_ */
