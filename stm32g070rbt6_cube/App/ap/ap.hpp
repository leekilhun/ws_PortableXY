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
#include "def.err.hpp"

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
