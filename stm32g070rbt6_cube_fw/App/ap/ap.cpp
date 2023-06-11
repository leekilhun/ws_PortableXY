/*
 * ap.cpp
 *
 *  Created on: Apr 23, 2023
 *      Author: gns2l
 */

#include "ap.hpp"


#ifdef _USE_HW_CLI
static void cliApp(cli_args_t *args);
#endif

/****************************************************
  0. mcu data and register
 ****************************************************/
ap_reg mcu_reg;
ap_dat ap_cfgdata;
ap_io mcu_io;

/****************************************************
  1. ap instances
 ****************************************************/
// engine layer
MOTOR::uart_moons moons_comm;

MOTOR::enMotor_moons moons_motors[AP_OBJ::MOTOR_MAX]{ M_SetMotorId(AP_OBJ::MOTOR_X),M_SetMotorId(AP_OBJ::MOTOR_Y) };
enOp op_panel;


// control
cnAuto autoManager;
//cnJob process;
//cnTasks tasks;
MOTOR::cnMotors motors;




void  apInit(void)
{

#ifdef _USE_HW_CLI
  if(cliOpen(HW_UART_LCD, 115200))
    logPrintf(">>cliOpen Success! ch[%d], baud[%d] \n",HW_UART_LCD, 115200);
  else
    logPrintf(">>cliOpen Fail! ch[%d], baud[%d] \n",HW_UART_LCD, 115200);
#endif


  // uart
  {
    using namespace MOTOR;
    uart_moons::cfg_t cfg{};
    cfg.ch = HW_UART_MOTOR;
    cfg.baud = 115200;
    moons_comm.Init(cfg);
  }

  /* operating panel sw initial */
  {
    enOp::cfg_t cfg = {0,};
    cfg.ptr_mcu_io      = &mcu_io;
    cfg.ptr_mcu_reg     = &mcu_reg;
    cfg.sw_pin_start    = _GPIO_OP_SW_START;
    cfg.sw_pin_stop     = _GPIO_OP_SW_STOP;
    cfg.sw_pin_reset    = _GPIO_OP_SW_RESET;
    cfg.sw_pin_estop    = _GPIO_OP_SW_ESTOP;

    cfg.lamp_pin_start  = _GPIO_OP_LAMP_START;
    cfg.lamp_pin_stop   = _GPIO_OP_LAMP_STOP;
    cfg.lamp_pin_reset  = _GPIO_OP_LAMP_RESET;
    op_panel.Init(cfg);
  }

  /* motor initial */
  {
    using namespace MOTOR;

    enMotor_moons::cfg_t cfg { };
    cfg.instance_no = AP_OBJ::MOTOR_X;
    cfg.ptr_apReg = &mcu_reg;
    cfg.ptr_cfgDat = &ap_cfgdata;
    //cfg.p_apAxisDat = &axis_data;
    cfg.p_comm = &moons_comm;
    cfg.motor_param.Init();
    moons_motors[AP_OBJ::MOTOR_X].Init(cfg);


    cfg = {};
    cfg.instance_no = AP_OBJ::MOTOR_Y;
    cfg.ptr_apReg = &mcu_reg;
    cfg.ptr_cfgDat = &ap_cfgdata;
    //cfg.p_apAxisDat = &axis_data;
    cfg.p_comm = &moons_comm;
    cfg.motor_param.Init();
    moons_motors[AP_OBJ::MOTOR_Y].Init(cfg);

  }



  /* automanager initial */
  {
    cnAuto::cfg_t auto_cfg = {0, };
    auto_cfg.ptr_apReg = &mcu_reg;
    //auto_cfg.p_apLog = &mcu_log;
    auto_cfg.ptr_op =&op_panel;
    auto_cfg.ptr_io = &mcu_io;;
    auto_cfg.ptr_motors = &motors;
    autoManager.Init(auto_cfg);
  }


  /*Assign Obj */
  mcu_io.Init();


#ifdef _USE_HW_CLI
  cliAdd("app", cliApp);
#endif

}


void  apMain(void)
{
  uint32_t pre_time;

  pre_time = millis();

  logPrintf("[start main loop] \n");
  while (1)
  {

    if (millis()-pre_time >= 1000)
    {
      pre_time = millis();
      ledToggle(_DEF_LED1);
    }

    cliMain();

  }

}



#ifdef _USE_HW_CLI
void cliApp(cli_args_t *args)
{
  bool ret = false;

  if (args->argc == 1)
  {
    if (args->isStr(0, "info") == true)
    {
      cliPrintf("motor X [%d] \n", &moons_motors[0]);
      cliPrintf("motor Y [%d] \n", &moons_motors[1]);
      ret = true;

    }
  }
  else if (args->argc == 2)
  {
    uint8_t axis_id = (uint8_t)args->getData(1);
    int ret = 0;
    if (args->isStr(0, "motor_on") == true)
    {
      if (axis_id == 1)
      {
        ret = moons_motors[0].MotorOnOff(true);
      }
      else if (axis_id == 2)
      {
        ret = moons_motors[1].MotorOnOff(true);
      }
      if (ret == ERROR_SUCCESS)
      {
        cliPrintf("motor Axis[%d] On \n", axis_id);
        ret = true;
      }
    }
    else if (args->isStr(0, "motor_off") == true)
    {
      if (axis_id == 1)
      {
        ret = moons_motors[0].MotorOnOff(false);
      }
      else if (axis_id == 2)
      {
        ret = moons_motors[1].MotorOnOff(false);
      }
      if (ret == ERROR_SUCCESS)
      {
        cliPrintf("motor Axis[%d] Off \n", axis_id);
        ret = true;
      }
    }

  }
  else if (args->argc == 4)
  {
    uint8_t id = (uint8_t)args->getData(1);
    uint32_t speed = (uint32_t)args->getData(2);
    int step_pulse = (uint32_t)args->getData(3);
    if (args->isStr(0, "run") == true)
    {
      //motor.SetStep(step_pulse, speed);
      //motor.RunAndWait();
      {
        cliPrintf("ID[%d] run step[%d] speed[%d] : OK\n",id, step_pulse, speed);
        ret = true;
      }
    }
    if (args->isStr(0, "rel") == true)
    {
      //motor.SetRel(step_pulse, speed);
      //motor.RunAndWait();
      {
        cliPrintf("ID[%d] rel step[%d] speed[%d] : OK\n",id, step_pulse, speed);
        ret = true;
      }
    }


  }


  if (ret == false)
  {
    cliPrintf( "app info \n");
    cliPrintf( "app motor_on [axis] \n");
    cliPrintf( "app motor_off [axis] \n");
    cliPrintf( "app run [axis][100:0 speed][step] \n");
    cliPrintf( "app rel [axis][100:0 speed][step] \n");

  }


}
#endif


