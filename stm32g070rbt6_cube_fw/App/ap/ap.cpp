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


static void updateApReg();
static void updateErr();
static void eventOpPanel();
static void updateLamp();
//static bool machineSelfTest();

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

MOTOR::enMotor_moons moons_motors[AP_OBJ::MOTOR_MAX]{
  M_SetMotorId(AP_OBJ::MOTOR_X),M_SetMotorId(AP_OBJ::MOTOR_Y),M_SetMotorId(AP_OBJ::MOTOR_R)
};
enOp op_panel;


// control
cnAuto autoManager;
//cnJob process;
cnTasks tasks;
MOTOR::cnMotors motors;




void  apInit(void)
{

#ifdef _USE_HW_CLI
  if(cliOpen(HW_UART_LCD, 115200))
    LOG_PRINT("cliOpen Success! ch[%d], baud[%d]",HW_UART_LCD, 115200);
  else
    LOG_PRINT("cliOpen Fail! ch[%d], baud[%d] \n",HW_UART_LCD, 115200);
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
    cfg.ptr_comm = &moons_comm;
    cfg.motor_param.Init();
    moons_motors[AP_OBJ::MOTOR_X].Init(cfg);


    cfg = {};
    cfg.instance_no = AP_OBJ::MOTOR_Y;
    cfg.ptr_apReg = &mcu_reg;
    cfg.ptr_cfgDat = &ap_cfgdata;
    //cfg.p_apAxisDat = &axis_data;
    cfg.ptr_comm = &moons_comm;
    cfg.motor_param.Init();
    moons_motors[AP_OBJ::MOTOR_Y].Init(cfg);

  }


  /* control motors */
  {
    using namespace MOTOR;

    cnMotors::cfg_t cfg = {0,};
    cfg.ptr_motor = moons_motors;
    //cfg.p_apAxisDat =  &axis_data;
    cfg.ptr_comm = &moons_comm;
    cfg.ptr_cfgDat = &ap_cfgdata;
    cfg.ptr_io = &mcu_io;
    cfg.ptr_apReg = &mcu_reg;
    motors.Init(cfg);
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

  /* task jos initial */
  {
      cnTasks::cfg_t cfg = {0, };
      cfg.ptr_apReg = &mcu_reg;
      cfg.ptr_io = &mcu_io;
      cfg.ptr_motors = &motors;
      //cfg.p_Cyl = cyl;
      //cfg.p_Vac = vac;
      cfg.ptr_op = &op_panel;
      cfg.ptr_AutoManger = &autoManager;
      //cfg.p_apAxisDat = &axis_data;
      //cfg.p_apCylDat = &cyl_data;
      //cfg.p_apVacDat = &vac_data;
      //cfg.p_apCfgDat = &apCfg_data;
      //cfg.p_apSeqDat = &seq_data;
      //cfg.p_linkPosDat = &linkPose_data;
      tasks.Init(cfg);
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

  LOG_PRINT("start! main loop");
  LOG_PRINT("tasks size = %d",sizeof(tasks));
  while (1)
  {

    if (millis()-pre_time >= 1000)
    {
      pre_time = millis();
      ledToggle(_DEF_LED1);
    }

    updateApReg();

    // non-block
    //motors.ThreadJob();

    // non-block�ڵ�.
    //op_lcd.ThreadJob();

    // non-block�ڵ�
    //remote_pc.ThreadJob();

    // non-block�ڵ�
    tasks.ThreadJob();

    eventOpPanel();

    // non-block�ڵ�
    autoManager.ThreadJob();

    updateLamp();

    updateErr();


#ifdef _USE_HW_CLI
    cliMain();
#endif

  }

}



/*
 * operating button event
 */
void eventOpPanel()
{
  /* 1. update op panel state*/
  op_panel.UpdateState();

  using reg_status_t = ap_reg::state_e;
  using panelbtn_t = enOp::panel_e;
  using opstatus_t = enOp::status_e;

  if(autoManager.IsModeAuto())
  {
    // auto
    constexpr panelbtn_t ext_start      = panelbtn_t::SW_START;
    constexpr panelbtn_t ext_stop       = panelbtn_t::SW_STOP;
    constexpr panelbtn_t ext_reset      = panelbtn_t::SW_RESET;
    constexpr panelbtn_t ext_n_estop    = panelbtn_t::SW_ESTOP;

    constexpr opstatus_t system_init   = opstatus_t::INIT;

    /* check estop */
    if (autoManager.SetEStop(op_panel.GetPressed(ext_n_estop)))/* mcu_io.m_sysio.estop_Sw */
    {
      return;
    }

    /* handshaking*/
    //Check if the system can be started
    volatile bool is_system_init = autoManager.GetOPStatus() == system_init;
    if (is_system_init || !mcu_reg.state_reg.system_origin_cplt)//not
    {

#if 0
      if (is_module_init == false)
      {
        return;
      }
      else
      {
        // case request system initial
        if(mcu_io.m_sysio.start_sw && mcu_io.m_sysio.start_sw && mcu_io.m_sysio.reset_sw)
        {
          //motors.AlarmReset();
          //tasks.m_requestMotor_idx=AP_OBJ::MOTOR_MAX;
          //tasks.Initialize();
          return;
        }

        /* reset */
        if (op_panel.GetPressed(ext_reset))
        {
          //motors.AlarmReset();
          autoManager.ResetSw();
        }
      }
#endif
    }
    else
    {
      // case work request
      if (op_panel.GetPressed(ext_start))
      {
        autoManager.StartSw();
      }
      // case stop request
      else if (op_panel.GetPressed(ext_stop))
      {
        autoManager.StopSw();
      }
      // case reset request
      else if (op_panel.GetPressed(ext_reset))
      {
        autoManager.ResetSw();
        //tasks.m_workCnt = 0;
        //tasks.m_product.Init();
        //motors.AlarmReset();
      }
    }


  }
  else
  {
    //manual
    /*check request initial*/
    /*
    if (autoManager.IsRequestSystemInit())
    {
      mcu_reg.state_reg.request_initial = false;
      //motors.AlarmReset();
      //tasks.m_requestMotor_idx=AP_OBJ::MOTOR_MAX;
      //tasks.Initialize();
    }
    */
    /* op key*/
    if (op_panel.GetPressed(panelbtn_t::SW_ESTOP))
    {
      mcu_reg.SetReg_State(reg_status_t::EMG_STOP, true);
      return;
    }
    else
    {
      mcu_reg.SetReg_State(reg_status_t::EMG_STOP, false);
    }

    if (op_panel.GetPressed(panelbtn_t::SW_START))
    {
      autoManager.StartSw();
    }
    else if (op_panel.GetPressed(panelbtn_t::SW_STOP))
    {
      mcu_reg.state_reg.auto_running = false;
      autoManager.StopSw();
    }
    else if (op_panel.GetPressed(panelbtn_t::SW_RESET))
    {
      mcu_reg.state_reg.alarm_status = false;
      autoManager.ResetSw();
      //motors.AlarmReset();
      uint32_t pre_ms = millis();
      while(op_panel.GetPressed(panelbtn_t::SW_RESET))
      {
        if(millis() - pre_ms >= (1000*3))
        {
          // do system initialize
          op_panel.LampOnOff(enOp::LAMP_RESET,true);
          op_panel.LampOnOff(enOp::LAMP_STOP,true);
          delay(1000);
          //tasks.m_requestMotor_idx=AP_OBJ::MOTOR_MAX;
          //tasks.Initialize();
          return;
        }
      }
    }
  }
}

/*
 * operating lamp and buzzer
 */
void updateLamp()
{

}

/*
 * mcu register update
 */
void updateApReg()
{
  /* 1. io register */
  mcu_io.Update_io();

  /* 2. mcu register */
  using reg = ap_reg::state_e;

  mcu_reg.SetReg_State(reg::DETECT_AREA_SEN, !mcu_io.m_in.in_safety_sensor/*IsOff(io::in_safety_sensor)*/);
  //mcu_reg.SetReg_State(reg::MOTOR_ON, motors.IsMotorOn());
  //mcu_reg.SetReg_State(reg::SYSTEM_INIT_COMPLETED, tasks.IsInitailzed());

}

/*
 * system error register, communication error and try to recovery
 */
void updateErr()
{

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
    else if (args->isStr(0, "io_in") == true)
    {
      cliPrintf("mcu io in [%d] \n", mcu_io.m_in.data);
      ret = true;
    }
  }
  else if (args->argc == 2)
  {
    
    int result = 0;
    if (args->isStr(0, "motor_on") == true)
    {
      uint8_t axis_id = (uint8_t)args->getData(1);
      if (axis_id == 1)
      {
        result = moons_motors[0].MotorOnOff(true);
      }
      else if (axis_id == 2)
      {
        result = moons_motors[1].MotorOnOff(true);
      }
      if (result == ERROR_SUCCESS)
      {
        cliPrintf("motor Axis[%d] On \n", axis_id);
        ret = true;
      }
    }
    else if (args->isStr(0, "motor_off") == true)
    {
      uint8_t axis_id = (uint8_t)args->getData(1);
      if (axis_id == 1)
      {
        result = moons_motors[0].MotorOnOff(false);
      }
      else if (axis_id == 2)
      {
        result = moons_motors[1].MotorOnOff(false);
      }
      if (result == ERROR_SUCCESS)
      {
        cliPrintf("motor Axis[%d] Off \n", axis_id);
        ret = true;
      }
    }
    else if (args->isStr(0, "io_on") == true)
    {
      uint8_t idx = (uint8_t)args->getData(1);
         
      if (mcu_io.OutputOn(idx + AP_DEF_START_OUT_ADDR)== ERROR_SUCCESS)
      {
        cliPrintf("io_on [%d] success! \n", idx + AP_DEF_START_OUT_ADDR);
        ret = true;
      }
    }
    else if (args->isStr(0, "io_off") == true)
    {
      uint8_t idx = (uint8_t)args->getData(1);
      if (mcu_io.OutputOff(idx + AP_DEF_START_OUT_ADDR) == ERROR_SUCCESS)
      {
        cliPrintf("io_off [%d] success! \n", idx + AP_DEF_START_OUT_ADDR);
        ret = true;
      }
    }
  }
  else if (args->argc == 3)
  {
    uint8_t lamp_idx = constrain((uint8_t)args->getData(1),(uint8_t)enOp::lamp_e::LAMP_START,(uint8_t)enOp::lamp_e::LAMP_RESET);
    bool on_off = args->getData(2)==0?false:true;
    if (args->isStr(0, "oplamp") == true)
    {
      op_panel.LampOnOff((enOp::lamp_e)lamp_idx, on_off);
      {
        cliPrintf("LampOnOff index[%d][%d]\n",lamp_idx, on_off);
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
    cliPrintf( "app io_in \n");
    cliPrintf( "app io_on [0:7] \n");
    cliPrintf( "app io_off [0:7] \n");
    cliPrintf( "app motor_on [axis] \n");
    cliPrintf( "app motor_off [axis] \n");
    cliPrintf( "app oplamp [0:2] [0:1 0 off, 1 on] \n");
    cliPrintf( "app run [axis][100:0 speed][step] \n");
    cliPrintf( "app rel [axis][100:0 speed][step] \n");
  }


}
#endif


