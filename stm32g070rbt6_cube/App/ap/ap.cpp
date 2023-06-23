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

task_dat task_data;



/****************************************************
  1. ap instances
 ****************************************************/
// engine layer
MOTOR::uart_moons moons_comm;

std::array<MOTOR::enMotor_moons, AP_OBJ::MOTOR_MAX> moons_motors{
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

    cfg = {};
    cfg.instance_no = AP_OBJ::MOTOR_R;
    cfg.ptr_apReg = &mcu_reg;
    cfg.ptr_cfgDat = &ap_cfgdata;
    //cfg.p_apAxisDat = &axis_data;
    cfg.ptr_comm = &moons_comm;
    cfg.motor_param.Init();
    moons_motors[AP_OBJ::MOTOR_R].Init(cfg);

  }


  /* control motors */
  {
    using namespace MOTOR;

    cnMotors::cfg_t cfg = {0,};
    cfg.ptr_motor = moons_motors.data();
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



  {

    using type_t = sequece_idx_data_st::linetype_e;
    using idx_t = pos_data_st::idx_e;

    constexpr uint8_t data_cnt = 20/*APDAT_SEQ_CNT_MAX*/;

    std::array<sequece_idx_data_st, data_cnt> line_datas = {0,};
    LOG_PRINT("sequece_idx_data_st size [%d] , start address[0x%X]", sizeof(sequece_idx_data_st{}), flash_data_start_addr);
    uint16_t idx = 0;
    uint32_t pre_time = millis();

    //1. earze
    if (flashErase(flash_data_start_addr, data_cnt* APDAT_SEQ_LENGTH) == false)
      LOG_PRINT("flash_data_start_addr[0x%X] fail length[%d] , pass ms[%d]", flash_data_start_addr, (data_cnt* APDAT_SEQ_LENGTH), millis()-pre_time  );
    else
    {
      for(auto& elm: line_datas)
      {
        elm.idx = idx++;
        elm.next_line = 1;
        elm.line_type = type_t::lt_sequence;
        elm.pos_data_idx = idx_t::mdi_null;
        elm.entry_setout = 2;
        elm.exit_setout = 3;
        elm.entry_delay = 4;
        elm.exit_delay = 5;
        elm.condition_in = 6;
        elm.condition_pass_line = 7;
        elm.condition_fail_line = 8;

        if (task_data.WriteData(idx++, elm) == false)
          LOG_PRINT("WriteData fail index[%d]", (idx - 1) );

      }
      LOG_PRINT("sequece %d line data write [%d]ms", data_cnt,  millis()-pre_time);

    }

  }

#ifdef _USE_HW_CLI
cliAdd("app", cliApp);
#endif

}




void  apMain(void)
{
  uint32_t pre_time;

  pre_time = millis();

  LOG_PRINT("start! main loop");
  //LOG_PRINT("tasks size = %d",sizeof(tasks));
  while (1)
  {

    if (millis()-pre_time >= 1000)
    {
      pre_time = millis();
      ledToggle(_DEF_LED1);
      //logPrintf(">>hdma_usart1_rx.Instance->CNDTR %d \n",hdma_usart1_rx.Instance->CNDTR);
    }

    updateApReg();

    // non-block
    motors.ThreadJob();

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

#if 0

    if (uartAvailable(_DEF_UART1) > 0)
    {
      uint8_t rx_data;

      rx_data = uartRead(_DEF_UART1);
      uartPrintf(_DEF_UART1, "rx data : 0x%02X (%c)\n", rx_data, rx_data);
    }
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
      cliPrintf("motor R [%d] \n", &moons_motors[2]);
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
      uint8_t axis_id = constrain((uint8_t)args->getData(1), 1, 4);
      result = moons_motors[(axis_id-1)].MotorOnOff(true);
      if (result == ERROR_SUCCESS)
      {
        cliPrintf("motor Axis[%d] On \n", axis_id);
        ret = true;
      }
    }
    else if (args->isStr(0, "motor_off") == true)
    {
      uint8_t axis_id = constrain((uint8_t)args->getData(1), 1, 4);
      result = moons_motors[(axis_id-1)].MotorOnOff(false);
      if (result == ERROR_SUCCESS)
      {
        cliPrintf("motor Axis[%d] Off \n", axis_id);
        ret = true;
      }
    }
    else if (args->isStr(0, "motor_state") == true)
    {
      uint8_t axis_idx = ((uint8_t)args->getData(1) - 1) ;
      motors.GetMotorState((AP_OBJ::MOTOR)axis_idx);

      cliPrintf("motor GetMotorState[%d] \n", axis_idx);
      ret = true;
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
    cliPrintf( "app motor_state [0:3] \n");
    cliPrintf( "app motor_off [axis] \n");
    cliPrintf( "app oplamp [0:2] [0:1 0 off, 1 on] \n");
    cliPrintf( "app run [axis][100:0 speed][step] \n");
    cliPrintf( "app rel [axis][100:0 speed][step] \n");
  }


}
#endif


