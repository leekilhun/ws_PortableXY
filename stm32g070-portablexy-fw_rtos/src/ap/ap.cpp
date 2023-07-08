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


firm_version_t *p_boot_ver = (firm_version_t *)(FLASH_ADDR_BOOT_VER);
firm_version_t *p_firm_ver = (firm_version_t *)(FLASH_ADDR_FW_VER);


static void updateApReg();
static void updateErr();
static void eventOpPanel();
static void updateLamp();
//static bool machineSelfTest();


static void threadTask(void const *argument);
static void threadCmd(void const *argument);
static void threadReceiveNonBlock(void const *argument);
#ifdef _USE_HW_CLI
static void threadCli(void const *argument);
#endif

/****************************************************
  0. mcu data and register
 ****************************************************/
ap_reg mcu_reg;
ap_dat ap_cfgdata;
ap_io mcu_io;

taskDat task_data;
mcu_data_st mcu_data;


/****************************************************
  1. ap instances
 ****************************************************/
// engine layer
MOTOR::uart_moons moons_comm;
RCTRL::uart_remote remote_comm;

#ifdef APP_USE_MOTOR_R

std::array<MOTOR::enMotor_moons, AP_OBJ::MOTOR_MAX> moons_motors{
  M_SetMotorId(AP_OBJ::MOTOR_X),M_SetMotorId(AP_OBJ::MOTOR_Y),M_SetMotorId(AP_OBJ::MOTOR_R)
};
#else
std::array<MOTOR::enMotor_moons, AP_OBJ::MOTOR_MAX> moons_motors{
  M_SetMotorId(AP_OBJ::MOTOR_X),M_SetMotorId(AP_OBJ::MOTOR_Y)
};//,M_SetMotorId(AP_OBJ::MOTOR_R)
#endif
enOp op_panel;


// control
cnAuto autoManager;
//cnJob process;
cnTasks tasks;
MOTOR::cnMotors motors;


// user interface.
api_remote remote_pc;



void  apInit(void)
{

  /* rtos initial*/
    {
      /**/
      osThreadDef(threadCmd, threadCmd, _HW_DEF_RTOS_THREAD_PRI_CMD, 0, _HW_DEF_RTOS_THREAD_MEM_CMD);
      if (osThreadCreate(osThread(threadCmd), NULL) != NULL)
      {
        LOG_PRINT("threadCmd : OK");
      }
      else
      {
        LOG_PRINT("threadCmd : Fail");
      }
      /**/
      osThreadDef(threadReceiveNonBlock, threadReceiveNonBlock, _HW_DEF_RTOS_THREAD_PRI_RECEIVE, 0, _HW_DEF_RTOS_THREAD_MEM_RECEIVE);
      if (osThreadCreate(osThread(threadReceiveNonBlock), NULL) != NULL)
      {
        LOG_PRINT("threadReceiveNonBlock : OK");
      }
      else
      {
        LOG_PRINT("threadReceiveNonBlock : Fail");
      }

      /**/
      osThreadDef(threadTask, threadTask, _HW_DEF_RTOS_THREAD_PRI_TASK, 0, _HW_DEF_RTOS_THREAD_MEM_TASK);
      if (osThreadCreate(osThread(threadTask), NULL) != NULL)
      {
        LOG_PRINT("threadTask : OK");
      }
      else
      {
        LOG_PRINT("threadTask : Fail");
      }


      /**/
      osThreadDef(threadCli, threadCli, _HW_DEF_RTOS_THREAD_PRI_CLI, 0, _HW_DEF_RTOS_THREAD_MEM_CLI);
      if (osThreadCreate(osThread(threadCli), NULL) != NULL)
      {
        LOG_PRINT("threadCli : OK");
      }
      else
      {
        LOG_PRINT("threadCLi : Fail");
      }


    }


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

  {
    using namespace RCTRL;
    uart_remote::cfg_t cfg{};
    cfg.ch = HW_UART_PC;
    cfg.baud = 115200;
    remote_comm.Init(cfg);
  }

  /* operating panel sw initial */
  {
    enOp::cfg_t cfg = {};
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

#ifdef APP_USE_MOTOR_R
    cfg = {};
    cfg.instance_no = AP_OBJ::MOTOR_R;
    cfg.ptr_apReg = &mcu_reg;
    cfg.ptr_cfgDat = &ap_cfgdata;
    //cfg.p_apAxisDat = &axis_data;
    cfg.ptr_comm = &moons_comm;
    cfg.motor_param.Init();
    moons_motors[AP_OBJ::MOTOR_R].Init(cfg);
#endif
  }


  /* control motors */
  {
    using namespace MOTOR;

    cnMotors::cfg_t cfg = {};
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
    cnAuto::cfg_t auto_cfg = {};
    auto_cfg.ptr_apReg = &mcu_reg;
    //auto_cfg.p_apLog = &mcu_log;
    auto_cfg.ptr_op =&op_panel;
    auto_cfg.ptr_io = &mcu_io;;
    auto_cfg.ptr_motors = &motors;
    autoManager.Init(auto_cfg);
  }

  /* task jos initial */
  {
      cnTasks::cfg_t cfg = {};
      cfg.ptr_apReg = &mcu_reg;
      cfg.ptr_io = &mcu_io;
      cfg.ptr_motors = &motors;
      cfg.ptr_taskDat = &task_data;
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


  /*remote control and monitor*/
    {
      api_remote::cfg_t cfg{};
      cfg.ptr_auto = &autoManager;
      cfg.ptr_cfg_data = &ap_cfgdata;
      cfg.ptr_comm = &remote_comm;
      cfg.ptr_io = &mcu_io;
      cfg.ptr_mcu_data = &mcu_data;
      cfg.ptr_mcu_reg = &mcu_reg;
      cfg.ptr_motors = &motors;
      cfg.ptr_task = &tasks;

      remote_pc.Init(cfg);
    }



  /*Assign Obj */
  mcu_io.Init();


  uint8_t idx = 0;
  //write configuration data
  {
    ap_dat::dat_t data[]
    {
        {20,20000},{56,20000},{20,20000},{20,2000},
        {282,100},{100,100},{100,100},{100,100}
    };
    ap_cfgdata.WriteData(ap_dat::mt_x_turn_dist, data[idx++]);
    ap_cfgdata.WriteData(ap_dat::mt_y_turn_dist, data[idx++]);
    ap_cfgdata.WriteData(ap_dat::mt_r_turn_dist, data[idx++]);
    ap_cfgdata.WriteData(ap_dat::cfg_4, data[idx++]);
    ap_cfgdata.WriteData(ap_dat::sycn_rate, data[idx++]);
    ap_cfgdata.WriteData(ap_dat::default_accdec, data[idx++]);
    ap_cfgdata.WriteData(ap_dat::cfg_7, data[idx++]);
    ap_cfgdata.WriteData(ap_dat::cfg_8, data[idx++]);
  }











  {

    using type_t = sequece_idx_data_st::linetype_e;
    using idx_t = pos_data_st::idx_e;

    constexpr uint8_t data_cnt = 20/*APDAT_SEQ_CNT_MAX*/;

    std::array<sequece_idx_data_st, data_cnt> line_datas = {};
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
        elm.idx = idx;
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

        if (task_data.WriteData((idx), elm) == false)
          LOG_PRINT("WriteData fail index[%d]", (idx) );

        idx++;

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
  uint32_t blink_time;
  constexpr uint32_t no_error_blink_time = 1'000;
  constexpr uint32_t error_blink_time = 300;

  pre_time = millis();

  LOG_PRINT("start! main loop");
  //LOG_PRINT("tasks size = %d",sizeof(tasks));
  while (1)
  {
    /* set led blink time */
    if (mcu_reg.error_reg.no_error)
      blink_time = no_error_blink_time;
    else
      blink_time = error_blink_time;

    if (millis()-pre_time >= blink_time)
    {
      pre_time = millis();
      ledToggle(_DEF_LED1);
      //logPrintf(">>hdma_usart1_rx.Instance->CNDTR %d \n",hdma_usart1_rx.Instance->CNDTR);
    }

    updateApReg();

    eventOpPanel();

    updateLamp();

    updateErr();


    delay(2);
  }

}

#ifdef _USE_HW_CLI
/*######################################################
  cli
  ######################################################*/
void threadCli(void const *argument)
{
  UNUSED(argument);
  while (1)
  {

    cliMain();

    delay(2);
  }
}
#endif


/*######################################################
  Task
  ######################################################*/
void threadTask(void const *argument)
{
  UNUSED(argument);
  while (1)
  {
    autoManager.ThreadJob();
    tasks.ThreadJob();


    delay(2);
  }
}


/*######################################################
  command
  ######################################################*/
void threadCmd(void const *argument)
{
  UNUSED(argument);
  while (1)
  {
    remote_pc.ThreadJob();
    motors.ThreadJob();

    delay(2);
  }
}

/*######################################################
   receive communication
  ######################################################*/
void threadReceiveNonBlock(void const *argument)
{
  UNUSED(argument);

  while (1)
  {
    remote_comm.ReceiveProcess();
    moons_comm.ReceiveProcess();

    delay(2);
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

  enum :uint8_t {io_1d,io_2d,io_3d, io_max};
  enum :uint8_t {motor_1d,motor_2d,motor_3d,motor_4d, motor_max};
  enum :uint8_t {data_1d,data_2d,data_3d,data_4d, data_max};
  mcu_data.reg_sys = mcu_io.m_sysio.system_io;
  mcu_data.reg_state = mcu_reg.state_reg.ap_state;
  mcu_data.reg_option = mcu_reg.option_reg.ap_option;
  mcu_data.reg_err = mcu_reg.error_reg.ap_error;
  mcu_data.io_in[io_1d] = mcu_io.m_in.data;
  mcu_data.io_in[io_2d] = 0;
  mcu_data.io_in[io_3d] = 0;
  mcu_data.io_out[io_1d] = mcu_io.m_out.data;
  mcu_data.io_out[io_2d] = 0;
  mcu_data.io_out[io_3d] = 0;
  mcu_data.motor_cnt= (uint16_t)AP_OBJ::MOTOR_MAX;
  mcu_data.motor_pulse[motor_1d] = moons_motors[AP_OBJ::MOTOR_X].m_motorData.encoder_position;//500'000;
  mcu_data.motor_status[motor_1d] = moons_motors[AP_OBJ::MOTOR_X].m_motorData.drv_status.sc_status;
  mcu_data.motor_pulse[motor_2d] = moons_motors[AP_OBJ::MOTOR_Y].m_motorData.encoder_position;
  mcu_data.motor_status[motor_2d] = moons_motors[AP_OBJ::MOTOR_Y].m_motorData.drv_status.sc_status;
#ifdef APP_USE_MOTOR_R
  mcu_data.motor_pulse[motor_3d] = moons_motors[AP_OBJ::MOTOR_R].m_motorData.encoder_position;//2'500'000;
  mcu_data.motor_status[motor_3d] = moons_motors[AP_OBJ::MOTOR_R].m_motorData.drv_status.sc_status;
#else
  mcu_data.motor_pulse[motor_3d] = 0;
  mcu_data.motor_status[motor_3d] = 0;
#endif
  mcu_data.motor_pulse[motor_4d] = 0;
  mcu_data.motor_status[motor_4d] = 0;
  //mcu_data.datas[data_1d] = 134'217'728;
  //mcu_data.datas[data_2d] = 2'281'736'192;
  //mcu_data.datas[data_3d] = 2'290'124'928;
  //mcu_data.datas[data_4d] = 2'290'649'224;


}

/*
 * system error register, communication error and try to recovery
 */
void updateErr()
{
  /* 3. motor communication */
  if (motors.GetCommStatus() == 0)
  {
    mcu_reg.error_reg.no_resp_mot = false;
  }
  else
  {
    mcu_reg.error_reg.no_resp_mot = true;
  }






  if (mcu_reg.error_reg.ap_error > 1)
  {
    mcu_reg.error_reg.no_error = false;
  }
  else
    mcu_reg.error_reg.no_error = true;


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
    if (args->isStr(0, "mt_on") == true)
    {
      uint8_t axis_id = constrain((uint8_t)args->getData(1), 1, 4);
      result = moons_motors[(axis_id-1)].MotorOnOff(true);
      if (result == ERROR_SUCCESS)
      {
        cliPrintf("motor Axis[%d] On \n", axis_id);
        ret = true;
      }
    }
    else if (args->isStr(0, "mt_off") == true)
    {
      uint8_t axis_id = constrain((uint8_t)args->getData(1), 1, 4);
      result = moons_motors[(axis_id-1)].MotorOnOff(false);
      if (result == ERROR_SUCCESS)
      {
        cliPrintf("motor Axis[%d] Off \n", axis_id);
        ret = true;
      }
    }
    else if (args->isStr(0, "mt_state") == true)
    {
      uint8_t axis_id = constrain((uint8_t)args->getData(1), 1, 4);
      motors.GetMotorState((AP_OBJ::MOTOR)(axis_id-1));

      cliPrintf("motor GetMotorState[%d] \n", (axis_id-1));
      ret = true;
    }
    else if (args->isStr(0, "mt_show") == true)
    {
      uint8_t axis_id = constrain((uint8_t)args->getData(1), 1, 4);
      MOTOR::enMotor_moons::moons_data_t* ptr_data =  &moons_motors[(axis_id-1)].m_motorData;
      cliPrintf( "motor id[%d] \n", (axis_id));
      cliPrintf( "motor al_code [%d] \n",ptr_data->al_code.al_status );
      cliPrintf( "motor drv_status [%d] \n",ptr_data->drv_status.sc_status);
      cliPrintf( "motor driver_board_inputs [%d] \n",ptr_data->al_code.al_status);
      cliPrintf( "motor driver_board_inputs [%d] \n",ptr_data->driver_board_inputs);
      cliPrintf( "motor encoder_position [%d] \n",ptr_data->encoder_position);
      cliPrintf( "motor immediate_abs_position [%d] \n",ptr_data->immediate_abs_position);
      cliPrintf( "motor abs_position_command [%d] \n",ptr_data->abs_position_command);
      ret = true;
    }
    else if (args->isStr(0, "line_run") == true)
    {
      uint16_t line_idx = args->getData(1);
      if (task_data.ReadData())
      {
        tasks.RunLineTask(line_idx);
        for (auto&[i, dat]  : task_data.task_dat)
        {
          cliPrintf("line : idx[%d]\tnext_line[%d]\tline_type[%d]\tpos_idx[%d]\ten_out[%d]\tex_out\n"
              ,  dat.idx, dat.next_line, dat.line_type, dat.pos_data_idx, dat.entry_setout, dat.exit_setout );

        }
       /* for (auto& elm : task_data.task_dat)
        {
          cliPrintf("line : idx[%d]\tnext_line[%d]\tline_type[%d]\tpos_idx[%d]\ten_out[%d]\tex_out\n"
              ,  elm.line_data.idx, elm.line_data.next_line, elm.line_data.line_type, elm.line_data.pos_data_idx, elm.line_data.entry_setout, elm.line_data.exit_setout );

        }*/

        cliPrintf("line_run [%d] \n", line_idx);
        ret = true;
      }
      else
      {
        cliPrintf("line_run ReadData fail \n");
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
    cliPrintf( "app mt_on [axis] \n");
    cliPrintf( "app mt_off [axis] \n");
    cliPrintf( "app mt_state [axis] \n");
    cliPrintf( "app mt_show [axis] \n");
    cliPrintf( "app line_run [idx] \n");
    cliPrintf( "app oplamp [0:2] [0:1 0 off, 1 on] \n");
    cliPrintf( "app run [axis][100:0 speed][step] \n");
    cliPrintf( "app rel [axis][100:0 speed][step] \n");
  }


}
#endif


