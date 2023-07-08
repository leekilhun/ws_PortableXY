/*
 * cnMotors.hpp
 *
 *  Created on: 2023. 6. 10.
 *      Author: gns2l
 */

#ifndef AP__INC_CNMOTORS_HPP_
#define AP__INC_CNMOTORS_HPP_



namespace MOTOR
{

  constexpr uint16_t CN_MOTORS_VELOCITY_RPM_MAX    = 3000;
  constexpr uint16_t CN_MOTORS_SYNC_DEC_RATE_MAX   = 2000;
  constexpr uint16_t CN_MOTORS_TURN_DIST_MM_MAX    = 1000;
  constexpr uint16_t CN_MOTORS_GEAR_RATE_MIN       = 2000;
  constexpr uint16_t CN_MOTORS_GEAR_RATE_MAX       = 20000;

  class cnMotors
  {
    /****************************************************
     *  data
     ****************************************************/
  public:
    struct cfg_t  {
      //axis_dat* p_apAxisDat;
      ap_reg* ptr_apReg{};
      enMotor_moons *ptr_motor{};
      uart_moons *ptr_comm{};
      ap_io *ptr_io{};
      ap_dat* ptr_cfgDat{};

      cfg_t() = default;

      // copy constructor
      cfg_t(const cfg_t& rhs) = default;
      // copy assignment operator
      cfg_t& operator=(const cfg_t& rhs) = default;
      // move constructor
      cfg_t(cfg_t&& rhs) = default;
      // move assignment operator
      cfg_t& operator=(cfg_t&& rhs) = default;

    };

    union CN_MOTORS_COMM
    {
      uint8_t comm_err{};
      struct
      {
        unsigned  idx_0_comm_err : 1;       // = 0x00000001;
        unsigned  idx_1_comm_err : 1;       // = 0x00000002;
        unsigned  idx_2_comm_err : 1;       // = 0x00000004;
        unsigned  idx_3_comm_err : 1;       // = 0x00000008;
        unsigned  idx_4_comm_err : 1;       // = 0x00000010;
        unsigned  idx_5_comm_err : 1;       // = 0x00000020;
        unsigned  idx_6_comm_err : 1;       // = 0x00000040;
        unsigned  idx_7_comm_err : 1;       // = 0x00000080;
      };
    };

  private:
    cfg_t m_cfg;
    uint32_t m_request_ms;
    CN_MOTORS_COMM m_commStatus;
    uint8_t m_motorCommErrCnt[AP_OBJ::MOTOR_MAX];

  public:
    uint8_t m_requestMotor_idx;
    prc_step_t m_step;
    uint8_t m_errCnt;
    std::array<bool, AP_OBJ::MOTOR_MAX> m_IsOriginCplt;
    /****************************************************
     *  Constructor
     ****************************************************/
  public:
    cnMotors () : m_cfg{}, m_request_ms{}
    ,m_commStatus{}, m_motorCommErrCnt{}
    ,m_requestMotor_idx{}, m_step{}, m_errCnt{},m_IsOriginCplt{}
    {

    }

    virtual  ~cnMotors (){}
    /****************************************************
     *  func
     ****************************************************/
  public:
    inline int Init(cnMotors::cfg_t &cfg)  {
      m_cfg = cfg;
      LOG_PRINT("Init Success!");
      return 0;
    }


    inline void ResetAlarmNon()  {
      constexpr uint8_t STEP_STATE_ALARM_RESET = 8;
      m_step.SetStep(STEP_STATE_ALARM_RESET);
    }

    inline void GetMotorState(AP_OBJ::MOTOR id = AP_OBJ::MOTOR_MAX)
    {
      m_requestMotor_idx = (uint8_t)id;
      m_step.SetStep(4);
    }

    //
    inline void ThreadJob()
    {
      doRunStep();
#ifndef _USE_HW_RTOS
      m_cfg.ptr_comm->ReceiveProcess();
#endif
    }


    inline void doRunStep()
    {
      enum : uint8_t {
        STEP_INIT,STEP_TODO,STEP_TIMEOUT,STEP_RESET_COMM_ALARM,
        STEP_STATE_UPDATE,STEP_STATE_UPDATE_START,STEP_STATE_UPDATE_WAIT,STEP_STATE_UPDATE_END,
        STEP_STATE_ALARM_RESET,STEP_STATE_ALARM_RESET_START,STEP_STATE_ALARM_RESET_WAIT,STEP_STATE_ALARM_RESET_END,
      };
      constexpr uint8_t step_retry_max = 3;
      constexpr uint8_t comm_timeout_max = 200;
      constexpr uint32_t step_wait_delay = 20;

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
          if (m_cfg.ptr_comm->IsAvailableComm())
          {
            m_step.SetStep(STEP_STATE_UPDATE);
            break;
          }

          //check timeout
          if (m_step.LessThan(comm_timeout_max))
            break;

          LOG_PRINT("STEP_TODO timeout!");
          m_step.SetStep(STEP_TIMEOUT);
        }
        break;
        /*######################################################
          timeout
        ######################################################*/
        case STEP_TIMEOUT:
        {
          if (m_cfg.ptr_motor[m_requestMotor_idx].IsCommAlarm())
          {
            m_step.SetStep(STEP_STATE_ALARM_RESET);
            break;
          }
          else
          {
            SetCommStatus((AP_OBJ::MOTOR)m_requestMotor_idx, true);
            LOG_PRINT("STEP_TIMEOUT idx [%d] , recovery result[%d]",  m_requestMotor_idx, m_cfg.ptr_comm->Recovery());
          }

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

          m_requestMotor_idx = m_requestMotor_idx % AP_OBJ::MOTOR_MAX;
          m_step.SetStep(STEP_STATE_UPDATE_START);

          /*
           check is_comm_alarm state  -> comm_reset.
           */
          if (m_cfg.ptr_motor[m_requestMotor_idx].IsCommAlarm())
          {
            LOG_PRINT("STEP_STATE_UPDATE_START idx [%d],al_code[%d]",m_requestMotor_idx, m_cfg.ptr_motor[m_requestMotor_idx].m_motorData.al_code);
            m_step.SetStep(STEP_STATE_ALARM_RESET);
          }

        }
        break;

        case STEP_STATE_UPDATE_START:
        {
          if (m_cfg.ptr_motor[m_requestMotor_idx].GetMotorData()== ERROR_SUCCESS)
            m_step.SetStep(STEP_STATE_UPDATE_WAIT);
          else
          {
            LOG_PRINT("STEP_STATE_UPDATE_START GetMotorData failed! idx [%d]",  m_requestMotor_idx);
            m_step.SetStep(STEP_TIMEOUT);
          }
        }
        break;

        case STEP_STATE_UPDATE_WAIT:
        {
          if(m_step.LessThan(50))
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
            LOG_PRINT("STEP_TIMEOUT idx [%d] , retry[%d]",  m_requestMotor_idx, m_step.retry_cnt);

          }
        }
        break;

        case STEP_STATE_UPDATE_END:
        {
          SetCommStatus((AP_OBJ::MOTOR)m_requestMotor_idx, false);
          ++m_requestMotor_idx;

          if (m_requestMotor_idx == AP_OBJ::MOTOR_MAX )
          {
            m_requestMotor_idx = 0;
            m_step.SetStep(STEP_TODO);
          }
          else
            m_step.SetStep(STEP_STATE_UPDATE);

        }
        break;

        /*######################################################
          STEP_STATE_ALARM_RESET
        ######################################################*/
        case STEP_STATE_ALARM_RESET:
        {
          m_step.wait_resp = false;
          m_step.wait_step = 0;
          m_step.retry_cnt = 0;

          m_step.SetStep(STEP_STATE_ALARM_RESET_START);
        }
        break;

        case STEP_STATE_ALARM_RESET_START:
        {
          if(m_step.LessThan(step_wait_delay))
            break;

          LOG_PRINT("STEP_STATE_ALARM_RESET_START m_requestMotor_idx[%d]",m_requestMotor_idx );
          m_cfg.ptr_motor[m_requestMotor_idx].ResetAlarmNon();
          m_step.SetStep(STEP_STATE_ALARM_RESET_WAIT);
        }
        break;

        case STEP_STATE_ALARM_RESET_WAIT:
        {
          if(m_step.LessThan(step_wait_delay))
            break;

          m_step.SetStep(STEP_STATE_ALARM_RESET_END);
        }
        break;

        case STEP_STATE_ALARM_RESET_END:
        {
          m_step.retry_cnt = 0;
          m_step.SetStep(STEP_TODO);
        }
        break;

        default:
          break;
      }
      // end of switch

    }


    inline uint8_t SetCommStatus(AP_OBJ::MOTOR motor_idx, bool set_err = true)
    {
      if ( set_err )
        m_commStatus.comm_err |= 1 << (int)motor_idx;
      else
        m_commStatus.comm_err &=  ~(1<< (int)motor_idx);
      return m_commStatus.comm_err;
    }


    inline uint8_t GetCommStatus() const{
      return m_commStatus.comm_err;
    }

    inline int MotorOnOff(bool on_off = true, AP_OBJ::MOTOR motor_idx = AP_OBJ::MOTOR_MAX){
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        int ret = 0;
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_X].MotorOnOff(on_off);
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_Y].MotorOnOff(on_off);
#ifdef APP_USE_MOTOR_R
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_R].MotorOnOff(on_off);
#endif
        return ret;
      }
      return m_cfg.ptr_motor[motor_idx].MotorOnOff(on_off);
    }

    inline int MotorClearAlarm(AP_OBJ::MOTOR motor_idx = AP_OBJ::MOTOR_MAX){
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        int ret = 0;
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_X].ClearState();
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_Y].ClearState();
#ifdef APP_USE_MOTOR_R
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_R].ClearState();
#endif
        return ret;
      }
      return m_cfg.ptr_motor[motor_idx].ClearState();
    }


    inline bool IsConnected() const{
      return m_cfg.ptr_comm->m_Isconnected;
    }

    inline bool IsMotorOn(AP_OBJ::MOTOR motor_idx = AP_OBJ::MOTOR_MAX){
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        bool ret = true;
        ret = m_cfg.ptr_motor[AP_OBJ::MOTOR_X].IsMotorOn();
        ret &= m_cfg.ptr_motor[AP_OBJ::MOTOR_Y].IsMotorOn();
#ifdef APP_USE_MOTOR_R
        ret &= m_cfg.ptr_motor[AP_OBJ::MOTOR_R].IsMotorOn();
#endif
        return ret;
      }
      return m_cfg.ptr_motor[motor_idx].IsMotorOn();
    }

    inline bool IsNoErrStopAndZero(AP_OBJ::MOTOR motor_idx = AP_OBJ::MOTOR_MAX){
      volatile bool ret = true;
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        ret &= m_cfg.ptr_motor[AP_OBJ::MOTOR_X].IsNoErrStopAndZero();
        ret &= m_cfg.ptr_motor[AP_OBJ::MOTOR_Y].IsNoErrStopAndZero();
#ifdef APP_USE_MOTOR_R
        ret &= m_cfg.ptr_motor[AP_OBJ::MOTOR_R].IsNoErrStopAndZero();
#endif
        return ret;
      }
      return m_cfg.ptr_motor[motor_idx].IsNoErrStopAndZero();
    }

    inline bool IsAlarmState(AP_OBJ::MOTOR motor_idx = AP_OBJ::MOTOR_MAX){
      if (motor_idx < AP_OBJ::MOTOR::MOTOR_MAX)
        return m_cfg.ptr_motor[motor_idx].IsAlarmState();
      if(m_cfg.ptr_motor[AP_OBJ::MOTOR_X].IsAlarmState())
        return true;
      if(m_cfg.ptr_motor[AP_OBJ::MOTOR_Y].IsAlarmState())
        return true;
#ifdef APP_USE_MOTOR_R
      if(m_cfg.ptr_motor[AP_OBJ::MOTOR_R].IsAlarmState())
        return true;
#endif
      return false;
    }

    inline bool IsMotorHoming(AP_OBJ::MOTOR motor_idx = AP_OBJ::MOTOR_MAX){
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        bool ret = true;
        ret = m_cfg.ptr_motor[AP_OBJ::MOTOR_X].IsMotorHoming();
        ret &= m_cfg.ptr_motor[AP_OBJ::MOTOR_Y].IsMotorHoming();
#ifdef APP_USE_MOTOR_R
        ret &= m_cfg.ptr_motor[AP_OBJ::MOTOR_R].IsMotorHoming();
#endif
        return ret;
      }
      return m_cfg.ptr_motor[motor_idx].IsMotorHoming();
    }

    inline bool IsOriginCpltAll(){
#ifdef APP_USE_MOTOR_R
        return (m_IsOriginCplt[AP_OBJ::MOTOR_X]& m_IsOriginCplt[AP_OBJ::MOTOR_Y]& m_IsOriginCplt[AP_OBJ::MOTOR_R]);
#else
        return (m_IsOriginCplt[AP_OBJ::MOTOR_X]& m_IsOriginCplt[AP_OBJ::MOTOR_Y]);
#endif
      }

    inline bool IsOriginProcessCplt(AP_OBJ::MOTOR motor_idx = AP_OBJ::MOTOR_MAX){
      if (motor_idx < AP_OBJ::MOTOR::MOTOR_MAX)
      {
        if (IsMotorHoming(motor_idx) == false && m_IsOriginCplt[motor_idx] == false)
        {
          m_IsOriginCplt[motor_idx] =  true;
          return m_IsOriginCplt[motor_idx];
        }

        return false;
      }
      else
      {
        if (IsMotorHoming(AP_OBJ::MOTOR_X) == false && m_IsOriginCplt[AP_OBJ::MOTOR_X] == false)
        {
          m_IsOriginCplt[AP_OBJ::MOTOR_X] =  true;
        }
        if (IsMotorHoming(AP_OBJ::MOTOR_Y) == false && m_IsOriginCplt[AP_OBJ::MOTOR_Y] == false)
        {
          m_IsOriginCplt[AP_OBJ::MOTOR_Y] =  true;
        }
#ifdef APP_USE_MOTOR_R
        if (IsMotorHoming(AP_OBJ::MOTOR_R) == false && m_IsOriginCplt[AP_OBJ::MOTOR_R] == false)
        {
          m_IsOriginCplt[AP_OBJ::MOTOR_R] =  true;
        }
        return (m_IsOriginCplt[AP_OBJ::MOTOR_X]& m_IsOriginCplt[AP_OBJ::MOTOR_Y]& m_IsOriginCplt[AP_OBJ::MOTOR_R]);
#else
        return (m_IsOriginCplt[AP_OBJ::MOTOR_X]& m_IsOriginCplt[AP_OBJ::MOTOR_Y]);
#endif
      }
    }

    inline bool IsLimitPos(AP_OBJ::MOTOR motor_idx, bool is_ccw = true) //b active
    {
      return m_cfg.ptr_motor[motor_idx].IsLimitPos(is_ccw);
    }

    inline bool IsMotorRun(AP_OBJ::MOTOR motor_idx = AP_OBJ::MOTOR_MAX){
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        bool ret = true;
        ret = m_cfg.ptr_motor[AP_OBJ::MOTOR_X].IsMotorRun();
        ret &= m_cfg.ptr_motor[AP_OBJ::MOTOR_Y].IsMotorRun();
#ifdef APP_USE_MOTOR_R
        ret &= m_cfg.ptr_motor[AP_OBJ::MOTOR_R].IsMotorRun();
#endif
        return ret;
      }
      return m_cfg.ptr_motor[motor_idx].IsMotorRun();
    }

    inline bool IsMotorStop(AP_OBJ::MOTOR motor_idx = AP_OBJ::MOTOR_MAX){
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        bool ret = true;
        ret = m_cfg.ptr_motor[AP_OBJ::MOTOR_X].IsMotorStop();
        ret &= m_cfg.ptr_motor[AP_OBJ::MOTOR_Y].IsMotorStop();
#ifdef APP_USE_MOTOR_R
        ret &= m_cfg.ptr_motor[AP_OBJ::MOTOR_R].IsMotorStop();
#endif
        return ret;
      }
      return m_cfg.ptr_motor[motor_idx].IsMotorStop();
    }

    inline bool IsInPosition (AP_OBJ::MOTOR motor_idx, int cmd_dist_10um){
      return m_cfg.ptr_motor[motor_idx].IsInPosition(cal_dist_10um_to_pulse(motor_idx, cmd_dist_10um));
    }

    inline bool IsInPose(AP_OBJ::MOTOR motor_idx = AP_OBJ::MOTOR_MAX){
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        bool ret = true;
        ret = m_cfg.ptr_motor[AP_OBJ::MOTOR_X].IsInPose();
        ret &=m_cfg.ptr_motor[AP_OBJ::MOTOR_Y].IsInPose();
#ifdef APP_USE_MOTOR_R
        ret &=m_cfg.ptr_motor[AP_OBJ::MOTOR_R].IsInPose();
#endif
        return ret;
      }
      return m_cfg.ptr_motor[motor_idx].IsInPose();
    }


    inline int MotorZeroEncode(AP_OBJ::MOTOR motor_idx = AP_OBJ::MOTOR_MAX){
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        int ret = 0;
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_X].ClearEncoder();
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_Y].ClearEncoder();
#ifdef APP_USE_MOTOR_R
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_R].ClearEncoder();
#endif
        return ret;
      }
      return m_cfg.ptr_motor[motor_idx].ClearEncoder();
    }

    inline int MoveToOffset(AP_OBJ::MOTOR motor_idx = AP_OBJ::MOTOR_MAX){

      /*must assign motor id*/
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        int ret = 0;
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_X].MoveToOffset();
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_Y].MoveToOffset();
#ifdef APP_USE_MOTOR_R
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_R].MoveToOffset();
#endif
        return ret;
      }
      return m_cfg.ptr_motor[motor_idx].MoveToOffset();
    }


    inline int MoveToLimit(AP_OBJ::MOTOR motor_idx = AP_OBJ::MOTOR_MAX){
      /*must assign motor id*/
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        int ret = 0;
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_X].MoveToLimit();
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_Y].MoveToLimit();
#ifdef APP_USE_MOTOR_R
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_R].MoveToLimit();
#endif
        return ret;
      }
      return m_cfg.ptr_motor[motor_idx].MoveToLimit();
    }


    inline int MotorJogMoveStop(AP_OBJ::MOTOR motor_idx = AP_OBJ::MOTOR_MAX){
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        int ret = 0;
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_X].JogStop();
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_Y].JogStop();
#ifdef APP_USE_MOTOR_R
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_R].JogStop();
#endif
        return ret;
      }
      return m_cfg.ptr_motor[motor_idx].JogStop();
    }

    inline int MotorMoveStop(AP_OBJ::MOTOR motor_idx = AP_OBJ::MOTOR_MAX){
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        int ret = 0;
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_X].MoveStop();
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_Y].MoveStop();
#ifdef APP_USE_MOTOR_R
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_R].MoveStop();
#endif
        return ret;
      }
      return m_cfg.ptr_motor[motor_idx].MoveStop();
    }

    inline int MotorJogMove(AP_OBJ::MOTOR motor_idx = AP_OBJ::MOTOR_MAX, bool is_cw = true){
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        int ret = 0;
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_X].JogMove(is_cw);
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_Y].JogMove(is_cw);
#ifdef APP_USE_MOTOR_R
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_R].JogMove(is_cw);
#endif
        return ret;
      }
      return m_cfg.ptr_motor[motor_idx].JogMove(is_cw);
    }

    inline int MotorJogMove(AP_OBJ::MOTOR motor_idx, uart_moons::speed_t& param, bool is_cw){
      param.speed = constrain(param.speed,1,CN_MOTORS_VELOCITY_RPM_MAX);
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        int ret = 0;
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_X].JogMove(param, is_cw);
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_Y].JogMove(param, is_cw);
#ifdef APP_USE_MOTOR_R
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_R].JogMove(param, is_cw);
#endif

        return ret;
      }
      return m_cfg.ptr_motor[motor_idx].JogMove(param,is_cw);
    }

    inline int SetParamDataMove(AP_OBJ::MOTOR motor_idx, uint32_t rpm, uint32_t acc, uint32_t dec){
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        int ret = 0;
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_X].SetParamDataMove(constrain(rpm,1,CN_MOTORS_VELOCITY_RPM_MAX), acc, dec);
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_Y].SetParamDataMove(constrain(rpm,1,CN_MOTORS_VELOCITY_RPM_MAX), acc, dec);
#ifdef APP_USE_MOTOR_R
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_R].SetParamDataMove(constrain(rpm,1,CN_MOTORS_VELOCITY_RPM_MAX), acc, dec);;
#endif
        return ret;
      }
      return m_cfg.ptr_motor[motor_idx].SetParamDataMove(constrain(rpm,1,CN_MOTORS_VELOCITY_RPM_MAX), acc, dec);
    }

    inline int SetParamDataInit(AP_OBJ::MOTOR motor_idx, MOTOR::enMotor_moons::origin_param_t& param){
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        int ret = 0;
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_X].SetOriginParam(param);
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_Y].SetOriginParam(param);
#ifdef APP_USE_MOTOR_R
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_R].SetOriginParam(param);
#endif
        return ret;
      }
      return m_cfg.ptr_motor[motor_idx].SetOriginParam(param);
    }


    inline int Move(AP_OBJ::MOTOR motor_idx, int cmd_dist_10um){
      /*must assign motor id*/
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        if (!IsOriginCpltAll())
          return -1;
        int ret = 0;
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_X].MoveAbsolutive(cal_dist_10um_to_pulse(AP_OBJ::MOTOR_X, cmd_dist_10um));
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_Y].MoveAbsolutive(cal_dist_10um_to_pulse(AP_OBJ::MOTOR_Y, cmd_dist_10um));
#ifdef APP_USE_MOTOR_R
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_R].MoveAbsolutive(cal_dist_10um_to_pulse(AP_OBJ::MOTOR_R, cmd_dist_10um));
#endif
        return ret;
      }
      if (!(m_IsOriginCplt[motor_idx]))
        return -1;
      return m_cfg.ptr_motor[motor_idx].MoveAbsolutive(cal_dist_10um_to_pulse(motor_idx, cmd_dist_10um));
    }

    inline int Move(AP_OBJ::MOTOR motor_idx, int cmd_dist_10um, uint32_t rpm, uint32_t acc = 100, uint32_t dec = 100){

      /*must assign motor id*/
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        if (!IsOriginCpltAll())
          return -1;

        int ret = 0;
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_X].MoveAbsolutive(
            cal_dist_10um_to_pulse(AP_OBJ::MOTOR_X, cmd_dist_10um)
            , constrain(rpm,1,CN_MOTORS_VELOCITY_RPM_MAX), acc, dec);
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_Y].MoveAbsolutive(
            cal_dist_10um_to_pulse(AP_OBJ::MOTOR_Y, cmd_dist_10um)
            , constrain(rpm,1,CN_MOTORS_VELOCITY_RPM_MAX), acc, dec);
#ifdef APP_USE_MOTOR_R
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_R].MoveAbsolutive(
            cal_dist_10um_to_pulse(AP_OBJ::MOTOR_R, cmd_dist_10um)
            , constrain(rpm,1,CN_MOTORS_VELOCITY_RPM_MAX), acc, dec);
#endif
        return ret;
      }
      if (!m_IsOriginCplt[motor_idx])
        return -1;
      return m_cfg.ptr_motor[motor_idx].MoveAbsolutive( cal_dist_10um_to_pulse(motor_idx, cmd_dist_10um), constrain(rpm,1,CN_MOTORS_VELOCITY_RPM_MAX), acc, dec);
    }

    inline int RelMove(AP_OBJ::MOTOR motor_idx, int cmd_dist_10um){
      /*must assign motor id*/
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        if (!IsOriginCpltAll())
          return -1;
        int ret = 0;
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_X].MoveRelative(cal_dist_10um_to_pulse(AP_OBJ::MOTOR_X, cmd_dist_10um));
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_Y].MoveRelative(cal_dist_10um_to_pulse(AP_OBJ::MOTOR_Y, cmd_dist_10um));
#ifdef APP_USE_MOTOR_R
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_R].MoveRelative(cal_dist_10um_to_pulse(AP_OBJ::MOTOR_R, cmd_dist_10um));
#endif
        return ret;
      }
      if (!m_IsOriginCplt[motor_idx])
        return -1;
      return m_cfg.ptr_motor[motor_idx].MoveRelative(cal_dist_10um_to_pulse(motor_idx, cmd_dist_10um));
    }

    inline int RelMove(AP_OBJ::MOTOR motor_idx, int cmd_dist_10um, uint32_t rpm, uint32_t acc = 100, uint32_t dec = 100){

      /*must assign motor id*/
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        if (!IsOriginCpltAll())
          return -1;
        int ret = 0;
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_X].MoveRelative(
            cal_dist_10um_to_pulse(AP_OBJ::MOTOR_X, cmd_dist_10um),
            constrain(rpm,1,CN_MOTORS_VELOCITY_RPM_MAX), acc, dec);
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_Y].MoveRelative(
            cal_dist_10um_to_pulse(AP_OBJ::MOTOR_Y, cmd_dist_10um),
            constrain(rpm,1,CN_MOTORS_VELOCITY_RPM_MAX), acc, dec);
#ifdef APP_USE_MOTOR_R
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_R].MoveRelative(
            cal_dist_10um_to_pulse(AP_OBJ::MOTOR_R, cmd_dist_10um),
            constrain(rpm,1,CN_MOTORS_VELOCITY_RPM_MAX), acc, dec);
#endif
        return ret;
      }
      if (!m_IsOriginCplt[motor_idx])
        return -1;
      return m_cfg.ptr_motor[motor_idx].MoveAbsolutive( cal_dist_10um_to_pulse(motor_idx, cmd_dist_10um), constrain(rpm,1,CN_MOTORS_VELOCITY_RPM_MAX), acc, dec);
    }

    inline int SetOutput(AP_OBJ::MOTOR motor_idx, uint16_t pin_no, bool is_on = true){
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        int ret = 0;
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_X].SetOutput(pin_no, is_on);
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_Y].SetOutput(pin_no, is_on);
#ifdef APP_USE_MOTOR_R
        ret += m_cfg.ptr_motor[AP_OBJ::MOTOR_R].SetOutput(pin_no, is_on);
#endif
        return ret;
      }
      return m_cfg.ptr_motor[motor_idx].SetOutput(pin_no, is_on);
    }


    inline int Origin(AP_OBJ::MOTOR motor_idx =AP_OBJ::MOTOR_MAX){
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        int ret = 0;
        for (int i = 0; i <AP_OBJ::MOTOR_MAX ; i++)
        {
          if (m_cfg.ptr_motor[i].OriginMotor() == ERROR_SUCCESS)
          {
            m_IsOriginCplt[i] = false;
            ret = ret + 1;
          }
        }
        return (ret == 3? ERROR_SUCCESS : -1 );
      }
      m_IsOriginCplt[motor_idx] = false;
      return m_cfg.ptr_motor[motor_idx].OriginMotor();
    }






























    inline float cal_sync_vel_dec(uint32_t source_rpm){
      float rate = 0;     ap_dat::dat_t*  p_data{};
      p_data = m_cfg.ptr_cfgDat->GetData(ap_dat::sycn_rate);
      rate = ((float)(source_rpm*100)/(float)constrain(p_data->parm1,1,CN_MOTORS_SYNC_DEC_RATE_MAX));
      return rate;
    }


    inline int cal_dist_10um_to_pulse(AP_OBJ::MOTOR motor_idx, int um10){
      ap_dat::dat_t*  p_data{};
      int ret = 0;
      int turn_dist_um = 0;
      int turn_by_pulse = 0;
      int dist_by_turn_um = 0;
      switch (motor_idx) {
        case AP_OBJ::MOTOR::MOTOR_X:
          p_data = m_cfg.ptr_cfgDat->GetData(ap_dat::mt_x_turn_dist);
          break;
        case AP_OBJ::MOTOR::MOTOR_Y:
          p_data = m_cfg.ptr_cfgDat->GetData(ap_dat::mt_y_turn_dist);
          break;
#ifdef APP_USE_MOTOR_R
        case AP_OBJ::MOTOR::MOTOR_R:
          p_data = m_cfg.ptr_cfgDat->GetData(ap_dat::mt_r_turn_dist);
          break;
#endif
        default:
          break;
      }
      turn_dist_um =um10*10;
      dist_by_turn_um = int(constrain(p_data->parm1,1,CN_MOTORS_TURN_DIST_MM_MAX))*1000;
      turn_by_pulse = (int)constrain(p_data->parm2,CN_MOTORS_GEAR_RATE_MIN,CN_MOTORS_GEAR_RATE_MAX);
      ret = int(((float)turn_dist_um/(float)dist_by_turn_um) * (float)turn_by_pulse);

      return ret;
    }


#if 0

    inline bool Recovery() {
      m_commStatus.comm_err = 0;

      m_cfg.p_motor[AP_OBJ::MOTOR_JIG].Recovery();
      m_cfg.p_motor[AP_OBJ::MOTOR_ROLL].Recovery();
      m_cfg.p_motor[AP_OBJ::MOTOR_HIGH].Recovery();

      return m_cfg.p_comm->Recovery();
    }

    inline bool IsConnected() {
      constexpr uint8_t rimit_err_cnt = 10;
      if(m_cfg.p_comm->IsConnected())
        return  (m_cfg.p_comm->GetErrCnt() > rimit_err_cnt) ? false: true;
      return false;
    }



    inline uint8_t AddErrCnt() {
      //m_commStatus.comm_err= 0;
      Recovery();
      return m_errCnt++;
    }



    inline MOTOR::enMotor_moons* GetMotorObject(AP_OBJ::MOTOR motor_idx){
      if (motor_idx < AP_OBJ::MOTOR::MOTOR_MAX)
        return &m_cfg.p_motor[motor_idx];
      return nullptr;
    }


    void UpdateMotorsState();

    inline int SetParamDataMove(AP_OBJ::MOTOR motor_idx, uint32_t rpm, uint32_t acc, uint32_t dec){
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        int ret = 0;
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_JIG].SetParamDataMove(constrain(rpm,1,CN_MOTORS_VELOCITY_RPM_MAX), acc, dec);
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_ROLL].SetParamDataMove(constrain(rpm,1,CN_MOTORS_VELOCITY_RPM_MAX), acc, dec);
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_HIGH].SetParamDataMove(constrain(rpm,1,CN_MOTORS_VELOCITY_RPM_MAX), acc, dec);
        return ret;
      }
      return m_cfg.p_motor[motor_idx].SetParamDataMove(constrain(rpm,1,CN_MOTORS_VELOCITY_RPM_MAX), acc, dec);
    }

    inline int SetParamDataInit(AP_OBJ::MOTOR motor_idx, MOTOR::enMotor_moons::origin_param_t& param){
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        int ret = 0;
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_JIG].SetOriginParam(param);
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_ROLL].SetOriginParam(param);
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_HIGH].SetOriginParam(param);
        return ret;
      }
      return m_cfg.p_motor[motor_idx].SetOriginParam(param);
    }

    inline bool IsLinkMoveCplt(){
      bool ret = true;
      ret = m_cfg.p_motor[AP_OBJ::MOTOR_JIG].IsInPose()
                           &m_cfg.p_motor[AP_OBJ::MOTOR_ROLL].IsInPose();
      return ret;
    }

    inline int LinkMove(int cmd_dist_10um, uint32_t rpm){
      int ret = 0;

      ret += m_cfg.p_motor[AP_OBJ::MOTOR_JIG].MoveRelative(
          cal_dist_10um_to_pulse(AP_OBJ::MOTOR_JIG, cmd_dist_10um)
          , constrain(rpm,1,CN_MOTORS_VELOCITY_RPM_MAX), 100, 100);
      ret += m_cfg.p_motor[AP_OBJ::MOTOR_ROLL].MoveRelative(
          cal_dist_10um_to_pulse(AP_OBJ::MOTOR_ROLL, cmd_dist_10um)
          , cal_sync_vel_dec(constrain(rpm,1,CN_MOTORS_VELOCITY_RPM_MAX)), 100, 100);

      return ret;
    }

    inline int Move(AP_OBJ::MOTOR motor_idx, int cmd_dist_10um){

      /*must assign motor id*/
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        if (!IsOriginCpltAll())
          return -1;
        int ret = 0;
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_JIG].MoveAbsolutive(cal_dist_10um_to_pulse(AP_OBJ::MOTOR_JIG, cmd_dist_10um));
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_ROLL].MoveAbsolutive(cal_dist_10um_to_pulse(AP_OBJ::MOTOR_ROLL, cmd_dist_10um));
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_HIGH].MoveAbsolutive(cal_dist_10um_to_pulse(AP_OBJ::MOTOR_HIGH, cmd_dist_10um));
        return ret;
      }
      if (!(m_IsOriginCplt[motor_idx]))
        return -1;
      return m_cfg.p_motor[motor_idx].MoveAbsolutive(cal_dist_10um_to_pulse(motor_idx, cmd_dist_10um));
    }

    inline int Move(AP_OBJ::MOTOR motor_idx, int cmd_dist_10um, uint32_t rpm, uint32_t acc = 100, uint32_t dec = 100){

      /*must assign motor id*/
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        if (!(m_IsOriginCplt[AP_OBJ::MOTOR_JIG]&m_IsOriginCplt[AP_OBJ::MOTOR_ROLL]&m_IsOriginCplt[AP_OBJ::MOTOR_HIGH]))
          return -1;

        int ret = 0;
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_JIG].MoveAbsolutive(
            cal_dist_10um_to_pulse(AP_OBJ::MOTOR_JIG, cmd_dist_10um)
            , constrain(rpm,1,CN_MOTORS_VELOCITY_RPM_MAX), acc, dec);
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_ROLL].MoveAbsolutive(
            cal_dist_10um_to_pulse(AP_OBJ::MOTOR_ROLL, cmd_dist_10um)
            , constrain(rpm,1,CN_MOTORS_VELOCITY_RPM_MAX), acc, dec);
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_HIGH].MoveAbsolutive(
            cal_dist_10um_to_pulse(AP_OBJ::MOTOR_HIGH, cmd_dist_10um)
            , constrain(rpm,1,CN_MOTORS_VELOCITY_RPM_MAX), acc, dec);
        return ret;
      }
      if (!m_IsOriginCplt[motor_idx])
        return -1;
      return m_cfg.p_motor[motor_idx].MoveAbsolutive( cal_dist_10um_to_pulse(motor_idx, cmd_dist_10um), constrain(rpm,1,CN_MOTORS_VELOCITY_RPM_MAX), acc, dec);
    }

    inline int RelMove(AP_OBJ::MOTOR motor_idx, int cmd_dist_10um){

      /*must assign motor id*/
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        if (!(m_IsOriginCplt[AP_OBJ::MOTOR_JIG]&m_IsOriginCplt[AP_OBJ::MOTOR_ROLL]&m_IsOriginCplt[AP_OBJ::MOTOR_HIGH]))
          return -1;
        int ret = 0;
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_JIG].MoveRelative(cal_dist_10um_to_pulse(AP_OBJ::MOTOR_JIG, cmd_dist_10um));
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_ROLL].MoveRelative(cal_dist_10um_to_pulse(AP_OBJ::MOTOR_ROLL, cmd_dist_10um));
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_HIGH].MoveRelative(cal_dist_10um_to_pulse(AP_OBJ::MOTOR_HIGH, cmd_dist_10um));
        return ret;
      }
      if (!m_IsOriginCplt[motor_idx])
        return -1;
      return m_cfg.p_motor[motor_idx].MoveRelative(cal_dist_10um_to_pulse(motor_idx, cmd_dist_10um));
    }

    inline int RelMove(AP_OBJ::MOTOR motor_idx, int cmd_dist_10um, uint32_t rpm, uint32_t acc = 100, uint32_t dec = 100){

      /*must assign motor id*/
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        if (!(m_IsOriginCplt[AP_OBJ::MOTOR_JIG]&m_IsOriginCplt[AP_OBJ::MOTOR_ROLL]&m_IsOriginCplt[AP_OBJ::MOTOR_HIGH]))
          return -1;
        int ret = 0;
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_JIG].MoveRelative(
            cal_dist_10um_to_pulse(AP_OBJ::MOTOR_JIG, cmd_dist_10um),
            constrain(rpm,1,CN_MOTORS_VELOCITY_RPM_MAX), acc, dec);
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_ROLL].MoveRelative(
            cal_dist_10um_to_pulse(AP_OBJ::MOTOR_ROLL, cmd_dist_10um),
            constrain(rpm,1,CN_MOTORS_VELOCITY_RPM_MAX), acc, dec);
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_HIGH].MoveRelative(
            cal_dist_10um_to_pulse(AP_OBJ::MOTOR_HIGH, cmd_dist_10um),
            constrain(rpm,1,CN_MOTORS_VELOCITY_RPM_MAX), acc, dec);
        return ret;
      }
      if (!m_IsOriginCplt[motor_idx])
        return -1;
      return m_cfg.p_motor[motor_idx].MoveAbsolutive( cal_dist_10um_to_pulse(motor_idx, cmd_dist_10um), constrain(rpm,1,CN_MOTORS_VELOCITY_RPM_MAX), acc, dec);
    }

    inline int SetOutput(AP_OBJ::MOTOR motor_idx, uint16_t pin_no, bool is_on = true){
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        int ret = 0;
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_JIG].SetOutput(pin_no, is_on);
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_ROLL].SetOutput(pin_no, is_on);
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_HIGH].SetOutput(pin_no, is_on);
        return ret;
      }
      return m_cfg.p_motor[motor_idx].SetOutput(pin_no, is_on);
    }

    inline int Origin(AP_OBJ::MOTOR motor_idx =AP_OBJ::MOTOR_MAX){
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        int ret = 0;

        for (int i = 0; i <AP_OBJ::MOTOR_MAX ; i++)
        {
          if (m_cfg.p_motor[i].OriginMotor() == ERROR_SUCCESS)
          {
            m_IsOriginCplt[i] = false;
            ret = ret + 1;
          }
        }
        return (ret == 3? ERROR_SUCCESS : -1 );
      }
      m_IsOriginCplt[motor_idx] = false;
      return m_cfg.p_motor[motor_idx].OriginMotor();
    }


    inline int MotorOnOff(bool on_off = true, AP_OBJ::MOTOR motor_idx =AP_OBJ::MOTOR_MAX){
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        int ret = 0;
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_JIG].MotorOnOff(on_off);
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_ROLL].MotorOnOff(on_off);
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_HIGH].MotorOnOff(on_off);
        return ret;
      }
      return m_cfg.p_motor[motor_idx].MotorOnOff(on_off);
    }

    inline int MotorClearAlarm(AP_OBJ::MOTOR motor_idx = AP_OBJ::MOTOR_MAX){
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        int ret = 0;
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_JIG].ClearState();
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_ROLL].ClearState();
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_HIGH].ClearState();
        return ret;
      }
      return m_cfg.p_motor[motor_idx].ClearState();
    }


    inline int MoveToOffset(AP_OBJ::MOTOR motor_idx = AP_OBJ::MOTOR_MAX){

      /*must assign motor id*/
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        int ret = 0;
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_JIG].MoveToOffset();
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_ROLL].MoveToOffset();
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_HIGH].MoveToOffset();
        return ret;
      }
      return m_cfg.p_motor[motor_idx].MoveToOffset();
    }


    inline int MotorJogMoveStop(AP_OBJ::MOTOR motor_idx = AP_OBJ::MOTOR_MAX){
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        int ret = 0;
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_JIG].JogStop();
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_ROLL].JogStop();
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_HIGH].JogStop();
        return ret;
      }
      return m_cfg.p_motor[motor_idx].JogStop();
    }

    inline int MotorMoveStop(AP_OBJ::MOTOR motor_idx = AP_OBJ::MOTOR_MAX){
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        int ret = 0;
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_JIG].MoveStop();
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_ROLL].MoveStop();
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_HIGH].MoveStop();
        return ret;
      }
      return m_cfg.p_motor[motor_idx].MoveStop();
    }

    inline int MotorJogMove(AP_OBJ::MOTOR motor_idx = AP_OBJ::MOTOR_MAX, bool is_cw = true){
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        int ret = 0;
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_JIG].JogMove(is_cw);
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_ROLL].JogMove(is_cw);
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_HIGH].JogMove(is_cw);
        return ret;
      }
      return m_cfg.p_motor[motor_idx].JogMove(is_cw);
    }

    inline int MotorJogMove(AP_OBJ::MOTOR motor_idx, uart_moons::speed_t& param, bool is_cw){
      param.speed = constrain(param.speed,1,CN_MOTORS_VELOCITY_RPM_MAX);
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        int ret = 0;
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_JIG].JogMove(param,is_cw);
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_ROLL].JogMove(param,is_cw);
        ret += m_cfg.p_motor[AP_OBJ::MOTOR_HIGH].JogMove(param,is_cw);
        return ret;
      }
      return m_cfg.p_motor[motor_idx].JogMove(param,is_cw);
    }


    inline bool IsConnected() const{
      return m_cfg.p_comm->IsConnected();
    }

    inline bool IsMotorOn(AP_OBJ::MOTOR motor_idx = AP_OBJ::MOTOR_MAX){
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        bool ret = true;
        ret = m_cfg.p_motor[AP_OBJ::MOTOR_JIG].IsMotorOn()
                             &m_cfg.p_motor[AP_OBJ::MOTOR_HIGH].IsMotorOn()
                             &m_cfg.p_motor[AP_OBJ::MOTOR_ROLL].IsMotorOn();
        return ret;
      }
      return m_cfg.p_motor[motor_idx].IsMotorOn();
    }



    inline bool IsMotorHomingAnyOne(){
      bool ret = true;
      ret = m_cfg.p_motor[AP_OBJ::MOTOR_JIG].IsMotorHoming()
                           |m_cfg.p_motor[AP_OBJ::MOTOR_HIGH].IsMotorHoming()
                           |m_cfg.p_motor[AP_OBJ::MOTOR_ROLL].IsMotorHoming();
      return ret;
    }



    inline bool IsMotorStop(AP_OBJ::MOTOR motor_idx = AP_OBJ::MOTOR_MAX){
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        bool ret = true;
        ret = m_cfg.p_motor[AP_OBJ::MOTOR_JIG].IsMotorStop()
                             &m_cfg.p_motor[AP_OBJ::MOTOR_HIGH].IsMotorStop()
                             &m_cfg.p_motor[AP_OBJ::MOTOR_ROLL].IsMotorStop();
        return ret;
      }
      return m_cfg.p_motor[motor_idx].IsMotorStop();
    }

    inline bool IsInPosition (AP_OBJ::MOTOR motor_idx, int cmd_dist_10um){
      return m_cfg.p_motor[motor_idx].IsInPosition(cal_dist_10um_to_pulse(motor_idx, cmd_dist_10um));
    }

    inline bool IsInPose(AP_OBJ::MOTOR motor_idx = AP_OBJ::MOTOR_MAX){
      if (motor_idx == AP_OBJ::MOTOR_MAX)
      {
        bool ret = true;
        ret = m_cfg.p_motor[AP_OBJ::MOTOR_JIG].IsInPose()
                                   &m_cfg.p_motor[AP_OBJ::MOTOR_HIGH].IsInPose()
                                   &m_cfg.p_motor[AP_OBJ::MOTOR_ROLL].IsInPose();
        return ret;
      }
      return m_cfg.p_motor[motor_idx].IsInPose();
    }

    inline bool IsNoErrStopAndZeroAll(){
      volatile bool ret = true;
      ret &= m_cfg.p_motor[AP_OBJ::MOTOR_JIG].IsNoErrStopAndZero();
      ret &= m_cfg.p_motor[AP_OBJ::MOTOR_HIGH].IsNoErrStopAndZero();
      ret &= m_cfg.p_motor[AP_OBJ::MOTOR_ROLL].IsNoErrStopAndZero();

      return ret;
    }




    inline float cal_sync_vel_dec(uint32_t source_rpm){
      float rate = 0;     ap_dat::dat_t*  p_data{};
      p_data = m_cfg.p_apCfgDat->GetData(ap_dat::sycn_rate);
      rate = ((float)(source_rpm*100)/(float)constrain(p_data->parm1,1,CN_MOTORS_SYNC_DEC_RATE_MAX));
      return rate;
    }

    inline int cal_dist_10um_to_pulse(AP_OBJ::MOTOR motor_idx, int um10){
      ap_dat::dat_t*  p_data{};
      int ret = 0;
      int turn_dist_um = 0;
      int turn_by_pulse = 0;
      int dist_by_turn_um = 0;
      switch (motor_idx) {
        case AP_OBJ::MOTOR::MOTOR_JIG:
          p_data = m_cfg.p_apCfgDat->GetData(ap_dat::mt_jig_turn_dist);
          break;
        case AP_OBJ::MOTOR::MOTOR_ROLL:
          p_data = m_cfg.p_apCfgDat->GetData(ap_dat::mt_roll_turn_dist);
          break;
        case AP_OBJ::MOTOR::MOTOR_HIGH:
          p_data = m_cfg.p_apCfgDat->GetData(ap_dat::mt_high_turn_dist);
          break;
        default:
          break;
      }
      turn_dist_um =um10*10;
      dist_by_turn_um = int(constrain(p_data->parm1,1,CN_MOTORS_TURN_DIST_MM_MAX))*1000;
      turn_by_pulse = (int)constrain(p_data->parm2,CN_MOTORS_GEAR_RATE_MIN,CN_MOTORS_GEAR_RATE_MAX);
      ret = int(((float)turn_dist_um/(float)dist_by_turn_um) * (float)turn_by_pulse);

      return ret;
    }

#endif
#if 0


    inline int EStop (uint8_t id = static_cast<uint8_t>(cnMotors::mot_Id::_max)){
      int ret = 0;
      if (id < static_cast<uint8_t>(cnMotors::mot_Id::_max))
      {
        ret = m_Cfg.p_motor[id].EStop();
      }
      else if (id == static_cast<uint8_t>(cnMotors::mot_Id::_max))
      {
        ret += m_Cfg.p_motor[AP_DEF_OBJ_MOTOR_ID_JIG].EStop();
        ret += m_Cfg.p_motor[AP_DEF_OBJ_MOTOR_ID_ROLL].EStop();
        ret += m_Cfg.p_motor[AP_DEF_OBJ_MOTOR_ID_HIGH].EStop();

      }
      return ret;
    }

#endif


  };
}// end of namespace MOTOR


#endif /* AP__INC_CNMOTORS_HPP_ */
