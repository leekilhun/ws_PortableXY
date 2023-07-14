/*
 * cnTasks.cpp
 *
 *  Created on: 2023. 6. 10.
 *      Author: gns2l
 */



#include "ap.hpp"
#include "cnTasks.hpp"

#define PROC_ALARM(p_head, p_err)                 m_cfg.ptr_AutoManger->AUTO_ALARM(p_head, p_err)

void cnTasks::ThreadJob()
  {
    switch (m_cfg.ptr_AutoManger->GetOPStatus())
    {
    case enOp::status_e::RUN_READY:
      break;

    case enOp::status_e::RUN:
    {
      switch (m_cfg.ptr_AutoManger->GetOPMode())
      {
      case enOp::mode_e::DRY_RUN:__attribute__((fallthrough));
      case enOp::mode_e::AUTORUN:
        if (!m_cfg.ptr_AutoManger->IsDetectAreaSensor()) //not
          doRunStep(m_idx);
        break;
      case enOp::mode_e::READY:     break;
      case enOp::mode_e::STOP:      break;
      default:          break;
      }
      // end of switch (GetOPMode())
    }
    break;

    case enOp::status_e::INIT:__attribute__((fallthrough));
    case enOp::status_e::STEP_STOP:__attribute__((fallthrough));
    case enOp::status_e::ERR_STOP:
      break;

    default:      break;
    }
    // end of switch (m_cfg.ptr_AutoManger->GetOPStatus())
  }

void cnTasks::doRunStep(uint8_t idx)
  {
    sequece_idx_data_st* line = m_lines[idx];
    constexpr uint32_t step_wait_delay = 300;
    constexpr uint8_t retry_max = 3;
    constexpr uint32_t move_timeout = 1'000 * 5;

    enum : uint8_t
    {
      STEP_INIT,
      STEP_TODO,
      STEP_TIMEOUT,
      STEP_DO_STEP_OUT,
      STEP_WAIT_DONE,
      STEP_WAIT_RETURN,
      STEP_DO_MOTOR_ORIGIN,
      STEP_DO_MOTOR_ORIGIN_START,
      STEP_DO_MOTOR_ORIGIN_WAIT,
      STEP_DO_MOTOR_ORIGIN_END,
      STEP_DO_INITAIL,
      STEP_DO_INITAIL_START,
      STEP_DO_INITAIL_WAIT,
      STEP_DO_INITAIL_END,
      STEP_DO_STANDBY,
      STEP_DO_STANDBY_START,
      STEP_DO_STANDBY_WAIT,
      STEP_DO_STANDBY_END,
      STEP_DO_IDX1,
      STEP_DO_IDX1_START,
      STEP_DO_IDX1_WAIT,
      STEP_DO_IDX1_END,
    };

    switch (m_step.GetStep())
    {
    case STEP_INIT:
    {
      LOG_PRINT("STEP_INIT");
      m_idx = 0;
      m_step.SetStep(STEP_TODO);
    }
    break;
    /*######################################################
       to do
      ######################################################*/
    case STEP_TODO:
    {
    }
    break;

    case STEP_TIMEOUT:
    {
    }
    break;
    case STEP_DO_STEP_OUT:
    {
      m_step.retry_cnt = 0; m_step.wait_step = 0; m_step.wait_resp = false;
      m_cfg.ptr_AutoManger->StopSw();
      m_step.SetStep(STEP_TODO);
    }
    break;
    case STEP_WAIT_DONE:
    {
    }
    break;

    case STEP_WAIT_RETURN:
    {
    }
    break;
    ////////////////////////////////////////////////////
    /*######################################################
         do motor origin process
      ######################################################*/
    case STEP_DO_MOTOR_ORIGIN:
    {
      if (!m_cfg.ptr_apReg->state_reg.motor_on /*|| !m_cfg.p_apReg->error_reg.no_error*/)
      {
        log_dat::head_t head_inf{};
        head_inf.error_no = static_cast<uint8_t>(ap_err_st::err_e::axis_origin_err);
        head_inf.obj_id = m_requestMotor_idx;
        head_inf.step_no = STEP_DO_MOTOR_ORIGIN;
        PROC_ALARM(head_inf, "axis origin not ready");
        m_step.SetStep(STEP_DO_STEP_OUT);
        break;
      }

      m_cfg.ptr_motors->MotorClearAlarm();
      m_step.retry_cnt = 0;
      m_step.wait_resp = false;

      switch (m_requestMotor_idx)
      {
        case AP_OBJ::MOTOR_X:
          m_step.wait_step = 4;
          break;
        case AP_OBJ::MOTOR_Y:
          m_step.wait_step = 8;
          break;
#ifdef APP_USE_MOTOR_R
        case AP_OBJ::MOTOR_R:__attribute__((fallthrough));
#endif
        case AP_OBJ::MOTOR_MAX:__attribute__((fallthrough));
        default:
          m_step.wait_step = 0;
          break;
      }

      m_step.SetStep(STEP_DO_MOTOR_ORIGIN_START);
    }
    break;
    case STEP_DO_MOTOR_ORIGIN_START:
    {
      if (m_step.LessThan(step_wait_delay))             { break; }
      enum : uint8_t{
        prc_move_to_limit_x, prc_org_x, prc_org_reset_x, prc_move_ready_x,
        prc_move_to_limit_y, prc_org_y, prc_org_reset_y, prc_move_ready_y,
#ifdef APP_USE_MOTOR_R
        prc_move_to_limit_r, prc_org_r, prc_org_reset_r, prc_move_ready_r,
#endif
      };

      uint8_t recursive_idx = m_step.wait_step;
      int ret {};
      switch (recursive_idx)
      {
        case prc_move_to_limit_x:
          ret = m_cfg.ptr_motors->MoveToLimit(AP_OBJ::MOTOR_X);
          break;
        case prc_org_x:
          ret = m_cfg.ptr_motors->Origin(AP_OBJ::MOTOR_X);
          break;
        case prc_org_reset_x:
          ret = m_cfg.ptr_motors->MotorZeroEncode(AP_OBJ::MOTOR_X);
          break;
        case prc_move_ready_x:
          if (m_cfg.ptr_motors->IsNoErrStopAndZero(AP_OBJ::MOTOR_X))
            ret = m_cfg.ptr_motors->MoveToOffset(AP_OBJ::MOTOR_X);
          break;
        case prc_move_to_limit_y:
          ret = m_cfg.ptr_motors->MoveToLimit(AP_OBJ::MOTOR_Y);
          break;
        case prc_org_y:
          ret = m_cfg.ptr_motors->Origin(AP_OBJ::MOTOR_Y);
          break;
        case prc_org_reset_y:
          ret = m_cfg.ptr_motors->MotorZeroEncode(AP_OBJ::MOTOR_Y);
          break;
        case prc_move_ready_y:
          if (m_cfg.ptr_motors->IsNoErrStopAndZero(AP_OBJ::MOTOR::MOTOR_Y))
            ret = m_cfg.ptr_motors->MoveToOffset(AP_OBJ::MOTOR_Y);
          break;
#ifdef APP_USE_MOTOR_R
        case prc_move_to_limit_r:
          ret = m_cfg.ptr_motors->MoveToLimit(AP_OBJ::MOTOR_R);
          break;
        case prc_org_r:
          ret = m_cfg.ptr_motors->Origin(AP_OBJ::MOTOR_R);
          break;
        case prc_org_reset_r:
          ret = m_cfg.ptr_motors->MotorZeroEncode(AP_OBJ::MOTOR_R);
          break;
        case prc_move_ready_r:
          if (m_cfg.ptr_motors->IsNoErrStopAndZero(AP_OBJ::MOTOR::MOTOR_R))
            ret = m_cfg.ptr_motors->MoveToOffset(AP_OBJ::MOTOR_R);
          break;
#endif
        default:
          break;
      }
      // end of switch

      if (ret != ERROR_SUCCESS || m_step.MoreThan(1000*5))
      {
        log_dat::head_t head_inf{};
        head_inf.error_no = static_cast<uint8_t>(ap_err_st::axis_origin_err);
        head_inf.obj_id = m_requestMotor_idx;
        head_inf.step_no = STEP_DO_MOTOR_ORIGIN_START;
        PROC_ALARM(head_inf, "axis origin start fail");
        m_step.SetStep(STEP_DO_STEP_OUT);
        break;

      }

      //logPrintf("cnTasks STEP_DO_MOTOR_ORIGIN_START recursive_idx[%d], check [%d] \n",recursive_idx, m_cfg.p_motors->IsLimitPos(AP_OBJ::MOTOR_ROLL, true));

      switch (recursive_idx)
      {
        case prc_move_to_limit_x:
          if(m_cfg.ptr_motors->IsLimitPos(AP_OBJ::MOTOR_X, true))
            m_step.wait_resp = true;
          else
            m_step.wait_resp = m_cfg.ptr_motors->IsMotorRun(AP_OBJ::MOTOR_X);
          break;
        case prc_org_x:
          m_step.wait_resp = true;
          break;
        case prc_move_ready_x:
          m_step.wait_resp = m_cfg.ptr_motors->IsMotorRun(AP_OBJ::MOTOR_X);
          m_step.wait_resp |= m_cfg.ptr_motors->IsInPose(AP_OBJ::MOTOR_X);
          break;
        case prc_org_reset_x:
          m_step.wait_resp = true;
          break;
        case prc_move_to_limit_y:
          if(m_cfg.ptr_motors->IsLimitPos(AP_OBJ::MOTOR_Y, true))
            m_step.wait_resp = true;
          else
            m_step.wait_resp = m_cfg.ptr_motors->IsMotorRun(AP_OBJ::MOTOR_Y);
          break;
        case prc_org_y:
          m_step.wait_resp = m_cfg.ptr_motors->IsMotorRun(AP_OBJ::MOTOR_Y);
          break;
        case prc_move_ready_y:
          m_step.wait_resp = m_cfg.ptr_motors->IsMotorRun(AP_OBJ::MOTOR_Y);
          m_step.wait_resp |= m_cfg.ptr_motors->IsInPose(AP_OBJ::MOTOR_Y);
          break;
        case prc_org_reset_y:
          m_step.wait_resp = true;
          break;
#ifdef APP_USE_MOTOR_R
        case prc_move_to_limit_r:
          if(m_cfg.ptr_motors->IsLimitPos(AP_OBJ::MOTOR_R, true))
            m_step.wait_resp = true;
          else
            m_step.wait_resp = m_cfg.ptr_motors->IsMotorRun(AP_OBJ::MOTOR_R);
          break;
        case prc_org_r:
          m_step.wait_resp = true;
          break;
        case prc_move_ready_r:
          m_step.wait_resp = m_cfg.ptr_motors->IsMotorRun(AP_OBJ::MOTOR_R);
          m_step.wait_resp |= m_cfg.ptr_motors->IsInPose(AP_OBJ::MOTOR_R);
          break;
        case prc_org_reset_r:
          m_step.wait_resp = true;
          break;
#endif
        default:
          break;
      }
      // end of switch

      if (m_step.wait_resp)
      {
        m_step.SetStep(STEP_DO_MOTOR_ORIGIN_WAIT);
      }
      else
      {
        m_step.SetStep(STEP_DO_MOTOR_ORIGIN_START);
      }

    }
    break;
    case STEP_DO_MOTOR_ORIGIN_WAIT:
    {
      if (m_step.LessThan(step_wait_delay))
      {
        break;
      }

      enum : uint8_t
      {
        prc_move_to_limit_x,
        prc_org_x,
        prc_org_reset_x,
        prc_move_ready_x,
        prc_move_to_limit_y,
        prc_org_y,
        prc_org_reset_y,
        prc_move_ready_y,
#ifdef APP_USE_MOTOR_R
        prc_move_to_limit_r,
        prc_org_r,
        prc_org_reset_r,
        prc_move_ready_r,
#endif
      };

      uint8_t recursive_idx = m_step.wait_step;
      uint8_t ret{};

      enum : uint8_t
      {
        step_none,
        step_cplt,
        step_retry,
        step_err
      };
      auto check_move_done = [&](auto id, bool is_check_pos = false) -> uint8_t
      {
        bool check{};
        if (is_check_pos)
          check = IsMotorMoveCplt(id);
        else
          check = IsMotorStop(id);

        if (check)
          return step_cplt;
        else
        {
          if (IsMotorRun(id) == false)
          {
            if (m_step.retry_cnt++ < retry_max) // retry
              return step_retry;
          }

          if (m_step.MoreThan(move_timeout * 10)) // error timeout for 50 second
            return step_err;
        }
        return step_none;
      };

      auto check_org_done = [&](auto id) -> uint8_t
      {
        if (m_cfg.ptr_motors->IsOriginProcessCplt(id))
          return step_cplt;
        else
        {
          if (m_cfg.ptr_motors->IsMotorHoming(id) == false)
          {
            if (m_step.retry_cnt++ < retry_max) // retry
              return step_retry;
          }

          if (m_step.MoreThan(move_timeout * 10)) // error
            return step_err;
        }
        return step_none;
      };

      switch (recursive_idx)
      {
      case prc_move_to_limit_x:
        ret = check_move_done(AP_OBJ::MOTOR::MOTOR_X);
        break;
      case prc_org_x:
        ret = check_org_done(AP_OBJ::MOTOR::MOTOR_X);
        break;
      case prc_org_reset_x:
        if (m_cfg.ptr_motors->IsNoErrStopAndZero(AP_OBJ::MOTOR::MOTOR_X))
            ret = step_cplt;
        else
            ret = step_retry;
        break;
      case prc_move_ready_x:
        ret = check_move_done(AP_OBJ::MOTOR::MOTOR_X, true);
        break;
      case prc_move_to_limit_y:
        ret = check_move_done(AP_OBJ::MOTOR::MOTOR_Y);
        break;
      case prc_org_y:
        ret = check_org_done(AP_OBJ::MOTOR::MOTOR_Y);
        break;
      case prc_org_reset_y:
        if (m_cfg.ptr_motors->IsNoErrStopAndZero(AP_OBJ::MOTOR::MOTOR_Y))
            ret = step_cplt;
        else
            ret = step_retry;
        break;
      case prc_move_ready_y:
        ret = check_move_done(AP_OBJ::MOTOR::MOTOR_Y, true);
        break;
#ifdef APP_USE_MOTOR_R
      case prc_move_to_limit_r:
        ret = check_move_done(AP_OBJ::MOTOR::MOTOR_R);
        break;
      case prc_org_r:
        ret = check_org_done(AP_OBJ::MOTOR::MOTOR_R);
        break;
      case prc_org_reset_r:
        if (m_cfg.ptr_motors->IsNoErrStopAndZero(AP_OBJ::MOTOR::MOTOR_R))
            ret = step_cplt;
        else
            ret = step_retry;
        break;
      case prc_move_ready_r:
        ret = check_move_done(AP_OBJ::MOTOR::MOTOR_R, true);
        break;
#endif
      default:
        break;
      }
      // end of switch

      // step_cplt, step_retry, step_err
      if (ret == step_cplt)
      {
        m_step.SetStep(STEP_DO_MOTOR_ORIGIN_END);
        break;
      }
      else if (ret == step_retry)
      {
        // m_step.SetStep(STEP_DO_MOTOR_ORIGIN_START);
        break;
      }
      else if (ret == step_err)
      {
        m_cfg.ptr_apReg->state_reg.initializing = false;
        m_step.retry_cnt = 0;
        log_dat::head_t head_inf;
        head_inf.error_no = static_cast<uint8_t>(ap_err_st::axis_move_timeout);
        head_inf.obj_id = m_requestMotor_idx;
        head_inf.step_no = STEP_DO_MOTOR_ORIGIN_WAIT;
        PROC_ALARM(head_inf, "axis origin  move timeou");
        m_step.SetStep(STEP_TIMEOUT);
      }
    }
    break;
    case STEP_DO_MOTOR_ORIGIN_END:
    {

      enum : uint8_t
      {
        prc_move_to_limit_x,
        prc_org_x,
        prc_org_reset_x,
        prc_move_ready_x,
        prc_move_to_limit_y,
        prc_org_y,
        prc_org_reset_y,
        prc_move_ready_y,
#ifdef APP_USE_MOTOR_R
        prc_move_to_limit_r,
        prc_org_r,
        prc_org_reset_r,
        prc_move_ready_r,
#endif
      };

      uint8_t recursive_idx = m_step.wait_step;
      m_step.retry_cnt = 0;
      m_step.wait_resp = false;
      m_step.SetStep(STEP_DO_MOTOR_ORIGIN_START);
      switch (recursive_idx)
      {
      case prc_move_to_limit_x:
        m_step.wait_step = prc_org_x;
        break;
      case prc_org_x:
        m_step.wait_step = prc_org_reset_x;
        break;
      case prc_org_reset_x:
        m_step.wait_step = prc_move_ready_x;
        break;
      case prc_move_ready_x:
        if (m_requestMotor_idx == AP_OBJ::MOTOR::MOTOR_MAX)
            m_step.wait_step = prc_move_to_limit_y;
        else
            m_step.wait_step = 0;
        break;

      case prc_move_to_limit_y:
        m_step.wait_step = prc_org_y;
        break;
      case prc_org_y:
        m_step.wait_step = prc_org_reset_y;
        break;
      case prc_org_reset_y:
        m_step.wait_step = prc_move_ready_y;
        break;
      case prc_move_ready_y:
#ifdef APP_USE_MOTOR_R
        if (m_requestMotor_idx == AP_OBJ::MOTOR::MOTOR_MAX)
            m_step.wait_step = prc_move_to_limit_r;
        else
            m_step.wait_step = 0;
        break;
      case prc_move_to_limit_r:
        m_step.wait_step = prc_org_r;
        break;
      case prc_org_r:
        m_step.wait_step = prc_org_reset_r;
        break;
      case prc_org_reset_r:
        m_step.wait_step = prc_move_ready_r;
        break;
      case prc_move_ready_r:
        m_step.wait_step = 0;
        break;
#else
        m_step.wait_step = 0;
#endif
        break;
        default:
        break;
      }
      // end of switch

      if (m_step.wait_step == 0)
      {
        if (m_requestMotor_idx == AP_OBJ::MOTOR::MOTOR_MAX)
            m_step.SetStep(STEP_DO_STANDBY);
        else
            m_step.SetStep(STEP_TODO);
      }
    }
    break;
    /*######################################################
       do app system initialize
      ######################################################*/
    case STEP_DO_INITAIL:
    {
      m_step.retry_cnt = 0;
			m_step.wait_step = 0;
			m_step.wait_resp = false;
			m_cfg.ptr_motors->MotorClearAlarm();
			m_step.SetStep(STEP_DO_INITAIL_START);
			m_cfg.ptr_apReg->SetReg_State(ap_reg::INITIALIZING, true);

    }
    break;
    case STEP_DO_INITAIL_START:
    {
      if (m_cfg.ptr_motors->IsOriginCpltAll() == false)
				m_step.SetStep(STEP_DO_MOTOR_ORIGIN);
			else
				m_step.SetStep(STEP_DO_STANDBY);
    }
    break;
    case STEP_DO_INITAIL_WAIT:
    {

    }
    break;
    case STEP_DO_INITAIL_END:
    {
      m_cfg.ptr_apReg->SetReg_State(ap_reg::INITIALIZING, false);
      m_step.SetStep(STEP_INIT);
    }
    break;
    /*######################################################
       do system standby state and wait start
      ######################################################*/
    case STEP_DO_STANDBY:
    {
      m_step.retry_cnt = 0;
			m_step.wait_step = 0;
			m_step.wait_resp = false;

			//constexpr uint8_t rec_io_and_cyl_state_init = 3; //rec -> recursive
    /*
     reset cylinder, vacuum, io-out    
     */

			m_step.SetStep(STEP_DO_STANDBY_START);

    }
    break;
    case STEP_DO_STANDBY_START:
    {
      if (m_step.LessThan(step_wait_delay))
        break;

      m_step.SetStep(STEP_DO_STANDBY_WAIT);
    }
    break;
    case STEP_DO_STANDBY_WAIT:
    {
      if (m_step.LessThan(step_wait_delay))
        break;

      m_step.SetStep(STEP_DO_STANDBY_END);
    }
    break;
    case STEP_DO_STANDBY_END:
    {
      m_step.SetStep(STEP_TODO);
    }
    break;
    /*######################################################
       line index
      ######################################################*/
    case STEP_DO_IDX1:
    {
      if (line->entry_setout)
      {
        LOG_PRINT("entry_setout : OutputOn Set [%d]",line->entry_setout + AP_DEF_START_OUT_ADDR );
        m_cfg.ptr_io->OutputOn((uint32_t)(line->entry_setout + AP_DEF_START_OUT_ADDR));
        // io set out
      }

      m_step.retry_cnt = 0;
      m_step.wait_step = 0;
      m_step.SetStep(STEP_DO_IDX1_START);
    }
    break;

    case STEP_DO_IDX1_START:
    {
      if (line->entry_delay > 0)
      {
        if (m_step.LessThan((uint32_t)line->entry_delay))
          break;
      }

      if (line->line_type == line_t::lt_pos)
      {
        // m_pos[line->pos_data_idx].data.pos_x
        if (MoveToPos(line->pos_data_idx) == ERROR_SUCCESS)
        {
          m_step.SetStep(STEP_DO_IDX1_WAIT);
        }
      }
      else
      {
      }
    }
    break;

    case STEP_DO_IDX1_WAIT:
    {
      if (m_step.LessThan(step_wait_delay))
        break;

      if (line->line_type == line_t::lt_pos)
      {
        if (this->IsMotorMoveCplt())
        {
          m_step.SetStep(STEP_DO_IDX1_END);
          break;
        }

        if (IsMotorRun() == false)
        {
          if (m_step.retry_cnt++ < retry_max)
          {
            m_step.SetStep(STEP_DO_IDX1_START);
            break;
          }
        }

        if (m_step.MoreThan(move_timeout))
        {
          // timeout error
        }
      }
      else
      {
        LOG_PRINT("condition_in : IsOn Set [%d]",line->condition_in + AP_DEF_START_IN_ADDR);
        if ( m_cfg.ptr_io->IsOn((uint32_t)(line->condition_in + AP_DEF_START_IN_ADDR))) //
          m_idx = line->condition_pass_line;
        else
          m_idx = line->condition_fail_line;
      }
    }
    break;

    case STEP_DO_IDX1_END:
    {
      if (line->exit_setout)
      {
        LOG_PRINT("line->exit_setout : OutputOn Set [%d]",line->exit_setout + AP_DEF_START_OUT_ADDR);
        // io set out
        m_cfg.ptr_io->OutputOn((uint32_t)(line->exit_setout + AP_DEF_START_OUT_ADDR));
      }

      if (line->exit_delay > 0)
      {
        if (m_step.LessThan((uint32_t)line->exit_delay))
          break;
      }
    }
    break;
    ////////////////////////////////////////////////////
    default:
      break;
    }
  }


