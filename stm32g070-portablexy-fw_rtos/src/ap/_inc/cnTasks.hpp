/*
 * cnTasks.hpp
 *
 *  Created on: 2023. 6. 10.
 *      Author: gns2l
 */

#ifndef AP__INC_CNTASKS_HPP_
#define AP__INC_CNTASKS_HPP_

constexpr uint8_t line_index_max = 200;

struct cnTasks
{

  using line_t = sequece_idx_data_st::linetype_e;
  using pos_data_t = pos_data_st;
  using pos_idx_t = pos_data_st::idx_e;


  struct cfg_t
  {
    ap_reg* ptr_apReg{};
    enOp* ptr_op{};
    MOTOR::cnMotors* ptr_motors{};
    taskDat* ptr_taskDat{};
    //ap_log* ptr_apLog;
    ap_io* ptr_io{};
    cnAuto* ptr_AutoManger{};

    cfg_t() = default;
    ~cfg_t() = default;

    // copy constructor
    cfg_t(const cfg_t& rhs) = default;
    // copy assignment operator
    cfg_t& operator=(const cfg_t& rhs) = default;
    // move constructor
    cfg_t(cfg_t&& rhs) = default;
    // move assignment operator
    cfg_t& operator=(cfg_t&& rhs) = default;

  };

  cfg_t m_cfg{};
  prc_step_t m_step{};
  uint8_t m_idx{};
  AP_OBJ::MOTOR m_requestMotor_idx{};
  std::array<sequece_idx_data_st*, line_index_max> m_lines{};
  //std::array<pos_data_t, pos_idx_t::mdi_max> m_pos;

  cnTasks() = default;
  ~cnTasks() = default;

  // copy constructor
  cnTasks(const cnTasks& rhs) = default;
  // copy assignment operator
  cnTasks& operator=(const cnTasks& rhs) = default;
  // move constructor
  cnTasks(cnTasks&& rhs) = default;
  // move assignment operator
  cnTasks& operator=(cnTasks&& rhs) = default;


  inline int Init(cnTasks::cfg_t& cfg) {
    LOG_PRINT("Init Success!");
    m_cfg = cfg;
    return ERROR_SUCCESS;
  }



  inline bool IsMotorRun(AP_OBJ::MOTOR motor_idx = AP_OBJ::MOTOR_MAX){
    return m_cfg.ptr_motors->IsMotorRun(motor_idx);
  }

  inline bool IsMotorMoveCplt(AP_OBJ::MOTOR motor_idx = AP_OBJ::MOTOR_MAX){
    return m_cfg.ptr_motors->IsInPose(motor_idx);
  }

  inline bool IsMotorStop(AP_OBJ::MOTOR motor_idx = AP_OBJ::MOTOR_MAX){
    return m_cfg.ptr_motors->IsMotorStop(motor_idx);
  }
  inline errno_t MoveToPos(pos_data_st::idx_e idx)
  {

		return 0;
	}
  void ThreadJob();
	void doRunStep(uint8_t idx = 0);

  inline void RunLineTask(uint8_t idx)
  {
    constexpr auto STEP_DO_IDX1 = 13;
    m_idx = idx;
    m_lines[0] =  &m_cfg.ptr_taskDat->task_dat[0].line_data;
    m_lines[1] =  &m_cfg.ptr_taskDat->task_dat[1].line_data;
    m_lines[2] =  &m_cfg.ptr_taskDat->task_dat[2].line_data;
    m_lines[3] =  &m_cfg.ptr_taskDat->task_dat[3].line_data;
    m_step.SetStep(STEP_DO_IDX1);
  }

};
//end of cnTasks




#endif /* AP__INC_CNTASKS_HPP_ */
