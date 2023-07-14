/*
 * api_remote.hpp
 *
 *  Created on: 2023. 6. 10.
 *      Author: gns2l
 */

#ifndef AP__INC_API_REMOTE_HPP_
#define AP__INC_API_REMOTE_HPP_



struct api_remote
{
  /****************************************************
   *  data
   ****************************************************/

public:
  struct cfg_t
  {
    RCTRL::uart_remote* ptr_comm{};
    ap_reg* ptr_mcu_reg{};
    cnTasks* ptr_task{};
    ap_io* ptr_io{};
    ap_dat* ptr_cfg_data{};
    mcu_data_st* ptr_mcu_data{};
    cnAuto* ptr_auto{};
    MOTOR::cnMotors* ptr_motors{};

    cfg_t() = default;
    // destructor
    ~cfg_t() = default;

    // copy constructor
    cfg_t(const cfg_t& other) = default;
    // copy assignment
    cfg_t& operator=(const cfg_t& other) = default;
    // move constructor
    cfg_t(cfg_t&& other) = default;
    // move assignment
    cfg_t& operator=(cfg_t&& other) = default;
  };

  bool m_IsInit;
  cfg_t m_cfg;
  RCTRL::uart_remote::packet_st m_receiveData;
  //std::array<uint8_t, RCTRL::CMD_MAX_PACKET_LENGTH> m_txBuffer;
  bool m_waitReplyOK;
  bool m_OkReply;

  prc_step_t m_step;
  uint32_t m_elaps;

  /****************************************************
   *  Constructor
   ****************************************************/
public:
  api_remote(): m_IsInit{}, m_cfg{},m_receiveData{}/*, m_txBuffer{}*/
  , m_waitReplyOK{}, m_OkReply{}
  , m_step{}, m_elaps{}{

  };

  ~api_remote(){};

  /****************************************************
   *  func
   ****************************************************/
private:
  void doRunStep();


  // callback function
  inline static void receiveDataFunc(void* obj, void* w_parm, void* l_parm) {
    api_remote* ptr_this = (api_remote*)obj;

    ptr_this->m_waitReplyOK = false;
    if (w_parm == nullptr && obj == nullptr && l_parm == nullptr)
      return;
    ptr_this->ProcessCmd(*(RCTRL::uart_remote::packet_st*)l_parm) ;
  }



public:
  inline int Init(api_remote::cfg_t &cfg){
    m_cfg = cfg;
    cfg.ptr_comm->AttCallbackFunc(this, receiveDataFunc);
    m_IsInit = true;
    LOG_PRINT("Init Success! ");
    return ERROR_SUCCESS;
  }

  // Retry the communication connection
  inline void CommRecovery() {
    m_cfg.ptr_comm->Recovery();
  }

  // It is processed as nonblock through machine step.
  void ThreadJob();

  // The PC processes command reception and returns the received ack signal.
  void ProcessCmd(RCTRL::uart_remote::packet_st& data);

  //sends mcu status information
  void PcUpdate();

  errno_t SendData(RCTRL::TX_TYPE type, uint8_t obj_id = 0);

  inline errno_t Ret_OkResponse() {
    return SendData(RCTRL::TX_TYPE::TX_OK_RESPONSE);
    }


};




#endif /* AP__INC_API_REMOTE_HPP_ */
