/*
 * uart_moons.hpp
 *
 *  Created on: 2023. 6. 10.
 *      Author: gns2l
 */

#ifndef AP__INC_UART_MOONS_HPP_
#define AP__INC_UART_MOONS_HPP_



namespace MOTOR
{


#ifdef _USE_HW_RTOS
#define AP_UART_MOONS_LOCK_BEGIN osMutexWait(m_mutex_id, osWaitForever)
#define AP_UART_MOONS_LOCK_END   osMutexRelease(m_mutex_id)
#else
#define AP_UART_MOONS_LOCK_BEGIN
#define AP_UART_MOONS_LOCK_END
#endif


  constexpr uint8_t MOONS_RETRY_CNT_MAX = 5;
  constexpr int MOONS_COMM_TIMEOUT = 100;
  constexpr int MOONS_PACKET_BUFF_LENGTH = 100;
  constexpr int MOONS_MULTI_WRITE_DATA_LENGTH = 10;

  struct uart_moons
  {
    /****************************************************
     *  data
     ****************************************************/
    enum func_e {
      read_HoldingReg = 0x03,
      read_InputReg = 0x04,
      write_SingleReg = 0x06,
      write_MultiReg = 0x10
    };

    enum reg_e{
      Alarm_Code_AL_f = 0x0000,
      Point_to_Point_Acceleration_AC_A = 0x001B,
      Point_to_Point_Distance_DI_D = 0x001E,
      Jog_Accel_JA_ = 0x002E,
      Command_Opcode  = 0x007c,
    };



  public:
    struct speed_t{
      uint16_t accel{};
      uint16_t decel{};
      uint16_t speed{};
    };
    struct cfg_t
    {
      uint8_t ch{}; uint32_t baud{};
    };

    struct rx_packet_t
    {
      uint8_t   node_id{};
      uint8_t   func_type{};
      uint16_t  reg_addr{};
      uint16_t  reg_length{};
      uint8_t   data_length{};
      uint8_t*  data{};
      uint16_t  check_sum{};
      uint16_t  check_sum_recv{};
      std::array <uint8_t, MOONS_PACKET_BUFF_LENGTH> buffer{};

      rx_packet_t() = default;
      
         // copy constructor
      rx_packet_t(const rx_packet_t& rhs) = default;
        // copy assignment operator
      rx_packet_t& operator=(const rx_packet_t& rhs) = default;
        // move constructor
      rx_packet_t(rx_packet_t&& rhs) = default;
        // move assignment operator
      rx_packet_t& operator=(rx_packet_t&& rhs) = default;

    };
    struct modbus_t {
      uint8_t   state{};
      uint32_t  prev_ms{};
      uint32_t  resp_ms{};
      uint32_t  index{};
      uint32_t  data_cnt{};
      uint8_t   error{};

      uint8_t   request_flag{};

      rx_packet_t  rx_packet{};
    };

    bool m_Isconnected;
    cfg_t m_cfg;
    modbus_t m_packet;

    std::array <void* ,AP_OBJ::MOTOR_MAX> m_objs;
    evt_cb m_func;
    uint32_t m_packet_sending_ms;


#ifdef _USE_HW_RTOS
    osMutexId m_mutex_id;
#endif

    /****************************************************
     *  Constructor
     ****************************************************/
  public:
    uart_moons():m_Isconnected{}, m_cfg{}, m_packet{}, m_objs{}, m_func{}
    ,m_packet_sending_ms{}
    {
#ifdef _USE_HW_RTOS
      osMutexDef(m_mutex_id);
      m_mutex_id = osMutexCreate (osMutex(m_mutex_id));
#endif

    }
    ~uart_moons (){}

    /****************************************************
     *  func
     ****************************************************/

    inline void Init(cfg_t &cfg) {
      m_cfg = cfg;
      if (uartOpen(cfg.ch, cfg.baud))
      {
        logPrintf(">>uart_moons Init Success! Uart ch[%d], baud[%d] \n",cfg.ch, cfg.baud);
        m_Isconnected = true;
      }
    }

    inline bool Recovery() {
      m_packet.request_flag = 0;
      uartClose(m_cfg.ch);

      m_Isconnected = uartOpen(m_cfg.ch, m_cfg.baud);
      return m_Isconnected;
    }

    inline bool ReceiveProcess(){
      bool ret = false;
      if (receivePacket())
      {
        receiveCplt();
        ret = true;
      }
      return ret;
    }

    inline void AttCallbackFunc(AP_OBJ::MOTOR instance_no, void* obj, evt_cb cb)  {
      m_objs[instance_no] = obj;
      m_func = cb;
    }

    inline errno_t RequestMotorData(uint8_t node_id){
      enum{id, fn,reg_h, reg_l, length_h, length_l, crc_l, crc_h,  _max};
      uint8_t  func = read_HoldingReg;
      uint16_t regist_no = Alarm_Code_AL_f;
      uint16_t data_cnt = 12;
      uint16_t crc = 0xffff;

      std::array<uint8_t, _max> send_data = {
          node_id,
          func,
          (uint8_t)(regist_no >> 8),
          (uint8_t)(regist_no >> 0),
          (uint8_t)(data_cnt >> 8),
          (uint8_t)(data_cnt >> 0),
          (uint8_t)(0),
          (uint8_t)(0)
      };

      for(uint16_t i = 0; i < crc_l; i++)
      {
        UTL::crc16_modbus_update(&crc, send_data[i]);
      }
      send_data[crc_l] = (uint8_t)(crc >> 0);
      send_data[crc_h] = (uint8_t)(crc >> 8);


      return (SendCmd(send_data.data(), (uint32_t)send_data.size()));
    }

    inline errno_t MotorOnOff(uint8_t node_id, bool on_off = true){
      if (m_Isconnected != true)
        return -1;

      constexpr uint16_t motor_on =  159;
      constexpr uint16_t motor_off =  158;
      enum{id, fn,reg_h, reg_l, length_h, length_l, crc_l, crc_h,  _max};
      uint8_t  func = write_SingleReg;
      uint16_t regist_no = Command_Opcode;
      uint16_t value = (on_off ? motor_on:motor_off);
      uint16_t crc = 0xffff;
      std::array<uint8_t, _max> send_data = {
          node_id,
          func,
          (uint8_t)(regist_no >> 8),
          (uint8_t)(regist_no >> 0),
          (uint8_t)(value >> 8),
          (uint8_t)(value >> 0),
          (uint8_t)(0),
          (uint8_t)(0)
      };

      for(uint16_t i = 0; i < crc_l; i++)
      {
        UTL::crc16_modbus_update(&crc, send_data[i]);
      }
      send_data[crc_l] = (uint8_t)(crc >> 0);
      send_data[crc_h] = (uint8_t)(crc >> 8);

      constexpr uint32_t timeout = 100;
      uint8_t retray_cnt = MOONS_RETRY_CNT_MAX;
      uint32_t ms =millis();
      while (retray_cnt)
      {
        errno_t result = SendCmdRxResp(send_data.data(), (uint32_t)send_data.size(), timeout);
        if (result == ERROR_SUCCESS)
        {
          logPrintf("uart_moons MotorOnOff Success! response time [%d]ms \n",millis() - ms);
          return ERROR_SUCCESS;
        }
        else
        {
          retray_cnt--;
        }
      }
      return -1;
      //return (SendCmd(send_data.data(), (uint32_t)send_data.size()));

    }

    inline errno_t DistancePoint(uint8_t node_id, int dist) {
      if (m_Isconnected != true)
        return -1;

      enum{id, fn, st_reg_h, st_reg_l, reg_cnt_h, reg_cnt_l,b_cnt, dist3,dist2,dist1,dist0,
        crc_l, crc_h,  _max};
      uint8_t  func = write_MultiReg;
      uint16_t start_reg_no = Point_to_Point_Distance_DI_D;
      constexpr uint16_t reg_cnt = 2;
      constexpr uint8_t byte_cnt = 4;
      uint16_t crc = 0xffff;
      uint32_t dist_point = (uint32_t)dist;
      std::array<uint8_t, _max> send_data = {
          node_id,
          func,
          (uint8_t)(start_reg_no >> 8),
          (uint8_t)(start_reg_no),
          (uint8_t)(reg_cnt >> 8),
          (uint8_t)reg_cnt,
          byte_cnt,
          (uint8_t)((dist_point >> 24) & 0xff),
          (uint8_t)((dist_point >> 16) & 0xff),
          (uint8_t)((dist_point >> 8)  & 0xff),
          (uint8_t)((dist_point >> 0)  & 0xff),
          (uint8_t)(0),
          (uint8_t)(0)
      };

      for(uint16_t i = 0; i < crc_l; i++)
      {
        UTL::crc16_modbus_update(&crc, send_data[i]);
      }
      send_data[crc_l] = (uint8_t)(crc >> 0);
      send_data[crc_h] = (uint8_t)(crc >> 8);

      constexpr uint32_t timeout = 100;
      uint8_t retray_cnt = MOONS_RETRY_CNT_MAX;
      while (retray_cnt)
      {
        errno_t result = SendCmdRxResp(send_data.data(), (uint32_t)send_data.size(), timeout);
        if (result == ERROR_SUCCESS)
        {
          return ERROR_SUCCESS;
        }
        else
        {
          retray_cnt--;
        }
      }
      return -1;

    }

    inline errno_t OriginAxis(uint8_t node_id, char xin, char level = 'F') {
      if (m_Isconnected != true)
        return -1;

      constexpr uint16_t seek_home = 110;
      uint8_t  func = write_MultiReg;
      uint16_t regist_no = Command_Opcode;
      uint16_t value = seek_home;

      enum { opcode, param1, param2, v_max };
      std::array<uint16_t, v_max> values = {
          value,
          (uint16_t)xin,
          (uint16_t)level
      };
      constexpr uint16_t reg_cnt = v_max;
      constexpr uint8_t byte_cnt = v_max * 2;

      uint16_t crc = 0xffff;
      enum{id, fn, reg_h, reg_l, reg_cnt_h, reg_cnt_l, b_cnt, v5, v4, v3, v2, v1, v0,
                    crc_l, crc_h,  _max};

      std::array<uint8_t, _max> send_data = {
          node_id,
          func,
          (uint8_t)(regist_no >> 8),
          (uint8_t)(regist_no >> 0),
          (uint8_t)(reg_cnt >> 8),
          (uint8_t)(reg_cnt >> 0),
          byte_cnt,
          (uint8_t)((values[opcode] >> 8) & 0xff),
          (uint8_t)((values[opcode] >> 0) & 0xff),
          (uint8_t)((values[param1] >> 8) & 0xff),
          (uint8_t)((values[param1] >> 0) & 0xff),
          (uint8_t)((values[param2] >> 8) & 0xff),
          (uint8_t)((values[param2] >> 0) & 0xff),
          (uint8_t)(0),
          (uint8_t)(0)
      };

      for(uint16_t i = 0; i < crc_l; i++)
      {
        UTL::crc16_modbus_update(&crc, send_data[i]);
      }
      send_data[crc_l] = (uint8_t)(crc >> 0);
      send_data[crc_h] = (uint8_t)(crc >> 8);

      constexpr uint32_t timeout = 100;
      uint8_t retray_cnt = MOONS_RETRY_CNT_MAX;
      while (retray_cnt)
      {
        errno_t result = SendCmdRxResp(send_data.data(), (uint32_t)send_data.size(), timeout);
        if (result == ERROR_SUCCESS)
        {
          return ERROR_SUCCESS;
        }
        else
        {
          retray_cnt--;
        }
      }
      return -1;

    }

    inline errno_t SetOriginParam(uint8_t node_id, uart_moons::speed_t& params, int dir){
      if (m_Isconnected != true)
        return -1;

      uint8_t  func = write_MultiReg;
      uint16_t start_reg_no = Point_to_Point_Acceleration_AC_A;
      enum {accel, decel, speed, dist_h, dist_l, v_max};
      constexpr uint16_t reg_cnt = v_max;
      constexpr uint8_t byte_cnt = v_max * 2;
      std::array<uint16_t, reg_cnt> value = {
          (uint16_t)params.accel,
          (uint16_t)params.decel,
          (uint16_t)params.speed,
          (uint16_t)(dir>>16),
          (uint16_t)(dir>>00),
      };

      uint16_t crc = 0xffff;
      enum{id, fn, reg_h, reg_l, reg_cnt_h, reg_cnt_l, b_cnt, v9, v8, v7, v6, v5, v4, v3, v2, v1, v0,
        crc_l, crc_h,  _max};
      std::array<uint8_t, _max> send_data = {
          node_id,
          func,
          (uint8_t)(start_reg_no >> 8),
          (uint8_t)(start_reg_no),
          (uint8_t)(reg_cnt >> 8),
          (uint8_t)(reg_cnt >> 0),
          byte_cnt,
          (uint8_t)((value[accel] >> 8) & 0xff),
          (uint8_t)((value[accel] >> 0) & 0xff),
          (uint8_t)((value[decel] >> 8) & 0xff),
          (uint8_t)((value[decel] >> 0) & 0xff),
          (uint8_t)((value[speed] >> 8) & 0xff),
          (uint8_t)((value[speed] >> 0) & 0xff),
          (uint8_t)((value[dist_h] >> 8) & 0xff),
          (uint8_t)((value[dist_h] >> 0) & 0xff),
          (uint8_t)((value[dist_l] >> 8) & 0xff),
          (uint8_t)((value[dist_l] >> 0) & 0xff),
          (uint8_t)(0),
          (uint8_t)(0)
      };

      for(uint16_t i = 0; i < crc_l; i++)
      {
        UTL::crc16_modbus_update(&crc, send_data[i]);
      }
      send_data[crc_l] = (uint8_t)(crc >> 0);
      send_data[crc_h] = (uint8_t)(crc >> 8);

      constexpr uint32_t timeout = 100;
      uint8_t retray_cnt = MOONS_RETRY_CNT_MAX;
      while (retray_cnt)
      {
        errno_t result = SendCmdRxResp(send_data.data(), (uint32_t)send_data.size(), timeout);
        if (result == ERROR_SUCCESS)
        {
          return ERROR_SUCCESS;
        }
        else
        {
          retray_cnt--;
        }
      }
      return -1;
    }

    inline errno_t SetMoveParam(uint8_t node_id, uart_moons::speed_t& params){
      if (m_Isconnected != true)
        return -1;

      uint8_t  func = write_MultiReg;
      uint16_t start_reg_no = Point_to_Point_Acceleration_AC_A;
      enum {accel, decel, speed, v_max};
      constexpr uint16_t reg_cnt = v_max;
      constexpr uint8_t byte_cnt = reg_cnt * 2;

      std::array<uint16_t, reg_cnt> value = {
          (uint16_t)params.accel,
          (uint16_t)params.decel,
          (uint16_t)params.speed,
      };

      uint16_t crc = 0xffff;
      enum{id, fn, reg_h, reg_l, reg_cnt_h, reg_cnt_l, b_cnt, v5, v4, v3, v2, v1, v0,
        crc_l, crc_h,  _max};

      std::array<uint8_t, _max> send_data = {
          node_id,
          func,
          (uint8_t)(start_reg_no >> 8),
          (uint8_t)(start_reg_no),
          (uint8_t)(reg_cnt >> 8),
          (uint8_t)(reg_cnt >> 0),
          byte_cnt,
          (uint8_t)((value[accel] >> 8) & 0xff),
          (uint8_t)((value[accel] >> 0) & 0xff),
          (uint8_t)((value[decel] >> 8) & 0xff),
          (uint8_t)((value[decel] >> 0) & 0xff),
          (uint8_t)((value[speed] >> 8) & 0xff),
          (uint8_t)((value[speed] >> 0) & 0xff),
          (uint8_t)(0),
          (uint8_t)(0)
      };

      for(uint16_t i = 0; i < crc_l; i++)
      {
        UTL::crc16_modbus_update(&crc, send_data[i]);
      }
      send_data[crc_l] = (uint8_t)(crc >> 0);
      send_data[crc_h] = (uint8_t)(crc >> 8);

      constexpr uint32_t timeout = 100;
      uint8_t retray_cnt = MOONS_RETRY_CNT_MAX;
      while (retray_cnt)
      {
        errno_t result = SendCmdRxResp(send_data.data(), (uint32_t)send_data.size(), timeout);
        if (result == ERROR_SUCCESS)
        {
          return ERROR_SUCCESS;
        }
        else
        {
          retray_cnt--;
        }
      }
      return -1;
    }

    inline errno_t SetMove(uint8_t node_id, uart_moons::speed_t& params, int dist){
      if (m_Isconnected != true)
        return -1;

      uint8_t  func = write_MultiReg;
      uint16_t start_reg_no = Point_to_Point_Acceleration_AC_A;
      enum {accel, decel, speed, dist_h, dist_l, v_max};
      constexpr uint16_t reg_cnt = v_max;
      constexpr uint8_t byte_cnt = v_max * 2;

      std::array<uint16_t, reg_cnt> value = {
          (uint16_t)params.accel,
          (uint16_t)params.decel,
          (uint16_t)params.speed,
          (uint16_t)(dist>>16),
          (uint16_t)(dist>>00),
      };

      uint16_t crc = 0xffff;
      enum{id, fn, reg_h, reg_l, reg_cnt_h, reg_cnt_l, b_cnt, v9, v8, v7, v6, v5, v4, v3, v2, v1, v0,
        crc_l, crc_h,  _max};

      std::array<uint8_t, _max> send_data = {
          node_id,
          func,
          (uint8_t)(start_reg_no >> 8),
          (uint8_t)(start_reg_no),
          (uint8_t)(reg_cnt >> 8),
          (uint8_t)(reg_cnt >> 0),
          byte_cnt,
          (uint8_t)((value[accel] >> 8) & 0xff),
          (uint8_t)((value[accel] >> 0) & 0xff),
          (uint8_t)((value[decel] >> 8) & 0xff),
          (uint8_t)((value[decel] >> 0) & 0xff),
          (uint8_t)((value[speed] >> 8) & 0xff),
          (uint8_t)((value[speed] >> 0) & 0xff),
          (uint8_t)((value[dist_h] >> 8) & 0xff),
          (uint8_t)((value[dist_h] >> 0) & 0xff),
          (uint8_t)((value[dist_l] >> 8) & 0xff),
          (uint8_t)((value[dist_l] >> 0) & 0xff),
          (uint8_t)(0),
          (uint8_t)(0)
      };

      for(uint16_t i = 0; i < crc_l; i++)
      {
        UTL::crc16_modbus_update(&crc, send_data[i]);
      }
      send_data[crc_l] = (uint8_t)(crc >> 0);
      send_data[crc_h] = (uint8_t)(crc >> 8);

      constexpr uint32_t timeout = 100;
      uint8_t retray_cnt = MOONS_RETRY_CNT_MAX;
      while (retray_cnt)
      {
        errno_t result = SendCmdRxResp(send_data.data(), (uint32_t)send_data.size(), timeout);
        if (result == ERROR_SUCCESS)
        {
          return ERROR_SUCCESS;
        }
        else
        {
          retray_cnt--;
        }
      }
      return -1;
    }

    inline errno_t MoveRelactive(uint8_t node_id) {
      if (m_Isconnected != true)
        return -1;

      constexpr uint16_t feed_to_length = 102;

      uint8_t  func = write_SingleReg;
      uint16_t regist_no = Command_Opcode;
      uint16_t value = feed_to_length;
      uint16_t crc = 0xffff;
      enum{id, fn, reg_h, reg_l, v1, v0,
        crc_l, crc_h,  _max};

      std::array<uint8_t, _max> send_data = {
          node_id,
          func,
          (uint8_t)(regist_no >> 8),
          (uint8_t)(regist_no),
          (uint8_t)((value >> 8) & 0xff),
          (uint8_t)((value >> 0) & 0xff),
          (uint8_t)(0),
          (uint8_t)(0)
      };

      for(uint16_t i = 0; i < crc_l; i++)
      {
        UTL::crc16_modbus_update(&crc, send_data[i]);
      }
      send_data[crc_l] = (uint8_t)(crc >> 0);
      send_data[crc_h] = (uint8_t)(crc >> 8);

      constexpr uint32_t timeout = 100;
      uint8_t retray_cnt = MOONS_RETRY_CNT_MAX;
      while (retray_cnt)
      {
        errno_t result = SendCmdRxResp(send_data.data(), (uint32_t)send_data.size(), timeout);
        if (result == ERROR_SUCCESS)
        {
          return ERROR_SUCCESS;
        }
        else
        {
          retray_cnt--;
        }
      }
      return -1;
    }

    inline errno_t SetOutput(uint8_t node_id, uint16_t pin_no, bool is_on = true){
      if (m_Isconnected != true)
        return -1;

      if (pin_no < 0  || pin_no > 4 )
        return -1;

      constexpr uint16_t set_output = 0x008b;
      uint8_t func = write_MultiReg;
      uint16_t regist_no = Command_Opcode;
      uint16_t value = set_output;
      uint16_t asc_port_0 = 0x0030;
      constexpr char out_on_state = 'L';
      constexpr char out_off_state = 'H';

      enum  { opcode, param1, param2,   v_max };
      std::array<uint16_t, v_max> values = {
          value,
          (uint16_t) (asc_port_0 + pin_no),
          (uint16_t)(is_on?out_on_state:out_off_state)
          };
      constexpr uint16_t reg_cnt = v_max;
      constexpr uint8_t byte_cnt = v_max * 2;
      enum  { id, fn, reg_h, reg_l, reg_cnt_h, reg_cnt_l,b_cnt,
          v5, v4, v3, v2, v1, v0, crc_l, crc_h, _max };

      std::array<uint8_t, _max> send_data = {
          node_id,
          func,
          (uint8_t)(regist_no >> 8),
          (uint8_t)(regist_no >> 0),
          (uint8_t)(reg_cnt >> 8),
          (uint8_t)(reg_cnt >> 0),
          byte_cnt,
          (uint8_t)((values[opcode] >> 8) & 0xff),
          (uint8_t)((values[opcode] >> 0) & 0xff),
          (uint8_t)((values[param1] >> 8) & 0xff),
          (uint8_t)((values[param1] >> 0) & 0xff),
          (uint8_t)((values[param2] >> 8) & 0xff),
          (uint8_t)((values[param2] >> 0) & 0xff),
          (uint8_t)(0),
          (uint8_t)(0)
      };

      uint16_t crc = 0xffff;
      for(uint16_t i = 0; i < crc_l; i++)
      {
        UTL::crc16_modbus_update(&crc, send_data[i]);
      }
      send_data[crc_l] = (uint8_t)(crc >> 0);
      send_data[crc_h] = (uint8_t)(crc >> 8);

      constexpr uint32_t timeout = 100;
      uint8_t retray_cnt = MOONS_RETRY_CNT_MAX;
      while (retray_cnt)
      {
        errno_t result = SendCmdRxResp(send_data.data(), (uint32_t)send_data.size(), timeout);
        if (result == ERROR_SUCCESS)
        {
          return ERROR_SUCCESS;
        }
        else
        {
          retray_cnt--;
        }
      }
      return -1;

    }

    inline errno_t moveAbsolutive(uint8_t node_id) {
      if (m_Isconnected != true)
        return -1;

      constexpr uint16_t feed_to_position = 103;

      uint8_t  func = write_SingleReg;
      uint16_t regist_no = Command_Opcode;
      uint16_t value = feed_to_position;
      uint16_t crc = 0xffff;
      enum{id, fn, reg_h, reg_l, v1, v0,
        crc_l, crc_h,  _max};
      std::array<uint8_t, _max> send_data = {
          node_id,
          func,
          (uint8_t)(regist_no >> 8),
          (uint8_t)(regist_no),
          (uint8_t)((value >> 8) & 0xff),
          (uint8_t)((value >> 0) & 0xff),
          (uint8_t)(0),
          (uint8_t)(0)
      };

      for(uint16_t i = 0; i < crc_l; i++)
      {
        UTL::crc16_modbus_update(&crc, send_data[i]);
      }
      send_data[crc_l] = (uint8_t)(crc >> 0);
      send_data[crc_h] = (uint8_t)(crc >> 8);

      constexpr uint32_t timeout = 100;
      uint8_t retray_cnt = MOONS_RETRY_CNT_MAX;
      while (retray_cnt)
      {
        errno_t result = SendCmdRxResp(send_data.data(), (uint32_t)send_data.size(), timeout);
        if (result == ERROR_SUCCESS)
        {
          return ERROR_SUCCESS;
        }
        else
        {
          retray_cnt--;
        }
      }
      return -1;
    }

    inline errno_t JogMove(uint8_t node_id, uart_moons::speed_t& params, bool is_cw = true) {
      if (m_Isconnected != true)
        return -1;

      uint8_t  func = write_MultiReg;
      uint16_t start_reg_no = Jog_Accel_JA_;
      enum {jog_accel, jog_decel, jog_speed, v_max};
      constexpr uint16_t reg_cnt = v_max;
      constexpr uint8_t byte_cnt = v_max * 2;
      std::array<uint16_t, reg_cnt> value = {
          (uint16_t)params.accel,
          (uint16_t)params.decel,
          (uint16_t)params.speed
      };
      if (is_cw == false)
      {
        int a = (int)(value[2]);
        a *= -1;
        value[2] = (uint16_t)a;
      }

      uint16_t crc = 0xffff;
      enum{id, fn, reg_h, reg_l, reg_cnt_h, reg_cnt_l, b_cnt, v5, v4, v3, v2, v1, v0,
        crc_l, crc_h,  _max};

      std::array<uint8_t, _max> send_data = {
          node_id,
          func,
          (uint8_t)(start_reg_no >> 8),
          (uint8_t)(start_reg_no),
          (uint8_t)(reg_cnt >> 8),
          (uint8_t)(reg_cnt >> 0),
          byte_cnt,
          (uint8_t)((value[jog_accel] >> 8) & 0xff),
          (uint8_t)((value[jog_accel] >> 0) & 0xff) ,
          (uint8_t)((value[jog_decel] >> 8) & 0xff),
          (uint8_t)((value[jog_decel] >> 0) & 0xff) ,
          (uint8_t)((value[jog_speed] >> 8) & 0xff),
          (uint8_t)((value[jog_speed] >> 0) & 0xff),
          (uint8_t)(0),
          (uint8_t)(0)
      };

      for(uint16_t i = 0; i < crc_l; i++)
      {
        UTL::crc16_modbus_update(&crc, send_data[i]);
      }
      send_data[crc_l] = (uint8_t)(crc >> 0);
      send_data[crc_h] = (uint8_t)(crc >> 8);

      constexpr uint32_t timeout = 100;
      uint8_t retray_cnt = MOONS_RETRY_CNT_MAX;
      while (retray_cnt)
      {
        errno_t result = SendCmdRxResp(send_data.data(), (uint32_t)send_data.size(), timeout);
        if (result == ERROR_SUCCESS)
        {
          //retray_cnt = 0;
          jogging(node_id);
          return ERROR_SUCCESS;
        }
        else
        {
          retray_cnt--;
        }
      }
      return -1;
    }

    inline errno_t MoveStop(uint8_t node_id) {
      if (m_Isconnected != true)
        return -1;

      constexpr int stop_move_kill_Buffer_normal_decel = 226;
      uint8_t  func = write_SingleReg;
      uint16_t regist_no = Command_Opcode;
      uint16_t value = stop_move_kill_Buffer_normal_decel;
      uint16_t crc = 0xffff;
      enum{id, fn, reg_h, reg_l, v1, v0,
        crc_l, crc_h,  _max};

      std::array<uint8_t, _max> send_data = {
          node_id,
          func,
          (uint8_t)(regist_no >> 8),
          (uint8_t)(regist_no >> 0),
          (uint8_t)(value >> 8),
          (uint8_t)(value >> 0),
          (uint8_t)(0),
          (uint8_t)(0)
      };

      for(uint16_t i = 0; i < crc_l; i++)
      {
        UTL::crc16_modbus_update(&crc, send_data[i]);
      }
      send_data[crc_l] = (uint8_t)(crc >> 0);
      send_data[crc_h] = (uint8_t)(crc >> 8);

      constexpr uint32_t timeout = 100;
      uint8_t retray_cnt = MOONS_RETRY_CNT_MAX;
      while (retray_cnt)
      {
        errno_t result = SendCmdRxResp(send_data.data(), (uint32_t)send_data.size(), timeout);
        if (result == ERROR_SUCCESS)
        {
          return ERROR_SUCCESS;
        }
        else
        {
          retray_cnt--;
        }
      }
      return -1;

    }

    inline errno_t JogStop(uint8_t node_id) {
      return jogging(node_id, false);
    }

    inline errno_t ClearEncoder(uint8_t node_id){
      if (m_Isconnected != true)
        return -1;

      constexpr uint16_t encoder_position = 152;

      uint8_t  func = write_MultiReg;
      uint16_t reg_no = Command_Opcode;
      uint16_t value = encoder_position;
      enum { op_code, encoder, v_max };
      std::array<uint16_t, v_max> values = {
          value,
          0
      };
      constexpr uint16_t reg_cnt = v_max;
      constexpr uint8_t byte_cnt = v_max * 2;
      uint16_t crc = 0xffff;
      enum{id, fn, reg_h, reg_l, reg_cnt_h, reg_cnt_l, b_cnt, v3, v2, v1, v0,
        crc_l, crc_h,  _max};
      std::array<uint8_t, _max> send_data ={
          node_id,
          func,
          (uint8_t)(reg_no >> 8),
          (uint8_t)reg_no,
          (uint8_t)(reg_cnt >> 8),
          (uint8_t)reg_cnt,
          (uint8_t)byte_cnt,
          (uint8_t)(values[op_code] >> 8),
          (uint8_t)(values[op_code] >> 0),
          (uint8_t)(values[encoder] >> 8),
          (uint8_t)(values[encoder] >> 0),
          (uint8_t)(0),
          (uint8_t)(0)
      };

      for(uint16_t i = 0; i < crc_l; i++)
      {
        UTL::crc16_modbus_update(&crc, send_data[i]);
      }
      send_data[crc_l] = (uint8_t)(crc >> 0);
      send_data[crc_h] = (uint8_t)(crc >> 8);

      constexpr uint32_t timeout = 100;
      uint8_t retray_cnt = MOONS_RETRY_CNT_MAX;
      while (retray_cnt)
      {
        errno_t result = SendCmdRxResp(send_data.data(), (uint32_t)send_data.size(), timeout);
        if (result == ERROR_SUCCESS)
        {
          return commandPosition(node_id, 0);
        }
        else
        {
          retray_cnt--;
        }
      }
      return -1;
    }

    inline errno_t ClearState(uint8_t node_id){
      if (m_Isconnected != true)
        return -1;

      constexpr uint16_t alarm_reset = 186;

      uint8_t  func = write_SingleReg;
      uint16_t reg_no = Command_Opcode;
      uint16_t value = alarm_reset;
      uint16_t crc = 0xffff;
      enum{id, fn, reg_h, reg_l, v1, v0,
        crc_l, crc_h,  _max};
      std::array<uint8_t, _max> send_data ={
          node_id,
          func,
          (uint8_t)(reg_no >> 8),
          (uint8_t)reg_no,
          (uint8_t)(value >> 8),
          (uint8_t)(value >> 0),
          (uint8_t)(0),
          (uint8_t)(0)
      };

      for(uint16_t i = 0; i < crc_l; i++)
      {
        UTL::crc16_modbus_update(&crc, send_data[i]);
      }
      send_data[crc_l] = (uint8_t)(crc >> 0);
      send_data[crc_h] = (uint8_t)(crc >> 8);

      constexpr uint32_t timeout = 100;
      uint8_t retray_cnt = MOONS_RETRY_CNT_MAX;
      while (retray_cnt)
      {
        errno_t result = SendCmdRxResp(send_data.data(), (uint32_t)send_data.size(), timeout);
        if (result == ERROR_SUCCESS)
        {
          return ERROR_SUCCESS;
        }
        else
        {
          retray_cnt--;
        }
      }
      return -1;
    }


  private:
    inline errno_t jogging(uint8_t node_id,bool start = true){
      if (m_Isconnected != true)
        return -1;

      constexpr uint16_t start_jogging = 150;
      constexpr uint16_t stop_jogging = 216;

      uint8_t  func = write_SingleReg;
      uint16_t regist_no = Command_Opcode;
      uint16_t value = (start ? start_jogging : stop_jogging);
      uint16_t crc = 0xffff;
      enum{id, fn, reg_h, reg_l, v1, v0,
        crc_l, crc_h,  _max};

      std::array<uint8_t, _max> send_data = {
          node_id,
          func,
          (uint8_t)(regist_no >> 8),
          (uint8_t)(regist_no >> 0),
          (uint8_t)(value >> 8),
          (uint8_t)(value >> 0),
          (uint8_t)(0),
          (uint8_t)(0)
      };

      for(uint16_t i = 0; i < crc_l; i++)
      {
        UTL::crc16_modbus_update(&crc, send_data[i]);
      }
      send_data[crc_l] = (uint8_t)(crc >> 0);
      send_data[crc_h] = (uint8_t)(crc >> 8);

      constexpr uint32_t timeout = 100;
      uint8_t retray_cnt = MOONS_RETRY_CNT_MAX;
      while (retray_cnt)
      {
        errno_t result = SendCmdRxResp(send_data.data(), (uint32_t)send_data.size(), timeout);
        if (result == ERROR_SUCCESS)
        {
          if (start == false)
            return MoveStop(node_id);
          return ERROR_SUCCESS;
        }
        else
        {
          retray_cnt--;
        }
      }
      return -1;

    }

    inline errno_t commandPosition(uint8_t node_id, uint16_t target_pos){
      if (m_Isconnected != true)
        return -1;

      constexpr uint16_t set_position = 165;

      uint8_t  func = write_MultiReg;
      uint16_t regist_no = Command_Opcode;
      uint16_t value = set_position;
      enum { op_code, cmd_pos, v_max };
      std::array<uint16_t, v_max> values = {
          value,
          target_pos
      };
      constexpr uint16_t reg_cnt = v_max;
      constexpr uint8_t byte_cnt = v_max * 2;
      uint16_t crc = 0xffff;
      enum{id, fn, reg_h, reg_l, reg_cnt_h, reg_cnt_l, b_cnt, v3, v2, v1, v0,
        crc_l, crc_h,  _max};
      std::array<uint8_t, _max> send_data = {
          node_id,
          func,
          (uint8_t)(regist_no >> 8),
          (uint8_t)(regist_no >> 0),
          (uint8_t)(reg_cnt >> 8),
          (uint8_t)(reg_cnt >> 0),
          (uint8_t)byte_cnt,
          (uint8_t)(values[op_code] >> 8),
          (uint8_t)(values[op_code] >> 0),
          (uint8_t)(values[cmd_pos] >> 8),
          (uint8_t)(values[cmd_pos] >> 0),
          (uint8_t)(0),
          (uint8_t)(0)
      };

      for(uint16_t i = 0; i < crc_l; i++)
      {
        UTL::crc16_modbus_update(&crc, send_data[i]);
      }
      send_data[crc_l] = (uint8_t)(crc >> 0);
      send_data[crc_h] = (uint8_t)(crc >> 8);


      constexpr uint32_t timeout = 100;
      uint8_t retray_cnt = MOONS_RETRY_CNT_MAX;
      while (retray_cnt)
      {
        errno_t result = SendCmdRxResp(send_data.data(), (uint32_t)send_data.size(), timeout);
        if (result == ERROR_SUCCESS)
        {
          return ERROR_SUCCESS;
        }
        else
        {
          retray_cnt--;
        }
      }
      return -1;

    }

#if 0

    inline void ResetCommFlag(){
      m_packet.request_flag = 0;
    }


    inline uint8_t GetErrCnt() const {
      return m_packet.error;
    }

    inline uint8_t AddErrCnt()  {
      return ++m_packet.error;
    }


    inline bool IsAvailableComm(){
      if (m_packet.request_flag == 0)
        return true;
      return false;
    }



#endif

    inline errno_t SendCmd(uint8_t* ptr_data, uint32_t size) {
      m_packet_sending_ms = millis();
      ++m_packet.request_flag;

      if (uartWrite(m_cfg.ch, ptr_data, size)){
        return ERROR_SUCCESS;
      }
      return -1;
    }


    inline errno_t SendCmdRxResp(uint8_t* ptr_data, uint32_t size, uint32_t timeout){
      if (SendCmd(ptr_data, size) == ERROR_SUCCESS)
      {
        uint32_t pre_ms = millis();
        bool result = true;
        while (receivePacket() == false)
        {
          if ((millis() - pre_ms) > timeout)
          {
            result = false;
            break;
          }
        }
        if (result)
        {
          receiveCplt();
          return ERROR_SUCCESS;
        }
      }
      return -1;
    }

    inline bool receivePacket() {

      enum : uint8_t
      {
        STATE_WAIT_ID,STATE_WAIT_FUNCNTION,STATE_WAIT_LENGTH,
        STATE_WAIT_REG_ADDR_L,STATE_WAIT_REG_ADDR_H,STATE_WAIT_DATA,
        STATE_WAIT_CHECKSUM_L,STATE_WAIT_CHECKSUM_H
      };


      bool ret = false;
      uint8_t rx_data = 0x00;

      uint8_t data_cnt = 0;
      std::array <uint8_t, (MOONS_PACKET_BUFF_LENGTH + 8)> data_arry = {0,};

      while(uartAvailable(m_cfg.ch))
      {
        if(data_cnt <(MOONS_PACKET_BUFF_LENGTH + 8))
        {
          data_arry[data_cnt++] = uartRead(m_cfg.ch);
        }
        else
        {
          break;
        }
      }//while

      if (data_cnt == 0)
      {
        return false;
      }
      // update flag and for debug
      {
        m_Isconnected = true;
      }
      rx_packet_t* rx_packet = &m_packet.rx_packet;

      if (millis() - m_packet.prev_ms >= 100)
      {
        m_packet.state = STATE_WAIT_ID;
        m_packet.index = 0;
        m_packet.data_cnt = 0;
        rx_packet->check_sum = 0xffff;
        memset(&rx_packet->buffer[0], 0x00, MOONS_PACKET_BUFF_LENGTH);
      }
      m_packet.prev_ms = millis();


      for (uint8_t i = 0 ; i < data_cnt ; i ++ )
      {
        rx_data = data_arry[i];

        switch (m_packet.state)
        {
          case STATE_WAIT_ID:
          {
            rx_packet->node_id = rx_data;
            m_packet.index = 0;
            m_packet.data_cnt = 0;
            rx_packet->check_sum = 0xffff;
            m_packet.state = STATE_WAIT_FUNCNTION;
            UTL::crc16_modbus_update(&rx_packet->check_sum, rx_data);
            rx_packet->buffer[m_packet.index] = rx_data;
            m_packet.index++;
          }
          break;

          case STATE_WAIT_FUNCNTION:
          {
            rx_packet->func_type = rx_data;
            UTL::crc16_modbus_update(&rx_packet->check_sum, rx_data);
            rx_packet->buffer[m_packet.index] = rx_data;
            m_packet.index++;

            switch (static_cast<func_e>(rx_data))
            {
              case func_e::read_HoldingReg:
                __attribute__((fallthrough));
              case func_e::read_InputReg:
                m_packet.state = STATE_WAIT_LENGTH;
                break;

              case func_e::write_SingleReg:  // skip break;
                __attribute__((fallthrough));
              case func_e::write_MultiReg:   // skip break;
                m_packet.state = STATE_WAIT_REG_ADDR_L;
                break;

              default:
                m_packet.state = STATE_WAIT_ID;
                m_packet.index = 0;
                m_packet.data_cnt = 0;
                rx_packet->check_sum = 0xffff;
                memset(&rx_packet->buffer[0], 0x00, MOONS_PACKET_BUFF_LENGTH);
                i = data_cnt;
                break;

            } //switch (static_cast<func_e>(rx_data))
          }
          break;

          case STATE_WAIT_LENGTH:
          {
            rx_packet->data_length = rx_data;
            m_packet.state = STATE_WAIT_DATA;
            UTL::crc16_modbus_update(&rx_packet->check_sum, rx_data);
            rx_packet->buffer[m_packet.index] = rx_data;
            m_packet.index++;
          }
          break;

          case STATE_WAIT_REG_ADDR_L:
          {
            rx_packet->reg_addr = rx_data;
            m_packet.state = STATE_WAIT_REG_ADDR_H;
            UTL::crc16_modbus_update(&rx_packet->check_sum, rx_data);
            rx_packet->buffer[m_packet.index] = rx_data;
            m_packet.index++;
          }
          break;

          case STATE_WAIT_REG_ADDR_H:
          {
            rx_packet->reg_addr |= (rx_data << 8);
            rx_packet->data_length = 2;// 0x06, 0x10 경우 2 byte;
            m_packet.state = STATE_WAIT_DATA;
            UTL::crc16_modbus_update(&rx_packet->check_sum, rx_data);
            rx_packet->buffer[m_packet.index] = rx_data;
            m_packet.index++;
          }
          break;

          case STATE_WAIT_DATA:
          {
            if (m_packet.data_cnt++ == 0)
            {
              rx_packet->data = &rx_packet->buffer[m_packet.index];
            }
            if (m_packet.data_cnt >= rx_packet->data_length)
            {
              m_packet.state = STATE_WAIT_CHECKSUM_L;
            }
            rx_packet->buffer[m_packet.index] = rx_data;
            m_packet.index++;
            UTL::crc16_modbus_update(&rx_packet->check_sum, rx_data);
          }
          break;

          case STATE_WAIT_CHECKSUM_L:
          {
            rx_packet->check_sum_recv = rx_data;
            m_packet.state = STATE_WAIT_CHECKSUM_H;
            rx_packet->buffer[m_packet.index] = rx_data;
            m_packet.index++;
          }
          break;

          case STATE_WAIT_CHECKSUM_H:
          {
            rx_packet->buffer[m_packet.index] = rx_data;
            m_packet.index++;
            rx_packet->check_sum_recv |= (rx_data << 8);
            if (rx_packet->check_sum == rx_packet->check_sum_recv)
            {
              ret = true;
              //buzzerBeep(1, 1);
            }
            m_packet.state = STATE_WAIT_ID;
          }
          break;
        }//switch (m_packet.state)
      }// for


      return ret;
    }

  private:
    inline void receiveCplt() {
      --m_packet.request_flag;
      m_packet.resp_ms = millis() - m_packet_sending_ms;
      m_Isconnected = true;
      uint8_t instance_no = M_GetMotorInstanceId(m_packet.rx_packet.node_id);
      if (instance_no < m_objs.size())
      {
        if (m_func && m_objs[instance_no])
        {
          m_func(m_objs[instance_no],&instance_no,&m_packet.rx_packet);
        }
      }

    }

  };


}


#endif /* AP__INC_UART_MOONS_HPP_ */
