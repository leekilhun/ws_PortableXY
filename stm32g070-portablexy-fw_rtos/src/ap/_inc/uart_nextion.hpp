/*
 * uart_nextion.hpp
 *
 *  Created on: 2023. 6. 10.
 *      Author: gns2l
 */
#pragma once
#ifndef AP__INC_UART_NEXTION_HPP_
#define AP__INC_UART_NEXTION_HPP_


/*

mcu - server, lcd - client


TX  (mcu -> lcd) provide information
obj_id [option] 0 is all or ignore


RX (lcd -> mcu) request information or action
obj_id [option] 0 is all or ignore
| SOF  | page | type |Obj Id| Data Length |Data          |   Checksum   | EOF  |
| :--- |:-----|:-----|:---- | :---------- |:-------------|:-------------| :--  |
| 0x4A |1 byte|1 byte|1 byte| 2 byte      |Data 0～Data n|2 byte(crc 16)| 0x4C |


| SOF  | page | type | request beep sound - return true
| :--- |:-----|:-----|
| 0x4A |1 byte|   0  |

| reply  | reply page |
| :---   |:-----------|
| 0x66   |   1 byte   |



 */


namespace NXLCD
{

  //TX  (mcu -> lcd) provide information
  enum TX_TYPE:uint8_t
  {
    TX_MCU_DATA                       = 0x01,
    TX_MOTOR_DATA                     = 0x02,

    TX_LCD_BKCMDLEVEL                 = 0x11,
		TX_LCD_START_REPARSEMODE          = 0x12,
		TX_LCD_END_REPARSEMODE            = 0x13,
		TX_LCD_CHANGE_PAGE                = 0x14,
		TX_LCD_REQUSET_PAGE               = 0x15,
  
    TX_FIRM_INFO                      = 0x50,
    TX_OK_RESPONSE                    = 0xAA
  };

  //RX (lcd -> mcu) request information or action
  enum CMD_TYPE:uint8_t
  {
    CMD_OK_RESPONSE                   = 0x00,
    CMD_READ_MCU_DATA                 = 0x01,
    CMD_READ_FIRM_INFO                = 0x02,

    CMD_CTRL_MOT_ORIGIN               = 0x20,
    CMD_CTRL_MOT_ONOFF                = 0x21,
    CMD_CTRL_MOT_MOVE                 = 0x22,
    CMD_CTRL_MOT_STOP                 = 0x23,
    CMD_CTRL_MOT_JOG                  = 0x24,
    CMD_CTRL_MOT_LIMIT                = 0x25,
    CMD_CTRL_MOT_ZEROSET              = 0x26,
    CMD_CTRL_MOT_RELMOVE              = 0x27,
    CMD_CTRL_MOT_CLEAR_ALARM          = 0x28,
    CMD_CTRL_MOT_CHANGE_VEL           = 0x29,
    CMD_CTRL_MOT_MOVE_VEL             = 0x2A,
    CMD_CTRL_MOT_RELMOVE_VEL          = 0x2B,
    CMD_CTRL_MOT_VEL_JOG              = 0x2C,

    CMD_CTRL_BKCMD                    = 0xAA,
    CMD_CTRL_REQ_BEEP                 = 0xBB,
  };



	constexpr uint8_t CMD_STX = 0x4A;
	constexpr uint8_t CMD_ETX = 0x4C;

	constexpr int NEXTION_COMM_TIMEOUT = 100;

	constexpr int CMD_MAX_DATA_LENGTH = 120;
	constexpr int CMD_MAX_PACKET_LENGTH = (CMD_MAX_DATA_LENGTH + 8);

	constexpr int CMD_STATE_REPLY_PAGE_NO = 11;


#ifdef _USE_HW_RTOS
#define AP_UART_NEXTION_LOCK_BEGIN osMutexWait(m_mutex_id, osWaitForever)
#define AP_UART_NEXTION_LOCK_END   osMutexRelease(m_mutex_id)
#else
#define AP_UART_NEXTION_LOCK_BEGIN
#define AP_UART_NEXTION_LOCK_END
#endif



  struct uart_nextion
  {
    /****************************************************
     *  data
     ****************************************************/
    struct cfg_t
    {
      uint8_t ch{};
      uint32_t baud{};

      cfg_t() = default;
      // copy constructor
      cfg_t(const cfg_t& other) = default;
      // copy assignment
      cfg_t& operator=(const cfg_t& other) = default;
      // move constructor
      cfg_t(cfg_t&& other) = default;
      // move assignment
      cfg_t& operator=(cfg_t&& other) = default;
    };


    struct packet_st {
      uint8_t         type{};
      uint8_t         page{};
      uint8_t         obj_id{};
      uint16_t        data_length{};
      uint8_t*        data{};
      uint16_t        checksum{};
      uint16_t        checksum_recv{};
      std::array <uint8_t, CMD_MAX_PACKET_LENGTH> buffer{};
      uint8_t         buffer_idx{};
      uint16_t        data_cnt{};
      uint32_t        resp_ms{};
      prc_step_t      state{};

      packet_st() = default;
      // copy constructor
      packet_st(const packet_st& other) = default;
      // copy assignment
      packet_st& operator=(const packet_st& other) = default;
      // move constructor
      packet_st(packet_st&& other) = default;
      // move assignment
      packet_st& operator=(packet_st&& other) = default;
      ~packet_st() = default;

      uint8_t BufferAdd(uint8_t rx_data)
      {
        buffer[buffer_idx] = rx_data;
        buffer_idx = (buffer_idx + 1) % CMD_MAX_PACKET_LENGTH;
        return buffer_idx;
      }

      void BufferClear()
      {
        buffer.fill(0);
        buffer_idx = 0;
        data_cnt = 0;
        checksum = 0xffff;
        state.SetStep(0);
      }


    };

    bool m_Isconnected;
    cfg_t m_cfg;
    packet_st m_packet;

    void* m_obj;
    evt_cb m_func;
    uint32_t m_packet_sending_ms;
    uint8_t m_reqFlag;

#ifdef _USE_HW_RTOS
    osMutexId m_mutex_id;
#endif

    /****************************************************
     *  Constructor
     ****************************************************/
  public:
    uart_nextion():m_Isconnected{}, m_cfg{}, m_packet{}, m_obj{}, m_func{}, m_packet_sending_ms{}, m_reqFlag{}
    {
#ifdef _USE_HW_RTOS
      osMutexDef(m_mutex_id);
      m_mutex_id = osMutexCreate (osMutex(m_mutex_id));
#endif

    }
    ~uart_nextion (){}

    /****************************************************
     *  func
     ****************************************************/

    inline void Init(cfg_t &cfg) {
      m_cfg = cfg;
      if (uartOpen(cfg.ch, cfg.baud))
      {
        LOG_PRINT("uart_nextion Init Success! Uart ch[%d], baud[%d]",cfg.ch, cfg.baud);
        m_Isconnected = true;
      }
    }


    inline bool Recovery() {
      uartClose(m_cfg.ch);
      /* */
      return uartOpen(m_cfg.ch, m_cfg.baud);
    }

    inline bool IsAvailableComm(){
      return (m_reqFlag == 0);
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

    inline void AttCallbackFunc(void* obj, evt_cb cb)
    {
      m_obj = obj;
      m_func = cb;
    }


    inline bool SendNextionData(TX_TYPE tx_cmd, uint8_t* ptr_data, uint32_t length)
    {
      constexpr int NEXTION_PACKET_BUFF_LENGTH = 32;
      std::array<uint8_t, NEXTION_PACKET_BUFF_LENGTH> temp = {0, };
      bool ret = false;
      switch (tx_cmd)
      {
      case TX_MCU_DATA:
        /* code */
        break;

      case TX_MOTOR_DATA:
        /* code */
        break;

      case TX_LCD_BKCMDLEVEL:
        sprintf((char *)temp.data(), "bkcmd=%d", (*(int *)ptr_data));
        ret = true;
        break;

      case TX_LCD_START_REPARSEMODE:
        sprintf((char *)temp.data(), "recmod=%d", (*(int *)ptr_data));
        ret = true;
        break;

      case TX_LCD_END_REPARSEMODE:
        sprintf((char *)temp.data(), "DRAKJHSUYDGBNCJHGJKSHBDN");
        ret = true;
        break;

      case TX_LCD_CHANGE_PAGE:
        sprintf((char *)temp.data(), "page %d", (*(int *)ptr_data));
        ret = true;
        break;

      case TX_LCD_REQUSET_PAGE:
        sprintf((char *)temp.data(), "sendme");
        ret = true;
        break;

      case TX_FIRM_INFO:
        sprintf((char *)temp.data(), "FIRM_INFO.val=%s","test_frim_1.1");
        ret = true;
        break;

      case TX_OK_RESPONSE:
        sprintf((char *)temp.data(), "isResp.val=1");
        ret = true;
        break;

      default:
        break;
      }
      // end of switch
      if (ret)
      {
        uint32_t len = strnlen((const char *)temp.data(), NEXTION_PACKET_BUFF_LENGTH);
        temp[len++] = 0xff;
        temp[len++] = 0xff;
        temp[len++] = 0xff;
        m_packet_sending_ms = millis();
        ret = (uartWrite(m_cfg.ch, temp.data(), len) > 0) ? true : false;
      }
      return ret;
    }


    inline bool receivePacket() {

      enum : uint8_t
      {  STATE_WAIT_STX, STATE_WAIT_PAGE_NO,
        STATE_WAIT_TYPE,  STATE_WAIT_OBJ_ID,  STATE_WAIT_LENGTH_L, STATE_WAIT_LENGTH_H ,
        STATE_WAIT_DATA, STATE_WAIT_CHECKSUM_L , STATE_WAIT_CHECKSUM_H ,
        STATE_WAIT_ETX
      };

      uint8_t rx_data = 0x00 ;

      if (m_packet.state.MoreThan(100))
      {
        m_packet.BufferClear();
      }

      while (uartAvailable(m_cfg.ch))
      {
        rx_data = uartRead(m_cfg.ch);
        //LOG_PRINT("rx_data %d", rx_data);

        switch (m_packet.state.GetStep())
        {
          case STATE_WAIT_STX:
            if (rx_data == CMD_STX)
            {
              m_packet.BufferClear();
              m_packet.BufferAdd(rx_data);
              m_packet.state.SetStep(STATE_WAIT_PAGE_NO);
            }
            break;
          
          case STATE_WAIT_PAGE_NO:
            m_packet.page = rx_data;
            m_packet.BufferAdd(rx_data);
            UTL::crc16_modbus_update(&m_packet.checksum, rx_data);
            m_packet.state.SetStep(STATE_WAIT_TYPE);
            break;


          case STATE_WAIT_TYPE:
            m_packet.type = rx_data;
            m_packet.BufferAdd(rx_data);
            UTL::crc16_modbus_update(&m_packet.checksum, rx_data);
            m_packet.state.SetStep(STATE_WAIT_OBJ_ID);
            break;

          case STATE_WAIT_OBJ_ID:
            m_packet.obj_id = rx_data;
            m_packet.BufferAdd(rx_data);
            UTL::crc16_modbus_update(&m_packet.checksum, rx_data);
            m_packet.state.SetStep(STATE_WAIT_LENGTH_L);
            break;

          case STATE_WAIT_LENGTH_L:
            m_packet.data_length = rx_data;
            m_packet.BufferAdd(rx_data);
            UTL::crc16_modbus_update(&m_packet.checksum, rx_data);
            m_packet.state.SetStep(STATE_WAIT_LENGTH_H);
            break;

          case STATE_WAIT_LENGTH_H:
            m_packet.data_length |= (rx_data << 8);
            m_packet.BufferAdd(rx_data);
            UTL::crc16_modbus_update(&m_packet.checksum, rx_data);
            if (m_packet.data_length == 0)
              m_packet.state.SetStep(STATE_WAIT_CHECKSUM_L);
            else if (m_packet.data_length < (CMD_MAX_DATA_LENGTH + 1))
              m_packet.state.SetStep(STATE_WAIT_DATA);
            else
              m_packet.state.SetStep(STATE_WAIT_STX);
            break;

          case STATE_WAIT_DATA:
            // assign data address
            if (m_packet.data_cnt++ == 0)
              m_packet.data = &m_packet.buffer[m_packet.buffer_idx];

            // check length
            if (m_packet.data_cnt == m_packet.data_length)
              m_packet.state.SetStep(STATE_WAIT_CHECKSUM_L);

            UTL::crc16_modbus_update(&m_packet.checksum, rx_data);
            m_packet.BufferAdd(rx_data);
            break;

          case STATE_WAIT_CHECKSUM_L:
            m_packet.checksum_recv = rx_data;
            m_packet.state.SetStep(STATE_WAIT_CHECKSUM_H);
            m_packet.BufferAdd(rx_data);
            break;

          case STATE_WAIT_CHECKSUM_H:
            m_packet.BufferAdd(rx_data);
            m_packet.checksum_recv |= (rx_data << 8);
            m_packet.state.SetStep(STATE_WAIT_ETX);
            break;

          case STATE_WAIT_ETX:
            m_packet.state.SetStep(STATE_WAIT_STX);
            if (rx_data == CMD_ETX)
            {
              m_packet.BufferAdd(rx_data);

              if (m_packet.checksum == m_packet.checksum_recv)
                return true;
            }
            break;

          default:
            return false;
        }
        // end of  switch
      }
      //while (uartAvailable(m_cfg.ch))




      return false;
    }


  private:
    inline void receiveCplt() {
      m_reqFlag = 0;
      m_packet.resp_ms = millis() - m_packet_sending_ms;
      if (m_func && m_obj)
      {
        m_func(m_obj,nullptr,&m_packet);
      }
    }


  };











}

#endif /* AP__INC_UART_NEXTION_HPP_ */
