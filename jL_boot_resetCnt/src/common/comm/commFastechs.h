/*
 * commFastechs.h
 *
 *  Created on: 2022. 5. 17.
 *      Author: gns2.lee
 */

#ifndef SRC_COMMON_COMM_COMMFASTECHS_H_
#define SRC_COMMON_COMM_COMMFASTECHS_H_

#include "hw_def.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FASTECHS_CMD_MAX_DATA_LENGTH 48

#define DEF_FASTECH_COMM_TYPE_GET_SLAVE_INFO          0x01
#define DEF_FASTECH_COMM_TYPE_GET_MOTOR_INFO          0x05
#define DEF_FASTECH_COMM_TYPE_SAVE_ROM_ALL            0x10
#define DEF_FASTECH_COMM_TYPE_GET_ROM_PARAM           0x11
#define DEF_FASTECH_COMM_TYPE_SET_RAM_PARAM           0x12
#define DEF_FASTECH_COMM_TYPE_GET_RAM_PARAM           0x13
#define DEF_FASTECH_COMM_TYPE_SET_IO_OUT_REG          0x20
#define DEF_FASTECH_COMM_TYPE_SET_IO_IN_REG           0x21
#define DEF_FASTECH_COMM_TYPE_GET_IO_IN_REG           0x22
#define DEF_FASTECH_COMM_TYPE_GET_IO_OUT_REG          0x23
#define DEF_FASTECH_COMM_TYPE_SET_ASSN_IO_MAP         0x24
#define DEF_FASTECH_COMM_TYPE_GET_ASSN_IO_MAP         0x25
#define DEF_FASTECH_COMM_TYPE_READ_IO_MAP             0x26
#define DEF_FASTECH_COMM_TYPE_TRG_OUT_RUN_A           0x27
#define DEF_FASTECH_COMM_TYPE_TRG_OUT_STATUS          0x28
#define DEF_FASTECH_COMM_TYPE_SERVO_ENABLE            0x2A
#define DEF_FASTECH_COMM_TYPE_ALARM_RESET             0x2B
#define DEF_FASTECH_COMM_TYPE_GET_ALARM_INFO          0x2E
#define DEF_FASTECH_COMM_TYPE_MOVE_STOP               0x31
#define DEF_FASTECH_COMM_TYPE_MOVE_EMG_STOP           0x32
#define DEF_FASTECH_COMM_TYPE_MOVE_ORG_SINGLE_AXIS    0x33
#define DEF_FASTECH_COMM_TYPE_MOVE_ABS_SINGLE_AXIS    0x34
#define DEF_FASTECH_COMM_TYPE_MOVE_REL_SINGLE_AXIS    0x35
#define DEF_FASTECH_COMM_TYPE_MOVE_TO_LIMIT           0x36
#define DEF_FASTECH_COMM_TYPE_MOVE_VELOCITY           0x37
#define DEF_FASTECH_COMM_TYPE_MOVE_ABS_OVERRIDE       0x38
#define DEF_FASTECH_COMM_TYPE_MOVE_REL_OVERRIDE       0x39
#define DEF_FASTECH_COMM_TYPE_MOVE_VELOCITY_OVERRIDE  0x3A
#define DEF_FASTECH_COMM_TYPE_MOVE_ALL_STOP           0x3B
#define DEF_FASTECH_COMM_TYPE_MOVE_ALL_EMG_STOP       0x3C
#define DEF_FASTECH_COMM_TYPE_MOVE_ALL_ORG_STOP       0x3D
#define DEF_FASTECH_COMM_TYPE_MOVE_ALL_ABS_POS        0x3E
#define DEF_FASTECH_COMM_TYPE_MOVE_ALL_REL_POS        0x3F
#define DEF_FASTECH_COMM_TYPE_GET_AXIS_STATUS         0x40
#define DEF_FASTECH_COMM_TYPE_GET_IO_AXIS_STATUS      0x41
#define DEF_FASTECH_COMM_TYPE_GET_MOTION_STATUS       0x42
#define DEF_FASTECH_COMM_TYPE_GET_ALL_STATUS          0x43
#define DEF_FASTECH_COMM_TYPE_CLEAR_POSITION          0x56



  #ifdef _USE_HW_CMD_FASTECHS

  typedef struct
  {
    uint8_t   mt_id;
    uint8_t   sync_no;
    uint8_t   type;
    uint8_t   response;
    uint16_t  length;
    uint32_t  check_sum;
    uint16_t  check_sum_recv;
    uint8_t   buffer[FASTECHS_CMD_MAX_DATA_LENGTH + 8];
    uint8_t*  data;
  } fm_packet_t;

  typedef struct
  {
    uint8_t   mt_id;
    uint8_t   sync_no;
  } fm_id_t;

  typedef struct
  {
    bool      wait_next;
    bool      skip_stuffing;
    uint8_t   state;
    uint32_t  pre_time;
    uint16_t  data_len;
    uint32_t  index;
    uint8_t   error;
    uint8_t   resp_time;
    fm_packet_t  rx_packet;
  } fastechs_t;

  void commFm_Init();
  bool commFm_Open(uint8_t ch, uint32_t baud);
  bool commFm_Recovery();
  bool commFm_Close(fastechs_t *p_cmd);
  bool commFm_ReceivePacket(fastechs_t *p_cmd);

  // data[0](cmd_type)  + data[1 ~ length] (send data)

  uint8_t commFm_SendCmd(fm_id_t* p_id, uint8_t* p_data, uint8_t length);
  void commFm_SendGetAllStatus(fastechs_t *p_cmd);
  void commFm_SendAlarmType(fastechs_t *p_cmd);






#endif /*_USE_HW_CMD_FASTECHS*/

#ifdef __cplusplus
 }
#endif

#endif /* SRC_COMMON_COMM_COMMFASTECHS_H_ */
