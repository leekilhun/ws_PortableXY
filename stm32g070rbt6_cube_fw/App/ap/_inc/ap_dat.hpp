/*
 * ap_dat.hpp
 *
 *  Created on: 2023. 6. 10.
 *      Author: gns2l
 */

#ifndef AP__INC_AP_DAT_HPP_
#define AP__INC_AP_DAT_HPP_




/**
 * @brief Define the address of eeprom
 *
 * st24c64c - 32byte 256 page  0x0000  ~ 0x2000 *
 */

#define APDAT_START_ADDRESS                0x0000

#define APDAT_START_ADDR                   APDAT_START_ADDRESS
#define APDAT_LENGTH                       4
#define APDAT_ADDR(x)                      APDAT_START_ADDR + ((x) * APDAT_LENGTH)
#define APDAT_CNT_MAX                      8
#define APDAT_END_ADDR                     APDAT_START_ADDR + (APDAT_LENGTH * APDAT_CNT_MAX)


// use machine test (2 byte * 16 data)
#define APDAT_SETUP_START_ADDR             APDAT_END_ADDR
#define APDAT_SETUP_DATA_LENGTH            2
#define APDAT_SETUP_DATA_ADDR(x)           APDAT_SETUP_START_ADDR + ((x) * APDAT_SETUP_DATA_LENGTH)
#define APDAT_SETUP_DATA_CNT_MAX           16
#define APDAT_SETUP_END_ADDR               APDAT_SETUP_START_ADDR + (APDAT_SETUP_DATA_LENGTH * APDAT_SETUP_DATA_CNT_MAX)




struct ap_dat
{
  struct dat_t
  {
    uint16_t  parm1;
    uint16_t  parm2;
  };

  enum addr_e:uint8_t //
  {
    mt_x_turn_dist, mt_y_turn_dist, cfg_3, cfg_4 ,sycn_rate, default_accdec, cfg_7, cfg_8,
    _max
  };

  std::array<dat_t,APDAT_CNT_MAX> apcfg_dat;

  inline void WriteData(addr_e addr, dat_t& data){
    apcfg_dat[addr]=data;
    uint16_t rcv_data[2] = {data.parm1, data.parm2};
    at24c64Write(APDAT_ADDR(addr), (uint8_t *)&rcv_data[0], APDAT_LENGTH);
  }

  inline dat_t* ReadData(addr_e addr){
    return &apcfg_dat[addr];
  }

  inline dat_t* LoadData(addr_e addr){
    uint16_t rcv_data[2] = {0,};
    uint8_t idx = addr;
    at24c64Read(APDAT_ADDR(idx), (uint8_t*)&rcv_data, APDAT_LENGTH);
    apcfg_dat[idx].parm1 =rcv_data[0];
    apcfg_dat[idx].parm2 =rcv_data[1];
    return &apcfg_dat[idx];
  }
  inline dat_t* GetData(addr_e addr){
    return &apcfg_dat[addr];
  }

  inline bool LoadRomData(){
    uint16_t rcv_data[2] = {0,};
    bool ret;
    for (uint8_t i = 0; i < APDAT_CNT_MAX ;  i++) {
      ret = at24c64Read(APDAT_ADDR(i), (uint8_t*)&rcv_data, APDAT_LENGTH);
      apcfg_dat[i].parm1 =rcv_data[0];
      apcfg_dat[i].parm2 =rcv_data[1];
    }
    return ret;
  }

  inline bool ClearRomData(){
    uint8_t data[APDAT_LENGTH] = {0,};
    bool ret = false;
    for (uint8_t i = 0; i < APDAT_CNT_MAX ;  i++) {
      ret = at24c64Write(APDAT_ADDR(i), (uint8_t*)&data, APDAT_LENGTH);
    }
    return ret;
  }
};













#endif /* AP__INC_AP_DAT_HPP_ */
