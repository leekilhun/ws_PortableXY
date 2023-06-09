/*
 * ap.cpp
 *
 *  Created on: Apr 23, 2023
 *      Author: gns2l
 */

#include "ap.hpp"








void  apInit(void)
{

}




void  apMain(void)
{
  uint32_t pre_time;

  pre_time = millis();

  while (1)
  {

    if (millis()-pre_time >= 500)
    {
      pre_time = millis();
    }


    if (uartAvailable(_DEF_UART1) > 0)
    {
      uint8_t rx_data;

      rx_data = uartRead(_DEF_LED1);
      uartPrintf(_DEF_UART1, "rx data : 0x%02X (%c)\n", rx_data, rx_data);
    }


   }

}


