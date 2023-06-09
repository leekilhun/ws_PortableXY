/*
 * main.cpp
 *
 *  Created on: Aug 21, 2022
 *      Author: gns2l
 */

#include "main.h"

#ifndef _USE_HW_RTOS

int main(void)
{
  bspInit();
  hwInit();
  exhwInit();
  apInit();
  apMain();

  return 0;
}

#else

static void mainThread(void const *argument);

int main(void)
{
  bspInit();

  //apMain();

  osThreadDef(mainThread, mainThread, _HW_DEF_RTOS_THREAD_PRI_MAIN, 0, _HW_DEF_RTOS_THREAD_MEM_MAIN);
  if (osThreadCreate(osThread(mainThread), NULL) == NULL)
  {
    ledInit();

    while(1)
    {
      delay(100);
    }
  }

  osKernelStart();

  return 0;
}


void mainThread(void const *argument)
{
  UNUSED(argument);

  hwInit();
  exhwInit();
  apInit();
  apMain();

}



#endif
