/*
 * exhw.c
 *
 *  Created on: 2022. 7. 24.
 *      Author: gns2l
 */



#include "exhw.h"


bool exhwInit(void)
{
	bool ret = true;

#ifdef _USE_EXHW_AT24C64
	ret = at24c64Init();
#endif

  return ret;
}
