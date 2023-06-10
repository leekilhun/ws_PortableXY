/*
 * bsp.h
 *
 *  Created on: 2023. 6. 10.
 *      Author: gns2l
 */

#ifndef BSP_BSP_H_
#define BSP_BSP_H_

#include "def.h"

#ifdef __cplusplus
extern "C" {
#endif


  bool bspInit(void);
  void bspDeInit(void);
  void delay(uint32_t time_ms);
  uint32_t millis(void);
  void logPrintf(const char *fmt, ...);


#ifdef __cplusplus
}
#endif


#endif /* BSP_BSP_H_ */
