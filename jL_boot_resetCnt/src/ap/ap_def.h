/*
 * ap_def.h
 *
 *  Created on: Dec 27, 2021
 *      Author: gns2.lee
 */

#ifndef SRC_AP_AP_DEF_H_
#define SRC_AP_AP_DEF_H_



#include "exhw.h"
//#include "thread/thread_def.h"

typedef struct
{
  bool (*keepLoop)(void);
} mode_args_t;



#endif /* SRC_AP_AP_DEF_H_ */
