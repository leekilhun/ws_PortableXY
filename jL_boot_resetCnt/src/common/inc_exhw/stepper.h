/*
 * stepper.h
 *
 *  Created on: 2022. 7. 31.
 *      Author: gns2l
 */

#ifndef SRC_COMMON_INC_EXHW_STEPPER_H_
#define SRC_COMMON_INC_EXHW_STEPPER_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "exhw_def.h"

#ifdef _USE_EXHW_STEPPER

typedef struct
{
    axis_dir            axis;
    tim_tbl_t *p_tim_handle;
    uint32_t channel;

    uint32_t acceleration;
    volatile uint32_t minStepInterval; // max speed, smaller is faster
    void (*func_dir)(axis_dir axis, int dir);
    void (*func_step)(axis_dir axis);

    /*derived parameters*/
    uint32_t c0; // step interval for first step, determines acceleration
    long stepPosition;//current position of stepper (total of all movements)

    /*per movement variables (only changed once per movement) */
    volatile int dir; // current direction of movement, used to keep
    volatile uint32_t   totalSteps ; // number of  steps requested for current movement
    volatile uint32_t   rampUpStepCount; //number of steps taken to reach either max
    volatile uint64_t   estStepsToSpeed;// estimated steps required to reach max speed
    volatile uint64_t   estTimeForMove;   // estimated time (interrupt ticks) required to complete movement
    volatile uint64_t   rampUpStepTime;
    volatile float      speedScale; // used to slow down this motor to make coordinated movement with other motors
    volatile bool       moveDone ;

    /* per iteration variables (potentially changed every interrupt) */
    volatile uint32_t   n; // index in acceleration curve, used to calculation
    volatile float d;// current interval length
    volatile uint32_t   di;// above variable truncated
    volatile uint32_t   stepCount; // number of steps completed in current movement

    volatile uint32_t   tim_cnt;
    bool                is_init;
    bool                is_start;

} stepper_tbl_t;


  bool stepperInit(void);
  bool stepperStart(axis_dir axis);


  void stepperConstantAccel(void);
  void stepperMove(axis_dir axis, long p, bool wait);

  void stepperPrepareMove(axis_dir axis, int steps);
  void stepperRunAndWait(void);


  void stepperEnable(axis_dir axis);
  void stepperDisable(axis_dir axis);


#endif


#ifdef __cplusplus
}
#endif


#endif /* SRC_COMMON_INC_EXHW_STEPPER_H_ */
