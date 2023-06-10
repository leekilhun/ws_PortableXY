/*
 * stepper.c
 *
 *  Created on: 2022. 7. 31.
 *      Author: gns2l
 */





#include "stepper.h"
#include "tim.h"


#ifdef _USE_EXHW_STEPPER


#define STEPS           800*10  // one turn steps 1600
#define DEF_MAX_SPEED   10     // high speed
#define DEF_START_DELAY 400   // 2000 * sqrt(2 * angle(1)/accel(0.01))*0.67703;


stepper_tbl_t  stepper_tbl[AXIS_MAX];
static tim_tbl_t *pTimer;
volatile uint8_t remainingSteppersFlag = 0;
volatile uint8_t nextStepperFlag = 0;
volatile uint32_t SetArr = 0;
volatile unsigned int intervals[100];


bool stepperInit(void)
{
  for (int i=0; i<(int)AXIS_MAX; i++)
  {
    stepper_tbl[i].axis             = (axis_dir)i ;
    stepper_tbl[i].p_tim_handle     = NULL;
    stepper_tbl[i].channel          = 0;

    stepper_tbl[i].acceleration     = 0;
    stepper_tbl[i].c0               = DEF_START_DELAY;
    stepper_tbl[i].func_dir         = NULL;
    stepper_tbl[i].func_step        = NULL;
    stepper_tbl[i].is_init          = true;
    stepper_tbl[i].is_start         = false;
    stepper_tbl[i].stepPosition     = 0;

    stepper_tbl[i].d                = 0;
    stepper_tbl[i].di               = 0;
    stepper_tbl[i].dir              = 0;
    stepper_tbl[i].minStepInterval  = 0;
    stepper_tbl[i].moveDone         = false;
    stepper_tbl[i].n                = 0;
    stepper_tbl[i].rampUpStepCount  = 0;
    stepper_tbl[i].stepCount        = 0;
    stepper_tbl[i].totalSteps       = 0;
    stepper_tbl[i].estStepsToSpeed  = 0;
    stepper_tbl[i].estTimeForMove   = 0;
    stepper_tbl[i].rampUpStepTime   = 0;
    stepper_tbl[i].speedScale       = 0;
    stepper_tbl[i].tim_cnt          = 0;

    stepperDisable((axis_dir)i);

  }

  return true;
}

static void stepperFuncStep(axis_dir axis);
static void stepperFuncDir(axis_dir axis, int dir);
static void moveNSteps(axis_dir axis, long steps);
static void stepperReset(volatile stepper_tbl_t* pMotor);

static void stepperISR_Axis_X(void);
static void stepperISR_Axis_Y(void);

static float getDurationOfAcceleration(volatile stepper_tbl_t* pMotor, uint32_t numSteps);
static void adjustSpeedScales(void);
static void startISR(void);

static void delay_us(uint32_t us);


bool stepperStart(axis_dir axis)
{
  switch (axis)
  {
    case AXIS_1:
      stepper_tbl[AXIS_1].func_dir        = stepperFuncDir;
      stepper_tbl[AXIS_1].func_step       = stepperFuncStep;
      stepper_tbl[AXIS_1].acceleration    = DEF_START_DELAY;
      stepper_tbl[AXIS_1].minStepInterval = DEF_MAX_SPEED;
      if (timStart(_DEF_TIM1) == false)   { return false; }
      stepper_tbl[AXIS_1].p_tim_handle    = timGetData(_DEF_TIM1);
      stepper_tbl[AXIS_1].p_tim_handle->func_callback = stepperISR_Axis_X;

      timChangePrescale(_DEF_TIM1, 612);
      //timEnableISR(_DEF_TIM1);

      break;
    case AXIS_2:
      stepper_tbl[AXIS_2].func_dir        = stepperFuncDir;
      stepper_tbl[AXIS_2].func_step       = stepperFuncStep;
      stepper_tbl[AXIS_2].acceleration    = DEF_START_DELAY;
      stepper_tbl[AXIS_2].minStepInterval = DEF_MAX_SPEED;
      if (timStart(_DEF_TIM2) == false)   { return false; }
      stepper_tbl[AXIS_2].p_tim_handle    = timGetData(_DEF_TIM2);
      stepper_tbl[AXIS_2].p_tim_handle->func_callback = stepperISR_Axis_Y;

      timChangePrescale(_DEF_TIM2, 612);
      //timEnableISR(_DEF_TIM2);

      break;

    case AXIS_3:      break;
    case AXIS_4:      break;
    case AXIS_MAX:
      break;
  }

  return true;
}



void delay_us(uint32_t us)
{
  volatile uint32_t i;
  for (i=0; i<us; i++) { }
}

void stepperEnable(axis_dir axis)
{
  switch (axis)
  {
    case AXIS_1:
      gpioPinWrite(_PIN_XMOTOR_EN,_DEF_LOW);
      break;
    case AXIS_2:
      gpioPinWrite(_PIN_YMOTOR_EN,_DEF_LOW);
      break;

    case AXIS_3:            break;
    case AXIS_4:            break;
    case AXIS_MAX:
      gpioPinWrite(_PIN_XMOTOR_EN,_DEF_LOW);
      gpioPinWrite(_PIN_YMOTOR_EN,_DEF_LOW);
      break;
  }
}
void stepperDisable(axis_dir axis)
{
  switch (axis)
  {
    case AXIS_1:
      gpioPinWrite(_PIN_XMOTOR_EN,_DEF_HIGH);
      break;
    case AXIS_2:
      gpioPinWrite(_PIN_YMOTOR_EN,_DEF_HIGH);
      break;

    case AXIS_3:            break;
    case AXIS_4:            break;
    case AXIS_MAX:
      gpioPinWrite(_PIN_XMOTOR_EN,_DEF_HIGH);
      gpioPinWrite(_PIN_YMOTOR_EN,_DEF_HIGH);
      break;
  }
}


//volatile byte remainingSteppersFlag = 0;

/* test code 1cycle for and backward*/
void stepperConstantAccel(void)
{
  uint32_t delays[STEPS];
  float angle =1;
  float accel = 0.01;
  float c0 = 2000 * sqrt(2 * angle/accel)*0.67703;
  float lastDelay = 0;
  uint32_t highSpeed = 500;


  float d= c0;

  for (uint32_t i =0; i <STEPS; i++)
  {
    if (i>0)
    {
      d = lastDelay - (2 * lastDelay)/(4*i+1);
    }
    if (d<highSpeed)
    {
      d = highSpeed;
    }

    delays[i] = d;
    lastDelay = d;
  }

  // use delays from the array, forward
  for (int i=0; i<STEPS;i++)
  {
    gpioPinWrite(_PIN_XMOTOR_STP,_DEF_HIGH);
    gpioPinWrite(_PIN_XMOTOR_STP,_DEF_LOW);

    delay_us(delays[i]);
  }

  // high speed
  for (int i=0; i<STEPS*4;i++)
  {
    gpioPinWrite(_PIN_XMOTOR_STP,_DEF_HIGH);
    gpioPinWrite(_PIN_XMOTOR_STP,_DEF_LOW);

    delay_us(highSpeed);
  }

  // use delays from the array, backward
  for (int i=0; i<STEPS;i++)
  {
    gpioPinWrite(_PIN_XMOTOR_STP,_DEF_HIGH);
    gpioPinWrite(_PIN_XMOTOR_STP,_DEF_LOW);

    delay_us(delays[STEPS-i-1]);
  }

}



void stepperMoveSteps(long steps)
{
  gpioPinWrite(_PIN_XMOTOR_DIR,steps<0?_DEF_HIGH:_DEF_LOW);
  //stepper_tbl[AXIS_X].dir
}


void stepperFuncStep(axis_dir axis)
{
  switch (axis)
  {
    case AXIS_1:

      LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_8);
      // break;
      //gpioPinWrite(_PIN_XMOTOR_STP,_DEF_HIGH);
      //gpioPinWrite(_PIN_XMOTOR_STP,_DEF_LOW);
      LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_13);
      LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_13);

      break;
    case AXIS_2:
      //break;

      //LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_8);
      //gpioPinWrite(_PIN_YMOTOR_STP,_DEF_HIGH);
      //gpioPinWrite(_PIN_YMOTOR_STP,_DEF_LOW);
      //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
      //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
      LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_10);
      LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_10);

      break;

    case AXIS_3:            break;
    case AXIS_4:            break;
    case AXIS_MAX:  break;

  }
}
void stepperFuncDir(axis_dir axis, int dir)
{
  switch (axis)
  {
    case AXIS_1:
      gpioPinWrite(_PIN_XMOTOR_DIR,dir);
      break;
    case AXIS_2:
      gpioPinWrite(_PIN_YMOTOR_DIR,dir);
      break;

    case AXIS_3:         break;
    case AXIS_4:         break;
    case AXIS_MAX:  break;

  }
}



void moveNSteps(axis_dir axis, long steps)
{
  stepper_tbl[axis].dir             = steps >0 ? 1:-1;
  stepper_tbl[axis].totalSteps      = abs(steps);
  stepper_tbl[axis].d               = stepper_tbl[axis].c0;
  //timUpdateARR(uint8_t ch,uint32_t val)

  pTimer->pHandle->Instance->ARR    = stepper_tbl[axis].d;

  //stepper_tbl[axis].p_tim_handle->pHandle->Instance->ARR = stepper_tbl[axis].d;
  stepper_tbl[axis].stepCount       = 0;
  stepper_tbl[axis].n               = 0;
  stepper_tbl[axis].rampUpStepCount = 0;
  stepper_tbl[axis].moveDone        = false;
  stepper_tbl[axis].is_start        = true;

  switch (axis)
  {
    case AXIS_1:
      gpioPinWrite(_PIN_XMOTOR_DIR,steps<0?_DEF_HIGH:_DEF_LOW);
      timEnableISR(_DEF_TIM1);
      break;

    case AXIS_2:
      gpioPinWrite(_PIN_YMOTOR_DIR,steps<0?_DEF_HIGH:_DEF_LOW);
      timEnableISR(_DEF_TIM2);
      break;

    case AXIS_3:  break;
    case AXIS_4:  break;
    case AXIS_MAX:  break;
  }

}

void stepperMove(axis_dir axis, long p, bool wait)
{
  moveNSteps(axis, p- stepper_tbl[axis].stepPosition);
  while (wait && !stepper_tbl[axis].moveDone);
}


void stepperReset(volatile stepper_tbl_t* pMotor)
{
  pMotor->c0              = pMotor->acceleration;
  pMotor->d               = pMotor->c0;
  pMotor->di              = pMotor->d;
  pMotor->stepCount       = 0;

  pMotor->n               = 0;
  pMotor->rampUpStepCount = 0;
  pMotor->speedScale      = 1;
  pMotor->tim_cnt         = 0;
  pMotor->moveDone        = false;

  /* */
  float a = pMotor->minStepInterval / (float)pMotor->c0;
  a *= 0.676;
  float m = ((a*a - 1) / (-2 * a));
  float n = m * m;
  pMotor->estStepsToSpeed = n;

  pMotor->is_start        = true;

}


float getDurationOfAcceleration(volatile stepper_tbl_t* pMotor, uint32_t numSteps)
{
  float d = pMotor->c0;
  float totalDuration = 0;
  for (unsigned int n = 1; n < numSteps; n++)
  {
    d = d - (2 * d) / (4 * n + 1);
    totalDuration += d;
  }
  return totalDuration;
}


void adjustSpeedScales()
{
  float maxTime = 0;
  bool isThisMotor;
  for (int i = 0; i < AXIS_MAX; i++)
  {
    isThisMotor = ((1 << i) & remainingSteppersFlag);
    if (isThisMotor == false)
      continue;
    if (stepper_tbl[i].estTimeForMove > maxTime )
      maxTime = stepper_tbl[i].estTimeForMove;
  }

  if ( maxTime != 0 )
  {
    for (int i = 0; i < AXIS_MAX; i++)
    {
      isThisMotor = ((1 << i) & remainingSteppersFlag);
      if (isThisMotor == false)
        continue;
      stepper_tbl[i].speedScale = maxTime / stepper_tbl[i].estTimeForMove;
    }
  }
}


void stepperPrepareMove(axis_dir axis, int steps)
{
  volatile stepper_tbl_t *si  =  &stepper_tbl[axis];
  volatile int moveStep       = 0;
  moveStep = steps - stepper_tbl[axis].stepPosition;
  si->func_dir(axis, moveStep < 0 ? _DEF_HIGH : _DEF_LOW );
  si->dir  = moveStep > 0 ? 1 : -1;
  si->totalSteps = abs(moveStep);

  stepperReset(si);

  remainingSteppersFlag |= (1 << axis);  //On motor timer_cal

  uint64_t stepsAbs = abs(moveStep);

  if ( (2 * si->estStepsToSpeed) < stepsAbs )
  {
    // there will be a period of time at full speed
    uint64_t stepsAtFullSpeed = stepsAbs - 2 * si->estStepsToSpeed;
    float accelDecelTime = getDurationOfAcceleration(si, si->estStepsToSpeed);
    si->estTimeForMove = (2 * accelDecelTime) + (stepsAtFullSpeed * si->minStepInterval);
  }
  else
  {
    // will not reach full speed before needing to slow down again
    float accelDecelTime = getDurationOfAcceleration( si, stepsAbs / 2 );
    si->estTimeForMove = 2 * accelDecelTime;
  }


  //INTERRUPTS_ON
  /*
  si->p_tim_handle->pHandle->Instance->CCR1 = 10;
  si->p_tim_handle->pHandle->Instance->ARR  = 10;
  switch (axis)
  {
    case AXIS_1:  timEnableISR(_DEF_TIM1);  break;
    case AXIS_2:  timEnableISR(_DEF_TIM2);  break;
    case AXIS_3:  break;
    case AXIS_4:  break;
    default:      break;
  }
 */

}

void startISR(void)
{
  bool isThisMotor;
  for (int i = 0; i < AXIS_MAX; i++)
   {
     isThisMotor = ((1 << i) & remainingSteppersFlag);

     if (isThisMotor == true)
     {
       stepper_tbl[i].p_tim_handle->pHandle->Instance->CCR1 =10;
       stepper_tbl[i].p_tim_handle->pHandle->Instance->ARR =10;
       switch (i)
       {
         case AXIS_1:  timEnableISR(_DEF_TIM1);  break;
         case AXIS_2:  timEnableISR(_DEF_TIM2);  break;
         case AXIS_3:  break;
         case AXIS_4:  break;
         default:      break;
       }
     }
   }
}

void stepperRunAndWait(void)
{
  adjustSpeedScales();
  //setNextInterruptInterval();
  startISR();
  while (1)
  {
    if (remainingSteppersFlag == 0)
      break;
  }

}

void stepperISR_Axis_X(void)
{
  stepper_tbl_t * pM   = &stepper_tbl[AXIS_1];


  /*### step and checking arrived ###*/
  if ( pM->stepCount < pM->totalSteps )
  {
    pM->func_step(AXIS_1);
    pM->stepCount++;
    pM->stepPosition += pM->dir;
    pM->tim_cnt = 0;
  }
  else //if( s->stepCount >= s->totalSteps )
  {
    pM->moveDone = true;
    remainingSteppersFlag &= ~(1 << AXIS_1); //Off  motor timer_cal
    pM->is_start = false;

    timDisableISR(_DEF_TIM1);
    return;
  }

  /*### set motor interval delay time ###*/

  if ((pM->stepCount > (pM->totalSteps/2)) == true)// reached halfway point
  {
    // ramp down phase
    if (pM->n > 0)
    {
      if (pM->n < pM->rampUpStepCount)
      {
        pM->d = (pM->d*(4*pM->n+1))/ (4*pM->n+1-2);
      }
      pM->n--;
    }
  }
  else // ramp up phase
  {
    pM->n++;
    pM->d = pM->d -(2 * pM->d)/(4 * pM->n + 1);

    if (pM->d <= pM->minStepInterval) //reached max speed
    {
      if (pM->rampUpStepCount == 0 )
      {
        pM->rampUpStepCount = pM->stepCount;
      }
      pM->d = pM->minStepInterval;
    }
  }

  pM->di = pM->d * pM->speedScale; // integer


  pM->p_tim_handle->pHandle->Instance->ARR = pM->di;
  timEnableISR(_DEF_TIM1);

}

void stepperISR_Axis_Y(void)
{
  stepper_tbl_t * pM   = &stepper_tbl[AXIS_2];

  /*### step and checking arrived ###*/
  if ( pM->stepCount < pM->totalSteps )
  {
    pM->func_step(AXIS_2);
    pM->stepCount++;
    pM->stepPosition += pM->dir;
    pM->tim_cnt = 0;
  }
  else //if( s->stepCount >= s->totalSteps )
  {
    pM->moveDone = true;
    remainingSteppersFlag &= ~(1 << AXIS_2); //Off  motor timer_cal
    pM->is_start = false;

    timDisableISR(_DEF_TIM2);
    return;
  }

  /*### set motor interval delay time ###*/

  if ((pM->stepCount > (pM->totalSteps/2)) == true)// reached halfway point
  {
    // ramp down phase
    if (pM->n > 0)
    {
      if (pM->n < pM->rampUpStepCount)
      {
        pM->d = (pM->d*(4*pM->n+1))/ (4*pM->n+1-2);
      }
      pM->n--;
    }
  }
  else // ramp up phase
  {
    pM->n++;
    pM->d = pM->d -(2 * pM->d)/(4 * pM->n + 1);

    if (pM->d <= pM->minStepInterval) //reached max speed
    {
      if (pM->rampUpStepCount == 0 )
      {
        pM->rampUpStepCount = pM->stepCount;
      }
      pM->d = pM->minStepInterval;
    }
  }

  pM->di = pM->d * pM->speedScale; // integer


  pM->p_tim_handle->pHandle->Instance->ARR = pM->di;
  timEnableISR(_DEF_TIM2);
}




#if 0
void moveNSteps(axis_dir axis, long steps)
{
  stepper_tbl[axis].dir             = steps >0 ? 1:-1;
  stepper_tbl[axis].totalSteps      = abs(steps);
  stepper_tbl[axis].d               = stepper_tbl[axis].c0;
  //timUpdateARR(uint8_t ch,uint32_t val)

  pTimer->pHandle->Instance->ARR    = stepper_tbl[axis].d;

  //stepper_tbl[axis].p_tim_handle->pHandle->Instance->ARR = stepper_tbl[axis].d;
  stepper_tbl[axis].stepCount       = 0;
  stepper_tbl[axis].n               = 0;
  stepper_tbl[axis].rampUpStepCount = 0;
  stepper_tbl[axis].moveDone        = false;
  stepper_tbl[axis].is_start        = true;

  switch (axis)
  {
    case AXIS_1:
      gpioPinWrite(_PIN_XMOTOR_DIR,steps<0?_DEF_HIGH:_DEF_LOW);
      timEnableISR(_DEF_TIM1);
      break;

    case AXIS_2:
      gpioPinWrite(_PIN_YMOTOR_DIR,steps<0?_DEF_HIGH:_DEF_LOW);
      timEnableISR(_DEF_TIM2);
      break;

    case AXIS_3:  break;
    case AXIS_4:  break;
    case AXIS_MAX:  break;
  }

}

void stepperMove(axis_dir axis, long p, bool wait)
{
  moveNSteps(axis, p- stepper_tbl[axis].stepPosition);
  while (wait && !stepper_tbl[axis].moveDone);
}


void stepperReset(volatile stepper_tbl_t* pMotor)
{
  pMotor->c0              = pMotor->acceleration;
  pMotor->d               = (float)pMotor->c0;
  pMotor->di              = (uint32_t)pMotor->d;
  pMotor->stepCount       = 0;

  pMotor->n               = 0;
  pMotor->rampUpStepCount = 0;
  pMotor->speedScale      = 1.0;
  pMotor->tim_cnt         = 0;
  pMotor->moveDone        = false;

  /* */
  float a = (float)pMotor->minStepInterval / (float)pMotor->c0;
  a *= 0.676;
  float m = ((a*a - 1.0) / (-2.0 * a));
  float n = m * m;
  pMotor->estStepsToSpeed = (uint64_t)n;

  pMotor->is_start        = true;

}


float getDurationOfAcceleration(volatile stepper_tbl_t* pMotor, uint32_t numSteps)
{
  float d = (float)pMotor->c0;
  float totalDuration = 0.0;
  for (uint32_t n = 1; n < numSteps; n++)
  {
    d = d - (2.0 * d) / (4.0 * (float)n + 1.0);
    totalDuration += d;
  }
  return totalDuration;
}


void adjustSpeedScales()
{
  float maxTime = 0;
  bool isThisMotor;
  for (int i = 0; i < AXIS_MAX; i++)
  {
    isThisMotor = ((1 << i) & remainingSteppersFlag);
    if (isThisMotor == false)
      continue;
    if (stepper_tbl[i].estTimeForMove > maxTime )
      maxTime = stepper_tbl[i].estTimeForMove;
  }

  if ( maxTime != 0 )
  {
    for (int i = 0; i < AXIS_MAX; i++)
    {
      isThisMotor = ((1 << i) & remainingSteppersFlag);
      if (isThisMotor == false)
        continue;
      stepper_tbl[i].speedScale = maxTime / (float)stepper_tbl[i].estTimeForMove;
    }
  }
}


void stepperPrepareMove(axis_dir axis, int steps)
{
  volatile stepper_tbl_t *si  =  &stepper_tbl[axis];
  volatile int moveStep       = 0;
  moveStep = steps - stepper_tbl[axis].stepPosition;
  si->func_dir(axis, moveStep < 0 ? _DEF_HIGH : _DEF_LOW );
  si->dir  = moveStep > 0 ? 1 : -1;
  si->totalSteps = abs(moveStep);

  stepperReset(si);

  remainingSteppersFlag |= (1 << axis);  //On motor timer_cal

  uint64_t stepsAbs = abs(moveStep);

  if ( (2 * si->estStepsToSpeed) < stepsAbs )
  {
    // there will be a period of time at full speed
    uint64_t stepsAtFullSpeed = stepsAbs - 2 * si->estStepsToSpeed;
    float accelDecelTime = getDurationOfAcceleration(si, si->estStepsToSpeed);
    si->estTimeForMove = (2 * accelDecelTime) + (stepsAtFullSpeed * si->minStepInterval);
  }
  else
  {
    // will not reach full speed before needing to slow down again
    float accelDecelTime = getDurationOfAcceleration( si, stepsAbs / 2 );
    si->estTimeForMove = 2 * accelDecelTime;
  }


  //INTERRUPTS_ON
  /*
  si->p_tim_handle->pHandle->Instance->CCR1 = 10;
  si->p_tim_handle->pHandle->Instance->ARR  = 10;
  switch (axis)
  {
    case AXIS_1:  timEnableISR(_DEF_TIM1);  break;
    case AXIS_2:  timEnableISR(_DEF_TIM2);  break;
    case AXIS_3:  break;
    case AXIS_4:  break;
    default:      break;
  }
 */

}

void startISR(void)
{
  bool isThisMotor;
  for (int i = 0; i < AXIS_MAX; i++)
   {
     isThisMotor = ((1 << i) & remainingSteppersFlag);

     if (isThisMotor == true)
     {
       stepper_tbl[i].p_tim_handle->pHandle->Instance->CCR1 =10;
       stepper_tbl[i].p_tim_handle->pHandle->Instance->ARR =10;
       switch (i)
       {
         case AXIS_1:  timEnableISR(_DEF_TIM1);  break;
         case AXIS_2:  timEnableISR(_DEF_TIM2);  break;
         case AXIS_3:  break;
         case AXIS_4:  break;
         default:      break;
       }
     }
   }
}

void stepperRunAndWait(void)
{
  adjustSpeedScales();
  //setNextInterruptInterval();
  startISR();
  while (1)
  {
    if (remainingSteppersFlag == 0)
      break;
  }

}

void stepperISR_Axis_X(void)
{
  stepper_tbl_t * pM   = &stepper_tbl[AXIS_1];


  /*### step and checking arrived ###*/
  if ( pM->stepCount < pM->totalSteps )
  {
    pM->func_step(AXIS_1);
    pM->stepCount++;
    pM->stepPosition += pM->dir;
    pM->tim_cnt = 0;
  }
  else //if( s->stepCount >= s->totalSteps )
  {
    pM->moveDone = true;
    remainingSteppersFlag &= ~(1 << AXIS_1); //Off  motor timer_cal
    pM->is_start = false;

    timDisableISR(_DEF_TIM1);
    return;
  }

  /*### set motor interval delay time ###*/

  if ((pM->stepCount > (pM->totalSteps/2)) == true)// reached halfway point
  {
    // ramp down phase
    if (pM->n > 0)
    {
      if (pM->n < pM->rampUpStepCount)
      {
        pM->d = (pM->d*(4.0*(float)pM->n+1.0))/ (4.0*(float)pM->n+1.0-2.0);
      }
      pM->n--;
    }
  }
  else // ramp up phase
  {
    pM->n++;
    pM->d = pM->d -(2.0 * pM->d)/(4.0 * (float)pM->n + 1.0);

    if (pM->d <= pM->minStepInterval) //reached max speed
    {
      if (pM->rampUpStepCount == 0 )
      {
        pM->rampUpStepCount = pM->stepCount;
      }
      pM->d = pM->minStepInterval;
    }
  }

  pM->di = pM->d * pM->speedScale; // integer


  pM->p_tim_handle->pHandle->Instance->ARR = pM->di;
  timEnableISR(_DEF_TIM1);

}

void stepperISR_Axis_Y(void)
{
  stepper_tbl_t * pM   = &stepper_tbl[AXIS_2];

  /*### step and checking arrived ###*/
  if ( pM->stepCount < pM->totalSteps )
  {
    pM->func_step(AXIS_2);
    pM->stepCount++;
    pM->stepPosition += pM->dir;
    pM->tim_cnt = 0;
  }
  else //if( s->stepCount >= s->totalSteps )
  {
    pM->moveDone = true;
    remainingSteppersFlag &= ~(1 << AXIS_2); //Off  motor timer_cal
    pM->is_start = false;

    timDisableISR(_DEF_TIM2);
    return;
  }

  /*### set motor interval delay time ###*/

  if ((pM->stepCount > (pM->totalSteps/2)) == true)// reached halfway point
  {
    // ramp down phase
    if (pM->n > 0)
    {
      if (pM->n < pM->rampUpStepCount)
      {
        pM->d = (pM->d*(4.0*(float)pM->n+1.0))/ (4.0*(float)pM->n+1.0-2.0);
      }
      pM->n--;
    }
  }
  else // ramp up phase
  {
    pM->n++;
    pM->d = pM->d -(2.0 * pM->d)/(4.0 * (float)pM->n + 1.0);

    if (pM->d <= pM->minStepInterval) //reached max speed
    {
      if (pM->rampUpStepCount == 0 )
      {
        pM->rampUpStepCount = pM->stepCount;
      }
      pM->d = pM->minStepInterval;
    }
  }

  pM->di = pM->d * pM->speedScale; // integer


  pM->p_tim_handle->pHandle->Instance->ARR = pM->di;
  timEnableISR(_DEF_TIM2);
}


#endif


#endif

