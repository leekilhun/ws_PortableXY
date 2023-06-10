/*
 * reset.c
 *
 *  Created on: 2022. 9. 13.
 *      Author: gns2l
 *
 *  STM32G070RBT6 M0+ 대한 api 구성
 *  system 초기화 과정에서 리셋 정보를 기록하는 RCC_CSR의 핀 리셋정보(Bit 26 PINRSTF: Pin reset flag)를 읽어서
 *  rtc backup register인  - TAMP backup x register (x = 0 to 4) 기록한다.
 *
 *  - 디버거 다운로드 후 리셋일 경우 리셋 카운터가 2가 되는
 *    경우를 제거하기 위해 Core Debug플래그 상태를 확인하여 예외처리를 하려 했으나
 *    M0+에서 위 플래그 확인을 못함
 *  - System boot(resetToSysBoot) 이동 후 DFU 모드에서 플래쉬 다운로드 경우
 *    STM32G070RBT6 M0+ UART3으로 지원하지만 검증, 구현은 못함.
 */



#include "reset.h"
#include "rtc.h"


#ifdef _USE_HW_RESET

static uint32_t reset_count = 0;
static uint32_t run_timeout_count = 0;


/*
 * Enter boot mode.
 */
static void resetToRunBoot(void);


void resetISR(void)
{
  if (run_timeout_count > 0)
  {
    run_timeout_count--;

    if (run_timeout_count == 0)
    {
      resetToRunBoot();
    }
  }
}


bool resetInit(void)
{
  bool ret = true;


#if 1
  // 만약 디버거가 연결된 경우
  //

 /* bool is_debug = false;
  if (DBG->CR & (1<<2))
  {
    is_debug = true;
  }*/

  // 만약 Reset 핀이 눌렸다면
  // if (((RCC->CSR & (1<<26)) == (1<<26))&& is_debug!= true)
  if (RCC->CSR & (1<<26) /*&& (is_debug!= true)*/)
  {
    rtcBackupRegWrite(RESET_REG_COUNT, rtcBackupRegRead(RESET_REG_COUNT) + 1);
    delay(500);
    reset_count = rtcBackupRegRead(RESET_REG_COUNT);
  }

  rtcBackupRegWrite(RESET_REG_COUNT, 0);
#endif

  return ret;
}

uint32_t resetGetCount(void)
{
  return reset_count;
}

void resetToBoot(uint32_t timeout)
{
  if (timeout == 0)
    resetToRunBoot();
  else
    run_timeout_count = timeout;
}

void resetToSysBoot(void)
{
  void (*SysMemBootJump)(void);
  volatile uint32_t addr = 0x1FFF0000; // STM32G07x System memory


  HAL_RCC_DeInit();

  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL  = 0;

  NVIC->ICER[0] = 0xFFFFFFFF;
  __DSB();
  __ISB();

  SysMemBootJump = (void (*)(void)) (*((uint32_t *)(addr + 4)));

  __set_MSP(*(uint32_t *)addr);
  SysMemBootJump();
}

void resetToRunBoot(void)
{
  uint32_t reg;

  reg = rtcBackupRegRead(RESET_REG_PARAM);

  reg |= (1<<0);
  rtcBackupRegWrite(RESET_REG_PARAM, reg);
  NVIC_SystemReset();
}

#endif
