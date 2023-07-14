/*
 * buzzer.c
 *
 *  Created on: Dec 28, 2021
 *      Author: gns2.lee
 */


#include "buzzer.h"
#include "cli.h"

#ifdef _USE_HW_BUZZER


#define DEF_BUZZER_PORT     GPIOC
#define DEF_BUZZER_PIN      GPIO_PIN_13
static bool is_buzzerOn;

#ifdef _USE_HW_CLI
static void cliBuzzer(cli_args_t *args);
#endif


bool buzzerInit(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure = {0,};
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStructure.Pin   = DEF_BUZZER_PIN;
  GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(DEF_BUZZER_PORT, &GPIO_InitStructure);

  is_buzzerOn = false;

  buzzerOff();

#ifdef _USE_HW_CLI
  cliAdd("buzzer", cliBuzzer);
#endif

  return true;
}

void buzzerOn(void)
{
  HAL_GPIO_WritePin(DEF_BUZZER_PORT, DEF_BUZZER_PIN, GPIO_PIN_SET);
  is_buzzerOn = true;
}


void buzzerOff(void)
{
  HAL_GPIO_WritePin(DEF_BUZZER_PORT, DEF_BUZZER_PIN, GPIO_PIN_RESET);
  is_buzzerOn = false;
}

void buzzerToggle(void)
{
  HAL_GPIO_TogglePin(DEF_BUZZER_PORT, DEF_BUZZER_PIN);
}

void buzzerBeep(uint8_t times, uint32_t delay_ms)
{
  if (is_buzzerOn)
    buzzerOff();

  while(times>0)
  {
    buzzerToggle();
    delay(delay_ms);
    buzzerToggle();
    delay(delay_ms);
    times--;
  }
  buzzerOff();
}

#ifdef _USE_HW_CLI
static void cliBuzzer(cli_args_t *args)
{
  bool ret = false;

  if (args->argc == 1 && args->isStr(0, "on") == true)
  {
  	buzzerOn();
  	cliPrintf("buzzer On");
    ret = true;
  }

  if (args->argc == 1 && args->isStr(0, "off") == true)
  {
  	buzzerOff();
  	cliPrintf("buzzer Off");
    ret = true;
  }

  if (args->argc == 2 && args->isStr(0, "beep") == true)
  {
    uint8_t type;
    const uint8_t max_type = 4;
    uint32_t type_ms[4]=  {1000, 500, 100, 10};
    type   = (uint8_t)args->getData(1);

    cliPrintf("buzzer beep type:%d[%d] \n", type, type_ms[type % max_type]);
    while(cliKeepLoop())
    {
    	buzzerBeep(1,type_ms[type % max_type]);
    	uint32_t set_ms = type_ms[type % max_type];
    	while(cliKeepLoop())
    	{
    		if (set_ms> 0)
    		{
    			set_ms -= 10;
     			delay(10);
    		}
    		else
    		{
    			break;
    		}
    	}
    }
    ret = true;
  }

  if (ret != true)
  {
    cliPrintf("buzzer on\n");
    cliPrintf("buzzer off\n");
    cliPrintf("buzzer beep type[0:1s, 1:0.5s, 2:0.1s, 3:0.01s] \n");
  }

}
#endif

#endif /*_USE_HW_BUZZ*/
