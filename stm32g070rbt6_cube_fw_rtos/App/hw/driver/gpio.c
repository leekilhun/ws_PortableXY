/*
 * gpio.c
 *
 *  Created on: 2023. 6. 11.
 *      Author: gns2l
 */



#include "gpio.h"
#include "cli.h"


#ifdef _USE_HW_GPIO

#ifdef _USE_HW_RTOS
#define PRIORITY_ORDER   3
#else
#define PRIORITY_ORDER   0
#endif


typedef struct
{
  GPIO_TypeDef *port;
  uint32_t      pin;
  GPIO_PinState on_state;
  GPIO_PinState off_state;
  bool          init_value;
  void (*cb_func)(void);
  //bool (*cb_func)(uint8_t,uint8_t*,uint8_t);
} gpio_tbl_t;


gpio_tbl_t gpio_tbl[GPIO_MAX_CH] =
{
    {SPI1_CS_GPIO_Port,     SPI1_CS_Pin,      GPIO_PIN_SET, GPIO_PIN_RESET,   _DEF_LOW, NULL},      // 0. spi ncs
    {SPI2_NSS_GPIO_Port,    SPI2_NSS_Pin,     GPIO_PIN_SET, GPIO_PIN_RESET,   _DEF_LOW, NULL},      // 1. spi nss

    {BTN_START_GPIO_Port,   BTN_START_Pin,    GPIO_PIN_RESET, GPIO_PIN_SET,   _DEF_LOW, NULL},      // 2. op switch start
    {BTN_STOP_GPIO_Port,    BTN_STOP_Pin,     GPIO_PIN_RESET, GPIO_PIN_SET,   _DEF_LOW, NULL},      // 3. op switch stop
    {BTN_RESET_GPIO_Port,   BTN_RESET_Pin,    GPIO_PIN_RESET, GPIO_PIN_SET,   _DEF_LOW, NULL},      // 4. op switch reset
    {BTN_ESTOP_GPIO_Port,   BTN_ESTOP_Pin,    GPIO_PIN_RESET, GPIO_PIN_SET,   _DEF_LOW, NULL},      // 5. op switch estop
    {LAMP_START_GPIO_Port,  LAMP_START_Pin,   GPIO_PIN_SET, GPIO_PIN_RESET,   _DEF_LOW, NULL},      // 6. op lamp start
    {LAMP_STOP_GPIO_Port,   LAMP_STOP_Pin,    GPIO_PIN_SET, GPIO_PIN_RESET,   _DEF_LOW, NULL},      // 7. op lamp stop
    {LAMP_RESET_GPIO_Port,  LAMP_RESET_Pin,   GPIO_PIN_SET, GPIO_PIN_RESET,   _DEF_LOW, NULL},      // 8. op lamp reset

    {IO_IN0_GPIO_Port, IO_IN0_Pin,            GPIO_PIN_RESET, GPIO_PIN_SET,   _DEF_LOW, NULL},      // 10.
    {IO_IN1_GPIO_Port, IO_IN1_Pin,            GPIO_PIN_RESET, GPIO_PIN_SET,   _DEF_LOW, NULL},      // .
    {IO_IN2_GPIO_Port, IO_IN2_Pin,            GPIO_PIN_RESET, GPIO_PIN_SET,   _DEF_LOW, NULL},      // .
    {IO_IN3_GPIO_Port, IO_IN3_Pin,            GPIO_PIN_RESET, GPIO_PIN_SET,   _DEF_LOW, NULL},      // .
    {IO_IN4_GPIO_Port, IO_IN4_Pin,            GPIO_PIN_RESET, GPIO_PIN_SET,   _DEF_LOW, NULL},      // .
    {IO_IN5_GPIO_Port, IO_IN5_Pin,            GPIO_PIN_RESET, GPIO_PIN_SET,   _DEF_LOW, NULL},      // .
    {IO_IN6_GPIO_Port, IO_IN6_Pin,            GPIO_PIN_RESET, GPIO_PIN_SET,   _DEF_LOW, NULL},      // .
    {IO_IN7_GPIO_Port, IO_IN7_Pin,            GPIO_PIN_RESET, GPIO_PIN_SET,   _DEF_LOW, NULL},      // 17.

    {IO_OUT0_GPIO_Port, IO_OUT0_Pin,          GPIO_PIN_SET, GPIO_PIN_RESET,   _DEF_LOW, NULL},      // 18.
    {IO_OUT1_GPIO_Port, IO_OUT1_Pin,          GPIO_PIN_SET, GPIO_PIN_RESET,   _DEF_LOW, NULL},      //
    {IO_OUT2_GPIO_Port, IO_OUT2_Pin,          GPIO_PIN_SET, GPIO_PIN_RESET,   _DEF_LOW, NULL},      //
    {IO_OUT3_GPIO_Port, IO_OUT3_Pin,          GPIO_PIN_SET, GPIO_PIN_RESET,   _DEF_LOW, NULL},      //
    {IO_OUT4_GPIO_Port, IO_OUT4_Pin,          GPIO_PIN_SET, GPIO_PIN_RESET,   _DEF_LOW, NULL},      //
    {IO_OUT5_GPIO_Port, IO_OUT5_Pin,          GPIO_PIN_SET, GPIO_PIN_RESET,   _DEF_LOW, NULL},      //
    {IO_OUT6_GPIO_Port, IO_OUT6_Pin,          GPIO_PIN_SET, GPIO_PIN_RESET,   _DEF_LOW, NULL},      //
    {IO_OUT7_GPIO_Port, IO_OUT7_Pin,          GPIO_PIN_SET, GPIO_PIN_RESET,   _DEF_LOW, NULL},      //25.

    //  i2c interrupt
    {I2C2_INT_GPIO_Port, I2C2_INT_Pin,  GPIO_PIN_RESET, GPIO_PIN_SET,   _DEF_LOW, NULL},      // .
    {SPI1_CS_ROM_GPIO_Port,     SPI1_CS_ROM_Pin,      GPIO_PIN_SET, GPIO_PIN_RESET,   _DEF_LOW, NULL},      //  rom cs
};



//static void delayUs(uint32_t us);

#ifdef _USE_HW_CLI
static void cliGpio(cli_args_t *args);
#endif



bool gpioInit(void)
{
  bool ret = true;

  for (int i=0; i<GPIO_MAX_CH; i++)
  {
    gpioPinWrite(i, gpio_tbl[i].init_value);
  }



#ifdef _USE_HW_CLI
  cliAdd("gpio", cliGpio);
#endif

  return ret;
}


void gpioPinWrite(uint8_t ch, bool value)
{
  if (ch >= GPIO_MAX_CH)
  {
    return;
  }

  if (value)
  {
    HAL_GPIO_WritePin(gpio_tbl[ch].port, gpio_tbl[ch].pin, gpio_tbl[ch].on_state);
  }
  else
  {
    HAL_GPIO_WritePin(gpio_tbl[ch].port, gpio_tbl[ch].pin, gpio_tbl[ch].off_state);
  }
}

bool gpioPinRead(uint8_t ch)
{
  bool ret = false;

  if (ch >= GPIO_MAX_CH)
  {
    return false;
  }

  if (HAL_GPIO_ReadPin(gpio_tbl[ch].port, gpio_tbl[ch].pin) == gpio_tbl[ch].on_state)
  {
    ret = true;
  }

  return ret;
}

void gpioPinToggle(uint8_t ch)
{
  if (ch < GPIO_MAX_CH)
  {
    HAL_GPIO_TogglePin(gpio_tbl[ch].port, gpio_tbl[ch].pin);
  }

}


bool gpioIsOn(uint8_t ch)
{
  return gpioPinRead(ch);
}


void gpioAttachCallbackFunc(uint8_t ch, void (*cb_func)(void)/*bool(*cb_func)(uint8_t,uint8_t*,uint8_t)*/)
{
  if (ch < GPIO_MAX_CH)
  {
    gpio_tbl[ch].cb_func = cb_func;
  }
}




#if 0

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  //UNUSED(GPIO_Pin);
  if(GPIO_Pin == GPIO_PIN_15)
  {
    if (gpio_tbl[_GPIO_I2C_INTERRUPT].cb_func)
    {
      gpio_tbl[_GPIO_I2C_INTERRUPT].cb_func();
    }
  }

}

/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
}


#endif

#ifdef _USE_HW_CLI
void cliGpio(cli_args_t *args)
{
  bool ret = false;


  if (args->argc == 1 && args->isStr(0, "show") == true)
  {
    while(cliKeepLoop())
    {
      for (int i=0; i<GPIO_MAX_CH; i++)
      {
        cliPrintf("%d", gpioPinRead(i));

      }
      cliPrintf("\n");
      cliPrintf("\x1B[%dA", 1);

      delay(100);
    }
    ret = true;
  }
  else if (args->argc == 2 && args->isStr(0, "read") == true)
  {
    uint8_t ch;

    ch = (uint8_t)args->getData(1);

    while(cliKeepLoop())
    {
      cliPrintf("gpio read %d : %d\n", ch, gpioPinRead(ch));
      delay(100);
    }

    ret = true;
  }
  else if (args->argc == 3 && args->isStr(0, "write") == true)
  {
    uint8_t ch;
    uint8_t data;

    ch   = (uint8_t)args->getData(1);
    data = (uint8_t)args->getData(2);

    gpioPinWrite(ch, data);

    cliPrintf("gpio write %d : %d\n", ch, data);
    ret = true;
  }
  else if (args->argc == 4 && args->isStr(0, "pulse") == true)
  {
    uint8_t port;
    uint32_t cnt;
    uint32_t ms;

    port   = (uint8_t)args->getData(1);
    cnt = (uint32_t)args->getData(2);
    ms = (uint32_t)args->getData(3);

    for ( int i = 0 ; i<cnt; i++)
    {
      gpioPinToggle(port);
      delay(ms);
    }


    cliPrintf("gpio pulse out[%d] cnt[%d] \n", port, cnt);
    ret = true;
  }

  if (ret != true)
  {
    cliPrintf("gpio show\n");
    cliPrintf("gpio read ch[0~%d]\n", GPIO_MAX_CH-1);
    cliPrintf("gpio write ch[0~%d] 0:1\n", GPIO_MAX_CH-1);
    cliPrintf("gpio pulse out[0~7] cnt[n] [ms]\n");
  }
}


#endif /*_USE_HW_GPIO*/


#endif
