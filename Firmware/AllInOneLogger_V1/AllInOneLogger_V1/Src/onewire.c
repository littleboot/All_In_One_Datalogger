/*
 * onewire.c
 *
 *  Created on: 31 jul. 2016
 *      Author: Tom
 */
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <stdint.h>
#include "onewire.h"

/*
 * For one wire to function a micro second delay function is needed
 * NOP could be used to delay
 *
 * else use a timer:
 * 72Mhz results in 0.01388 us
 * else timer + ISR should be used. don't leave timer running
 *
 * Other way is using the systick current value register
 * http://micromouseusa.com/?p=296
 */

uint8_t
TM_OneWire_Reset()
{
  uint8_t i = 0;

//	/* Line low, and wait 480us */
//	HAL_GPIO(GPIOA, GPIO_PIN_);
//
//	ONEWIRE_LOW(OneWireStruct);
//	ONEWIRE_OUTPUT(OneWireStruct);
//	ONEWIRE_DELAY(480);
//
//	/* Release line and wait for 70us */
//	ONEWIRE_INPUT(OneWireStruct);
//	ONEWIRE_DELAY(70);
//
//	/* Check bit value */
//	i = TM_GPIO_GetInputPinValue(OneWireStruct->GPIOx, OneWireStruct->GPIO_Pin);
//
//	/* Delay for 410 us */
//	ONEWIRE_DELAY(410);
//
//	/* Return value of presence pulse, 0 = OK, 1 = ERROR */
  return i;
}
