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


uint8_t TM_OneWire_Reset() {
	uint8_t i;

	/* Line low, and wait 480us */
	HAL_GPIO(GPIOA, GPIO_PIN_)

	ONEWIRE_LOW(OneWireStruct);
	ONEWIRE_OUTPUT(OneWireStruct);
	ONEWIRE_DELAY(480);

	/* Release line and wait for 70us */
	ONEWIRE_INPUT(OneWireStruct);
	ONEWIRE_DELAY(70);

	/* Check bit value */
	i = TM_GPIO_GetInputPinValue(OneWireStruct->GPIOx, OneWireStruct->GPIO_Pin);

	/* Delay for 410 us */
	ONEWIRE_DELAY(410);

	/* Return value of presence pulse, 0 = OK, 1 = ERROR */
	return i;
}
