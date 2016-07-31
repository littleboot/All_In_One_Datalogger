/*
 * si7021.c
 *
 *  Created on: 31 jul. 2016
 *      Author: Tom
 */

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "si7021.h"


float get_airtemp(void) {
	//Variables needed
	uint8_t cmd = 0xE3;
	uint8_t I2CDataReceived[2] = { 0, 0 };

	HAL_I2C_Master_Transmit(&hi2c1, (0x40 << 1), &cmd, 1, 0xFF); //this function takes a byte as input adress. because the adress in the datasheet is 0x40 (7-Bit) it needs to be shifted left by one
	HAL_I2C_Master_Receive(&hi2c1, (0x40 << 1), I2CDataReceived, 2, 0xFF);
	uint16_t tempCode = (uint16_t) I2CDataReceived[0] << 8
			| (uint16_t) I2CDataReceived[1]; //convert the received MSB en LSB to one 16-bit word.
	return ((175.72 * tempCode) / 65536) - 46.85;
}

int get_humidity(void) {
	//Variables needed
	uint8_t cmd = 0xE5;
	uint8_t I2CDataReceived[2] = { 0, 0 };

	HAL_I2C_Master_Transmit(&hi2c1, (0x40 << 1), &cmd, 1, 0xFF); //this function takes a byte as input adress. because the adress in the datasheet is 0x40 (7-Bit) it needs to be shifted left by one
	HAL_I2C_Master_Receive(&hi2c1, (0x40 << 1), I2CDataReceived, 2, 0xFF); //Note timeout is higher then the get_temp function
	uint16_t RHCode = (uint16_t) I2CDataReceived[0] << 8
			| (uint16_t) I2CDataReceived[1]; //convert the received MSB en LSB to one 16-bit word.

	//make sure value is between 0 and 100%, recommended in datasheet sensor
	int humidity = ((125 * RHCode) / 65536) - 6;
	if (humidity > 100)
		humidity = 100;
	if (humidity < 0)
		humidity = 0;
	return humidity;
}
