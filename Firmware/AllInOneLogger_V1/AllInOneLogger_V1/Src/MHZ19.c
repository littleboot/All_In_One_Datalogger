/*
 * MHZ19.c
 *
 *  Created on: 31 jul. 2016
 *      Author: Tom
 */
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <stdint.h>
#include "MHZ19.h"

int get_CO2(void) {
	uint8_t cmd[9] = { 0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79 }; //Starting byte fixed; sensor no.; Get gas concentration cmd; ; ; ; ; ; ;check value;
	uint8_t response[9]; // stores received data

	HAL_UART_Transmit(&huart2, cmd, 9, 2000); //Send commands to MHZ-19 Co2 sensor
	HAL_UART_Receive(&huart2, response, 9, 2000); //save response to response buffer

	int responseHigh = (int) response[2];
	int responseLow = (int) response[3];
	int ppm = ((256 * responseHigh) + responseLow); //convert to ppm
	//printf("ppm: %d\n", ppm); //Debug

	return ppm;
}
