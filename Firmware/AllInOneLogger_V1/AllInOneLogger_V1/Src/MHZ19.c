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

uint16_t
getCO2(void)
{
  uint8_t cmd[9] =
    { 0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79 }; //Starting byte fixed; sensor no.; Get gas concentration cmd; ; ; ; ; ; ;check value;
  uint8_t response[9]; // stores received data

  HAL_UART_Transmit (&huart2, cmd, 9, UART2Timeout); //Send commands to MHZ-19 Co2 sensor
  HAL_UART_Receive (&huart2, response, 9, UART2Timeout); //save response to response buffer

  uint16_t responseHigh = (uint16_t) response[2];
  uint16_t responseLow = (uint16_t) response[3];
  uint16_t ppm = ((256 * responseHigh) + responseLow); //convert to ppm

  return ppm;
}
