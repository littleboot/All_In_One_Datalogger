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

uint8_t
getCheckSum(uint8_t *packet)
{
  uint8_t i, checksum;
  for (i = 1; i < 8; i++)
    {
      checksum += packet[i];
    }
  checksum = (0xff - checksum);
  checksum += 1;
  return checksum;
}

uint16_t
getCO2(void)
{
  uint8_t cmd[9] =
    { 0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79 }; //Starting byte fixed; sensor no.; Get gas concentration cmd; ; ; ; ; ; ;check value;
  uint8_t response[9]; // stores received data

  HAL_UART_Transmit (&huart2, cmd, 9, UART2Timeout); //Send commands to MHZ-19 Co2 sensor
  HAL_UART_Receive (&huart2, response, 9, UART2Timeout); //save response to response buffer

//  uint8_t crc1 = getCheckSum (&response);
//  uint32_t crc = response[8];
//  if (crc != crc1)
//    return 0;

  uint32_t responseHigh = (uint32_t) response[2];
  uint32_t responseLow = (uint32_t) response[3];
  uint32_t ppm = ((256 * responseHigh) + responseLow); //convert to ppm

  return (uint16_t) ppm;
}
