/*
 * ESP8266.c
 *
 *  Created on: 17 sep. 2016
 *      Author: Tom
 */

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <stdint.h>
#include "ESP8266.h"

void
sendCommand (void)
{
  uint8_t cmd[11] =
    { 0xFF, 0xFF, 0xFF, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9 }; //Starting byte fixed; sensor no.; Get gas concentration cmd; ; ; ; ; ; ;check value;

  HAL_UART_Transmit (&huart1, cmd, 11, 100);
//  uint8_t data[4] =
//    { 0xff, 0x01, 0xff, 0x01 };
//
//  HAL_GPIO_WritePin (ESP8266_CS_GPIO_Port, ESP8266_CS_Pin, GPIO_PIN_RESET);
//
//  HAL_SPI_Transmit (&hspi1, data, 4, 100);
//
//  HAL_GPIO_WritePin (ESP8266_CS_GPIO_Port, ESP8266_CS_Pin, GPIO_PIN_SET);
}
