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

static uint8_t
getCheckSum(uint8_t * packet)
{
  /* The checksum = (invert (byte 3 +... + 11)) + 1 */
  uint8_t i;
  uint8_t checksum = 0;

  for (i = 3; i < 12; i++)
    {
      checksum += packet[i];
    }
  checksum = (0xff - checksum);
  checksum += 1;
  return checksum;
}

/* messageBuffer[bufferSize]

 Thingsspeak data update cmd 0x00
 +---------------------------------------------------------------------------------+
 |Start | cmd  | LightLevel | AirTemp | Humidity | CO2 | WaterTemp | PH | EC | CRC |
 +---------------------------------------------------------------------------------+
 | 0-2  |  3   |     4      |    5    |    6     | 7-8 |    9      | 10 | 11 | 12  |
 +---------------------------------------------------------------------------------+


 */
void
sendToESP8266(uint8_t lightLevel, float airtemp, uint8_t humidity, uint16_t co2)
{
  uint8_t temp = (uint8_t) airtemp;
  uint8_t msbCo2 = (uint8_t) (co2 >> 8);
  uint8_t lsbCo2 = (uint8_t) (co2 & 0x00FF);

  uint8_t message[13] =
    { 0xFF, 0xFF, 0xFF, 0x00, lightLevel, temp, humidity, msbCo2, lsbCo2, 0, 0, 0, 0 }; //Starting byte fixed; sensor no.; Get gas concentration cmd; ; ; ; ; ; ;check value;
  message[12] = getCheckSum (message);

  HAL_UART_Transmit (&huart1, message, 13, UART1Timeout);

  //TO:DO wait for ACK
}
