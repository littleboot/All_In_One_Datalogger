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

/* messageBuffer[bufferSize]

 +------+------------+---------+----------+-----+-----------+----+----+-----+
 |Start | LightLevel | AirTemp | Humidity | CO2 | WaterTemp | PH | EC | CRC |
 +------+------------+---------+----------+-----+-----------+----+----+-----+
 Byte: | 0-2  |     3      |    4    |    5     | 6-7 |    8      |  9 | 10 | 11  |

 start == 0xFF 0xFF 0xFF
 */

void
sendToESP8266(uint8_t lightLevel, float airtemp, uint8_t humidity, uint16_t co2)
{
  airtemp = (uint8_t) airtemp;
  uint8_t msbCo2 = (uint8_t) (co2 >> 8);
  uint8_t lsbCo2 = (uint8_t) (co2 | 0x00FF);

  uint8_t cmd[cmdSize] =
    { 0xFF, 0xFF, 0xFF, lightLevel, airtemp, humidity, msbCo2, lsbCo2, 0, 0, 0 }; //Starting byte fixed; sensor no.; Get gas concentration cmd; ; ; ; ; ; ;check value;

  HAL_UART_Transmit (&huart1, cmd, cmdSize, UART1Timeout);

  //TO:DO wait for ACK
}
