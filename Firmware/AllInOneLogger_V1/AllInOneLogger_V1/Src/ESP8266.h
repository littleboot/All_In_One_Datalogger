/*
 * ESP8266.h
 *
 *  Created on: 17 sep. 2016
 *      Author: Tom
 */

#ifndef ESP8266_H_
#define ESP8266_H_

#define cmdSize 12
#define UART1Timeout 100

extern UART_HandleTypeDef huart1;

void
sendToESP8266(uint8_t lightLevel, float airtemp, uint8_t humidity, uint16_t co2);

#endif /* ESP8266_H_ */
