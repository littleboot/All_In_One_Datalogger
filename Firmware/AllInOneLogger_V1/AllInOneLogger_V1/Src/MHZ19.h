/*
 * MHZ19.h
 *
 *  Created on: 31 jul. 2016
 *      Author: Tom
 */

#ifndef MHZ19_H_
#define MHZ19_H_

#define UART2Timeout 100

extern UART_HandleTypeDef huart2;

uint8_t
getCheckSum(uint8_t *packet);
uint16_t
getCO2(void);

#endif /* MHZ19_H_ */
