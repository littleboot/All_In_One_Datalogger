/*
 * si7021.h
 *
 *  Created on: 31 jul. 2016
 *      Author: Tom
 */

#ifndef SI7021_H_
#define SI7021_H_

#define si7021_adress (0x40 << 1)//7-bit address
extern I2C_HandleTypeDef hi2c1;

float
getAirtemp (void);
int
getHumidity (void);

#endif /* SI7021_H_ */
