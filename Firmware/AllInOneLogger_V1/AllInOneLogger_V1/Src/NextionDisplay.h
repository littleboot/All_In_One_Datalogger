/*
 * NextionDisplay.h
 *
 *  Created on: 22 sep. 2016
 *      Author: Tom
 */

#ifndef NEXTIONDISPLAY_H_
#define NEXTIONDISPLAY_H_

#define UART3Timeout 100 //timeout in ticks
#define CO2SensorStartupTime 3 //(3*60*1000) //3min startuptime for CO2 sensor

extern UART_HandleTypeDef huart3;
extern RTC_HandleTypeDef hrtc;

void
nextionSleepAfter(uint8_t seconds);
void
nextionUpdateLightlevel(uint8_t light);
void
nextionUpdateAirtemp(float airtemp);
void
nextionUpdateHumidity(uint8_t humidity);
void
nextionUpdateCO2(uint16_t co2);
void
nextionUpdateTime();
void
nextionUpdateDayCounter();

#endif /* NEXTIONDISPLAY_H_ */
