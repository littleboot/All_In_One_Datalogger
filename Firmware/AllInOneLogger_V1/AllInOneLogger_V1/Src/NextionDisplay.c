/*
 * NextionDisplay.c
 *
 *  Created on: 22 sep. 2016
 *      Author: Tom
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "NextionDisplay.h"

void
nextionSleepAfter(uint8_t seconds)
{
  char buffer[40]; //stores string to be send

  sprintf (buffer, "thsp=%dÿÿÿthup=1ÿÿÿ", seconds); //thsp=30(No touch operation within 30 seconds, it will auto enter into sleep mode).
  int len = strlen (buffer);
  HAL_UART_Transmit (&huart3, (uint8_t *) buffer, len, 1000); //Send commands to nextion display
}

void
nextionUpdateLightlevel(uint8_t light)
{
  char buffer[40]; //stores string to be send
  sprintf (buffer, "t2.txt=\"%d %%\"ÿÿÿ", light);
  uint16_t len = strlen (buffer);
  HAL_UART_Transmit (&huart3, (uint8_t *) buffer, len, UART3Timeout); //Send commands to nextion display
}

void
nextionUpdateAirtemp(float airtemp)
{
  char buffer[40]; //stores string to be send
  sprintf (buffer, "t3.txt=\"%.1f C\"ÿÿÿ", airtemp);
  uint16_t len = strlen (buffer);
  HAL_UART_Transmit (&huart3, (uint8_t *) buffer, len, UART3Timeout); //Send commands to nextion display
}

void
nextionUpdateHumidity(uint8_t humidity)
{
  char buffer[40]; //stores string to be send
  sprintf (buffer, "t4.txt=\"%d %%\"ÿÿÿ", humidity);
  uint16_t len = strlen (buffer);
  HAL_UART_Transmit (&huart3, (uint8_t *) buffer, len, UART3Timeout); //Send commands to nextion display
}

void
nextionUpdateCO2(uint16_t co2)
{
  char buffer[50]; //stores string to be send

  if (HAL_GetTick () < CO2SensorStartupTime) //check if startuptime has passed
    sprintf (buffer, "t5.txt=\"warmup\"ÿÿÿ");
  else
    sprintf (buffer, "t5.txt=\"%d ppm\"ÿÿÿ", co2);

  uint16_t len = strlen (buffer);
  HAL_UART_Transmit (&huart3, (uint8_t *) buffer, len, UART3Timeout); //Send commands to nextion display
}

void
nextionUpdateTime()
{
  RTC_TimeTypeDef currentTime; //store time returned from the RTC
  HAL_RTC_GetTime (&hrtc, &currentTime, RTC_FORMAT_BIN); //get time from RTC and store in above variable

  char buffer[40]; //stores string to be send

  if (currentTime.Seconds % 2 == 0) //display : when seconds are even
    sprintf (buffer, "t0.txt=\"%02d:%02d:%02d\"ÿÿÿ", currentTime.Hours, currentTime.Minutes,
             currentTime.Seconds);
  else
    sprintf (buffer, "t0.txt=\"%02d %02d:%02d\"ÿÿÿ", currentTime.Hours, currentTime.Minutes,
             currentTime.Seconds);

  uint16_t len = strlen (buffer);
  HAL_UART_Transmit (&huart3, (uint8_t *) buffer, len, UART3Timeout); //Send commands to nextion display
}

void
nextionUpdateDayCounter()
{
  static int dayCounter = 1;

  char buffer[40]; //stores string to be send

  sprintf (buffer, "t1.txt=\"Day: %d\"ÿÿÿ", dayCounter);

  int len = strlen (buffer);
  HAL_UART_Transmit (&huart3, (uint8_t *) buffer, len, UART3Timeout); //Send commands to nextion display
}

