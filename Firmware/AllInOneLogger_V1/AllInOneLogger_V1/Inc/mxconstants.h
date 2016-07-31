/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MXCONSTANT_H
#define __MXCONSTANT_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define OUTPUT__LED_Pin GPIO_PIN_13
#define OUTPUT__LED_GPIO_Port GPIOC
#define LDR_light_sensor_Pin GPIO_PIN_1
#define LDR_light_sensor_GPIO_Port GPIOA
#define UART2_TX__CO2_Sens_Pin GPIO_PIN_2
#define UART2_TX__CO2_Sens_GPIO_Port GPIOA
#define UART2_RX__CO2_Sens_Pin GPIO_PIN_3
#define UART2_RX__CO2_Sens_GPIO_Port GPIOA
#define CS_ESP8266_Pin GPIO_PIN_4
#define CS_ESP8266_GPIO_Port GPIOA
#define UART3_TX__Nextion_Pin GPIO_PIN_10
#define UART3_TX__Nextion_GPIO_Port GPIOB
#define UART3_RX__Nextion_Pin GPIO_PIN_11
#define UART3_RX__Nextion_GPIO_Port GPIOB
#define RST_ESP8266_Pin GPIO_PIN_8
#define RST_ESP8266_GPIO_Port GPIOA
#define UART1_TX__USB_Serial_Pin GPIO_PIN_9
#define UART1_TX__USB_Serial_GPIO_Port GPIOA
#define UART1_RX__USB_Serial_Pin GPIO_PIN_10
#define UART1_RX__USB_Serial_GPIO_Port GPIOA
#define OneWire__WaterTemp_Pin GPIO_PIN_12
#define OneWire__WaterTemp_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MXCONSTANT_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
