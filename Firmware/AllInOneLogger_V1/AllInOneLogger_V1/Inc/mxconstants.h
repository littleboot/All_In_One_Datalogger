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

#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define LDR_Pin GPIO_PIN_1
#define LDR_GPIO_Port GPIOA
#define CO2_Sens_TX_Pin GPIO_PIN_2
#define CO2_Sens_TX_GPIO_Port GPIOA
#define CO2_sens_RX_Pin GPIO_PIN_3
#define CO2_sens_RX_GPIO_Port GPIOA
#define ESP8266_RST_Pin GPIO_PIN_4
#define ESP8266_RST_GPIO_Port GPIOA
#define Display_TX_Pin GPIO_PIN_10
#define Display_TX_GPIO_Port GPIOB
#define Display_RX_Pin GPIO_PIN_11
#define Display_RX_GPIO_Port GPIOB
#define ESP8266_NRST_Pin GPIO_PIN_8
#define ESP8266_NRST_GPIO_Port GPIOA
#define USB_Serial_TX_Pin GPIO_PIN_9
#define USB_Serial_TX_GPIO_Port GPIOA
#define USB_Serial_RX_Pin GPIO_PIN_10
#define USB_Serial_RX_GPIO_Port GPIOA
#define WaterTemp_Pin GPIO_PIN_12
#define WaterTemp_GPIO_Port GPIOA
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
