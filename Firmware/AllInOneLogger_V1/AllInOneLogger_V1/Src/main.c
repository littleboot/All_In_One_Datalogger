/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "si7021.h"
#include "MHZ19.h"
#include "ESP8266.h"
#include "NextionDisplay.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define displayUpdateInterval 500 //0.5hz = 500ms
#define espUpdateInterval 60000 //every minute = 60000ms

bool CO2Ready = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void
SystemClock_Config(void);
void
Error_Handler(void);
static void
MX_GPIO_Init(void);
static void
MX_ADC1_Init(void);
static void
MX_I2C1_Init(void);
static void
MX_RTC_Init(void);
static void
MX_USART1_UART_Init(void);
static void
MX_USART2_UART_Init(void);
static void
MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int
map(int x, int in_min, int in_max, int out_min, int out_max);
int
getLight(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/**
 * TO DO:
 * -add cmd byte to ESP8266 message structure
 * -Move update interval settings to STM32 instead of ESP8266
 * -Divide floating point numbers to a integer-part and fractional-part
 * -Save and recall variables from RTC BKP Like thingsspeak channel key
 * -Add function that gets temp from previous humidity measurement to speed up the code
 * -Receive commands from display (like starting the day counter)
 *
 * TO DO Later:
 * -Add option to configure dark and light voltage levels LDR
 * -Add option to implement offset to CO2 level @Display
 * -add firmware version to settings->about screen @display
 * -increase clock speed (if necessary)
 *
 * Bugs:
 * -CubeMX generated code sets RTC time, Every time code is generated it needs a manual fix
 * -When I2C is busy and SDA is disconnected and reconnected. transmit and receive I2c functions return HAL_TIMEOUT and do nothing else.
 * -UART mismatch CO2 level wrong 3334 ppm or something doesn't change, check with logic analyser if data is transmitted correctly. (fault probably in MCU because reset fixes it)
 * -
 */
/* USER CODE END 0 */

int
main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init ();

  /* Configure the system clock */
  SystemClock_Config ();

  /* Initialize all configured peripherals */
  MX_GPIO_Init ();
  MX_ADC1_Init ();
  MX_I2C1_Init ();
  MX_RTC_Init ();
  MX_USART1_UART_Init ();
  MX_USART2_UART_Init ();
  MX_USART3_UART_Init ();

  /* USER CODE BEGIN 2 */
  if (HAL_RTCEx_BKUPRead (&hrtc, 0x01) != 0x01) //checks if RTC time has been set before, If this is the case, skip set time
    { //time has not been set
      ///Configure RTC manually
      RTC_DateTypeDef date; //stores date to be configured
      RTC_TimeTypeDef time; //stores time to be configured

      time.Hours = 17;
      time.Minutes = 34;
      time.Seconds = 0;
      date.WeekDay = RTC_WEEKDAY_SATURDAY;
      date.Date = 17;
      date.Month = RTC_MONTH_SEPTEMBER;
      date.Year = 16;

      HAL_RTC_SetTime (&hrtc, &time, RTC_FORMAT_BIN);
      HAL_RTC_SetDate (&hrtc, &date, RTC_FORMAT_BIN);

      HAL_RTCEx_BKUPWrite (&hrtc, 0x01, 0x01);
    }

  HAL_Delay (3000); //Delay to prevent CO2 sensor at startup from doing weird stuff. and make sure Nextion display is fully booted

  nextionSleepAfter (30); //nextion display goes to sleep mode after 30 seconds

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
      /* update sensors data */
      uint8_t lightLevel = getLight ();
      //float airTemp = getAirtemp ();
      //uint8_t humidity = getHumidity ();
      uint16_t co2 = getCO2 ();

      static uint32_t prevDisplayUpdate = 0;
      if (HAL_GetTick () - prevDisplayUpdate >= displayUpdateInterval)
        { //update sensor data and display every second
          prevDisplayUpdate = HAL_GetTick ();

          HAL_GPIO_TogglePin (LED_GPIO_Port, LED_Pin); //LED toggle

          nextionUpdateLightlevel (lightLevel);
          //nextionUpdateAirtemp (airTemp);
          //nextionUpdateHumidity (humidity);
          nextionUpdateCO2 (co2);
          nextionUpdateTime (); //Updates the time on the display

        }

      static uint32_t prevESPUpdate = 0;
      if (HAL_GetTick () - prevESPUpdate >= espUpdateInterval)
        { //update wifi sensor data and display every minute
          prevESPUpdate = HAL_GetTick ();

          //sendToESP8266 (lightLevel, airTemp, humidity, co2);
        }

      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */

}

/** System Clock Configuration
 */
void
SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig (&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler ();
    }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
      | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
      Error_Handler ();
    }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig (&PeriphClkInit) != HAL_OK)
    {
      Error_Handler ();
    }

  HAL_RCC_EnableCSS ();

  HAL_SYSTICK_Config (HAL_RCC_GetHCLKFreq () / 1000);

  HAL_SYSTICK_CLKSourceConfig (SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority (SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void
MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

  /**Common config 
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init (&hadc1) != HAL_OK)
    {
      Error_Handler ();
    }

  /**Configure Regular Channel 
   */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel (&hadc1, &sConfig) != HAL_OK)
    {
      Error_Handler ();
    }

}

/* I2C1 init function */
static void
MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
  if (HAL_I2C_Init (&hi2c1) != HAL_OK)
    {
      Error_Handler ();
    }

}

/* RTC init function */
static void
MX_RTC_Init(void)
{

  /**Initialize RTC and set the Time and Date 
   */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init (&hrtc) != HAL_OK)
    {
      Error_Handler ();
    }

}

/* USART1 init function */
static void
MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init (&huart1) != HAL_OK)
    {
      Error_Handler ();
    }

}

/* USART2 init function */
static void
MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init (&huart2) != HAL_OK)
    {
      Error_Handler ();
    }

}

/* USART3 init function */
static void
MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init (&huart3) != HAL_OK)
    {
      Error_Handler ();
    }

}

/** Configure pins as 
 * Analog 
 * Input 
 * Output
 * EVENT_OUT
 * EXTI
 */
static void
MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE ();
  __HAL_RCC_GPIOD_CLK_ENABLE ();
  __HAL_RCC_GPIOA_CLK_ENABLE ();
  __HAL_RCC_GPIOB_CLK_ENABLE ();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin (LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin (GPIOA, ESP8266_RST_Pin | ESP8266_NRST_Pin | WaterTemp_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ESP8266_RST_Pin */
  GPIO_InitStruct.Pin = ESP8266_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init (ESP8266_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ESP8266_NRST_Pin WaterTemp_Pin */
  GPIO_InitStruct.Pin = ESP8266_NRST_Pin | WaterTemp_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int
map(int x, int in_min, int in_max, int out_min, int out_max)
{
  if (x < in_min)
    return out_min;
  if (x > in_max)
    return out_max;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int
getLight()
{
  HAL_ADC_Start (&hadc1);
  HAL_ADC_PollForConversion (&hadc1, 1000);
  uint32_t ldr = HAL_ADC_GetValue (&hadc1);
  HAL_ADC_Stop (&hadc1);
  return map (ldr, 300, 3500, 0, 100);
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void
Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while (1)
    {
    }
  /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
  {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */

  }

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
