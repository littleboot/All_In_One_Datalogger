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


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;



/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define intervalUpdateDisplaySensorData 1000 //time in ms

RTC_DateTypeDef date; //stores date to be configured
RTC_TimeTypeDef time; //stores time to be configured

bool CO2SensorReady = false; //false if still warming up (3min), if ready changed to true
uint32_t prevSensorUpdateDisplay = 0;


struct timeColors{
	uint8_t hoursColor;
	uint8_t minutesColor;
	uint8_t dateColor;
	uint8_t monthColor;
	uint8_t yearColor;
};

struct timeColors tc;

typedef enum {HOURS, MINUTES, DATE, MONTH, YEAR} time_select_type;
time_select_type timeSelected = HOURS;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
 
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
	
	
	return ch;
}

int map(int x, int in_min, int in_max, int out_min, int out_max);

float get_temperature(void);
int get_humidity(void);
float get_temp_from_RH(void); //Not implemented yet
int get_CO2(void);

int get_lightlevel(void);

void configure_display_sleepmode(void);
void update_display_time(void);
void update_display_date(void);
void update_display_sensordata(void);

void set_time(uint8_t hours, uint8_t minutes, uint8_t seconds);
void sete_date(uint8_t day, uint8_t month, uint8_t year);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/**
	* TO DO:
	* -Save and recall variables from RTC BKP Like thingsspeak channel key
	* -Add fucntion that gets temp from previous humidity measurement to speed up the code
	* -After power on the CO2 sensor shows wrong values. show dots the first 3 minutes at CO2 level istead of wrong values, 
	* 
	* TO DO Later:
	* -Add option to configure dark and light voltage levels LDR
	* -Add option to implement offset to CO2 level @Display
	* -add firmware version to settings->about screen @display
	* -increase clock speed (if necessary)
	*
	* Bugs:
	* -Date not saved after power off like the time
	* -When I2C is busy and SDA is disconnected and reconnected. transmit and receive I2c functions return HAL_TIMEOUT and do nothing else. Need debugger to fix this
	*/
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
	///Configure RTC manually 
	time.Hours = 0;
	time.Minutes = 0;
	time.Seconds = 0;
	date.WeekDay = RTC_WEEKDAY_THURSDAY;
	date.Date = 10;
	date.Month = RTC_MONTH_JULY;
	date.Year = 16;
	
//	HAL_RTC_SetTime(&hrtc,&time,RTC_FORMAT_BIN);
	HAL_RTC_SetDate(&hrtc,&date,RTC_FORMAT_BIN);
	
	HAL_Delay(3000); //Delay to prevent CO2 sensor at startup from doing wierd stuff. and make sure Nextion display is fully booted
	
	configure_display_sleepmode(); //screensaver for display
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		uint32_t millis  = HAL_GetTick(); //counter amount of miliseconds the program is running (createde in systick isr)
		
		///Blink LED, Used to check if MCU isn't stuck in a long loop
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); //LED toggle
		
		if(prevSensorUpdateDisplay + intervalUpdateDisplaySensorData < millis) { //update sensor data and display every second
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); //LED toggle
			update_display_sensordata(); //Updates display sensor values, at this moment only temp RH and CO2
			update_display_date(); //Updates date on display
		}
			
		update_display_time(); //Updates the time on the display

		uint8_t response[2]; // stores received cmd
		HAL_UART_Receive(&huart3, response, 2, 2000); //save response to response buffer
		
		switch(response[1]) {

			 case 0x01 :
					//
					break; /* optional */
			
			 case 0x02  :
					//
					break; 
			 case 0x03  : /* time setting */
					//
					//time.Hours
					
					break; /* optional */
		}
			
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  } 
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
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
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

//  RTC_TimeTypeDef sTime;
//  RTC_DateTypeDef DateToUpdate;

    /**Initialize RTC and set the Time and Date 
    */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

//  sTime.Hours = 0x1;
//  sTime.Minutes = 0x0;
//  sTime.Seconds = 0x0;

//  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
//  DateToUpdate.Month = RTC_MONTH_JANUARY;
//  DateToUpdate.Date = 0x1;
//  DateToUpdate.Year = 0x0;

//  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    Error_Handler();
//  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DEV_BOARD_LED_GPIO_Port, DEV_BOARD_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS_ESP8266_Pin|RST_ESP8266_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DEV_BOARD_LED_Pin */
  GPIO_InitStruct.Pin = DEV_BOARD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DEV_BOARD_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_ESP8266_Pin RST_ESP8266_Pin */
  GPIO_InitStruct.Pin = CS_ESP8266_Pin|RST_ESP8266_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int map(int x, int in_min, int in_max, int out_min, int out_max)
{
	if(x < in_min) return out_min;
	if(x > in_max) return out_max;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float get_temperature(void)
{
	//Variables needed
	uint8_t cmd = 0xE3;
	uint8_t I2CDataReceived[2] = {0, 0};

	HAL_I2C_Master_Transmit(&hi2c1, (0x40<<1),&cmd, 1, 0xFF); //this function takes a byte as input adress. because the adress in the datasheet is 0x40 (7-Bit) it needs to be shifted left by one
	HAL_I2C_Master_Receive(&hi2c1, (0x40<<1), I2CDataReceived, 2, 0xFF);
	uint16_t tempCode = (uint16_t)I2CDataReceived[0]<<8 | (uint16_t)I2CDataReceived[1]; //convert the received MSB en LSB to one 16-bit word.
	return ((175.72*tempCode)/65536)-46.85;
}

int get_humidity(void)
{
	//Variables needed
	uint8_t cmd = 0xE5;
	uint8_t I2CDataReceived[2] = {0, 0};

	HAL_I2C_Master_Transmit(&hi2c1, (0x40<<1),&cmd, 1, 0xFF); //this function takes a byte as input adress. because the adress in the datasheet is 0x40 (7-Bit) it needs to be shifted left by one
	HAL_I2C_Master_Receive(&hi2c1, (0x40<<1), I2CDataReceived, 2, 0xFF); //Note timeout is higher then the get_temp function
	uint16_t RHCode = (uint16_t)I2CDataReceived[0]<<8 | (uint16_t)I2CDataReceived[1]; //convert the received MSB en LSB to one 16-bit word.
	
	//make sure value is between 0 and 100%, recommended in datasheet sensor
	int humidity = ((125*RHCode)/65536)-6;
	if(humidity > 100) humidity = 100;
	if(humidity < 0 ) humidity = 0;
	return humidity;
}

int get_CO2(void)
{
	uint8_t cmd[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79}; //Starting byte fixed; sensor no.; Get gas concentration cmd; ; ; ; ; ; ;check value;
	uint8_t response[9]; // stores received data
			
	HAL_UART_Transmit(&huart2, cmd, 9, 2000); //Send commands to MHZ-19 Co2 sensor
	HAL_UART_Receive(&huart2, response, 9, 2000); //save response to response buffer
	
	int responseHigh = (int)response[2];
	int responseLow = (int)response[3];
	int ppm = ((256*responseHigh)+responseLow); //convert to ppm
	//printf("ppm: %d\n", ppm); //Debug
	
	return ppm;
}

int get_lightlevel()
{
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	uint32_t ldr = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return map(ldr, 300, 3500, 0, 100);
}

void update_display_sensordata(void)
{
	///Get sensor data from sensors
	float temp = get_temperature();
	int RH = get_humidity();
	int CO2 = get_CO2();
	int ldr = get_lightlevel();
	
	char buffer[120]; //stores string to be send
	sprintf(buffer,"t2.txt=\"%d %%\"ÿÿÿt3.txt=\"%.1f C\"ÿÿÿt4.txt=\"%d %%\"ÿÿÿt5.txt=\"%d ppm\"ÿÿÿ",ldr, temp, RH, CO2); //Example: t2.txt="Tom"ÿÿÿ
	int len = strlen(buffer);
	
	HAL_UART_Transmit(&huart3, (uint8_t *)buffer, len, 1000); //Send commands to nextion display
}

void update_display_time()
{
	RTC_TimeTypeDef currentTime; //store time returned from the RTC
	HAL_RTC_GetTime(&hrtc, &currentTime,RTC_FORMAT_BIN); //get time from RTC and store in above variable
	
	char buffer[40]; //stores string to be send
	
	if(currentTime.Seconds%2 == 0) //display : when seconds are even
	{
		sprintf(buffer,"t0.txt=\"%02d:%02d\"ÿÿÿ",currentTime.Hours, currentTime.Minutes); // %02d is format printf so the minutes always consists of two numbers starting with zero's
	}else {
		sprintf(buffer,"t0.txt=\"%02d %02d\"ÿÿÿ",currentTime.Hours, currentTime.Minutes); 
	}
	
	int len = strlen(buffer);
	HAL_UART_Transmit(&huart3, (uint8_t *)buffer, len, 1000); //Send commands to nextion display	
}

void update_display_date()
{
	RTC_DateTypeDef currentDate; //store date returned from the RTC
	HAL_RTC_GetDate(&hrtc, &currentDate,RTC_FORMAT_BIN); //get time from RTC and store in above variable
	
	char buffer[40]; //stores string to be send
	
	sprintf(buffer,"t1.txt=\"%02d-%02d-20%02d\"ÿÿÿ",currentDate.Date, currentDate.Month,currentDate.Year);
	
	int len = strlen(buffer);
	HAL_UART_Transmit(&huart3, (uint8_t *)buffer, len, 1000); //Send commands to nextion display		
}

void configure_display_sleepmode()
{
	char buffer[40]; //stores string to be send
	sprintf(buffer,"thsp=30ÿÿÿthup=1ÿÿÿ"); //thsp=30(No touch operation within 30 seconds, it will auto enter into sleep mode). //Touch will autom awake switch during sleep mode
	int len = strlen(buffer);
	HAL_UART_Transmit(&huart3, (uint8_t *)buffer, len, 1000); //Send commands to nextion display
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
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
