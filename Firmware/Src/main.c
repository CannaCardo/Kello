
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h" // USB
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

TSC_HandleTypeDef htsc;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define MULTIPLEXING		0

#define	EEPROM_ADDRESS		0b1010000
#define TIMEFORMAT_12		12
#define TIMEFORMAT_24		24

uint8_t		NIXIE_Digit1;
uint8_t		NIXIE_Digit2;
uint8_t		NIXIE_Digit3;
uint8_t		NIXIE_Digit4;

RTC_TimeTypeDef RTC_Time;
RTC_DateTypeDef RTC_Date;

uint8_t		HV_Enable;
uint8_t		TimeFormat;

uint16_t		TIM_Prescaler;
const uint16_t	TIM_Period = 500;
uint16_t		TIM_Compare;

// used in usbd_cdc_if.c
uint8_t			USB_Update_Time;
uint8_t			USB_Update_Brightness;
uint8_t			USB_Update_RefFreq;
uint8_t     	USB_Update_TimeFormat;

uint8_t			USB_Hours;
uint8_t			USB_Minutes;
uint8_t			USB_Seconds;
uint8_t			USB_TimeFormat;

uint8_t 		USB_Brightness;
uint16_t 		USB_RefreshFreq;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_TSC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t Current_Digit = 1;
	uint8_t	Current_Value = 0;

    if( htim == &htim1 ){
    	//Check if High Voltage Driver is enabled
    	if( HV_Enable ){
			// Poll time information
			HAL_RTC_GetTime(&hrtc, &RTC_Time, RTC_FORMAT_BCD);
			HAL_RTC_GetDate(&hrtc, &RTC_Date, RTC_FORMAT_BCD);

			// Determine value of currently shown digit
			if( TimeFormat == TIMEFORMAT_24 ){
				if( Current_Digit == 1 ) 		Current_Value = ((RTC_Time.Hours & 0xF0)>>4);
				else if( Current_Digit == 2 ) 	Current_Value =  (RTC_Time.Hours & 0x0F);
				else if( Current_Digit == 3 ) 	Current_Value = ((RTC_Time.Minutes & 0xF0)>>4);
				else if( Current_Digit == 4 ) 	Current_Value =  (RTC_Time.Minutes & 0x0F);
				else							Current_Value = 0;
			}else{
				if( Current_Digit == 1 ) 		Current_Value = (((RTC_Time.Hours%12) & 0xF0)>>4);
				else if( Current_Digit == 2 ) 	Current_Value =  ((RTC_Time.Hours%12) & 0x0F);
				else if( Current_Digit == 3 ) 	Current_Value = ((RTC_Time.Minutes & 0xF0)>>4);
				else if( Current_Digit == 4 ) 	Current_Value =  (RTC_Time.Minutes & 0x0F);
				else							Current_Value = 0;
			}

			// Disable all anodes
			HAL_GPIO_WritePin( Anode_1_GPIO_Port, Anode_1_Pin, GPIO_PIN_RESET );
			HAL_GPIO_WritePin( Anode_2_GPIO_Port, Anode_2_Pin, GPIO_PIN_RESET );
			HAL_GPIO_WritePin( Anode_3_GPIO_Port, Anode_3_Pin, GPIO_PIN_RESET );
			HAL_GPIO_WritePin( Anode_4_GPIO_Port, Anode_4_Pin, GPIO_PIN_RESET );
			// Disable all lamps (this should already be done by TIM Compare Callback, but just in case
			HAL_GPIO_WritePin( Digit_0_GPIO_Port, Digit_0_Pin, GPIO_PIN_RESET );
			HAL_GPIO_WritePin( Digit_1_GPIO_Port, Digit_1_Pin, GPIO_PIN_RESET );
			HAL_GPIO_WritePin( Digit_2_GPIO_Port, Digit_2_Pin, GPIO_PIN_RESET );
			HAL_GPIO_WritePin( Digit_3_GPIO_Port, Digit_3_Pin, GPIO_PIN_RESET );
			HAL_GPIO_WritePin( Digit_4_GPIO_Port, Digit_4_Pin, GPIO_PIN_RESET );
			HAL_GPIO_WritePin( Digit_5_GPIO_Port, Digit_5_Pin, GPIO_PIN_RESET );
			HAL_GPIO_WritePin( Digit_6_GPIO_Port, Digit_6_Pin, GPIO_PIN_RESET );
			HAL_GPIO_WritePin( Digit_7_GPIO_Port, Digit_7_Pin, GPIO_PIN_RESET );
			HAL_GPIO_WritePin( Digit_8_GPIO_Port, Digit_8_Pin, GPIO_PIN_RESET );
			HAL_GPIO_WritePin( Digit_9_GPIO_Port, Digit_9_Pin, GPIO_PIN_RESET );

			// Drive current digit
			if( Current_Value == 0 ) 		HAL_GPIO_WritePin( Digit_0_GPIO_Port, Digit_0_Pin, GPIO_PIN_SET );
			else if( Current_Value == 1 ) 	HAL_GPIO_WritePin( Digit_1_GPIO_Port, Digit_1_Pin, GPIO_PIN_SET );
			else if( Current_Value == 2 ) 	HAL_GPIO_WritePin( Digit_2_GPIO_Port, Digit_2_Pin, GPIO_PIN_SET );
			else if( Current_Value == 3 ) 	HAL_GPIO_WritePin( Digit_3_GPIO_Port, Digit_3_Pin, GPIO_PIN_SET );
			else if( Current_Value == 4 ) 	HAL_GPIO_WritePin( Digit_4_GPIO_Port, Digit_4_Pin, GPIO_PIN_SET );
			else if( Current_Value == 5 ) 	HAL_GPIO_WritePin( Digit_5_GPIO_Port, Digit_5_Pin, GPIO_PIN_SET );
			else if( Current_Value == 6 ) 	HAL_GPIO_WritePin( Digit_6_GPIO_Port, Digit_6_Pin, GPIO_PIN_SET );
			else if( Current_Value == 7 ) 	HAL_GPIO_WritePin( Digit_7_GPIO_Port, Digit_7_Pin, GPIO_PIN_SET );
			else if( Current_Value == 8 ) 	HAL_GPIO_WritePin( Digit_8_GPIO_Port, Digit_8_Pin, GPIO_PIN_SET );
			else if( Current_Value == 9 ) 	HAL_GPIO_WritePin( Digit_9_GPIO_Port, Digit_9_Pin, GPIO_PIN_SET );
			else						 	HAL_GPIO_WritePin( Digit_0_GPIO_Port, Digit_0_Pin, GPIO_PIN_SET );

			// Enable current lamp
			switch(Current_Digit){
			case 1:
				HAL_GPIO_WritePin( Anode_1_GPIO_Port, Anode_1_Pin, GPIO_PIN_SET );
				HAL_GPIO_WritePin( Anode_2_GPIO_Port, Anode_2_Pin, GPIO_PIN_RESET );
				HAL_GPIO_WritePin( Anode_3_GPIO_Port, Anode_3_Pin, GPIO_PIN_RESET );
				HAL_GPIO_WritePin( Anode_4_GPIO_Port, Anode_4_Pin, GPIO_PIN_RESET );
				Current_Digit = 2;
				break;
			case 2:
				HAL_GPIO_WritePin( Anode_1_GPIO_Port, Anode_1_Pin, GPIO_PIN_RESET );
				HAL_GPIO_WritePin( Anode_2_GPIO_Port, Anode_2_Pin, GPIO_PIN_SET );
				HAL_GPIO_WritePin( Anode_3_GPIO_Port, Anode_3_Pin, GPIO_PIN_RESET );
				HAL_GPIO_WritePin( Anode_4_GPIO_Port, Anode_4_Pin, GPIO_PIN_RESET );
				Current_Digit = 3;
				break;
			case 3:
				HAL_GPIO_WritePin( Anode_1_GPIO_Port, Anode_1_Pin, GPIO_PIN_RESET );
				HAL_GPIO_WritePin( Anode_2_GPIO_Port, Anode_2_Pin, GPIO_PIN_RESET );
				HAL_GPIO_WritePin( Anode_3_GPIO_Port, Anode_3_Pin, GPIO_PIN_SET );
				HAL_GPIO_WritePin( Anode_4_GPIO_Port, Anode_4_Pin, GPIO_PIN_RESET );
				Current_Digit = 4;
				break;
			case 4:
				HAL_GPIO_WritePin( Anode_1_GPIO_Port, Anode_1_Pin, GPIO_PIN_RESET );
				HAL_GPIO_WritePin( Anode_2_GPIO_Port, Anode_2_Pin, GPIO_PIN_RESET );
				HAL_GPIO_WritePin( Anode_3_GPIO_Port, Anode_3_Pin, GPIO_PIN_RESET );
				HAL_GPIO_WritePin( Anode_4_GPIO_Port, Anode_4_Pin, GPIO_PIN_SET );
				Current_Digit = 1;
				break;
			default:
				HAL_GPIO_WritePin( Anode_1_GPIO_Port, Anode_1_Pin, GPIO_PIN_RESET );
				HAL_GPIO_WritePin( Anode_2_GPIO_Port, Anode_2_Pin, GPIO_PIN_RESET );
				HAL_GPIO_WritePin( Anode_3_GPIO_Port, Anode_3_Pin, GPIO_PIN_RESET );
				HAL_GPIO_WritePin( Anode_4_GPIO_Port, Anode_4_Pin, GPIO_PIN_RESET );
				Current_Digit = 1;
			}
    	}else{
    		// If high voltage driver is disabled, then null all lines
    		HAL_GPIO_WritePin( Anode_1_GPIO_Port, Anode_1_Pin, GPIO_PIN_RESET );
			HAL_GPIO_WritePin( Anode_2_GPIO_Port, Anode_2_Pin, GPIO_PIN_RESET );
			HAL_GPIO_WritePin( Anode_3_GPIO_Port, Anode_3_Pin, GPIO_PIN_RESET );
			HAL_GPIO_WritePin( Anode_4_GPIO_Port, Anode_4_Pin, GPIO_PIN_RESET );

			HAL_GPIO_WritePin( Digit_0_GPIO_Port, Digit_0_Pin, GPIO_PIN_RESET );
			HAL_GPIO_WritePin( Digit_1_GPIO_Port, Digit_1_Pin, GPIO_PIN_RESET );
			HAL_GPIO_WritePin( Digit_2_GPIO_Port, Digit_2_Pin, GPIO_PIN_RESET );
			HAL_GPIO_WritePin( Digit_3_GPIO_Port, Digit_3_Pin, GPIO_PIN_RESET );
			HAL_GPIO_WritePin( Digit_4_GPIO_Port, Digit_4_Pin, GPIO_PIN_RESET );
			HAL_GPIO_WritePin( Digit_5_GPIO_Port, Digit_5_Pin, GPIO_PIN_RESET );
			HAL_GPIO_WritePin( Digit_6_GPIO_Port, Digit_6_Pin, GPIO_PIN_RESET );
			HAL_GPIO_WritePin( Digit_7_GPIO_Port, Digit_7_Pin, GPIO_PIN_RESET );
			HAL_GPIO_WritePin( Digit_8_GPIO_Port, Digit_8_Pin, GPIO_PIN_RESET );
			HAL_GPIO_WritePin( Digit_9_GPIO_Port, Digit_9_Pin, GPIO_PIN_RESET );
    	}
    }else if( htim == &htim2 ){
    	// Flashing dots, 1Hz
    	//HAL_GPIO_TogglePin( DOT_H_GPIO_Port, DOT_H_Pin );
    	//HAL_GPIO_TogglePin( DOT_L_GPIO_Port, DOT_L_Pin );
    }
}
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
	// this function serves the purpose of variable brightness
	if( htim == &htim1 ){
		// due to ghosting (lamp to lamp) turn off the cathodes while leaving the anode on
		// this way the potential on cathodes will rise (during dead time) and prohibit unwanted glow

		HAL_GPIO_WritePin( Digit_0_GPIO_Port, Digit_0_Pin, GPIO_PIN_RESET );
		HAL_GPIO_WritePin( Digit_1_GPIO_Port, Digit_1_Pin, GPIO_PIN_RESET );
		HAL_GPIO_WritePin( Digit_2_GPIO_Port, Digit_2_Pin, GPIO_PIN_RESET );
		HAL_GPIO_WritePin( Digit_3_GPIO_Port, Digit_3_Pin, GPIO_PIN_RESET );
		HAL_GPIO_WritePin( Digit_4_GPIO_Port, Digit_4_Pin, GPIO_PIN_RESET );
		HAL_GPIO_WritePin( Digit_5_GPIO_Port, Digit_5_Pin, GPIO_PIN_RESET );
		HAL_GPIO_WritePin( Digit_6_GPIO_Port, Digit_6_Pin, GPIO_PIN_RESET );
		HAL_GPIO_WritePin( Digit_7_GPIO_Port, Digit_7_Pin, GPIO_PIN_RESET );
		HAL_GPIO_WritePin( Digit_8_GPIO_Port, Digit_8_Pin, GPIO_PIN_RESET );
		HAL_GPIO_WritePin( Digit_9_GPIO_Port, Digit_9_Pin, GPIO_PIN_RESET );

		HAL_GPIO_WritePin( Anode_1_GPIO_Port, Anode_1_Pin, GPIO_PIN_SET );
		HAL_GPIO_WritePin( Anode_2_GPIO_Port, Anode_2_Pin, GPIO_PIN_SET );
		HAL_GPIO_WritePin( Anode_3_GPIO_Port, Anode_3_Pin, GPIO_PIN_SET );
		HAL_GPIO_WritePin( Anode_4_GPIO_Port, Anode_4_Pin, GPIO_PIN_SET );
	}
}
/**
  * @brief  Set Nixie Brightness
  * @param  Specifies the brightness of Nixie Tubes in %.
  *          This parameter can take value from range of 1 - 70
  * @note  Due to ghosting (lamp to lamp) brightness is limited to 70 - this ensures some dead time in-between
  * @retval HAL status
  */
HAL_StatusTypeDef SetBrightness( uint8_t val ){
	// check the value range
	if( val < 1 || val > 70 ) return HAL_ERROR;

	// Constant Period of 500
	TIM_Compare = TIM_Period / 100 * val;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, TIM_Compare);

	return HAL_OK;
}
/**
  * @brief  Set Nixie Refresh rate
  * @param  Specifies the Refresh rate of Nixie Tubes in Hz.
  *          This parameter can take value from range of 10 - 1000
  * @retval HAL status
  */
HAL_StatusTypeDef SetRefreshRate( uint16_t freq ){
	// check the value range
	if( freq < 10 || freq > 1000 ) return HAL_ERROR;

	// 24 	- 1000 Hz
	// 240 	-  100 Hz
	// 2400 -   10 Hz
	TIM_Prescaler = 24000/freq;
	__HAL_TIM_SET_PRESCALER(&htim1, TIM_Prescaler);

	return HAL_OK;
}
/**
  * @brief  Initialize Nixie Refresh timer
  */
void NixieTimInit( void ){
	HAL_StatusTypeDef ret = HAL_OK;
	uint8_t 	Brightness;
	uint16_t 	RefreshFreq;

	// poll data from EEprom
	ret |= HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS, 0x00, 1, &Brightness, 1, 100);
	ret |= HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS, 0x01, 1, ((uint8_t*)&RefreshFreq), 2, 100);
	ret |= HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS, 0x02, 1, &TimeFormat, 1, 100);
	// if EEprom is broken or not installed, assume default values
	if( ret == HAL_ERROR ){
		// 100 Hz Refresh, 70% brightness, 24h format
		Brightness  = 70;
		RefreshFreq = 100;
		TimeFormat	= TIMEFORMAT_24;
	}else{
		// check for correctness of received data
		// EEprom chips from factory usually have mem wiped to 0x00 or 0xFF (no info in datasheet)
		if( Brightness < 1 || Brightness > 100 ){
			Brightness = 70;

			HAL_GPIO_WritePin(EE_WP_GPIO_Port, EE_WP_Pin, GPIO_PIN_RESET);
			HAL_Delay(10);
			HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, 0x00, 1, &Brightness, 1, 100);
			HAL_Delay(10);
			HAL_GPIO_WritePin(EE_WP_GPIO_Port, EE_WP_Pin, GPIO_PIN_SET);
		}
		if( RefreshFreq < 10 || RefreshFreq > 1000 ){
			RefreshFreq = 100;

			HAL_GPIO_WritePin(EE_WP_GPIO_Port, EE_WP_Pin, GPIO_PIN_RESET);
			HAL_Delay(10);
			HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, 0x01, 1, ((uint8_t*)&RefreshFreq), 1, 100);
			HAL_Delay(10);
			HAL_GPIO_WritePin(EE_WP_GPIO_Port, EE_WP_Pin, GPIO_PIN_SET);
		}
		if( TimeFormat != TIMEFORMAT_12 && TimeFormat != TIMEFORMAT_24 ){
			TimeFormat = TIMEFORMAT_24;

			HAL_GPIO_WritePin(EE_WP_GPIO_Port, EE_WP_Pin, GPIO_PIN_RESET);
			HAL_Delay(10);
			HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, 0x02, 1, &TimeFormat, 1, 100);
			HAL_Delay(10);
			HAL_GPIO_WritePin(EE_WP_GPIO_Port, EE_WP_Pin, GPIO_PIN_SET);
		}
	}

	SetBrightness( Brightness );
	SetRefreshRate( RefreshFreq );

	// Start timer
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim2);

	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	USB_Update_Time = 0;
	USB_Update_Brightness = 0;
	USB_Update_RefFreq = 0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_TSC_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  NixieTimInit();
  HV_Enable = 1;
  HAL_GPIO_WritePin(HV_Enable_GPIO_Port, HV_Enable_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // wait for system driver to initialize
  HAL_Delay(1000);
  //USB_Hours 	= 0x17;
  //USB_Minutes 	= 0x45;
  //USB_Seconds 	= 0x00;
  //USB_Update_Time = 1;

  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  if( USB_Update_Time ){
		  RTC_TimeTypeDef USB_RTC_Time;
		  USB_RTC_Time.Hours = USB_Hours;
		  USB_RTC_Time.Minutes = USB_Minutes;
		  USB_RTC_Time.Seconds = USB_Seconds;
		  USB_RTC_Time.TimeFormat = TIMEFORMAT_24;
		  HAL_RTC_SetTime(&hrtc, &USB_RTC_Time, RTC_FORMAT_BCD);
		  USB_Update_Time = 0;
	  }else if( USB_Update_Brightness ){
		  SetBrightness(USB_Brightness);
		  USB_Update_Brightness = 0;
	  }else if( USB_Update_RefFreq ){
		  SetRefreshRate(USB_RefreshFreq);
		  USB_Update_RefFreq = 0;
	  }else if( USB_Update_TimeFormat ){
		  TimeFormat = USB_TimeFormat;
		  USB_Update_RefFreq = 0;
	  }
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure LSE Drive Capability 
    */
  HAL_PWR_EnableBkUpAccess();

  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_HIGH);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20303E5D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

  /* USER CODE BEGIN RTC_Init 1 */
  //if( RTC->ISR == 7 ){
  	  if(RTC->ISR == 7){ // Register Status Flag cold start value
          return;
      }
	  /* USER CODE END RTC_Init 1 */

		/**Initialize RTC Only
		*/
	  hrtc.Instance = RTC;
	  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	  hrtc.Init.AsynchPrediv = 127;
	  hrtc.Init.SynchPrediv = 255;
	  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	  if (HAL_RTC_Init(&hrtc) != HAL_OK)
	  {
		_Error_Handler(__FILE__, __LINE__);
	  }
	  /* USER CODE BEGIN RTC_Init 2 */

	  /* USER CODE END RTC_Init 2 */

		/**Initialize RTC and set the Time and Date
		*/
	  sTime.Hours = 0x0;
	  sTime.Minutes = 0x0;
	  sTime.Seconds = 0x0;
	  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	  {
		_Error_Handler(__FILE__, __LINE__);
	  }
	  /* USER CODE BEGIN RTC_Init 3 */

	  /* USER CODE END RTC_Init 3 */

	  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
	  sDate.Month = RTC_MONTH_JANUARY;
	  sDate.Date = 0x1;
	  sDate.Year = 0x0;

	  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
	  {
		_Error_Handler(__FILE__, __LINE__);
	  }
	  /* USER CODE BEGIN RTC_Init 4 */
	//}
  /* USER CODE END RTC_Init 4 */

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 24;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 500;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 48000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TSC init function */
static void MX_TSC_Init(void)
{

    /**Configure the TSC peripheral 
    */
  htsc.Instance = TSC;
  htsc.Init.CTPulseHighLength = TSC_CTPH_2CYCLES;
  htsc.Init.CTPulseLowLength = TSC_CTPL_2CYCLES;
  htsc.Init.SpreadSpectrum = DISABLE;
  htsc.Init.SpreadSpectrumDeviation = 1;
  htsc.Init.SpreadSpectrumPrescaler = TSC_SS_PRESC_DIV1;
  htsc.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV4;
  htsc.Init.MaxCountValue = TSC_MCV_8191;
  htsc.Init.IODefaultMode = TSC_IODEF_OUT_PP_LOW;
  htsc.Init.SynchroPinPolarity = TSC_SYNC_POLARITY_FALLING;
  htsc.Init.AcquisitionMode = TSC_ACQ_MODE_NORMAL;
  htsc.Init.MaxCountInterrupt = DISABLE;
  htsc.Init.ChannelIOs = TSC_GROUP2_IO1|TSC_GROUP2_IO2|TSC_GROUP2_IO3;
  htsc.Init.ShieldIOs = TSC_GROUP3_IO3;
  htsc.Init.SamplingIOs = TSC_GROUP2_IO4|TSC_GROUP3_IO2;
  if (HAL_TSC_Init(&htsc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Digit_3_GPIO_Port, Digit_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HV_Enable_GPIO_Port, HV_Enable_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Anode_4_Pin|Anode_3_Pin|Anode_2_Pin|Anode_1_Pin 
                          |Digit_0_Pin|Digit_9_Pin|Digit_8_Pin|Digit_7_Pin 
                          |Digit_6_Pin|Digit_4_Pin|Digit_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EE_WP_Pin|Digit_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Digit_5_GPIO_Port, Digit_5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Digit_3_Pin */
  GPIO_InitStruct.Pin = Digit_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Digit_3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HV_Enable_Pin */
  GPIO_InitStruct.Pin = HV_Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HV_Enable_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Anode_4_Pin Anode_3_Pin Anode_2_Pin Anode_1_Pin 
                           Digit_0_Pin Digit_9_Pin Digit_8_Pin Digit_7_Pin 
                           Digit_6_Pin Digit_4_Pin Digit_2_Pin */
  GPIO_InitStruct.Pin = Anode_4_Pin|Anode_3_Pin|Anode_2_Pin|Anode_1_Pin 
                          |Digit_0_Pin|Digit_9_Pin|Digit_8_Pin|Digit_7_Pin 
                          |Digit_6_Pin|Digit_4_Pin|Digit_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : EE_WP_Pin Digit_1_Pin */
  GPIO_InitStruct.Pin = EE_WP_Pin|Digit_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Digit_5_Pin */
  GPIO_InitStruct.Pin = Digit_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Digit_5_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
