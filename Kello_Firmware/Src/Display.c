/*
 * Display.c
 *
 *  Created on: 28 paz 2018
 *      Author: Krzysztof Belewicz
 *
 *
 *	TODO: variable neon lamp brightness
 */

#include "Display.h"
#include "math.h"
#include "stdlib.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t *	TopDot_Buffer;
uint32_t *	BotDot_Buffer;

uint8_t		FadeIO_Style; // TODO read from EEprom, add functions
uint8_t		DotStyle;

uint32_t 	FadeIO_Stable0;
uint32_t 	FadeIO_Rise;
uint32_t 	FadeIO_Stable1;
uint32_t 	FadeIO_Fall;
uint32_t	FadeIO_Period;

uint32_t	Blink_On;
uint32_t	Blink_Off;
uint32_t	Blink_Period;

uint32_t 	Dot_StaticOn[DOT_BUFFER_SIZE];
uint32_t 	Dot_StaticOff[DOT_BUFFER_SIZE];
uint32_t 	Dot_Blink[DOT_BUFFER_SIZE];
uint32_t *	Dot_Custom;
uint32_t 	Dot_FadeIO[DOT_BUFFER_SIZE];

TIM_HandleTypeDef * htim_Nixie;
TIM_HandleTypeDef * htim_Dots;

uint16_t		TIM_Prescaler;
const uint16_t	TIM_Period = 500;
uint16_t		TIM_Compare;

uint8_t			HV_Enable;

volatile uint8_t			TimeFormat;
RTC_TimeTypeDef RTC_Time;
RTC_DateTypeDef RTC_Date;
uint8_t			AM_notPM;

uint8_t		NIXIE_Digit1;
uint8_t		NIXIE_Digit2;
uint8_t		NIXIE_Digit3;
uint8_t		NIXIE_Digit4;


void DotArray_Init(void) {
	uint32_t PWM_Period = DOT_COUNTER / DOT_FREQ; // in ms
	uint32_t imod;

	FadeIO_Stable0 = DOT_FADEIO_STABLE0 / PWM_Period;
	FadeIO_Rise = DOT_FADEIO_RISE / PWM_Period;
	FadeIO_Stable1 = DOT_FADEIO_STABLE1 / PWM_Period;
	FadeIO_Fall = DOT_FADEIO_FALL / PWM_Period;
	FadeIO_Period = FadeIO_Stable0 + FadeIO_Rise + FadeIO_Stable1 + FadeIO_Fall;

	Blink_On = DOT_BLINK_ONTIME / PWM_Period;
	Blink_Off = DOT_BLINK_OFFTIME / PWM_Period;
	Blink_Period = Blink_On + Blink_Off;

	// allocate memmory for DMA buffers
	if( TopDot_Buffer ) free(TopDot_Buffer);
	if( BotDot_Buffer ) free(BotDot_Buffer);

	TopDot_Buffer = (uint32_t *)malloc( sizeof *TopDot_Buffer * (DOT_BUFFER_SIZE) );
	BotDot_Buffer = (uint32_t *)malloc( sizeof *TopDot_Buffer * (DOT_BUFFER_SIZE) );

	// calculate fade io values TODO depend on variable not define
	for (uint32_t i = 0; i<DOT_BUFFER_SIZE; ++i) {
		Dot_StaticOn[i] = DOT_FULLBRIGHTNESS;
		Dot_StaticOff[i] = 0;

		// blink
		imod = i%Blink_Period;
		if (imod <= Blink_On)	Dot_Blink[i] = DOT_FULLBRIGHTNESS;
		else					Dot_Blink[i] = 0;

		// Fade In/Out
		imod = i%FadeIO_Period;
		if (imod <= FadeIO_Stable0) {
			Dot_FadeIO[i] = 0;
		}
		else if (imod <= FadeIO_Rise + FadeIO_Stable0) {
			// 0-1 progress of transition
			double lin = ((double)(imod - FadeIO_Stable0)) / FadeIO_Rise;
			// transition shape
			#if DOT_FADEIO_TRANS == DOT_FADEIO_LINEAR
				Dot_FadeIO[i] = ((uint32_t)(lin*DOT_FULLBRIGHTNESS));
			#elif DOT_FADEIO_TRANS == DOT_FADEIO_SIN
				// sin from -pi/2 to pi/2
				double s = sin(M_PI*(lin - 0.5));
				Dot_FadeIO[i] = ((uint32_t)(((double)DOT_FULLBRIGHTNESS)*(s + 1.0) / 2.0));
			#elif DOT_FADEIO_TRANS == DOT_FADEIO_LOG
				// log from 1 to 1000
				double l = log10(lin * 999 + 1);
				Dot_FadeIO[i] = ((uint32_t)(((double)DOT_FULLBRIGHTNESS)*l / 3.0));
			#elif DOT_FADEIO_TRANS == DOT_FADEIO_INVLOG
				// log from 1000 to 1
				double l = 3.0 - log10(1000 - lin * 999);
				Dot_FadeIO[i] = ((uint32_t)(((double)DOT_FULLBRIGHTNESS)*l / 3.0));
			#endif
		}
		else if (imod <= FadeIO_Stable1 + FadeIO_Rise + FadeIO_Stable0) {
			Dot_FadeIO[i] = DOT_FULLBRIGHTNESS;
		}
		else { // fall
			// 0-1 progress of transition
			double lin = ((double)(imod - FadeIO_Stable1 - FadeIO_Rise - FadeIO_Stable0)) / FadeIO_Fall;
			#if DOT_FADEIO_TRANS == DOT_FADEIO_LINEAR
				Dot_FadeIO[i] = ((uint32_t)((1.0 - lin)*DOT_FULLBRIGHTNESS));
			#elif DOT_FADEIO_TRANS == DOT_FADEIO_SIN
				// sin from pi/2 to -pi/2
				double s = sin(M_PI*(-lin + 0.5));
				Dot_FadeIO[i] = ((uint32_t)(((double)DOT_FULLBRIGHTNESS)*(s + 1.0) / 2.0));
			#elif DOT_FADEIO_TRANS == DOT_FADEIO_LOG
				// log from 1000 to 1
				double l = log10(1000 - lin * 999);
				Dot_FadeIO[i] = ((uint32_t)(((double)DOT_FULLBRIGHTNESS)*l / 3.0));
			#elif DOT_FADEIO_TRANS == DOT_FADEIO_INVLOG
				// log from 1 to 1000
				double l = 3.0 - log10(lin * 999 + 1);
				Dot_FadeIO[i] = ((uint32_t)(((double)DOT_FULLBRIGHTNESS)*l / 3.0));
			#endif
		}
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

//void TIMx_DMA_IRQHandler(void) {
//	HAL_DMA_IRQHandler(htim_Dots.hdma[DMACHANNEL_TOP_DOT]);
//	HAL_DMA_IRQHandler(htim_Dots.hdma[DMACHANNEL_BOT_DOT]);
//}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t Current_Digit = 1;
	uint8_t	Current_Value = 0;

    if( htim == htim_Nixie ){
    	//Check if High Voltage Driver is enabled
    	if( HV_Enable ){
			// Poll time information
    		HAL_StatusTypeDef 	ret = HAL_OK;
    		ret |= HAL_RTC_GetTime(&hrtc, &RTC_Time, RTC_FORMAT_BCD);
    		ret |= HAL_RTC_GetDate(&hrtc, &RTC_Date, RTC_FORMAT_BCD);

    		if( ret != HAL_OK ){
    			RTC_Time.Hours 		= (1)<<4 | (3);
				RTC_Time.Minutes 	= (3)<<4 | (7);
				RTC_Time.Seconds 	= (0)<<4 | (0);
    		}


			// Determine value of currently shown digit
			if( TimeFormat == TIMEFORMAT_24 ){
				if( Current_Digit == 1 ) 		Current_Value = ((RTC_Time.Hours & 0xF0)>>4);
				else if( Current_Digit == 2 ) 	Current_Value =  (RTC_Time.Hours & 0x0F);
				else if( Current_Digit == 3 ) 	Current_Value = ((RTC_Time.Minutes & 0xF0)>>4);
				else if( Current_Digit == 4 ) 	Current_Value =  (RTC_Time.Minutes & 0x0F);
				else							Current_Value = 0;
			}else{
				uint8_t h = (((RTC_Time.Hours) & 0xF0)>>4)*10 + ((RTC_Time.Hours) & 0x0F);
				if( h >= 12 ) 	AM_notPM = 0;
				else			AM_notPM = 1;
				h = h%12;

				if( Current_Digit == 1 && h >= 10 ) 		Current_Value = 1;
				else if( Current_Digit == 1 && h <= 9 ) 	Current_Value = 0;
				else if( Current_Digit == 2 && h >= 10 ) 	Current_Value = h-10;
				else if( Current_Digit == 2 && h <= 9 ) 	Current_Value = h;
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
    }else if( htim == htim_Dots ){
    }
}


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
	// this function serves the purpose of variable brightness
	if( htim == htim_Nixie ){
		// due to ghosting (lamp to lamp) turn off the cathodes while leaving the anode on
		// this way the potential on cathodes will rise (during dead time) and lessen the unwanted glow

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

HAL_StatusTypeDef DOTTIM_PWM_Start_DMA(TIM_HandleTypeDef *htim, uint32_t *pData_bot, uint32_t *pData_top, uint16_t Length)
{
  /* Check the parameters */
  assert_param(IS_TIM_CCX_INSTANCE(htim->Instance, TIMCHANNEL_TOP_DOT));
  assert_param(IS_TIM_CCX_INSTANCE(htim->Instance, TIMCHANNEL_BOT_DOT));

  if ((htim->State == HAL_TIM_STATE_BUSY))
  {
    return HAL_BUSY;
  }
  else if ((htim->State == HAL_TIM_STATE_READY))
  {
    if ((pData_bot == NULL || pData_top == NULL) && (Length > 0U))
    {
      return HAL_ERROR;
    }
    else
    {
      htim->State = HAL_TIM_STATE_BUSY;
    }
  }
  else
  {
    /* nothing to do */
  }

  // Bottom Dot
  /* Set the DMA Period elapsed callback */
  htim->hdma[TIM_DMA_ID_CC3]->XferCpltCallback = TIM_DMADelayPulseCplt;

  /* Set the DMA error callback */
  htim->hdma[TIM_DMA_ID_CC3]->XferErrorCallback = TIM_DMAError ;

  /* Enable the DMA channel */
  if (HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC3], (uint32_t)pData_bot, (uint32_t)&htim->Instance->CCR3, Length) != HAL_OK)
  {
	return HAL_ERROR;
  }
  /* Enable the TIM Output Capture/Compare 3 request */
  __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC3);

  /* Set the DMA Period elapsed callback */
  htim->hdma[TIM_DMA_ID_CC4]->XferCpltCallback = TIM_DMADelayPulseCplt;

  /* Set the DMA error callback */
  htim->hdma[TIM_DMA_ID_CC4]->XferErrorCallback = TIM_DMAError ;

  /* Enable the DMA channel */
  if (HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC4], (uint32_t)pData_top, (uint32_t)&htim->Instance->CCR4, Length) != HAL_OK)
  {
	return HAL_ERROR;
  }
  /* Enable the TIM Capture/Compare 4 DMA request */
  __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC4);

  /* Enable the Capture compare channel */
  TIM_CCxChannelCmd(htim->Instance, TIMCHANNEL_TOP_DOT, TIM_CCx_ENABLE);
  TIM_CCxChannelCmd(htim->Instance, TIMCHANNEL_BOT_DOT, TIM_CCx_ENABLE);


  if (IS_TIM_BREAK_INSTANCE(htim->Instance) != RESET)
  {
    /* Enable the main output */
    __HAL_TIM_MOE_ENABLE(htim);
  }

  /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
  if (!IS_TIM_SLAVEMODE_TRIGGER_ENABLED(htim))
  {
    __HAL_TIM_ENABLE(htim);
  }

  /* Return function status */
  return HAL_OK;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

/** @brief  Initialize Display routine
  * @param  hrtc RTC handle
  * @param  htim TIM handle for driving Nixie refresh routine
  * @param  htim TIM handle for driving Neon lamps PWM signal
  * @retval status
  */
HAL_StatusTypeDef	Display_Init( 	TIM_HandleTypeDef * htim_Nixie_,
									TIM_HandleTypeDef * htim_Dots_ ){
	HAL_StatusTypeDef 	ret = HAL_OK;

	htim_Nixie 	= htim_Nixie_;
	htim_Dots 	= htim_Dots_;

	Display_Stop();

	ret |= HAL_TIM_Base_Start_IT(htim_Nixie);
	ret |= HAL_TIM_Base_Start(htim_Dots);

	// PWM no output mode
	// pulse end sets display brightness
	ret |= HAL_TIM_PWM_Start_IT(htim_Nixie, TIMCHANNEL_OFFTIME);

	// initialise dot dma arrays
	DotArray_Init();

	// set initial dot style and copy its value
	DotStyle = DOTS_BLINK;
	Display_Set_DotStyle( DOTS_FADEIO );

	// PWM signals driving Neon lamp dots
	ret |= DOTTIM_PWM_Start_DMA(htim_Dots, BotDot_Buffer, TopDot_Buffer, DOT_BUFFER_SIZE);
	//ret |= HAL_TIM_PWM_Start_DMA(htim_Dots, TIMCHANNEL_TOP_DOT, (uint32_t *)TopDot_Buffer, DOT_BUFFER_SIZE);
	//ret |= HAL_TIM_PWM_Start_DMA(htim_Dots, TIMCHANNEL_BOT_DOT, (uint32_t *)BotDot_Buffer, DOT_BUFFER_SIZE);

	return ret;
}

/** @brief  Enables high voltage converter
  * @param  none
  * @retval none
  */
void 				Display_Start( void ){
	HV_Enable = 1;
	HAL_GPIO_WritePin(HV_Enable_GPIO_Port, HV_Enable_Pin, GPIO_PIN_RESET);
}

/** @brief  Disables high voltage converter
  * @param  none
  * @retval none
  */
void 				Display_Stop( void ){
	HV_Enable = 0;
	HAL_GPIO_WritePin(HV_Enable_GPIO_Port, HV_Enable_Pin, GPIO_PIN_SET);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

/** @brief  Set display refresh rate
  * @param  Specifies the Refresh rate of Nixie Tubes in Hz.
  *          This parameter can take value from range of 10 - 1000
  * @retval none
  */
void 				Display_Set_RefreshRate( uint16_t freq ){
	uint16_t freq_;

	// clipp value to range
	if( freq < MIN_REFRESHRATE )		freq_= MIN_REFRESHRATE;
	else if( freq > MAX_REFRESHRATE ) 	freq_= MAX_REFRESHRATE;
	else								freq_= freq;

	// 24 	- 1000 Hz
	// 240 	-  100 Hz
	// 2400 -   10 Hz
	TIM_Prescaler = 24000/freq_;
	__HAL_TIM_SET_PRESCALER(htim_Nixie, TIM_Prescaler);
}

/** @brief  Set display Brightness
  * @param  Specifies the brightness of Nixie Tubes in %.
  *          This parameter can take value from range of 1 - 100
  * @note  	Due to ghosting (lamp to lamp) brightness is limited to 70 - this ensures some dead time in-between
  * @retval none
  */
void 				Display_Set_Brightness( uint8_t val ){
	uint8_t val_;

	// clipp value to range
	if( val < MIN_BRIGHTNESS )			val_= MIN_BRIGHTNESS;
	else if( val > MAX_BRIGHTNESS ) 	val_= MAX_BRIGHTNESS;
	else								val_= val;

	// Constant Period of 500
	TIM_Compare = (uint16_t)(((float)TIM_Period) / 100 * val_);
	__HAL_TIM_SET_COMPARE(htim_Nixie, TIMCHANNEL_OFFTIME, TIM_Compare);
}


/** @brief  Set display Time format
  * @param  12 or 24 time format
  * @retval none
  */
void 				Display_Set_TimeFormat( uint8_t val ){
	uint8_t val_;

	if( val != TIMEFORMAT_12 && val != TIMEFORMAT_24 ) val_ = TIMEFORMAT_24;
	else											   val_= val;

	TimeFormat = val_;
}

/** @brief  Set neon lamp custom style
  * @param  custom style vector
  * @retval none
  */
void 				Display_Set_DotCustom( uint32_t * vect ){
	Dot_Custom = vect;
}

/** @brief  Set neon lamp style
  * @param  new display style
  * @retval none
  */
void 				Display_Set_DotStyle( uint8_t val ){
	uint8_t style;

	if( val != DOTS_STATIC_ON 	&&
		val != DOTS_STATIC_OFF 	&&
		val != DOTS_BLINK 		&&
		val != DOTS_CUSTOM 		&&
		val != DOTS_FADEIO 	)		style = DOTS_FADEIO;
	else							style = val;

	// check if anything has changed
	if( style == DotStyle ) return;

	DotStyle = style;
	// the pwm signal is driven accordingly to contents of 2 arrays (via DMA)
	// different styles are achieved by varying arrays contents and timer frequency
	uint32_t * top;
	uint32_t * bot;

	switch( DotStyle ){
	case DOTS_STATIC_ON:
		top = Dot_StaticOn;
		bot = Dot_StaticOn;
		break;
	case DOTS_STATIC_OFF:
		top = Dot_StaticOff;
		bot = Dot_StaticOff;
		break;
	case DOTS_BLINK:
		top = Dot_Blink;
		bot = Dot_Blink;
		break;
	case DOTS_CUSTOM:
		top = Dot_Custom;
		bot = Dot_Custom;
		break;
	case DOTS_FADEIO:
	default:
		top = Dot_FadeIO;
		bot = Dot_FadeIO;
		break;
	}

	// copy values to target arrays
	// TODO vary by brightness
	for(uint8_t i=0; i<DOT_BUFFER_SIZE; ++i){
		TopDot_Buffer[i] = top[i];
		BotDot_Buffer[i] = bot[i];
	}
}

/** @brief  Get current hours decimal digit
  * @param  none
  * @retval current digit
  */
uint8_t				Display_Get_HH( void ){
	if( TimeFormat == TIMEFORMAT_24 ){
		return ((RTC_Time.Hours & 0xF0)>>4);
	}else{
		return(((RTC_Time.Hours%12) & 0xF0)>>4);
	}
}

/** @brief  Get current hours ones digit
  * @param  none
  * @retval current digit
  */
uint8_t				Display_Get_HL( void ){
	if( TimeFormat == TIMEFORMAT_24 ){
		return  (RTC_Time.Hours & 0x0F);
	}else{
		return ((RTC_Time.Hours%12) & 0x0F);
	}
}

/** @brief  Get current minutes decimal digit
  * @param  none
  * @retval current digit
  */
uint8_t				Display_Get_MH( void ){
	if( TimeFormat == TIMEFORMAT_24 ){
		return ((RTC_Time.Minutes & 0xF0)>>4);
	}else{
		return ((RTC_Time.Minutes & 0xF0)>>4);
	}
}

/** @brief  Get current minutes ones digit
  * @param  none
  * @retval current digit
  */
uint8_t				Display_Get_ML( void ){
	if( TimeFormat == TIMEFORMAT_24 ){
		return  (RTC_Time.Minutes & 0x0F);
	}else{
		return  (RTC_Time.Minutes & 0x0F);
	}
}

/** @brief  Get current seconds decimal digit
  * @param  none
  * @retval current digit
  */
uint8_t				Display_Get_SH( void ){
	if( TimeFormat == TIMEFORMAT_24 ){
		return ((RTC_Time.Seconds & 0xF0)>>4);
	}else{
		return ((RTC_Time.Seconds & 0xF0)>>4);
	}
}

/** @brief  Get current seconds ones digit
  * @param  none
  * @retval current digit
  */
uint8_t				Display_Get_SL( void ){
	if( TimeFormat == TIMEFORMAT_24 ){
		return  (RTC_Time.Seconds & 0x0F);
	}else{
		return  (RTC_Time.Seconds & 0x0F);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
