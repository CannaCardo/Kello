/*
 * Display.h
 *
 *  Created on: 28 paz 2018
 *      Author: Krzysztof Belewicz
 *
 *     	 brief: This file takes care of Nixie Display
 */


#ifndef DISPLAY_KELLO
#define DISPLAY_KELLO

#include "Defines.h"
#include "stm32l4xx_hal.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////


// high voltage converter enable pin
#define 	HV_Enable_Pin 		GPIO_PIN_2
#define 	HV_Enable_GPIO_Port GPIOB

// neon lamp blink parameters
#define		DOT_BLINK_ONTIME	(1000) //ms
#define		DOT_BLINK_OFFTIME	(1000) //ms

// neon lamp fade in/out parameters
#define		DOT_FADEIO_STABLE0	(100) //ms
#define		DOT_FADEIO_RISE		(900) //ms
#define		DOT_FADEIO_STABLE1	(100) //ms
#define		DOT_FADEIO_FALL		(900) //ms
#define		DOT_FADEIO_TRANS	(DOT_FADEIO_SIN)

// neon lamp calibration and timer parameters
#define		DOT_FULLBRIGHTNESS	(DOT_COUNTER)	// to be tuned with nixie lamps
#define		DOT_FREQ			(100) //kHz
#define 	DOT_COUNTER			(1000)
#define		DOT_BUFFER_SIZE		(200)

#define		TIMCHANNEL_OFFTIME	TIM_CHANNEL_1
#define		TIMCHANNEL_TOP_DOT 	TIM_CHANNEL_4
#define		TIMCHANNEL_BOT_DOT	TIM_CHANNEL_3
#define		DMACHANNEL_TOP_DOT	TIM_DMA_ID_CC1
#define		DMACHANNEL_BOT_DOT	TIM_DMA_ID_CC1

extern RTC_HandleTypeDef hrtc;

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

/** @brief  Initialize Display routine
  * @param  hrtc RTC handle
  * @param  htim TIM handle for driving Nixie refresh routine
  * @param  htim TIM handle for driving Neon lamps PWM signal
  * @retval status
  */
HAL_StatusTypeDef	Display_Init( 	TIM_HandleTypeDef * htim_Nixie_,
									TIM_HandleTypeDef * htim_Dots_ );

/** @brief  Enables high voltage converter
  * @param  none
  * @retval none
  */
void 				Display_Start( void );

/** @brief  Disables high voltage converter
  * @param  none
  * @retval none
  */
void 				Display_Stop( void );

/** @brief  Set display refresh rate
  * @param  Specifies the Refresh rate of Nixie Tubes in Hz.
  *          This parameter can take value from range of 10 - 1000
  * @retval none
  */
void 				Display_Set_RefreshRate( uint16_t freq );

/** @brief  Set display Brightness
  * @param  Specifies the brightness of Nixie Tubes in %.
  *          This parameter can take value from range of 1 - 100
  * @note  	Due to ghosting (lamp to lamp) brightness is limited to 70 - this ensures some dead time in-between
  * @retval none
  */
void 				Display_Set_Brightness( uint8_t val );

/** @brief  Set display Time format
  * @param  12 or 24 time format
  * @retval none
  */
void 				Display_Set_TimeFormat( uint8_t val );

/** @brief  Set neon lamp custom style
  * @param  custom style vector
  * @retval none
  */
void 				Display_Set_DotCustom( uint32_t * vect );

/** @brief  Set neon lamp style
  * @param  new display style
  * @retval none
  */
void 				Display_Set_DotStyle( uint8_t val );

uint8_t				Display_Get_HH( void );
uint8_t				Display_Get_HL( void );
uint8_t				Display_Get_MH( void );
uint8_t				Display_Get_ML( void );
uint8_t				Display_Get_SH( void );
uint8_t				Display_Get_SL( void );

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

#endif // DISPLAY_KELLO
