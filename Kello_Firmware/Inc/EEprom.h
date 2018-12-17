/*
 * EEprom.h
 *
 *  Created on: 28 paz 2018
 *      Author: Krzysztof Belewicz
 *
 *     	 brief: This file takes care of non-volatile settings
 *     	 		The device is AT24C04D (4 Kbit - 512x8)
 *     	 		Factory default value is all 0xFF
 */


#ifndef EEPROM_KELLO
#define EEPROM_KELLO

#include "stm32l4xx_hal.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////


// ----------------------------------------------------------------------
//							MEMORY STRUCTURE
// ----------------------------------------------------------------------
//
//		Address		Type		Name			Desctiption
//		0x000		boolean		FirstPwrUp		0xFF at first powerup
//		0x001		uint16_t	RefreshRate		Range 10-1000 Hz
//		0x002
//		0x003		uint8_t		Brightness		Range 1-100%
//		0x004		uint8_t		TimeFormat		12 or 24
//		0x005		uint8_t		DotStyle		1-4 representing dot styles
//		0x006		uint16_t	DotCustom		200 values (400 Bytes)
//
// ----------------------------------------------------------------------


#define		EEPROM_ADDRESS		0b10100000 // already shifted
#define		EEPROM_ADDRESS_R	0b10100001
#define		EEPROM_ADDRESS_W	0b10100000
#define 	WP_Pin 				GPIO_PIN_8
#define 	WP_GPIO_Port 		GPIOA

#define		FIRSTPWRUPVAL		(0xAA)

#define		DOT_BUFFER_SIZE		(200)

extern I2C_HandleTypeDef hi2c1;

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

/** @brief  Initialize EEprom routine
  * @param  hi2c I2C handle
  * @retval status
  */
HAL_StatusTypeDef	EEprom_Init( void );

/** @brief  returs hardware state
  * @param  none
  * @retval status
  */
HAL_StatusTypeDef	EEprom_HWState( void );

/** @brief  Pull all settings from EEprom
  * @param  none
  * @retval status
  */
HAL_StatusTypeDef 	EEprom_Pull( void );

/** @brief  Push all settings to EEprom
  * @param  none
  * @retval status
  */
HAL_StatusTypeDef 	EEprom_Push( void );

/** @brief  Read one byte from EEprom
  * @param  address 0-511
  * @retval register value
  */
uint8_t 			EEprom_GetByte( uint16_t addr );

/** @brief  Save one byte to EEprom
  * @param  address 0-511
  * @param  one byte of data
  * @retval status
  */
HAL_StatusTypeDef 	EEprom_PutByte( uint16_t addr, uint8_t data );

/** @brief  Get display refresh rate
  * @param  none
  * @retval RefreshRate in Hz
  */
uint16_t			EEprom_Get_RefreshRate( void );

/** @brief  Get display brightness
  * @param  none
  * @retval Brightness in %
  */
uint8_t				EEprom_Get_Brightness( void );

/** @brief  Get display time format
  * @param  none
  * @retval 12 or 24 time format
  */
uint8_t				EEprom_Get_TimeFormat( void );

/** @brief  Get neon lamp display format
  * @param  none
  * @retval current setting
  */
uint8_t				EEprom_Get_DotStyle( void );

/** @brief  Get neon lamp custom transition
  * @param  none
  * @retval current setting
  */
uint32_t *			EEprom_Get_DotCustom( void );

/** @brief  Set display refresh rate
  * @param  RefreshRate in Hz
  * @retval status
  */
HAL_StatusTypeDef	EEprom_Set_RefreshRate( uint16_t val );

/** @brief  Set display brightness
  * @param  Brightness in %
  * @retval status
  */
HAL_StatusTypeDef	EEprom_Set_Brightness( uint8_t val );

/** @brief  Set display time format
  * @param  12 or 24 time format
  * @retval status
  */
HAL_StatusTypeDef	EEprom_Set_TimeFormat( uint8_t val );

/** @brief  Set neon lamp display format
  * @param  new setting
  * @retval status
  */
HAL_StatusTypeDef	EEprom_Set_DotStyle( uint8_t val );

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

#endif // EEPROM_KELLO
