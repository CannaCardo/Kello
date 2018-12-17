/*
 * EEprom.c
 *
 *  Created on: 28 paz 2018
 *      Author: Krzysztof Belewicz
 *
 */


#include "EEprom.h"
#include "Defines.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

// this flag indicates hardware error detection
// if set, all EEprom activities are disabled
uint8_t		EEPROM_HWERROR;

uint16_t 	RefreshRate;
uint8_t		Brightness;
uint8_t		TimeFormat;
uint8_t		DotStyle;

uint32_t 	Dot_Custom[DOT_BUFFER_SIZE];

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

void				EEprom_RestoreDefault( void ){
	RefreshRate = 	DEF_REFRESHRATE;
	Brightness	= 	DEF_BRIGHTNESS;
	TimeFormat	=  	DEF_TIMEFORMAT;
	DotStyle	=	DEF_DOTSTYLE;

	for(uint32_t i=0; i<DOT_BUFFER_SIZE; ++i) Dot_Custom[i] = 0;
}

/** @brief  Initialize EEprom routine
  * @param  hi2c I2C handle
  * @retval status
  */
HAL_StatusTypeDef	EEprom_Init( void ){
	HAL_StatusTypeDef 	ret = HAL_OK;
	uint8_t buf;
	EEPROM_HWERROR = 0;

	EEprom_RestoreDefault();
	// Check if the device is powered up for the first time
	buf = EEprom_GetByte(0x00);

	if( buf == 0xFF ){	// first time power up
		//push default to EEprom
		ret |= EEprom_Push();
		ret |= EEprom_PutByte(0x00, FIRSTPWRUPVAL);
	}else if(buf == FIRSTPWRUPVAL){		// Standard power up
		ret |= EEprom_Pull();
	}else{ 						// Invalid value
		EEPROM_HWERROR = 1;
		ret = 1;
	}

	if( ret )	EEPROM_HWERROR = ret;
	return ret;
}

/** @brief  returs hardware state
  * @param  none
  * @retval status
  */
HAL_StatusTypeDef	EEprom_HWState( void ){
	return (HAL_StatusTypeDef)EEPROM_HWERROR;
}

/** @brief  Pull all settings from EEprom
  * @param  none
  * @retval status
  */
HAL_StatusTypeDef	EEprom_Pull( void ){
	HAL_StatusTypeDef 	ret = HAL_OK;
	uint8_t 			buf;
	uint16_t 			val;

	if( EEPROM_HWERROR ) return HAL_ERROR;

	HAL_GPIO_WritePin(WP_GPIO_Port, WP_Pin, GPIO_PIN_SET);

	ret |= HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS_R, 0x01, 1, &buf, 1, 100);
	val = buf<<8;
	ret |= HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS_R, 0x02, 1, &buf, 1, 100);
	val = val | buf;

	if( EEprom_Set_RefreshRate( val ) ){
		// invalid value read
		// set default
		RefreshRate = 	DEF_REFRESHRATE;
		ret |= HAL_ERROR;
	}

	ret |= HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS_R, 0x03, 1, &buf, 1, 100);
	if( EEprom_Set_Brightness( buf ) ){
		// invalid value read
		// set default
		Brightness = 	DEF_BRIGHTNESS;
		ret |= HAL_ERROR;
	}

	ret |= HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS_R, 0x04, 1, &buf, 1, 100);
	if( EEprom_Set_TimeFormat( buf ) ){
		// invalid value read
		// set default
		TimeFormat = 	DEF_TIMEFORMAT;
		ret |= HAL_ERROR;
	}

	ret |= HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS_R, 0x05, 1, &buf, 1, 100);
	if( EEprom_Set_DotStyle( buf ) ){
		// invalid value read
		// set default
		DotStyle = 	DEF_DOTSTYLE;
		ret |= HAL_ERROR;
	}

	return ret;
}

/** @brief  Push all settings to EEprom
  * @param  none
  * @retval status
  */
HAL_StatusTypeDef	EEprom_Push( void ){
	HAL_StatusTypeDef 	ret = HAL_OK;
	uint8_t buf;
	if( EEPROM_HWERROR ) return HAL_ERROR;

	buf = (RefreshRate & 0xFF00)>>8;
	ret |= EEprom_PutByte(0x01, buf);
	buf = (RefreshRate & 0x00FF)>>0;
	ret |= EEprom_PutByte(0x02, buf);

	ret |= EEprom_PutByte(0x03, Brightness);
	ret |= EEprom_PutByte(0x04, TimeFormat);
	ret |= EEprom_PutByte(0x05, DotStyle);

	return ret;
}
/** @brief  Read one byte from EEprom
  * @param  address 0-511
  * @retval register value
  */
uint8_t 			EEprom_GetByte( uint16_t addr ){
	uint8_t val;

	HAL_I2C_Mem_Read(&hi2c1, (EEPROM_ADDRESS_R | (addr&0x100)>>7), (uint8_t)(addr & 0x0FF), 1, &val, 1, 100);

	return val;
}

/** @brief  Save one byte to EEprom
  * @param  address 0-511
  * @param  one byte of data
  * @retval status
  */
HAL_StatusTypeDef 	EEprom_PutByte( uint16_t addr, uint8_t data ){
	HAL_StatusTypeDef 	ret = HAL_OK;

	HAL_GPIO_WritePin(WP_GPIO_Port, WP_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);

	ret |= HAL_I2C_Mem_Write(&hi2c1, (EEPROM_ADDRESS_W | (addr&0x100)>>7), (uint8_t)(addr & 0x0FF), 1, &data, 1, 100);

	HAL_Delay(10);
	HAL_GPIO_WritePin(WP_GPIO_Port, WP_Pin, GPIO_PIN_SET);

	return ret;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

/** @brief  Get display refresh rate
  * @param  none
  * @retval RefreshRate in Hz
  */
uint16_t	EEprom_Get_RefreshRate( void ){
	return RefreshRate;
}

/** @brief  Get display brightness
  * @param  none
  * @retval Brightness in %
  */
uint8_t		EEprom_Get_Brightness( void ){
	return Brightness;
}

/** @brief  Get display time format
  * @param  none
  * @retval 12 or 24 time format
  */
uint8_t		EEprom_Get_TimeFormat( void ){
	return TimeFormat;
}

/** @brief  Get neon lamp display format
  * @param  none
  * @retval current setting
  */
uint8_t		EEprom_Get_DotStyle( void ){
	return DotStyle;
}

/** @brief  Get neon lamp custom transition
  * @param  none
  * @retval current setting
  */
uint32_t *	EEprom_Get_DotCustom( void ){
	return Dot_Custom;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

/** @brief  Set display refresh rate
  * @param  RefreshRate in Hz
  * @retval status
  */
HAL_StatusTypeDef	EEprom_Set_RefreshRate( uint16_t val ){
	if( val < MIN_REFRESHRATE || val > MAX_REFRESHRATE ) return HAL_ERROR;
	RefreshRate = val;

	return HAL_OK;
}

/** @brief  Set display brightness
  * @param  Brightness in %
  * @retval status
  */
HAL_StatusTypeDef	EEprom_Set_Brightness( uint8_t val ){
	if( val < MIN_BRIGHTNESS || val > MAX_BRIGHTNESS ) return HAL_ERROR;
	Brightness = val;

	return HAL_OK;
}

/** @brief  Set display time format
  * @param  12 or 24 time format
  * @retval status
  */
HAL_StatusTypeDef	EEprom_Set_TimeFormat( uint8_t val ){
	if( val != TIMEFORMAT_12 && val != TIMEFORMAT_24 ) return HAL_ERROR;
	TimeFormat = val;

	return HAL_OK;
}

/** @brief  Set neon lamp display format
  * @param  new setting
  * @retval status
  */
HAL_StatusTypeDef	EEprom_Set_DotStyle( uint8_t val ){
	if( val != DOTS_STATIC_ON 	&&
		val != DOTS_STATIC_OFF 	&&
		val != DOTS_BLINK 		&&
		val != DOTS_FADEIO 	) return HAL_ERROR;
	DotStyle = val;

	return HAL_OK;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
