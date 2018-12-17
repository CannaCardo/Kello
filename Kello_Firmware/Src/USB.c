/*
 * USB.c
 *
 *  Created on: 10 lis 2018
 *      Author: canna_000
 */

#include "USB.h"
#include "Display.h"
#include "EEprom.h"
#include "Defines.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

HAL_StatusTypeDef TX_Buffer_Status;

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

/** @brief  Initialize USB routine
  * @param  none
  * @retval status
  */
HAL_StatusTypeDef	USB_Init( void ){
	HAL_StatusTypeDef 	ret = HAL_OK;

	TX_Buffer_Status = HAL_BUSY;

	USB_Update_Time 		= USB_UPDATE_IDLE;
	USB_Update_Brightness 	= USB_UPDATE_IDLE;
	USB_Update_RefFreq 		= USB_UPDATE_IDLE;
	USB_Update_TimeFormat 	= USB_UPDATE_IDLE;
	USB_Update_DotStyle 	= USB_UPDATE_IDLE;
	USB_SaveSettings 		= USB_UPDATE_IDLE;
	USB_EEprom_read 		= USB_UPDATE_IDLE;
	USB_EEprom_save 		= USB_UPDATE_IDLE;
	USB_Info 				= USB_UPDATE_IDLE;
	USB_UnknownRequest 		= USB_UPDATE_IDLE;
	USB_Help 				= HELP_NONE;

	USB_TX_Size = 0;
	USB_RX_Size = 0;
	TX_Buffer_Status = HAL_OK;

	return ret;
}

/** @brief  Get TX buffer status
  * @param  none
  * @retval status
  */
HAL_StatusTypeDef	USB_TXBuffer_Get_Status( void ){
	return TX_Buffer_Status;
}

/** @brief  Set TX buffer status
  * @param  status
  * @retval none
  */
void				USB_TXBuffer_Set_Status( HAL_StatusTypeDef status ){
	TX_Buffer_Status = status;
}

/** @brief  Returns true if anything has been received
  * @param  none
  * @retval 1 if true, 0 otherwise
  */
uint8_t				USB_RX_IsReceived( void ){
	if( USB_Update_Time 		!= USB_UPDATE_IDLE ||
		USB_Update_Brightness 	!= USB_UPDATE_IDLE ||
		USB_Update_RefFreq 		!= USB_UPDATE_IDLE ||
		USB_Update_TimeFormat 	!= USB_UPDATE_IDLE ||
		USB_Update_DotStyle 	!= USB_UPDATE_IDLE ||
		USB_SaveSettings 		!= USB_UPDATE_IDLE ||
		USB_EEprom_read 		!= USB_UPDATE_IDLE ||
		USB_EEprom_save 		!= USB_UPDATE_IDLE ||
		USB_Info 				!= USB_UPDATE_IDLE ||
		USB_UnknownRequest 		!= USB_UPDATE_IDLE ||
		USB_Help 				!= HELP_NONE
		) 	return 1;
	else 	return 0;
}


// ------------------------------------------------------------------

/** @brief  Sends responses to incomming USB communication and clears flags
  * @param  none
  * @retval none
  */
void				USB_RX_Respond( void ){
	uint8_t *	buf = USB_TX_Data;
	uint32_t * 	Len = &USB_TX_Size;
	uint8_t		EE_value = 0;
	uint8_t		EE_error = 0;

	// Update Time
	if( USB_Update_Time == USB_UPDATE_VALUE ){
		RTC_TimeTypeDef RTC_Time;
		RTC_DateTypeDef RTC_Date;

		HAL_RTC_GetTime(&hrtc, &RTC_Time, RTC_FORMAT_BCD);
		HAL_RTC_GetDate(&hrtc, &RTC_Date, RTC_FORMAT_BCD);

		RTC_Time.Hours   = 	USB_Hours;
		RTC_Time.Minutes = 	USB_Minutes;
		RTC_Time.Seconds = 	USB_Seconds;

		HAL_RTC_SetTime(&hrtc, &RTC_Time, RTC_FORMAT_BCD);
		HAL_RTC_SetDate(&hrtc, &RTC_Date, RTC_FORMAT_BCD);
	}
	// Update Brightness
	if( USB_Update_Brightness == USB_UPDATE_VALUE ){
		Display_Set_Brightness( USB_Brightness );
		EEprom_Set_Brightness( USB_Brightness );
	}
	// Update Refresh Rate
	if( USB_Update_RefFreq == USB_UPDATE_VALUE ){
		Display_Set_RefreshRate( USB_RefreshFreq );
		EEprom_Set_RefreshRate( USB_RefreshFreq );
	}
	// Update Time Format
	if( USB_Update_TimeFormat == USB_UPDATE_VALUE ){
		Display_Set_TimeFormat( USB_TimeFormat );
		EEprom_Set_TimeFormat( USB_TimeFormat );
	}
	// Update Dot Style
	if( USB_Update_DotStyle == USB_UPDATE_VALUE ){
		Display_Set_DotStyle( USB_DotStyle );
		EEprom_Set_DotStyle( USB_DotStyle );
	}
	// Save Settings to EEprom
	if( USB_SaveSettings == USB_UPDATE_VALUE ){
		EEprom_Push();
	}
	// Read byte from EEprom
	if( USB_EEprom_read == USB_UPDATE_VALUE ){
		EE_value = EEprom_GetByte(USB_EEprom_addr);
	}
	// Save byte to EEprom
	if( USB_EEprom_save == USB_UPDATE_VALUE ){
		EE_error = EEprom_PutByte(USB_EEprom_addr, USB_EEprom_data);
	}
	if( USB_Info == USB_UPDATE_VALUE ){
		USB_TX_Info();
		USB_Info = USB_UPDATE_IDLE;
	}

	// Wait for the TX buffer to be unused
	while( USB_TXBuffer_Get_Status() != HAL_OK );
	USB_TXBuffer_Set_Status( HAL_BUSY );

	*Len = 0;
	if( USB_Update_Time == USB_UPDATE_VALUE ){
		*Len = sprintf( (char*)buf + *Len, "Time Set\r\n") + *Len;
		USB_Update_Time = USB_UPDATE_IDLE;
	}else if( USB_Update_Time == USB_UPDATE_INVALID_ARG ){
		*Len = sprintf( (char*)buf + *Len, "Invalid parameter: Set Time *\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tIn case of problems type: help Set Time\r\n") + *Len;
		USB_Update_Time = USB_UPDATE_IDLE;
	}
	if( *Len != 0 ) CDC_Transmit_FS(buf, *Len);

	*Len = 0;
	if( USB_Update_Brightness == USB_UPDATE_VALUE ){
		*Len = sprintf( (char*)buf + *Len, "Brightness Set\r\n") + *Len;
		USB_Update_Brightness = USB_UPDATE_IDLE;
	}else if( USB_Update_Brightness == USB_UPDATE_INVALID_ARG ){
		*Len = sprintf( (char*)buf + *Len, "Invalid parameter: Set Brightness *\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tIn case of problems type: help Set Brightness\r\n") + *Len;
		USB_Update_Brightness = USB_UPDATE_IDLE;
	}
	if( *Len != 0 ) CDC_Transmit_FS(buf, *Len);

	*Len = 0;
	if( USB_Update_RefFreq == USB_UPDATE_VALUE ){
		*Len = sprintf( (char*)buf + *Len, "Refresh Rate Set\r\n") + *Len;
		USB_Update_RefFreq = USB_UPDATE_IDLE;
	}else if( USB_Update_RefFreq == USB_UPDATE_INVALID_ARG ){
		*Len = sprintf( (char*)buf + *Len, "Invalid parameter: Set Refresh Rate *\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tIn case of problems type: help Set Refresh Rate\r\n") + *Len;
		USB_Update_RefFreq = USB_UPDATE_IDLE;
	}
	if( *Len != 0 ) CDC_Transmit_FS(buf, *Len);

	*Len = 0;
	if( USB_Update_TimeFormat == USB_UPDATE_VALUE ){
		*Len = sprintf( (char*)buf + *Len, "Time Format Set\r\n") + *Len;
		USB_Update_TimeFormat = USB_UPDATE_IDLE;
	}else if( USB_Update_TimeFormat == USB_UPDATE_INVALID_ARG ){
		*Len = sprintf( (char*)buf + *Len, "Invalid parameter: Set Time Format *\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tIn case of problems type: help Set Time Format\r\n") + *Len;
		USB_Update_TimeFormat = USB_UPDATE_IDLE;
	}
	if( *Len != 0 ) CDC_Transmit_FS(buf, *Len);

	*Len = 0;
	if( USB_Update_DotStyle == USB_UPDATE_VALUE ){
		*Len = sprintf( (char*)buf + *Len, "Dot Style Set\r\n") + *Len;
		USB_Update_DotStyle = USB_UPDATE_IDLE;
	}else if( USB_Update_DotStyle == USB_UPDATE_INVALID_ARG ){
		*Len = sprintf( (char*)buf + *Len, "Invalid parameter: Set Dot Style *\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tIn case of problems type: help Set Dot Style\r\n") + *Len;
		USB_Update_DotStyle = USB_UPDATE_IDLE;
	}
	if( *Len != 0 ) CDC_Transmit_FS(buf, *Len);

	*Len = 0;
	if( USB_SaveSettings == USB_UPDATE_VALUE ){
		*Len = sprintf( (char*)buf + *Len, "Settings Saved\r\n") + *Len;
		USB_SaveSettings = USB_UPDATE_IDLE;
	}else if( USB_SaveSettings == USB_UPDATE_INVALID_ARG ){
		*Len = sprintf( (char*)buf + *Len, "Invalid parameter: Save Settings\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tIn case of problems type: help Save Settings\r\n") + *Len;
		USB_SaveSettings = USB_UPDATE_IDLE;
	}
	if( *Len != 0 ) CDC_Transmit_FS(buf, *Len);

	*Len = 0;
	if( USB_EEprom_read == USB_UPDATE_VALUE ){
		*Len = sprintf( (char*)buf + *Len, "EEprom read: 0x%02X\r\n", EE_value) + *Len;
		USB_EEprom_read = USB_UPDATE_IDLE;
	}else if( USB_EEprom_read == USB_UPDATE_INVALID_ARG ){
		*Len = sprintf( (char*)buf + *Len, "Invalid parameter: EEprom read\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tIn case of problems type: help eeprom read\r\n") + *Len;
		USB_EEprom_read = USB_UPDATE_IDLE;
	}
	if( *Len != 0 ) CDC_Transmit_FS(buf, *Len);

	*Len = 0;
	if( USB_EEprom_save == USB_UPDATE_VALUE ){
		if( EE_error )	*Len = sprintf( (char*)buf + *Len, "EEprom save error 0x%02X\r\n", EE_error) + *Len;
		else			*Len = sprintf( (char*)buf + *Len, "EEprom succesfully saved\r\n") + *Len;
		USB_EEprom_save = USB_UPDATE_IDLE;
	}else if( USB_EEprom_save == USB_UPDATE_INVALID_ARG ){
		*Len = sprintf( (char*)buf + *Len, "Invalid parameter: EEprom save\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tIn case of problems type: help eeprom save\r\n") + *Len;
		USB_EEprom_save = USB_UPDATE_IDLE;
	}
	if( *Len != 0 ) CDC_Transmit_FS(buf, *Len);

	*Len = 0;
	if( USB_UnknownRequest ){
		*Len = sprintf( (char*)buf + *Len, "Unknown command\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tIn case of problems type: help\r\n") + *Len;
		USB_UnknownRequest = 0;
	}
	if( *Len != 0 ) CDC_Transmit_FS(buf, *Len);

	*Len = 0;
	switch( USB_Help ){
	case HELP_LIST_COMMANDS:
		*Len = sprintf( (char*)buf + *Len, "Command list:\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tset time <HH:MM:SS>\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tset time format <12/24>\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tset refresh rate <val Hz>\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tset brightness <val %%>\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tset dot style <dot style>\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tset custom dot style <200 16bit values>\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tsave settings\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tread eeprom <Addr>\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tsave eeprom <Addr> <Data>\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tinfo\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\thelp\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\thelp <Command>\r\n") + *Len;
		break;
	case HELP_HELP:
		*Len = sprintf( (char*)buf + *Len, "help - lists available commands, every command is terminated with CRLF\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tcommands are case sensitive, all lowercase\r\n") + *Len;
		break;
	case HELP_READ_EEPROM:
		*Len = sprintf( (char*)buf + *Len, "read eeprom <Addr> - Reads one byte at <addr> of 4Kbit EEprom. \r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tAddress is specified in plain text as hex number, range 0-511\r\n") + *Len;
		break;
	case HELP_SAVE_EEPROM:
		*Len = sprintf( (char*)buf + *Len, "save eeprom <Addr> <Data> - Saves <Data> at <Addr> of 4Kbit EEprom. \r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tAddress is specified in plain text as hex number, range 0-511\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tData is one byte separated from address with space\r\n") + *Len;
		break;
	case HELP_SAVE_SETTINGS:
		*Len = sprintf( (char*)buf + *Len, "save settings - Saves current settings in EEprom\r\n") + *Len;
		break;
	case HELP_SET_BRIGHTNESS:
		*Len = sprintf( (char*)buf + *Len, "set brightness <val>%% - sets nixie brightness, decimal range 1 to 100%%\r\n") + *Len;
		break;
	case HELP_SET_CUSTOM_STYLE:
		*Len = sprintf( (char*)buf + *Len, "set custom dot style <200 16bit values> - sets custom dot pattern\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tfollowed by 400 bytes of data, big endian, MSB first\r\n") + *Len;
		break;
	case HELP_SET_DOT_STYLE:
		*Len = sprintf( (char*)buf + *Len, "set dot style <dot style> - sets dot pattern. Possible styles:\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tblink\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tcustom\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tstatic on\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tstatic off\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tfade in-out\r\n") + *Len;
		break;
	case HELP_SET_REFRESH_RATE:
		*Len = sprintf( (char*)buf + *Len, "set refresh rate <val>Hz - sets nixie refresh rate, decimal range 10 to 1000Hz\r\n") + *Len;
		break;
	case HELP_SET_TIME:
		*Len = sprintf( (char*)buf + *Len, "set time <HH:MM:SS> - sets current time (24h format)\r\n") + *Len;
		break;
	case HELP_SET_TIME_FORMAT:
		*Len = sprintf( (char*)buf + *Len, "set time format <12/24> - sets either 12 or 24h time format\r\n") + *Len;
		break;
	case HELP_INFO:
		*Len = sprintf( (char*)buf + *Len, "info - displays system info, automagically sent every 1s\r\n") + *Len;
		break;
	case HELP_INVALID_ARG:
	default:
		*Len = sprintf( (char*)buf + *Len, "Invalid parameter: help\r\n") + *Len;
		*Len = sprintf( (char*)buf + *Len, "\tIn case of problems type: help help\r\n") + *Len;
		break;
	}
	USB_Help = HELP_NONE;
	if( *Len != 0 ) CDC_Transmit_FS(buf, *Len);

	USB_TXBuffer_Set_Status( HAL_OK );
}

void				USB_TX_Info( void ){
	// Wait for the TX buffer to be unused
	while( USB_TXBuffer_Get_Status() != HAL_OK );
	USB_TXBuffer_Set_Status( HAL_BUSY );

	// TX frame:
	// Current Time: HH:MM:SS
	// Time Format:  12/24
	// Refresh Rate: XXXXHz
	// Brightness:   XX%
	// Dot Style:    current dot style
	// Firmware ver: current firmware version
	// Error messages

	USB_TX_Size = sprintf((char*)USB_TX_Data, "Current Time: %d%d:%d%d:%d%d\n\r",   Display_Get_HH(),
																				    Display_Get_HL(),
																				    Display_Get_MH(),
																				    Display_Get_ML(),
																				    Display_Get_SH(),
																				    Display_Get_SL());
	USB_TX_Size = sprintf((char*)USB_TX_Data+USB_TX_Size, "Time Format:  %d\n\r",   EEprom_Get_TimeFormat())  + USB_TX_Size;
	USB_TX_Size = sprintf((char*)USB_TX_Data+USB_TX_Size, "Refresh Rate: %dHz\n\r", EEprom_Get_RefreshRate()) + USB_TX_Size;
	USB_TX_Size = sprintf((char*)USB_TX_Data+USB_TX_Size, "Brightness:   %d%%\n\r", EEprom_Get_Brightness())  + USB_TX_Size;

	switch( EEprom_Get_DotStyle() ){
	case DOTS_STATIC_ON:
		USB_TX_Size = sprintf((char*)USB_TX_Data+USB_TX_Size, "Dot Style:    Static On\n\r") + USB_TX_Size;
		break;
	case DOTS_STATIC_OFF:
		USB_TX_Size = sprintf((char*)USB_TX_Data+USB_TX_Size, "Dot Style:    Static Off\n\r") + USB_TX_Size;
		break;
	case DOTS_BLINK:
		USB_TX_Size = sprintf((char*)USB_TX_Data+USB_TX_Size, "Dot Style:    Blink\n\r") + USB_TX_Size;
		break;
	case DOTS_FADEIO:
	default:
			USB_TX_Size = sprintf((char*)USB_TX_Data+USB_TX_Size, "Dot Style:    Fade in-out\n\r") + USB_TX_Size;
			break;
	}

	USB_TX_Size = sprintf((char*)USB_TX_Data+USB_TX_Size, "Firmware ver: %s\n\r---\n\r",  FIRMWARE_VER) + USB_TX_Size;

	switch( HAL_RTC_GetState(&hrtc) ){
	case HAL_RTC_STATE_RESET:
		USB_TX_Size = sprintf((char*)USB_TX_Data+USB_TX_Size, "RTC state: RESET\n\r") + USB_TX_Size;
		break;
	case HAL_RTC_STATE_READY:
		USB_TX_Size = sprintf((char*)USB_TX_Data+USB_TX_Size, "RTC state: READY\n\r") + USB_TX_Size;
		break;
	case HAL_RTC_STATE_BUSY:
		USB_TX_Size = sprintf((char*)USB_TX_Data+USB_TX_Size, "RTC state: BUSY\n\r") + USB_TX_Size;
		break;
	case HAL_RTC_STATE_TIMEOUT:
		USB_TX_Size = sprintf((char*)USB_TX_Data+USB_TX_Size, "RTC state: TIMEOUT\n\r") + USB_TX_Size;
		break;
	case HAL_RTC_STATE_ERROR:
	default:
		USB_TX_Size = sprintf((char*)USB_TX_Data+USB_TX_Size, "RTC state: ERROR\n\r") + USB_TX_Size;
		break;
	}

	switch( DisplayStatus ){
	case HAL_OK:
		USB_TX_Size = sprintf((char*)USB_TX_Data+USB_TX_Size, "TIM state: OK\n\r") + USB_TX_Size;
		break;
	case HAL_ERROR:
		USB_TX_Size = sprintf((char*)USB_TX_Data+USB_TX_Size, "TIM state: ERROR\n\r") + USB_TX_Size;
		break;
	case HAL_BUSY:
		USB_TX_Size = sprintf((char*)USB_TX_Data+USB_TX_Size, "TIM state: BUSY\n\r") + USB_TX_Size;
		break;
	case HAL_TIMEOUT:
	default:
		USB_TX_Size = sprintf((char*)USB_TX_Data+USB_TX_Size, "TIM state: TIMEOUT\n\r") + USB_TX_Size;
		break;
	}

	switch( EEpromStatus ){
	case HAL_OK:
		USB_TX_Size = sprintf((char*)USB_TX_Data+USB_TX_Size, "I2C state: OK\n\r") + USB_TX_Size;
		break;
	case HAL_ERROR:
		USB_TX_Size = sprintf((char*)USB_TX_Data+USB_TX_Size, "I2C state: ERROR\n\r") + USB_TX_Size;
		break;
	case HAL_BUSY:
		USB_TX_Size = sprintf((char*)USB_TX_Data+USB_TX_Size, "I2C state: BUSY\n\r") + USB_TX_Size;
		break;
	case HAL_TIMEOUT:
	default:
		USB_TX_Size = sprintf((char*)USB_TX_Data+USB_TX_Size, "I2C state: TIMEOUT\n\r") + USB_TX_Size;
		break;
	}

	// Error messages
	if( EEprom_HWState() != HAL_OK ) USB_TX_Size = sprintf((char*)USB_TX_Data+USB_TX_Size, "EEprom error 0x0%d\n\r",EEprom_HWState()) + USB_TX_Size;


	CDC_Transmit_FS(USB_TX_Data, USB_TX_Size);
	USB_TXBuffer_Set_Status( HAL_OK );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////


