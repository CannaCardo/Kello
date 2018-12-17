/*
 * USB.h
 *
 *  Created on: 28 paz 2018
 *      Author: Krzysztof Belewicz
 *
 *     	 brief: This file takes care of USB communication
 */


#ifndef USB_KELLO
#define USB_KELLO

#include "stm32l4xx_hal.h"
#include "usbd_cdc_if.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////


#define 	USB_TX_BUFFER_SIZE 		400
#define 	USB_RX_BUFFER_SIZE 		100 // TODO unused probably


////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////


uint8_t		USB_TX_Data[USB_TX_BUFFER_SIZE];
uint32_t	USB_TX_Size;

uint8_t		USB_RX_Data[USB_RX_BUFFER_SIZE];
uint32_t	USB_RX_Size;


uint8_t     USB_Update_Time;
uint8_t     USB_Update_Brightness;
uint8_t     USB_Update_RefFreq;
uint8_t     USB_Update_TimeFormat;
uint8_t     USB_Update_DotStyle;
uint8_t	 	USB_SaveSettings;
uint8_t	 	USB_UnknownRequest;

uint8_t		USB_EEprom_read;
uint8_t	 	USB_EEprom_save;
uint8_t	 	USB_EEprom_addr;
uint8_t	 	USB_EEprom_data;

uint8_t	 	USB_Info;

uint8_t	 	USB_Help;

uint8_t     USB_Hours;
uint8_t     USB_Minutes;
uint8_t     USB_Seconds;

uint8_t     USB_TimeFormat;
uint8_t     USB_Brightness;
uint8_t     USB_DotStyle;
uint16_t    USB_RefreshFreq;

extern HAL_StatusTypeDef DisplayStatus;
extern HAL_StatusTypeDef EEpromStatus;

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

HAL_StatusTypeDef	USB_Init( void );

HAL_StatusTypeDef	USB_TXBuffer_Get_Status( void );
void				USB_TXBuffer_Set_Status( HAL_StatusTypeDef status );

uint8_t				USB_RX_IsReceived( void );

void				USB_RX_Respond( void );
void				USB_TX_Info( void );

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

#endif // USB_KELLO
