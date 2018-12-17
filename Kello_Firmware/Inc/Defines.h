/*
 * Defines.h
 *
 *  Created on: 28 paz 2018
 *      Author: Krzysztof Belewicz
 *
 *     	 brief: parametric define statements used by multiple files
 */


#ifndef DEFINES_KELLO
#define DEFINES_KELLO

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////


#define USB_UPDATE_IDLE			(0)
#define USB_UPDATE_VALUE		(1)
#define USB_UPDATE_INVALID_ARG	(2)

#define HELP_NONE				(0)
#define HELP_LIST_COMMANDS 		(1)
#define HELP_SET_TIME			(2)
#define HELP_SET_TIME_FORMAT	(3)
#define HELP_SET_REFRESH_RATE	(4)
#define HELP_SET_BRIGHTNESS		(5)
#define HELP_SET_DOT_STYLE		(6)
#define HELP_SET_CUSTOM_STYLE	(7)
#define HELP_SAVE_SETTINGS		(8)
#define HELP_READ_EEPROM		(9)
#define HELP_SAVE_EEPROM		(10)
#define HELP_INFO				(11)
#define HELP_HELP				(12)
#define HELP_INVALID_ARG		(99)

// neon lamp styles
#define		DOTS_STATIC_ON		(0x01)
#define		DOTS_STATIC_OFF		(0x02)
#define		DOTS_BLINK			(0x03)
#define		DOTS_CUSTOM			(0x04)
#define		DOTS_FADEIO			(0x05)

// Default values
#define 	DEF_REFRESHRATE		(100)
#define 	DEF_BRIGHTNESS		(70)
#define 	DEF_TIMEFORMAT		(TIMEFORMAT_24)
#define 	DEF_DOTSTYLE		(DOTS_FADEIO)

// limiting values
#define		MAX_REFRESHRATE		(1000)
#define		MIN_REFRESHRATE		(10)
#define		MAX_BRIGHTNESS		(100)
#define		MIN_BRIGHTNESS		(1)

#define		TIMEFORMAT_12		(12)
#define		TIMEFORMAT_24		(24)

#define		DOT_FADEIO_LINEAR	(0)
#define		DOT_FADEIO_SIN		(1)
#define		DOT_FADEIO_LOG		(2)
#define		DOT_FADEIO_INVLOG	(3)


#define		FIRMWARE_VER		"11.12._1.0"

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

#endif // DEFINES_KELLO
