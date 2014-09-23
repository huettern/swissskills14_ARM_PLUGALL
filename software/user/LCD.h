/*****************************************************************************
* Project:            PLUGALL
* Target:             
* Type:               LCD controlls
* Version:            1.0
* Author:             Noah Huetter
* Creation-Date:      07.2014
******************************************************************************
* Modification History:
* 
* [1.0]     07.2014   NHU   first release
*
*****************************************************************************/  
#ifndef __LCD_H
#define __LCD_H




/*****************************************************************************
* Defines
*****************************************************************************/ 

/* COMMANDS */
#define LCD_CLEARDISPLAY 		0x01
#define LCD_RETURNHOME 			0x02
#define LCD_ENTRYMODESET 		0x04
#define LCD_DISPLAYCONTROL 	0x08
#define LCD_CURSORSHIFT 		0x10
#define LCD_FUNCTIONSET 		0x20
#define LCD_SETCGRAMADDR 		0x40
#define LCD_SETDDRAMADDR 		0x80


//LCD_DISPLAYCONTROL
#define LCD_ON							0x04
#define LCD_CURSOR					0x02
#define LCD_BLINK						0x01

/*****************************************************************************
* Public Functions
*****************************************************************************/ 
void LCD_fInit (void);

void LCD_fClearScreen (void);

void LCD_fSetCursor (uint8_t x_data, uint8_t y_data);
void LCD_fGoHome (void);

void LCD_fLCDCommand (uint8_t command);
void LCD_fLCDData (uint8_t data);

void LCD_printf(const char * __restrict string/*format*/, ...);

void LCD_fStartDataBlock (void);

void LCD_fStopDataBlock (void);


void LCD_fSetCustomChar (uint8_t location, uint8_t charmap[]);
void LCD_fPrintCustomChar (uint8_t location);

void LCD_fControlBacklight (uint8_t state);
	
	
	
	void fput_c (char c);
	
#endif
