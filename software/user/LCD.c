/*****************************************************************************
* Project:            PLUGALL
* Target:             
* Type:               LCD controlls for KS0066 compilant controller
* Version:            1.0
* Author:             Noah Huetter
* Creation-Date:      07.2014
******************************************************************************
* Modification History:
* 
* [1.0]     07.2014   NHU   first release
*
*****************************************************************************/  
#include "stm32f37x_conf.h"
#include "lcd.h"
#include "plugall.h"
#include "delay.h"


#include <stdio.h>
#include <stdarg.h>



//LCD commands
/* function set: set interface data length and number of display lines */
#define LCD_FUNCTION_4BIT_1LINE  0x20   /* 4-bit interface, single line, 5x7 dots */
#define LCD_FUNCTION_4BIT_2LINES 0x28   /* 4-bit interface, dual line,   5x7 dots */
#define LCD_FUNCTION_8BIT_1LINE  0x30   /* 8-bit interface, single line, 5x7 dots */
#define LCD_FUNCTION_8BIT_2LINES 0x38   /* 8-bit interface, dual line,   5x7 dots */

/* set entry mode: display shift on/off, dec/inc cursor move direction */
#define LCD_ENTRY_DEC            0x04   /* display shift off, dec cursor move dir */
#define LCD_ENTRY_DEC_SHIFT      0x05   /* display shift on,  dec cursor move dir */
#define LCD_ENTRY_INC            0x06   /* display shift off, inc cursor move dir */
#define LCD_ENTRY_INC_SHIFT      0x07   /* display shift on,  inc cursor move dir */

/* display on/off, cursor on/off, blinking char at cursor position */
#define LCD_DISP_OFF             0x08   /* display off                            */
#define LCD_DISP_ON              0x0C   /* display on, cursor off                 */
#define LCD_DISP_ON_BLINK        0x0D   /* display on, cursor off, blink char     */
#define LCD_DISP_ON_CURSOR       0x0E   /* display on, cursor on                  */
#define LCD_DISP_ON_CURSOR_BLINK 0x0F   /* display on, cursor on, blink char      */

/* move cursor/shift display */
#define LCD_MOVE_CURSOR_LEFT     0x10   /* move cursor left  (decrement)          */
#define LCD_MOVE_CURSOR_RIGHT    0x14   /* move cursor right (increment)          */
#define LCD_MOVE_DISP_LEFT       0x18   /* shift display left                     */
#define LCD_MOVE_DISP_RIGHT      0x1C   /* shift display right                    */

/* lines */
#define LCD_START_LINE1  0x00     /**< DDRAM address of first char of line 1 */
#define LCD_START_LINE2  0x40     /**< DDRAM address of first char of line 2 */


/*****************************************************************************
* Private Data
*****************************************************************************/   
uint8_t m_BusIsMine = 0;

uint8_t m_CurrenLine = 0;
uint8_t m_CurrenPos = 0;

uint8_t m_LCD_Backlight = 1;

uint8_t m_delay;

uint8_t m_DataBlockRunning = 0;

/*****************************************************************************
* Private Functions Prototypes
*****************************************************************************/ 
void fGetBus (void);

void fEnablePulse (void);

void fWrite (uint8_t cmd) ;
void fWriteCMD (uint8_t cmd);

void fNewLine (void);

//void fput_c (char c);
void fput_s(const char *s);


/*****************************************************************************
* Private Functions 
*****************************************************************************/ 
/*=======================================================================*//**
  @brief Disable Bus controll and sets Adress and data for LCD control
*//*========================================================================*/
void fGetBus (void)
{
	if(m_BusIsMine == 1) return;
	m_BusIsMine = 1;
	
	PLG_StopBusControl();
	
	GPIOA->BSRR = GPIO_Pin_3;						//disable bus
	
	PLG_fSetAdrBus(PLG_LCD);						//set address
	PLG_fSetDataDirection(PLG_OUTPUT);	//make bus Hi-Z to change adr
	PLG_fWriteDataBus(m_LCD_Backlight << 5);						//Backlight ON
	GPIOA->BRR = GPIO_Pin_3;						//set last adress bit
	
	
	//PLG_fWriteDataBus(0x00);
}

/*=======================================================================*//**
  @brief Releases Bus and continue Auto Bus controll
*//*========================================================================*/
void fReleaseBus (void)
{
	//don't release bus, if data block is running
	if(m_DataBlockRunning == 1) return;
	
	if(m_BusIsMine == 0) return;
	m_BusIsMine = 0;
	
	PLG_fWriteDataBus(m_LCD_Backlight << 5);						//Backlight ON
		
	
	m_delay = 100;			//wait min 450ns data setup time
	while(--m_delay);	//666ns measured
	
	GPIOA->BSRR = GPIO_Pin_3;						//disable address DOESNT WORK
	
	PLG_StartBusControl();
}

/*=======================================================================*//**
  @brief Outputs a positive pulse on Enable pin
*//*========================================================================*/
void fEnablePulse (void)
{
	GPIOB->BRR = GPIO_Pin_8;	//set Enable Low
	
	m_delay = 10;			//wait min 450ns data setup time
	while(--m_delay);	//666ns measured
	
	GPIOB->BSRR = GPIO_Pin_8;	//set Enable High
	
	m_delay = 10;			//wait min 450ns
	while(--m_delay);	//666ns measured
	
	GPIOB->BRR = GPIO_Pin_8;	//set Enable Low
	
	m_delay = 10;			//wait min 450ns
	while(--m_delay);	//666ns measured
}

/*=======================================================================*//**
  @brief Sends data to LCD
*//*========================================================================*/
void fWrite (uint8_t cmd) 
{	
	GPIOC->BRR = 0x03C0;	//clear 4 data bits
	GPIOC->BSRR = ((cmd >> 4) & 0x0F) << 6;	//set 4 data bits
	
	fEnablePulse();
	
	GPIOC->BRR = 0x03C0;	//clear 4 data bits
	GPIOC->BSRR = ((cmd) & 0x0F) << 6;	//set 4 data bits
	
	fEnablePulse();
	
	GPIOB->BRR = GPIO_Pin_8;	//set Enable Low
	
	m_delay = 0;			//wait min 450ns
	while(--m_delay);	//666ns measured
	//DLY_fDelayMs(5);	//Wait
}

/*=======================================================================*//**
  @brief Writes display cmd byte
*//*========================================================================*/
void fWriteCMD (uint8_t cmd)
{
	GPIOA->BRR = GPIO_Pin_11;	//RS = 0
	GPIOF->BRR = GPIO_Pin_6;	//RW = 0
	
	fWrite(cmd);
}

/*=======================================================================*//**
  @brief Writes Display data byte
*//*========================================================================*/
void fWriteData (uint8_t data)
{
	GPIOA->BSRR = GPIO_Pin_11;	//RS = 1
	GPIOF->BRR = GPIO_Pin_6;	//RW = 0
	
	fWrite(data);
}

/*=======================================================================*//**
  @brief Goes to next line
*//*========================================================================*/
void fNewLine (void)
{
	if(m_CurrenLine == 0) 
	{
		fWriteCMD(0x80 | (LCD_START_LINE2) );
		m_CurrenLine = 2;
	}
	else
	{
		fWriteCMD(0x80 | (LCD_START_LINE1) );
		m_CurrenLine = 1;
	}
		
}

/*=======================================================================*//**
  @brief Puts char on current display position
*//*========================================================================*/
void fput_c (char c)
{
	if (c=='\n')
	{
			fNewLine();
	}
	else
	{
		fWriteData((uint8_t)c);
		m_delay = 100;			//wait min 450ns
		while(--m_delay);	//666ns measured
		
	}
}


/*=======================================================================*//**
  @brief Display string without auto linefeed 
*//*========================================================================*/
void fput_s(const char *s)
{
    char c;

		c = *s;
	
	__disable_irq();
	fGetBus();
	
    while ( c ) 
		{
      fput_c(c);
			c = *++s;
    }
		
	fReleaseBus();
	__enable_irq();

}/* lcd_puts */


/*****************************************************************************
* Public Functions 
*****************************************************************************/ 
/*=======================================================================*//**
  @brief Init LCD, runtime 50ms
	@attention Interrupts PGL BUS!!
*//*========================================================================*/
void LCD_fInit (void)
{
	LCD_fStartDataBlock();
	fGetBus();
	
	
	DLY_fDelayMs(30);		//LCD Power-up time
	
	GPIOA->BRR = GPIO_Pin_11;	//RS = 0
	GPIOF->BRR = GPIO_Pin_6;	//RW = 0
	
	DLY_fDelayMs(1);		//
	
	/********* Init Display HERE **************/
	GPIOC->BSRR = GPIO_Pin_6;	//DB4
	GPIOC->BSRR = GPIO_Pin_7; //DB5
	
	DLY_fDelayMs(1);		//
	
	fEnablePulse();
	DLY_fDelayMs(5);	//Wait
	
	//repeat last command
	fEnablePulse();
	DLY_fDelayMs(5);	//Wait
	
	//repeat last command
	fEnablePulse();
	DLY_fDelayMs(5);	//Wait
	
	//configure for 4 bit mode
	GPIOC->BRR = GPIO_Pin_6;	//DB4
	DLY_fDelayMs(1);		//
	fEnablePulse();
	DLY_fDelayMs(5);	//Wait
	
	//Now configure LCD
	//FUNCTION SET
	fWriteCMD(LCD_FUNCTION_4BIT_2LINES);
	DLY_fDelayMs(1);		//
	fWriteCMD(LCD_DISP_OFF);
	DLY_fDelayMs(1);		//
  LCD_fClearScreen();
	DLY_fDelayMs(1);		//
  fWriteCMD(LCD_ENTRY_INC);	
	DLY_fDelayMs(1);		//
  
	fWriteCMD(LCD_DISP_ON_CURSOR_BLINK); //LCD_DISP_ON_CURSOR_BLINK
	DLY_fDelayMs(1);		//
	
	//LCD_fGoHome();
	
	LCD_fStopDataBlock();
	
}

/*=======================================================================*//**
  @brief Clears Display RAM
*//*========================================================================*/
void LCD_fClearScreen (void)
{
	fGetBus();
	
	fWriteCMD(0x01);
	DLY_fDelayMs(3);
	
	fReleaseBus();
}

/*=======================================================================*//**
  @brief Sets Cursor to XY coordinates
*//*========================================================================*/
void LCD_fSetCursor (uint8_t x_data, uint8_t y_data)
{
	__disable_irq();
	fGetBus();
	
		m_delay = 100;			//wait min 450ns
		while(--m_delay);	//666ns measured
	
	if(y_data == 0) fWriteCMD(0x80 | (LCD_START_LINE1+x_data) );
	if(y_data == 1) fWriteCMD(0x80 | (LCD_START_LINE2+x_data) );
	
		m_delay = 100;			//wait min 450ns
		while(--m_delay);	//666ns measured
	
	fReleaseBus();
	__enable_irq();
}

/*=======================================================================*//**
  @brief Sets Cursor to home position and reset shift
*//*========================================================================*/
void LCD_fGoHome (void)
{
	__disable_irq();
	fGetBus();
	
		m_delay = 100;			//wait min 450ns
		while(--m_delay);	//666ns measured
	
	fWriteCMD(0x02);
	
		m_delay = 100;			//wait min 450ns
		while(--m_delay);	//666ns measured
	
	fReleaseBus();
	__enable_irq();
}

/*=======================================================================*//**
  @brief Sends any command to the LCD
*//*========================================================================*/
void LCD_fLCDCommand (uint8_t command)
{
	__disable_irq();
	fGetBus();
	
		m_delay = 100;			//wait min 450ns
		while(--m_delay);	//666ns measured
	
	fWriteCMD(command);
	
		m_delay = 100;			//wait min 450ns
		while(--m_delay);	//666ns measured
	
	fReleaseBus();
	__enable_irq();
}


/*=======================================================================*//**
  @brief Sends any data to the LCD
*//*========================================================================*/
void LCD_fLCDData (uint8_t data)
{
	__disable_irq();
	fGetBus();
	
		m_delay = 100;			//wait min 450ns
		while(--m_delay);	//666ns measured
	
	fWrite(data);
	
		m_delay = 100;			//wait min 450ns
		while(--m_delay);	//666ns measured
	
	fReleaseBus();
	__enable_irq();
}



/*=======================================================================*//**
  @brief Display printf to display
*//*========================================================================*/
void LCD_printf(const char * __restrict string/*format*/, ...)
{
	char buffer [40];
	
	va_list a_list;
  va_start( a_list, string );
	
	vsprintf (buffer, string, a_list);
	
	fput_s(buffer);
	
  va_end (a_list);
}

/*=======================================================================*//**
  @brief Initializes Start of LCD Data block. 
  @attention Must be finished with LCD_fStopDataBlock!
*//*========================================================================*/
void LCD_fStartDataBlock (void)
{
	m_DataBlockRunning = 1;
}

/*=======================================================================*//**
  @brief Stops a LCD Data Block
*//*========================================================================*/
void LCD_fStopDataBlock (void)
{
	m_DataBlockRunning = 0;
	fReleaseBus();
	__enable_irq();
}

/*=======================================================================*//**
  @brief Saves a custom char into the display charmap
	@param location:	 where to save the char 0..7
	@param charmap:			array of pixel data like this

	uint8_t charmap[8] = {
		B11111,
		B11111,
		B11111,
		B11111,
		B11111,
		B11111,
		B11111,
		B11111};

*//*========================================================================*/
void LCD_fSetCustomChar (uint8_t location, uint8_t charmap[])
{
	uint8_t ctr;
	
	location &= 0x07; // we only have 8 locations 0-7

	//LCD_fStartDataBlock();
	fGetBus();
	__disable_irq();
	
	m_delay = 0;			//wait min 450ns
	while(--m_delay){};	//666ns measured
	
  fWriteCMD(LCD_SETCGRAMADDR | (location << 3));
	
	m_delay = 0;			//wait min 450ns
	while(--m_delay){};	//666ns measured
	
  for (ctr=0; ctr < 8; ctr++) 
	{
    fWriteData(charmap[ctr]);
		m_delay = 0;			//wait min 450ns
		while(--m_delay){};	//666ns measured
  }
	
  fWriteCMD(LCD_SETDDRAMADDR | (0));	//next write is in DD ram again
	
	m_delay = 100;			//wait min 450ns
	while(--m_delay);	//666ns measured
	
	//LCD_fStopDataBlock();
	fReleaseBus();
	__enable_irq();
}

/*=======================================================================*//**
  @brief prints the saved custom char
*//*========================================================================*/
void LCD_fPrintCustomChar (uint8_t location)
{
	location &= 0x7; // we only have 8 locations 0-7
	
	fGetBus();
	__disable_irq();
	
	fWriteData(location);
	m_delay = 100;			//wait min 450ns
	while(--m_delay);	//666ns measured

	fReleaseBus();
	__enable_irq();
}

/*=======================================================================*//**
  @brief Controlls LCD backlight
*//*========================================================================*/
void LCD_fControlBacklight (uint8_t state)
{
	if(state) m_LCD_Backlight = 1;
	else m_LCD_Backlight = 0;
	
	fGetBus();
	fReleaseBus();
}

/* EOF */
