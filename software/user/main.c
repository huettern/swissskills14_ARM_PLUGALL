/*****************************************************************************
* Project:            
* Target:             STM32F372RBT6
* Type:               Main-module
* Description:        main
* Compiler:           ANSI-C
* Filename:           main.c
* Version:            1.0
* Author:             Noah Huetter
* Creation-Date:      07.08.14
******************************************************************************
* Modification History:
* 
* [1.0]     07.08.14   NHU   first release
*
*****************************************************************************/  
#include "stm32f37x_conf.h"
#include "plugall.h"
#include "dac.h"
#include "spi.h"
#include "i2c.h"
#include "adc.h"
#include "int.h"
#include "shell.h"
#include "LCD.h"
#include "delay.h"

#include <stdio.h>
#include <stdlib.h>

/*****************************************************************************
* Defines and Makros
*****************************************************************************/  

/*****************************************************************************
* Private Data
*****************************************************************************/   

RCC_ClocksTypeDef RCC_Clocks;

unsigned int m_MSCounter = 0;
	
/*****************************************************************************
* Private Function Declaration
*****************************************************************************/ 
void fInit (void);

void cbGPTimeOut (void);
void cbGPTimeOut1 (void);
void cbGPTimeOut2 (void);
void cbGPTimeOut3 (void);
void cbGPTimeOut4 (void);

/*****************************************************************************
* MAIN ROUTINE
*****************************************************************************/ 
int main (void)
{	
	RCC_GetClocksFreq(&RCC_Clocks);
	
	fInit(); //Running at 64MHz
	
	for(;;)
	{
		/* PLACE YOUR CODE HERE */
		
		
		
		
		
		/* Periodic Handler */
		SHELL_HandleTx();
		SHELL_LineParser();
		DLY_fHandleTasks();
		PLG_fHandleEvents();
	}
	
}
	
/*****************************************************************************
* Private Functions
*****************************************************************************/ 
/*=======================================================================*//**
  @brief Inits peripherals
*//*========================================================================*/
void fInit (void)
{
	SHELL_fInit();
	DLY_fInit();
	
	LED_YEL_OFF();
	LED_RED_OFF();

	SHELL_fClearTerminal();
	printf("ARM PLUGALL v1 Started!                                   Copyright Noah Huetter\r\n\r\n");
	printf("SYSCLK=%dMHz HCLK=%dMHz \r\n",RCC_Clocks.SYSCLK_Frequency / 1000000, RCC_Clocks.HCLK_Frequency / 1000000);
	SHELL_fDrawLineOfChar('*');
	SHELL_fEchoControl(ECHO_ON);
	
	PLG_fInit();
	PLG_StartBusControl();
	
	DAC_fInit();
	SPI_fInit();
	I2C_fInit();
	I2C_fInitTempSensor();
	ADC_fInit();		
	INT_fInit();
	LCD_fInit();

	SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000); //@1ms
	
	//DLY_fAddTimerTick(DLY_CONTINUOUS, 100, cbGPTimeOut);
	//DLY_fAddTimerTick(DLY_SINGLE_SHOT, 5000, cbGPTimeOut1);
	//DLY_fAddTimerTick(DLY_SINGLE_SHOT, 10000, cbGPTimeOut2);
	//DLY_fAddTimerTick(DLY_CONTINUOUS, 100, cbGPTimeOut3);
	//DLY_fAddTimerTick(DLY_CONTINUOUS, 100, cbGPTimeOut4);
	
	//PLG_SubscribeLongPressHandler(PLG_CLAV_STOP_LONGPRESS, 	1000, cbGPTimeOut1);
	//PLG_SubscribeLongPressHandler(PLG_CLAV_ENTER_LONGPRESS, 2000, cbGPTimeOut2);
	//PLG_SubscribeLongPressHandler(PLG_CLAV_DOWN_LONGPRESS, 	 100, cbGPTimeOut3);
	//PLG_SubscribeLongPressHandler(PLG_CLAV_UP_LONGPRESS, 	  5000, cbGPTimeOut4);
}	
	
/*=======================================================================*//**
  @brief Sys Tick Interrupt called every 1ms
*//*========================================================================*/
void SysTick_Handler (void)
{
	PLG_fHandleBus();
	m_MSCounter++;
}

/*=======================================================================*//**
  @brief General Purpose Callback 100ms
*//*========================================================================*/
void cbGPTimeOut (void)
{
	
	
}


/*=======================================================================*//**
  @brief General Purpose Callback
*//*========================================================================*/
void cbGPTimeOut1 (void)
{
	
}


/*=======================================================================*//**
  @brief General Purpose Callback
*//*========================================================================*/
void cbGPTimeOut2 (void)
{
	
}


/*=======================================================================*//**
  @brief General Purpose Callback
*//*========================================================================*/
void cbGPTimeOut3 (void)
{
	
}


/*=======================================================================*//**
  @brief General Purpose Callback
*//*========================================================================*/
void cbGPTimeOut4 (void)
{	
	
}


#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
	{ 
	while (1){}
	}
#endif
