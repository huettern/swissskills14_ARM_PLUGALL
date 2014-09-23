/*****************************************************************************
* Project:            
* Target:             STM32F3
* Type:               Main-module
* Description:        main
* Compiler:           ANSI-C
* Filename:           main.c
* Version:            1.0
* Author:             Noah Huetter
* Creation-Date:      05.06.2014
******************************************************************************
* Modification History:
* 
* [1.0]     05.06.2014   NHU   first release
*
*****************************************************************************/  
#include "stm32f37x_conf.h"
#include "shell.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "lcd.h"


#pragma import (__use_no_semihosting)

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

#define TX_BUF_SIZE 512
#define RX_BUF_SIZE 256


/*****************************************************************************
* Private Data
*****************************************************************************/   
static uint8_t m_bTxBuffer [TX_BUF_SIZE];
static uint8_t m_bRxBuffer [RX_BUF_SIZE];

static uint16_t m_bTxBufCnt = 0;
static uint16_t m_bRxBufCnt = 0;

static uint8_t m_fEcho = 0;

static uint8_t m_fLineComplete = 0;

/*****************************************************************************
* Private Function Declaration
*****************************************************************************/ 

/*==========================================================================*/
static void fFillTxBufByte (char c)
/*----------------------------------------------------------------------------
  Description: Fills single character in Tx Buffer
  ==========================================================================*/
{
  static uint16_t bWriteIndex = 0;
  
  /* Write start byte and increment counters */
  m_bTxBuffer[bWriteIndex++] = c;
  bWriteIndex %= TX_BUF_SIZE;
  m_bTxBufCnt++;
}

/*==========================================================================*/
static char fGetRxBufByte (void)
/*----------------------------------------------------------------------------
  Description: Pulls single char out of Rx buffer
  ==========================================================================*/
{
  static uint16_t bReadIndex = 0;
  char data;
  
  if(m_bRxBufCnt == 0) return 0;
  
  /* Write start byte and increment counters */
  data = m_bRxBuffer[bReadIndex++];
  bReadIndex %= RX_BUF_SIZE;
  m_bRxBufCnt--;
  
  return data;
}

/*****************************************************************************
* Public Functions
*****************************************************************************/ 
/*==========================================================================*/
void SHELL_fInit (void)
/*----------------------------------------------------------------------------
  Description: Initializes all Peripherals
  ==========================================================================*/
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_7);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_7);
  
  /* Configure USART1 pins:  Rx and Tx ----------------------------*/
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15; //0 highest, 15 lowest
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  
  USART_Cmd(USART1,ENABLE);
}

/*==========================================================================*/
void SHELL_fClearTerminal (void)
/*----------------------------------------------------------------------------
  Description: Clears the terminal screen
  ==========================================================================*/
{
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
  USART_SendData(USART1, 0x0C);
}

/*==========================================================================*/
void SHELL_fDrawLineOfChar (char c)
/*----------------------------------------------------------------------------
  Description: Draws a line of characters to the terminal
  ==========================================================================*/
{
  uint8_t cnt = 0;
  fFillTxBufByte('\r');
  fFillTxBufByte('\n');
  for(cnt = 0; cnt < 80; cnt++) 
  {
    fFillTxBufByte(c);
  }
  fFillTxBufByte('\r');
  fFillTxBufByte('\n');
  
}

/*=======================================================================*//**
  @brief Handle Tx Buffer
  @attention Must be called periodically!
*//*========================================================================*/
uint16_t SHELL_HandleTx (void)
{
  static uint16_t bSendIndex = 0;
  
  /* If buffer is filled and Tx Reg is empty / ready to send */
  if(m_bTxBufCnt > 0)
  {
    if(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == SET)
    {
        USART_SendData(USART1, m_bTxBuffer[bSendIndex++]);
        bSendIndex %= TX_BUF_SIZE;
        m_bTxBufCnt--;
    }
  }
  return m_bTxBufCnt;
}

/*=======================================================================*//**
  @brief Writes Tx buffer out until its empty
*//*========================================================================*/
void SHELL_HandleTxEmpty (void)
{
  while(SHELL_HandleTx() > 0);
}

/*==========================================================================*/
int fputc (int c, FILE * stream)
/*----------------------------------------------------------------------------
  Description: Gets called by printf function
  ==========================================================================*/
{
  //while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
  fFillTxBufByte(c);
  return 0;
}

/*==========================================================================*/
void _sys_exit(int return_code)
/*----------------------------------------------------------------------------
  Description: Don't really know...
  ==========================================================================*/
{
label:  goto label;  /* endless loop */
}

/*=======================================================================*//**
  @brief Sets echo on or of
*//*========================================================================*/
void SHELL_fEchoControl (uint8_t status)
{
  m_fEcho = status;
}

/*=======================================================================*//**
  @brief Moves input buffer from RxBuf to data until line brake
  @details
  This is a more detailed description.
  ...
  @param data -> data output
  @retval 0 if fail, >0: number of chars copied
*//*========================================================================*/
uint8_t USART_fGetLine (char* data)
{
  char BufData;
  uint8_t counter = 0;
  
  if (m_fLineComplete != 1) return 0; //check if complete
  m_fLineComplete = 0;
  
  BufData = fGetRxBufByte();  //get a byte
  
  while( (BufData != '\r') && (BufData != 0) )  //move until line brake
  {
    *data = BufData;
    data++;
    counter++;
    BufData = fGetRxBufByte();
  }
  *data = 0;  //indicate last char in the string
  
  return counter; //return amount of chars copied
}

/*=======================================================================*//**
  @brief Parses the shell input line and executes commands
	@attention must be called periodically
*//*========================================================================*/
void SHELL_LineParser (void)
{
	char ShellInput[40];
	int value;
	
	if( m_fLineComplete != 1) return;
	
	USART_fGetLine(ShellInput);	//get line
	
	if( strstr (ShellInput, "dac0") != NULL )
	{
		value = atoi(&ShellInput[5]);
		TIM_SetCompare1(TIM19, value);
	}
	else if( strstr (ShellInput, "dac1") != NULL )
	{
		value = atoi(&ShellInput[5]);
		TIM_SetCompare2(TIM19, value);
	}
	else if( strstr (ShellInput, "lcd init") != NULL )
	{
		LCD_fInit();
	}
}

/****************************************************************************
 * INTERRUPT SERVICE FUNCTIONS
 ****************************************************************************/
/*=======================================================================*//**
  @brief Handles USART1 received bytes
*//*========================================================================*/
void USART1_IRQHandler (void)
{
  static uint8_t bWriteIndex = 0;
  static uint8_t data;
  
  data = USART_ReceiveData(USART1);  //get data
  
  /* Control if Buffer overflows */
  if(m_bRxBufCnt >= RX_BUF_SIZE) 
    {
    return;
    }
  
  /* fill SCI_Buffer */
  m_bRxBuffer[bWriteIndex] = data;
  
  /* increment Write-Index and BufferCounter*/
  bWriteIndex++; 
  bWriteIndex %= RX_BUF_SIZE;
  m_bRxBufCnt++;
    
  if(m_fEcho == ECHO_ON) fFillTxBufByte( data );
    
  if(data == '\r') 
  {
    m_fLineComplete = 1;
    if(m_fEcho == ECHO_ON) fFillTxBufByte( '\n' );
  }
}

