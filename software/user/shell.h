
/*****************************************************************************
* Project:            
* Target:             STM32F051R8T6
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

#define ECHO_ON 1
#define ECHO_OFF 0

/*****************************************************************************
* Public Function Declaration
*****************************************************************************/ 

void SHELL_fInit (void);

void SHELL_fClearTerminal (void);

void SHELL_fDrawLineOfChar (char c);

uint16_t SHELL_HandleTx (void);

void SHELL_HandleTxEmpty (void);

void SHELL_fEchoControl (uint8_t status);

uint8_t USART_fGetLine (char* data);

void SHELL_LineParser (void);
