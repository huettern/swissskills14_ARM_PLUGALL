/*****************************************************************************
* Project:            PLUGALL
* Target:             
* Type:               INT controlls
* Version:            1.0
* Author:             Noah Huetter
* Creation-Date:      07.2014
******************************************************************************
* Modification History:
* 
* [1.0]     07.2014   NHU   first release
*
*****************************************************************************/  
#ifndef __INT_H
#define __INT_H




/*****************************************************************************
* Defines
*****************************************************************************/ 
#define INT0	0
#define INT1	1

/*****************************************************************************
* Public Data
*****************************************************************************/ 


/*****************************************************************************
* Public Functions
*****************************************************************************/ 
void INT_fInit (void);

uint32_t INT_fGetLastValue (uint8_t intx);
uint32_t INT_fGetValue (uint8_t intx);


uint8_t INT_fGetINT0Event (void);
uint8_t INT_fGetINT1Event (void);
void INT_fResetINT0Event (void);
void INT_fResetINT1Event (void);


#endif
