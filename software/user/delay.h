/*****************************************************************************
* Project:            PLUGALL
* Target:             
* Type:               DELAY
* Version:            1.0
* Author:             Noah Huetter
* Creation-Date:      07.2014
******************************************************************************
* Modification History:
* 
* [1.0]     07.2014   NHU   first release
*
*****************************************************************************/  
#ifndef __DELAY_H
#define __DELAY_H


/*****************************************************************************
* Defines
*****************************************************************************/ 
#define NULL	0

#define DLY_CONTINUOUS  0
#define DLY_SINGLE_SHOT	1

typedef void(*TICK_tpfVoidCallback)(void);

/*****************************************************************************
* Public Functions
*****************************************************************************/ 
void DLY_fInit(void);

void DLY_fDelayMs (uint32_t time);

void DLY_fAddTimerTick(uint8_t SingleMulti,
											 uint16_t wTimeValueMS, 
                       TICK_tpfVoidCallback pfNewTimerTickCallback);

void DLY_fStopTimerTick(TICK_tpfVoidCallback pfCallback);

void DLY_fRestartTimerTick(TICK_tpfVoidCallback pfCallback);


void DLY_fHandleTasks (void);


#endif
