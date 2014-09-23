/*****************************************************************************
* Project:            PLUGALL
* Target:             
* Type:               delay functions
* Version:            1.0
* Author:             Noah Huetter
* Creation-Date:      07.2014
******************************************************************************
* Modification History:
* 
* [1.0]     08.07.2014   NHU   first release
*
*****************************************************************************/  
#include "stm32f37x_conf.h"
#include "delay.h"



/*****************************************************************************
* Defines
*****************************************************************************/ 
#define MAXIMUM_NUM_OF_CB  20 //maximum number of timer callbacks

typedef struct {
	TICK_tpfVoidCallback 	pfTimerTickCallback;
	uint16_t 							wTimeoutValue;
	uint16_t 							wTimeoutReloadValue;
	uint8_t 							SingleContinuous;
} TickStruct;

/*****************************************************************************
* Private Data
*****************************************************************************/   
static uint32_t m_MSCtr;

static TickStruct Ticks [MAXIMUM_NUM_OF_CB];



/*****************************************************************************
* Private Functions Prototypes
*****************************************************************************/ 



/*****************************************************************************
* Private Functions 
*****************************************************************************/ 

/*****************************************************************************
* Public Functions 
*****************************************************************************/ 
/*=======================================================================*//**
  @brief Inits timer for delay routines
*//*========================================================================*/
void DLY_fInit (void)
{
	static TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStruct;
  static NVIC_InitTypeDef NVIC_InitStructure;
	uint8_t ctr;
	
	for(ctr = 0; ctr < MAXIMUM_NUM_OF_CB; ctr++)
	{
		Ticks[ctr].pfTimerTickCallback = 0;
		Ticks[ctr].SingleContinuous = 0;
		Ticks[ctr].wTimeoutReloadValue = 0;
		Ticks[ctr].wTimeoutValue = 0;
	}
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);

	TIM_TimeBaseInitStruct.TIM_Prescaler = 63;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 1000;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM13, &TIM_TimeBaseInitStruct);
	TIM_Cmd(TIM13, ENABLE);
	
	TIM_ITConfig(TIM13, TIM_IT_Update, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM13_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*=======================================================================*//**
  @brief Waits n milliseconds here
*//*========================================================================*/
void DLY_fDelayMs (uint32_t time)
{
	m_MSCtr = 0;
	//TIM_ITConfig(TIM13, TIM_IT_Update, ENABLE);
	while(m_MSCtr < time);
	//TIM_ITConfig(TIM13, TIM_IT_Update, DISABLE);
}

/*=======================================================================*//**
  @brief Waits n milliseconds here
*//*========================================================================*/
void DLY_fAddTimerTick(uint8_t SingleMulti,
											 uint16_t wTimeValueMS, 
                       TICK_tpfVoidCallback pfNewTimerTickCallback)
/*----------------------------------------------------------------------------
  Description: starts the timer tick
  IN: fTimerTickCallback -> a callback function that gets called 
        every wTimeValueMS [ms]
  ==========================================================================*/ 
  {
  uint8_t bCallbackNumber;
		
  if(pfNewTimerTickCallback == NULL) return;   //FATAL
  
  /* search next free space in callback array */
  for(bCallbackNumber = 0; (bCallbackNumber < MAXIMUM_NUM_OF_CB ) &&
      Ticks[bCallbackNumber].pfTimerTickCallback != NULL; bCallbackNumber++);

  /* if callback array is full */
  if(bCallbackNumber == MAXIMUM_NUM_OF_CB) return;

  /* start timer, save callback and reload time */
  Ticks[bCallbackNumber].pfTimerTickCallback = pfNewTimerTickCallback;
  Ticks[bCallbackNumber].wTimeoutReloadValue = wTimeValueMS; 
  Ticks[bCallbackNumber].wTimeoutValue = wTimeValueMS;  
	Ticks[bCallbackNumber].SingleContinuous = SingleMulti;
  } /* End of TICK_fAddTimerTick */
  
/*=======================================================================*//**
  @brief Waits n milliseconds here
*//*========================================================================*/
void DLY_fStopTimerTick(TICK_tpfVoidCallback pfCallback)
/*----------------------------------------------------------------------------
  Description: stops the timer tick
  IN: pfCallback -> callback function name
  ==========================================================================*/
  {
  uint8_t bCallbackNumber;

  /* search for position in callback array */
  for(bCallbackNumber = 0; (bCallbackNumber < MAXIMUM_NUM_OF_CB ) &&
      Ticks[bCallbackNumber].pfTimerTickCallback != pfCallback; 
      bCallbackNumber++);

  /* if callback not found in array */
  if(bCallbackNumber == MAXIMUM_NUM_OF_CB) return;  

  /* remove from callback array and reload time */
  Ticks[bCallbackNumber].pfTimerTickCallback = NULL;
  Ticks[bCallbackNumber].wTimeoutValue = 0;   
  Ticks[bCallbackNumber].wTimeoutReloadValue = 0; 
	Ticks[bCallbackNumber].SingleContinuous = 0;
  } /* end of TICK_fStopTimerTick */

/*=======================================================================*//**
  @brief 
*//*========================================================================*/
void TICK_fRestartTimerTick(TICK_tpfVoidCallback pfCallback)
/*----------------------------------------------------------------------------
  Description: restarts the timer tick: count down from the original value
  IN: pfCallback -> callback function name
			SingleMulti -> Single shot event or continuous
  ==========================================================================*/
  {
  uint8_t bCallbackNumber;

  /* search for position in callback array */
  for(bCallbackNumber = 0; 
			(bCallbackNumber < MAXIMUM_NUM_OF_CB ) && Ticks[bCallbackNumber].pfTimerTickCallback != pfCallback; 
      bCallbackNumber++);

  /* if callback not found in array */
  if(bCallbackNumber >= MAXIMUM_NUM_OF_CB) return;  

  Ticks[bCallbackNumber].wTimeoutValue = Ticks[bCallbackNumber].wTimeoutReloadValue;  
  } /* end of TICK_fStopTimerTick */
	
/*=======================================================================*//**
  @brief Handles Timer Tasks: Checks if timeout has reached and executes task
	@attention Must be called periodically!
*//*========================================================================*/
void DLY_fHandleTasks (void)
{
	uint8_t bCtr;
  //execute callbacks if available
  for(bCtr = 0; bCtr < MAXIMUM_NUM_OF_CB; bCtr++)
	{
		if(Ticks[bCtr].wTimeoutValue == 0 && Ticks[bCtr].pfTimerTickCallback != NULL) 
		{
			Ticks[bCtr].pfTimerTickCallback();
			if(Ticks[bCtr].SingleContinuous == DLY_SINGLE_SHOT)
			{
				DLY_fStopTimerTick(Ticks[bCtr].pfTimerTickCallback);
			}
			else
			{
				Ticks[bCtr].wTimeoutValue = Ticks[bCtr].wTimeoutReloadValue;
			}
		}
	}
}
	
	
/*****************************************************************************
* Interrupts
*****************************************************************************/ 
	
/*=======================================================================*//**
  @brief Called every 1 ms
*//*========================================================================*/
void TIM13_IRQHandler (void)
{
  uint8_t bCtr;  
	
	TIM_ClearITPendingBit(TIM13, TIM_IT_Update);
	m_MSCtr++;
  
  //decrement all timer values that are not 0
  for(bCtr = 0; bCtr < MAXIMUM_NUM_OF_CB; bCtr++)
  {
    if(Ticks[bCtr].wTimeoutValue > 0) Ticks[bCtr].wTimeoutValue -= 1;
  }
  
}


/* EOF */
