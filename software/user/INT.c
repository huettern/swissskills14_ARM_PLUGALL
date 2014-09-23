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
#include "stm32f37x_conf.h"
#include "INT.h"

#include <stdio.h>


#define NUM_OF_DELTAS	10

/*****************************************************************************
* Private Data
*****************************************************************************/   
typedef struct
{
	uint32_t OldValue;
	uint32_t NewValue;
	uint32_t Deltas[NUM_OF_DELTAS];
}	INT_DataTypedef;

INT_DataTypedef INTData[2];

uint8_t m_INT1BufPointer = 0;
uint8_t m_INT0BufPointer = 0;

uint8_t INT_INT0Event = 0;
uint8_t INT_INT1Event = 0;

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
  @brief Inits 2 timers to capture INT0..1. Count at 1us
  @retval RETVAL
*//*========================================================================*/
void INT_fInit (void)
{
  //static TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStruct;
	static TIM_ICInitTypeDef				ICInitStruct;
	static NVIC_InitTypeDef					NVIC_InitStructure;
	
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource15,  GPIO_AF_2);	//INT0
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);		//INT1
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //INT0
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); //INT1
	
	TIM_InternalClockConfig(TIM2);
	TIM_InternalClockConfig(TIM5);
	
	TIM_DeInit(TIM2);
	TIM_DeInit(TIM5);	
	/*	
  TIM_TimeBaseInitStruct.TIM_Prescaler =        63;  //0..65535 divide SYSCLK by (63 + 1)
  TIM_TimeBaseInitStruct.TIM_CounterMode =      TIM_CounterMode_Up;
  TIM_TimeBaseInitStruct.TIM_Period =           0;  //count maximum
  TIM_TimeBaseInitStruct.TIM_ClockDivision =    TIM_CKD_DIV1;
  TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);//ticks now at 1us
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStruct);//ticks now at 1us
	*/
	TIM_PrescalerConfig(TIM2, 63, TIM_PSCReloadMode_Immediate);
	TIM_PrescalerConfig(TIM5, 63, TIM_PSCReloadMode_Immediate);
	
	
	/* INT0 configuration */
	ICInitStruct.TIM_Channel 			= TIM_Channel_1;
	ICInitStruct.TIM_ICPolarity		= TIM_ICPolarity_Rising;	//Rising, Falling, BothEdge
	ICInitStruct.TIM_ICPrescaler 	= TIM_ICPSC_DIV1;					//1,2,4,8
	ICInitStruct.TIM_ICFilter 		= 4; 											//0..15
	ICInitStruct.TIM_ICSelection	= TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM2, &ICInitStruct);
	
	/* INT1 configuration */
	ICInitStruct.TIM_Channel 			= TIM_Channel_1;
	ICInitStruct.TIM_ICPolarity		= TIM_ICPolarity_Rising;	//Rising, Falling, BothEdge
	ICInitStruct.TIM_ICPrescaler 	= TIM_ICPSC_DIV1;					//1,2,4,8
	ICInitStruct.TIM_ICFilter 		= 4; 											//0..15
	ICInitStruct.TIM_ICSelection	= TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM5, &ICInitStruct);
	
	
	/* Interrupts */
	TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);	//capture interrupt enable
	TIM_ITConfig(TIM5, TIM_IT_CC1, ENABLE); //capture interrupt enable
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//priority
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//priority
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM2->SR = 0;	//clear all flags
	TIM5->SR = 0;
	
  TIM_Cmd(TIM2, ENABLE);
  TIM_Cmd(TIM5, ENABLE);
}

/*=======================================================================*//**
  @brief Returns last valid time data
*//*========================================================================*/
uint32_t INT_fGetLastValue (uint8_t intx)
{
	return INTData[intx].Deltas[m_INT0BufPointer];
}

/*=======================================================================*//**
  @brief Returns a filtered value
*//*========================================================================*/
uint32_t INT_fGetValue (uint8_t intx)
{
	uint32_t maximum, minimum, ctr;
	uint64_t value;

	maximum = 0;
	minimum = 0xffffffff;
	value = 0;
	
	//add 'em and save peaks
	for(ctr = 0; ctr < NUM_OF_DELTAS; ctr++)
	{
		value += INTData[intx].Deltas[ctr];
		if(INTData[intx].Deltas[ctr] > maximum) maximum = INTData[intx].Deltas[ctr];
		if(INTData[intx].Deltas[ctr] < minimum) minimum = INTData[intx].Deltas[ctr];
	}
	
	value -= maximum;
	value -= minimum;
	
	value /= NUM_OF_DELTAS-2;
	
	return (uint32_t)value;
}

/*=======================================================================*//**
  @brief Returns 1 if INT0 Event occured
*//*========================================================================*/
uint8_t INT_fGetINT0Event (void)
{
	return INT_INT0Event;
}

/*=======================================================================*//**
  @brief Returns 1 if INT1 Event occured
*//*========================================================================*/
uint8_t INT_fGetINT1Event (void)
{
	return INT_INT1Event;
}

/*=======================================================================*//**
  @brief Resets INT0 Event 
*//*========================================================================*/
void INT_fResetINT0Event (void)
{
	INT_INT0Event = 0;
}

/*=======================================================================*//**
  @brief Resets INT1 Event 
*//*========================================================================*/
void INT_fResetINT1Event (void)
{
	INT_INT1Event = 0;
}


/*****************************************************************************
* IRQ
*****************************************************************************/ 
/*=======================================================================*//**
  @brief Interrupt TIM2 - INT0
*//*========================================================================*/
void TIM2_IRQHandler (void)
{
	static uint8_t m_INT0BufPointer = 0;
	TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
	
	INT_INT0Event = 1;
	
	//Get Value
	INTData[0].OldValue = INTData[0].NewValue;
	INTData[0].NewValue = TIM2->CCR1;
	
	//Calculate Delta
	if(INTData[0].NewValue >= INTData[0].OldValue)
		INTData[0].Deltas[m_INT0BufPointer] = INTData[0].NewValue - INTData[0].OldValue;
	else
		INTData[0].Deltas[m_INT0BufPointer] = (0xFFFFFFFF - INTData[0].OldValue) + INTData[0].NewValue;
	
	//Inc counter
	m_INT0BufPointer++;
	m_INT0BufPointer %= NUM_OF_DELTAS;
}



/*=======================================================================*//**
  @brief Interrupt TIM5 - INT1
*//*========================================================================*/
void TIM5_IRQHandler (void)
{
	static uint8_t m_INT1BufPointer = 0;
	TIM_ClearITPendingBit(TIM5, TIM_IT_CC1);
	
	INT_INT1Event = 1;
	
	//Get Value
	INTData[1].OldValue = INTData[1].NewValue;
	INTData[1].NewValue = TIM5->CCR1;
	
	//Calculate Delta
	if(INTData[1].NewValue >= INTData[1].OldValue)
		INTData[1].Deltas[m_INT1BufPointer] = INTData[1].NewValue - INTData[1].OldValue;
	else
		INTData[1].Deltas[m_INT1BufPointer] = (0xFFFFFFFF - INTData[1].OldValue) + INTData[1].NewValue;

	//Inc counter
	m_INT1BufPointer++;
	m_INT1BufPointer %= NUM_OF_DELTAS;
	
}


/* EOF */
