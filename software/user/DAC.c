/*****************************************************************************
* Project:            PLUGALL
* Target:             
* Type:               DAC controlls
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
#include "dac.h"


/*****************************************************************************
* Private Data
*****************************************************************************/   


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
  @brief Init function
*//*========================================================================*/
void DAC_fInit (void)
{
  static TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStruct;
  static TIM_OCInitTypeDef        outputChannelInit;
	
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11,  GPIO_AF_2);
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM19, ENABLE);
	
  TIM_TimeBaseInitStruct.TIM_Prescaler =        63;  //0..65535 divide SYSCLK by (63 + 1)
  TIM_TimeBaseInitStruct.TIM_CounterMode =      TIM_CounterMode_Up;
  TIM_TimeBaseInitStruct.TIM_Period =           1000;  //0..65535 maximum pwm duty and 1kHz freq
  TIM_TimeBaseInitStruct.TIM_ClockDivision =    TIM_CKD_DIV1;
  TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
  
  TIM_TimeBaseInit(TIM19, &TIM_TimeBaseInitStruct);
  TIM_Cmd(TIM19, ENABLE);
	
  // init output pin
  outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
  outputChannelInit.TIM_Pulse = 200;
  outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
  outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
	
  TIM_OC1Init(TIM19, &outputChannelInit);	//DAC0
  TIM_OC2Init(TIM19, &outputChannelInit);	//DAC1
	
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	TIM_SetCompare1(TIM19, 0);	//sets PWM duty of DAC0 to 0
	TIM_SetCompare2(TIM19, 0);	//sets PWM duty of DAC1 to 0
}




/* EOF */
