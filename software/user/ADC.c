/*****************************************************************************
* Project:            PLUGALL
* Target:             
* Type:               ADC controlls
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
#include "adc.h"


/*****************************************************************************
* Private Data
*****************************************************************************/   
typedef struct {
	uint16_t AIN0[4];
	uint16_t AIN1[4];
	uint16_t AIN2[4];
	uint16_t AIN3[4];
} m_ADCResultsStruct;

m_ADCResultsStruct m_ADCResults;

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
  @brief init function
*//*========================================================================*/
void ADC_fInit (void)
{
	static ADC_InitTypeDef	 				ADC_InitStructure;
	static DMA_InitTypeDef   				DMA_InitStructure;
	
	//Init DMA
	DMA_StructInit(&DMA_InitStructure);
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)m_ADCResults.AIN0;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 16;	//4ADC channels sampled 4 times each
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize  = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	
	//calibrate
  ADC_ResetCalibration(ADC1);
  while(ADC_GetResetCalibrationStatus(ADC1));
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1)); 
	
	//ADC: set clock and enable it
  RCC_ADCCLKConfig(RCC_PCLK2_Div4); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 16;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	//configure channels
  ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, ADC_SampleTime_71Cycles5);	//AIN0
  ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 2, ADC_SampleTime_71Cycles5);	//AIN0
  ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 3, ADC_SampleTime_71Cycles5);	//AIN0
  ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 4, ADC_SampleTime_71Cycles5);	//AIN0
	
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 5, ADC_SampleTime_71Cycles5);	//AIN1
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 6, ADC_SampleTime_71Cycles5);	//AIN1
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 7, ADC_SampleTime_71Cycles5);	//AIN1
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 8, ADC_SampleTime_71Cycles5);	//AIN1
	
  ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 9, ADC_SampleTime_71Cycles5);	//AIN2
  ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 10, ADC_SampleTime_71Cycles5);	//AIN2
  ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 11, ADC_SampleTime_71Cycles5);	//AIN2
  ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 12, ADC_SampleTime_71Cycles5);	//AIN2
	
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 13, ADC_SampleTime_71Cycles5);	//AIN3
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 14, ADC_SampleTime_71Cycles5);	//AIN3
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 15, ADC_SampleTime_71Cycles5);	//AIN3
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 16, ADC_SampleTime_71Cycles5);	//AIN3
	
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
	
	DMA_Cmd(DMA1_Channel1, ENABLE);
	
	ADC_SoftwareStartConv(ADC1);
}

/*=======================================================================*//**
  @brief Returns non filtered ADC values
*//*========================================================================*/
void ADC_fGetRawValues (uint16_t* values )
{
	values[0] = m_ADCResults.AIN0[0];
	values[1] = m_ADCResults.AIN1[1];
	values[2] = m_ADCResults.AIN2[2];
	values[3] = m_ADCResults.AIN3[3];
}

/*=======================================================================*//**
  @brief Returns filtered ADC values in [mV]
*//*========================================================================*/
void ADC_fGetFilteredValues (uint16_t* values )
{
	uint8_t ctr;
	uint32_t temp_value;
	
	/* AIN 0 */
	temp_value = 0;
	for(ctr = 0; ctr < 4; ctr++)
	{
		temp_value += m_ADCResults.AIN0[ctr];
	}
	temp_value = (5000 * (temp_value / 4)) / 0x0FFF;	//calculate average in [mV]
	values[0] = temp_value;
	
	/* AIN 1 */
	temp_value = 0;
	for(ctr = 0; ctr < 4; ctr++)
	{
		temp_value += m_ADCResults.AIN1[ctr];
	}
	temp_value = (5000 * (temp_value / 4)) / 0x0FFF;	//calculate average in [mV]
	values[1] = temp_value;
	
	/* AIN 2 */
	temp_value = 0;
	for(ctr = 0; ctr < 4; ctr++)
	{
		temp_value += m_ADCResults.AIN2[ctr];
	}
	temp_value = (5000 * (temp_value / 4)) / 0x0FFF;	//calculate average in [mV]
	values[2] = temp_value;
	
	/* AIN 3 */
	temp_value = 0;
	for(ctr = 0; ctr < 4; ctr++)
	{
		temp_value += m_ADCResults.AIN3[ctr];
	}
	temp_value = (5000 * (temp_value / 4)) / 0x0FFF;	//calculate average in [mV]
	values[3] = temp_value;
}



/* EOF */
