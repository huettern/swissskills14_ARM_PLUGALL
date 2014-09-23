/*****************************************************************************
* Project:            PLUGALL
* Target:             
* Type:               SPI controlls
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
#include "SPI.h"


/*****************************************************************************
* DEFINES and MAKROS
*****************************************************************************/   

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
  @brief Init
*//*========================================================================*/
void SPI_fInit (void)
{
	static SPI_InitTypeDef   			  SPI_InitStruct;
	static GPIO_InitTypeDef					GPIO_InitStruct;
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;	//Pull MISO Up
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_2;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	//GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_5);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	/*******************************
	CPHA |	CPOL | read  |	idle
	-----|------------------------
	 0   |   0   |   p   |   0
	 0   |   1   |   n   |   1
	 1   |   0   |   n   |   0
	 1   |   1   |   p   |   1
	 
	CPHA 0 = SPI_CPHA_1Edge
	CPHA 1 = SPI_CPHA_2Edge
	********************************/
	
  SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
  SPI_InitStruct.SPI_DataSize = SPI_DataSize_16b; //SPI_DataSize_4b .. SPI_DataSize_16b
  SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;	
  SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;	//software managed NSS pin
	//APB2 freq = 64MHz -> SPI freq = 64MHz / SPI_BaudRatePrescale
	//with PS=128 SPI_clk = 1MHz
  SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
  SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStruct.SPI_CRCPolynomial = 7;
	
	SPI_Init(SPI1, &SPI_InitStruct);
	
	SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_HF);
	
	//interrupts
	//SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE); //Tx buffer empty interrupt
	//SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, DISABLE); //Rx buffer not empty interrupt
	
	//enable SPI module
	SPI_Cmd(SPI1, ENABLE);
}


/*=======================================================================*//**
  @brief Sends n bits over SPI, returns received word
*//*========================================================================*/
uint16_t SPI_fSend (uint8_t nbits, uint16_t data)
{
	uint16_t receive;
	
	NSS_HIGH();
					
	//set n bits
	SPI1->CR2 = ((SPI1->CR2 & ~SPI_CR2_DS) | ( ( (nbits-1) << 8) & SPI_CR2_DS) );
	//start transmission
	SPI1->DR = data;
	
	
	while( (SPI1->SR & SPI_SR_RXNE) == 0 );	//wait til Rx is full
	receive = SPI1->DR;	//save it
	while( (SPI1->SR & SPI_SR_BSY) );	//wait til done
	
	NSS_LOW();

	return receive;
}


/*=======================================================================*//**
  @brief Sends n bits over SPI, returns received word 
	@attention DOESNT CONTROLL NSS
*//*========================================================================*/
uint16_t SPI_fSendCont (uint8_t nbits, uint16_t data)
{
	uint16_t receive;
					
	//set n bits
	SPI1->CR2 = ((SPI1->CR2 & ~SPI_CR2_DS) | ( ( (nbits-1) << 8) & SPI_CR2_DS) );
	//start transmission
	SPI1->DR = data;
	
	
	while( (SPI1->SR & SPI_SR_RXNE) == 0 );	//wait til Rx is full
	receive = SPI1->DR;	//save it
	while( (SPI1->SR & SPI_SR_BSY) );	//wait til done
	
	return receive;
}

/*=======================================================================*//**
  @brief Changes settings of CPOL and CPHA
	@param cpol can be SPI_CPOL_Low or SPI_CPOL_High
	@param cpha can be SPI_CPHA_1Edge or SPI_CPHA_2Edge
*//*========================================================================*/
void SPI_fChangeClockSettings (uint16_t cpol, uint16_t cpha)
{
	uint16_t tmpreg;
	
  assert_param(IS_SPI_CPOL(cpol));
  assert_param(IS_SPI_CPHA(cpha));
	
	while( (SPI1->SR & SPI_SR_BSY) );	//wait til ready
	SPI1->CR1 &= ~SPI_CR1_SPE;	//disable SPI to change edegs
	
	tmpreg = SPI1->CR1;	//get configuration
	
	tmpreg &= 0xFFFC;	//clear CPHA and CPOL
	
	tmpreg |= (cpol | cpha);	//set new settings
	
	SPI1->CR1 = tmpreg;	//set conf settings
	
	SPI1->CR1 |= SPI_CR1_SPE;	//enable SPI 
}











/* EOF */
