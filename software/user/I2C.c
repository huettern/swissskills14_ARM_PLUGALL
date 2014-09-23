/*****************************************************************************
* Project:            PLUGALL
* Target:             
* Type:               I2C controlls
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
#include "I2C.h"
#include "shell.h"

#include <stdio.h>

#define MAX_DATA_BYTES 64

#define I2C_SCL_LOW_PULSE		5000 //[ns]
#define I2C_SCL_HIGH_PULSE	5000//[ns]

#define I2C_SDA_SETUP_TIME  250 //[ns]
#define I2C_SDA_HOLD_TIME   250 //[ns]

#define I2C_PRESC	0x70000000	//0x0..0xf

#define TPRESC	125 //[ns]


/*****************************************************************************
* Private Data
*****************************************************************************/   
typedef struct {
	uint8_t address;
	uint8_t data[MAX_DATA_BYTES];
	uint8_t nbytes;
	uint8_t nbytes_left;
	uint8_t* read_reg;
} SlaveStruct;

SlaveStruct m_slave;

uint8_t m_TransmissionRuning = 0;


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
void I2C_fInit (void)
{
	static I2C_InitTypeDef   				I2C_InitStruct;
	static NVIC_InitTypeDef					NVIC_InitStruct;
	
	uint32_t timing_temp_reg;
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_4);
	
	RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK); //Sysclock -> 64MHz 
	
	
	
	
	
	timing_temp_reg = ( ( (I2C_PRESC) ) | 	//PRESCALER
											( ( (I2C_SDA_SETUP_TIME / TPRESC ) -1 ) << 20 ) | 	//SDA Setup Time
											( (I2C_SDA_HOLD_TIME / TPRESC ) << 16 ) | 	//SDA Hold Time
											( ( (I2C_SCL_HIGH_PULSE / TPRESC ) -1 ) <<  8 ) | 	//SCL High Pulse
											( ( (I2C_SCL_LOW_PULSE / TPRESC ) -1 ) <<  0 ) ); 	//SCL Low Pulse
	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	NVIC_InitStruct.NVIC_IRQChannel = I2C1_EV_IRQn;	//I2C event interrupt
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =  10; //0 highest, 15 lowest
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;	//dont care
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel = I2C1_ER_IRQn;	//I2C error interrupt
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =  10; //0 highest, 15 lowest
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;	//dont care
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
  I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;  
  I2C_InitStruct.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
  I2C_InitStruct.I2C_DigitalFilter = 0x00;
  I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStruct.I2C_Timing = timing_temp_reg; //100kHz Clock: 0x00222727
	I2C_Init(I2C1, &I2C_InitStruct);
	
	
	I2C_Cmd(I2C1, ENABLE);
	
	//I2C_ITConfig(I2C1, I2C_IT_TCI, ENABLE);	//transfer complete
	I2C_ITConfig(I2C1, I2C_IT_TXI, ENABLE);	//Transmit buffer interrupt status
	I2C_ITConfig(I2C1, I2C_IT_NACKF, ENABLE);	//Not acknowledge
	I2C_ITConfig(I2C1, I2C_IT_RXI, ENABLE);	//receiver full
	
	
}


/*=======================================================================*//**
  @brief Inits DS1621 temperature sensor
*//*========================================================================*/
void I2C_fInitTempSensor (void)
{
	static uint8_t data[5];
	static uint32_t delay;
	
	data[0] = 0xAC; //command: Access config
	data[1] = 0x02;	//data: Output is active High, continuous mode
	
	delay = 1;
	while(I2C_fGetTransmissionComplete() && ++delay){};
	if(delay == 0)
	{
		printf("I2C tr complete timeout");
		return;	//waited too long
	}
	I2C_fSendBytes(0x90, data, 2);
		
	data[0] = 0xA1; //command: Access TH - High temp treshold
	data[1] = 30;	//data: MSB - 30.0 C
	data[1] = 0;	//data: LSB
	
	delay = 1;
	while(I2C_fGetTransmissionComplete() && ++delay){};
	if(delay == 0)
	{
		printf("I2C tr complete timeout");
		return;	//waited too long
	}
	I2C_fSendBytes(0x90, data, 3);
		
	data[0] = 0xA2; //command: Access TL - Low temp treshold
	data[1] = 18;	//data: MSB - 18.0 C
	data[1] = 0;	//data: LSB
	
	delay = 1;
	while(I2C_fGetTransmissionComplete() && ++delay){};
	if(delay == 0)
	{
		printf("I2C tr complete timeout");
		return;	//waited too long
	}
	I2C_fSendBytes(0x90, data, 3);
		
	data[0] = 0xEE; //command: Start conversion
	
	delay = 1;
	while(I2C_fGetTransmissionComplete() && ++delay){};
	if(delay == 0)
	{
		printf("I2C tr complete timeout");
		return;	//waited too long
	}
	I2C_fSendBytes(0x90, data, 1);
	
}


/*=======================================================================*//**
  @brief 
*//*========================================================================*/
void I2C_fSendBytes (uint8_t addr, uint8_t* data, uint8_t bytes)
{
	uint8_t ctr;
	
	if (m_TransmissionRuning == 1) return;
	m_TransmissionRuning = 1;
	
	//save transmission data
	m_slave.address = addr;
	m_slave.nbytes = bytes;
	m_slave.nbytes_left = bytes;
	
	for(ctr = 0; ctr < bytes; ctr++)
	{
		m_slave.data[ctr] = data[ctr];
	}
	
	//start transmission
	I2C_TransferHandling(I2C1, m_slave.address = addr, m_slave.nbytes, 
											I2C_AutoEnd_Mode, I2C_Generate_Start_Write);	//write
}

/*=======================================================================*//**
  @brief Reads a number of bytes from a device
	@param	addr -> device address
	@param	start -> sub address of device
*//*========================================================================*/
void I2C_fReadBytes (uint8_t addr, uint8_t start, uint8_t* data, uint8_t bytes)
{
	if (m_TransmissionRuning == 1) return;
	m_TransmissionRuning = 1;
	
	//save transmission data
	m_slave.address = addr;
	m_slave.nbytes = 1;	//only send one byte: sub address
	m_slave.nbytes_left = 1;
	
	m_slave.data[0] = start;	//save sub address
	
	//start transmission
	I2C_TransferHandling(I2C1, m_slave.address = addr, 1, 
											I2C_SoftEnd_Mode, I2C_Generate_Start_Write);	//write
	
	while(m_TransmissionRuning == 1);	//wait for transmission to complete
	m_TransmissionRuning = 1;
	
	//save transmission data
	m_slave.address = addr;
	m_slave.nbytes = bytes;
	m_slave.nbytes_left = bytes;
	m_slave.read_reg = data;
	
	//start transmission to read bytes
	I2C_TransferHandling(I2C1, m_slave.address = addr, m_slave.nbytes, 
											I2C_AutoEnd_Mode, I2C_Generate_Start_Read);	//write
}

/*=======================================================================*//**
  @brief Returns the transmission status
*//*========================================================================*/
uint8_t I2C_fGetTransmissionComplete (void)
{
	return m_TransmissionRuning;
}

/*=======================================================================*//**
  @brief Reads out temperature data
*//*========================================================================*/
uint8_t I2C_fGetTemp (void)
{
	uint8_t temperature;
	uint8_t data[2];
	
	while(I2C_fGetTransmissionComplete()){};
	I2C_fReadBytes(0x91, 0xAA, data, 2);	//receive 2 bytes of data.
		
	while(I2C_fGetTransmissionComplete()){};
	temperature = data[0];
		
	return temperature;
}


/*****************************************************************************
* ISR
*****************************************************************************/ 

/*=======================================================================*//**
  @brief I2C1 Event Interrupt
*//*========================================================================*/
void I2C1_EV_IRQHandler (void)
{
	//check if new data must be written to TXDR
	if( I2C1->ISR & I2C_ISR_TXIS )
	{
		if(m_TransmissionRuning == 0) return;
		
		I2C1->TXDR = m_slave.data[m_slave.nbytes - m_slave.nbytes_left];
		
		//if this was the last byte
		if(m_slave.nbytes_left == 1) {m_TransmissionRuning = 0;}
		else m_slave.nbytes_left--;
	}
	
	//if transmission was not acknowledged
	if( I2C1->ISR & I2C_ISR_NACKF )
	{
		I2C1->ICR |= I2C_ICR_NACKCF;	//clear flag
		m_TransmissionRuning = 0;	//stop transmission
		printf("i2c1 NACK!\r\n");
	}
	
	//receive register full
	if( I2C1->ISR & I2C_ISR_RXNE )
	{
		if(m_TransmissionRuning == 0) return;
		
		m_slave.read_reg[m_slave.nbytes - m_slave.nbytes_left] = (uint8_t) I2C1->RXDR;
		
		//if this was the last byte
		if(m_slave.nbytes_left == 1) {m_TransmissionRuning = 0;}
		else m_slave.nbytes_left--;
	}
	
	
}

/*=======================================================================*//**
  @brief I2C1 Error Interrupt
*//*========================================================================*/
void I2C1_ER_IRQHandler (void)
{
	__asm("NOP");
	printf("err!\r\n");
	m_TransmissionRuning = 0;
}



/* EOF */
