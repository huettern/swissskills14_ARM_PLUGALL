/*****************************************************************************
* Project:            PLUGALL
* Target:             
* Type:               PLUGALL Bus controller module
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
#include "plugall.h"
#include "delay.h"

/*****************************************************************************
* Bus Settings
*****************************************************************************/  
#define PLG_BUS_SPEED 10	//in [ms] how fast to change address. MIN 2!



/*****************************************************************************
* Private Data
*****************************************************************************/   

//contains PLG device settings
typedef struct
{
	uint8_t						address;
	GPIOMode_TypeDef	data_dir;
	uint8_t						data;
} PLG_DeviceType;

//create devices
PLG_DeviceType PLG_Devices [PLG_NUM_OF_DEVICES];

uint8_t PLG_BusRunning = 0;

uint8_t m_ClavRising = 0;
uint8_t m_ClavFalling = 0;

uint32_t m_ClavStopCtr = 0;
uint32_t m_ClavEnterCtr = 0;
uint32_t m_ClavDownCtr = 0;
uint32_t m_ClavUpCtr = 0;

uint8_t m_ClavStopLPFired = 0;
uint8_t m_ClavEnterLPFired = 0;
uint8_t m_ClavDownLPFired = 0;
uint8_t m_ClavUpLPFired = 0;

PLG_tpfVoidCallback m_Clav_Handlers[12] = {0};
uint32_t m_Clav_Handlers_times[4] = {0};

/*****************************************************************************
* PRIVATE Functions
*****************************************************************************/ 

/*=======================================================================*//**
  @brief Write single GPIO bit
*//*========================================================================*/
void fWrite_Bit (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t data)
{
	if(data == 0)
		GPIOx->BRR = GPIO_Pin ;
	else
		GPIOx->BSRR = GPIO_Pin;
}

/*=======================================================================*//**
  @brief Enables Bus by setting CS3 to 0
*//*========================================================================*/
void fEnableBus (void)
{
	fWrite_Bit(GPIOA, GPIO_Pin_3, 0); //CS3 is used as enable
}

/*=======================================================================*//**
  @brief Enables Bus by setting CS3 to 1
*//*========================================================================*/
void fDisableBus (void)
{
	fWrite_Bit(GPIOA, GPIO_Pin_3, 1); //CS3 is used as enable
}

/*****************************************************************************
* Public Functions
*****************************************************************************/ 
/*==========================================================================*/
void PLG_fInit (void)
/*----------------------------------------------------------------------------
  Description: Initializes Plugall Bus
  ==========================================================================*/
{
  static GPIO_InitTypeDef  		 		GPIO_InitStruct;
	
	static RCC_ClocksTypeDef RCC_Clocks;
	
	RCC_GetClocksFreq(&RCC_Clocks);
	
	
	/************* GPIOA **************/
	//Port A: CS0..3, D4..5, NSS -> OUTPUT
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);	//set NSS 
	
	//Port A: SPI0..3, USART0..1, INT0..1    -> Alternate Funtcion
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | //SPI
														GPIO_Pin_9 | GPIO_Pin_10 | //USART
														GPIO_Pin_8 | GPIO_Pin_15;	//INT0..1
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	
	/************* GPIOB **************/
  //Port B: D6 -> OUTPUT
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
  //Port B: I2C0..1 -> Alternate Funtcion
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_OType  = GPIO_OType_OD;	 //opendrain
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;	//pullup
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	
	/************* GPIOC **************/
  //Port C: D0..3, LED0 -> OUTPUT
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 |GPIO_Pin_7 |GPIO_Pin_8 |GPIO_Pin_9 | //D0..3
														 GPIO_Pin_12;	//LED0
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	
  //Port C: DAC0..1 -> Alternate Function 
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;	//DAC0..1
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	//Port C: AIN0..3 -> GPIO_Mode_AN
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1 |GPIO_Pin_2 |GPIO_Pin_3; //AIN0..3
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	
	/************* GPIOD **************/
  //Port D: LED1 -> OUTPUT
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;	//LED1
	GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	
	
	/************* GPIOE **************/
  //Port D: DATA_TR -> OUTPUT
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;	//DATA_TR
	GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	
	/************* GPIOF **************/
  //Port D: D7 -> INPUT
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;	//D7
	GPIO_Init(GPIOF, &GPIO_InitStruct);
	
	//Set Databus Start state to input
	PLG_fSetDataDirection(PLG_OUTPUT);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);    //4 bits for preemp priority 0 bit for sub priority
	
	
	DLY_fAddTimerTick(DLY_CONTINUOUS, 10, PLG_fHandleLongPress);	//init long press handlers
	
	/************* Bus Devices ***********/
	//Switches
	PLG_Devices[PLG_SWITCH].address = 0;
	PLG_Devices[PLG_SWITCH].data_dir = PLG_INPUT;
	
	//LED
	PLG_Devices[PLG_LED].address = 1;
	PLG_Devices[PLG_LED].data_dir = PLG_OUTPUT;
	PLG_Devices[PLG_LED].data = 0xFF;
	
	//Clav
	PLG_Devices[PLG_CLAV].address = 2;
	PLG_Devices[PLG_CLAV].data_dir = PLG_INPUT;
	PLG_Devices[PLG_CLAV].data = 0xFF;
	
}

/*=======================================================================*//**
  @brief sets the databus direction
*//*========================================================================*/
void PLG_fSetDataDirection (GPIOMode_TypeDef Mode)
{
  static GPIO_InitTypeDef GPIO_InitStruct;
	
	
	//check to not put two driving sources on the bus
	if(Mode == GPIO_Mode_OUT)
	{
		//Set Buffer direction 
			GPIO_WriteBit(GPIOE, GPIO_Pin_9, Bit_SET);
	}		
	
	//D0..3
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode = Mode;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	//D4..5
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode = Mode;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	//D6
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode = Mode;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	//D7
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode = Mode;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOF, &GPIO_InitStruct);	
	
	
	if(Mode == GPIO_Mode_IN)
	{
		//Set Buffer direction 
			GPIO_WriteBit(GPIOE, GPIO_Pin_9, Bit_RESET);
	}		
}

/*=======================================================================*//**
  @brief Writes Data Bus Bits D0..7
*//*========================================================================*/
void PLG_fWriteDataBus (uint8_t data)
{	
	fWrite_Bit(GPIOC, GPIO_Pin_6,  (data & 0x01) );	//D0
	fWrite_Bit(GPIOC, GPIO_Pin_7,  ((data & 0x02)  >> 1));	//D1
	fWrite_Bit(GPIOC, GPIO_Pin_8,  ((data & 0x04)  >> 2));	//D2
	fWrite_Bit(GPIOC, GPIO_Pin_9,  ((data & 0x08)  >> 3));	//D3
	fWrite_Bit(GPIOA, GPIO_Pin_11, ((data & 0x10)  >> 4));	//D4
	fWrite_Bit(GPIOA, GPIO_Pin_12, ((data & 0x20)  >> 5));	//D5
	fWrite_Bit(GPIOB, GPIO_Pin_8,  ((data & 0x40)  >> 6));	//D6
	fWrite_Bit(GPIOF, GPIO_Pin_6,  ((data & 0x80)  >> 7));	//D7
}

/*=======================================================================*//**
  @brief Reads in the Data Bus D0..7 
*//*========================================================================*/
uint8_t PLG_fReadDataBus (void)
{
	uint8_t data;
	
	data = 0;
	
	data |= (GPIOC->IDR & GPIO_Pin_6) >> 6; 	//D0
	data |= (GPIOC->IDR & GPIO_Pin_7) >> 6; 	//D1
	data |= (GPIOC->IDR & GPIO_Pin_8) >> 6; 	//D2
	data |= (GPIOC->IDR & GPIO_Pin_9) >> 6; 	//D3
	data |= (GPIOA->IDR & GPIO_Pin_11) >> 7; //D4
	data |= (GPIOA->IDR & GPIO_Pin_12) >> 7; //D5
	data |= (GPIOB->IDR & GPIO_Pin_8) >> 2; 	//D6
	data |= (GPIOF->IDR & GPIO_Pin_6) << 1; 	//D7
	
	return data;
}

/*=======================================================================*//**
  @brief Writes Adress Bus Bits CS0..3
*//*========================================================================*/
void PLG_fSetAdrBus (uint8_t adr)
{
	fWrite_Bit(GPIOA, GPIO_Pin_0,  (adr & 0x01)); //CS0
	fWrite_Bit(GPIOA, GPIO_Pin_1, ((adr & 0x02)  >> 1)); //CS1
	fWrite_Bit(GPIOA, GPIO_Pin_2, ((adr & 0x04)  >> 2)); //CS2
	//fWrite_Bit(GPIOA, GPIO_Pin_3, ((adr & 0x04)  >> 3)); //CS3 is used as enable
}

/*=======================================================================*//**
  @brief Writes Data to the device Buffer
*//*========================================================================*/
void PLG_fWriteDeviceData (uint8_t device, uint8_t data)
{
	PLG_Devices[device].data = data;
}

/*=======================================================================*//**
  @brief Reads Data from the device Buffer
*//*========================================================================*/
uint8_t PLG_fReadDeviceData (uint8_t device)
{
	return PLG_Devices[device].data;
}

/*=======================================================================*//**
  @brief Returns last read Switch value
*//*========================================================================*/
uint8_t PLG_fReadSwitch (void)
{
	return PLG_Devices[PLG_SWITCH].data;
}

/*=======================================================================*//**
  @brief Writes LED Data
*//*========================================================================*/
void PLG_fWriteLED (uint8_t data)
{
	PLG_Devices[PLG_LED].data = data ^ 0xFF;
}

/*=======================================================================*//**
  @brief Returns the clavier value
*//*========================================================================*/
uint8_t PLG_fGetClav (void)
{
	return (PLG_Devices[PLG_CLAV].data & 0x0F) ^ 0x0F;
}

/*=======================================================================*//**
  @brief Returns the clavier value in a Union
*//*========================================================================*/
void PLG_fGetClavUnion (PLG_ClavUnion* data)
{
	data->Raw = (PLG_Devices[PLG_CLAV].data & 0x0F) ^ 0x0F;
}

/*=======================================================================*//**
  @brief Stops automatic Bus controlling
*//*========================================================================*/
void PLG_StopBusControl (void)
{
	PLG_BusRunning = 0;
}

/*=======================================================================*//**
  @brief Starts automatic Bus controlling
*//*========================================================================*/
void PLG_StartBusControl (void)
{
	PLG_BusRunning = 1;
}

/*=======================================================================*//**
  @brief Subscribes a Handler to call on a Clav btn rising or falilng edge
	@param type -> which button and rise/fall -> see header
*//*========================================================================*/
void PLG_SubscribeHandler (uint8_t type, PLG_tpfVoidCallback callback)
{
	m_Clav_Handlers[type] = callback;
}

/*=======================================================================*//**
  @brief Subscribes a Handler to call on a Clav btn rising or falilng edge
	@param type -> which button and rise/fall -> see header
	@param time -> longpress time in ms (10ms stepts!)
*//*========================================================================*/
void PLG_SubscribeLongPressHandler (uint8_t type, uint16_t time, PLG_tpfVoidCallback callback)
{
	m_Clav_Handlers[type] = callback;
	m_Clav_Handlers_times[type - 8] = time / 10;	//timesteps in 10 ms
}

/*=======================================================================*//**
  @brief Enables PWM output on all Databus pins and turns em as outputs
*//*========================================================================*/
void PLG_EnablePWM (void)
{
	static TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStruct;
  static TIM_OCInitTypeDef        outputChannelInit;
  static GPIO_InitTypeDef  		 		GPIO_InitStruct;
	
	PLG_StopBusControl();
	PLG_fSetAdrBus(PLG_LED);
	PLG_fSetDataDirection(PLG_OUTPUT);
	
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 |GPIO_Pin_7 |GPIO_Pin_8 |GPIO_Pin_9;//D0..3
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;	//D7
	GPIO_Init(GPIOF, &GPIO_InitStruct);
	
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_10);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_10);
  GPIO_PinAFConfig(GPIOF, GPIO_PinSource6, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_2);
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
  TIM_TimeBaseInitStruct.TIM_Prescaler =        63;  //0..65535 divide SYSCLK by (63 + 1)
  TIM_TimeBaseInitStruct.TIM_CounterMode =      TIM_CounterMode_Up;
  TIM_TimeBaseInitStruct.TIM_Period =           1000;  //0..65535 maximum pwm duty and 1kHz freq
  TIM_TimeBaseInitStruct.TIM_ClockDivision =    TIM_CKD_DIV1;
  TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
  
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);
  TIM_Cmd(TIM3, ENABLE);
  TIM_Cmd(TIM4, ENABLE);
	
  // init output pin
  outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
  outputChannelInit.TIM_Pulse = 200;
  outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
  outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_Low;
	
  TIM_OC1Init(TIM3, &outputChannelInit);	//D0
  TIM_OC2Init(TIM3, &outputChannelInit);	//D1
  TIM_OC3Init(TIM3, &outputChannelInit);	//D2
  TIM_OC4Init(TIM3, &outputChannelInit);	//D3
  TIM_OC1Init(TIM4, &outputChannelInit);	//D4
  TIM_OC2Init(TIM4, &outputChannelInit);	//D5
  TIM_OC3Init(TIM4, &outputChannelInit);	//D6
  TIM_OC4Init(TIM4, &outputChannelInit);	//D7
	
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	TIM_SetCompare1(TIM3, 0);	//sets PWM duty to 0
	TIM_SetCompare2(TIM3, 125);	//sets PWM duty to 0
	TIM_SetCompare3(TIM3, 250);	//sets PWM duty to 0
	TIM_SetCompare4(TIM3, 375);	//sets PWM duty to 0
	TIM_SetCompare1(TIM4, 500);	//sets PWM duty to 0
	TIM_SetCompare2(TIM4, 625);	//sets PWM duty to 0
	TIM_SetCompare3(TIM4, 750);	//sets PWM duty to 0
	TIM_SetCompare4(TIM4, 1000);	//sets PWM duty to 0
}

/*=======================================================================*//**
  @brief Disables PWM and reinits as normal bus controll
*//*========================================================================*/
void PLG_DisablePWM (void)
{
	
	PLG_fInit();
	PLG_StartBusControl();
}

/*=======================================================================*//**
  @brief Handles Bus Controll. Must be called in 1 ms tick
*//*========================================================================*/
void PLG_fHandleBus (void)
{
	static uint16_t msCtr = 0;
	static uint8_t current_device = 0;
	static uint8_t bus_restart = 0;
	static uint8_t clavNew, clavOld;
	
	if(PLG_BusRunning == 0) 
	{
		bus_restart = 1;
		return;
	}
	
	if(bus_restart == 1) 
	{
		msCtr = 0;
		//current_device = 0;
		bus_restart = 0;
	}
	
	msCtr++;
	
	if (msCtr != PLG_BUS_SPEED) return;
	
	msCtr = 0;
	
	if(current_device >= (PLG_NUM_OF_DEVICES-1) ) current_device = 0;
	else current_device++;
	
	fDisableBus();
	
	//set address and data direction
	PLG_fSetAdrBus(PLG_Devices[current_device].address);
	PLG_fSetDataDirection(PLG_Devices[current_device].data_dir);
	
	//if output type, set data
	if(	PLG_Devices[current_device].data_dir == PLG_OUTPUT )
	{
		PLG_fWriteDataBus(PLG_Devices[current_device].data);
	}
	
	//re-enable bus
	fEnableBus();
	
	//if input type, read data
	if(PLG_Devices[current_device].data_dir == PLG_INPUT)
	{
		//if Clav is selected, controll rising and falling edges
		if(PLG_Devices[current_device].address == PLG_CLAV)
		{
			clavNew = PLG_fReadDataBus();
			clavNew = (clavNew ^ 0xff) & 0x0f;
			
			clavOld = (PLG_Devices[current_device].data ^ 0xff) & 0x0f;
			
			/* check for rising edges */
			m_ClavRising = ((clavOld ^ clavNew) & clavNew) & 0x0f;
			/* check for falling edges */
			m_ClavFalling = ((clavOld ^ clavNew) & ~clavNew) & 0x0f;
			
			/* save new value */
			PLG_Devices[current_device].data = clavNew ^ 0x0f;
		}
		
		else
		{
			PLG_Devices[current_device].data = PLG_fReadDataBus();
		}
	}
}

/*=======================================================================*//**
  @brief Is called periodically. Controlls Push Events on CLAV
*//*========================================================================*/
void PLG_fHandleEvents (void)
{
	/* check rising edges on clav */
	
	//nothing todo
	//if ( (m_ClavRising == 0) && (m_ClavFalling == 0) ) return;
	
	/* check every case and execute callback if saved */
	if( (m_ClavFalling & 0x01) && (m_Clav_Handlers[PLG_CLAV_STOP_FALL]) ) 
		m_Clav_Handlers[PLG_CLAV_STOP_FALL]();
	if( (m_ClavFalling & 0x02) && (m_Clav_Handlers[PLG_CLAV_ENTER_FALL]) ) 
		m_Clav_Handlers[PLG_CLAV_ENTER_FALL]();
	if( (m_ClavFalling & 0x04) && (m_Clav_Handlers[PLG_CLAV_DOWN_FALL]) ) 
		m_Clav_Handlers[PLG_CLAV_DOWN_FALL]();
	if( (m_ClavFalling & 0x08) && (m_Clav_Handlers[PLG_CLAV_UP_FALL]) ) 
		m_Clav_Handlers[PLG_CLAV_UP_FALL]();
	if( (m_ClavRising & 0x01) && (m_Clav_Handlers[PLG_CLAV_STOP_RISE]) ) 
		m_Clav_Handlers[PLG_CLAV_STOP_RISE]();
	if( (m_ClavRising & 0x02) && (m_Clav_Handlers[PLG_CLAV_ENTER_RISE]) ) 
		m_Clav_Handlers[PLG_CLAV_ENTER_RISE]();
	if( (m_ClavRising & 0x04) && (m_Clav_Handlers[PLG_CLAV_DOWN_RISE]) ) 
		m_Clav_Handlers[PLG_CLAV_DOWN_RISE]();
	if( (m_ClavRising & 0x08) && (m_Clav_Handlers[PLG_CLAV_UP_RISE]) ) 
		m_Clav_Handlers[PLG_CLAV_UP_RISE]();
	
	/* longpress handler */
	if( (m_Clav_Handlers_times[0] == m_ClavStopCtr) && (m_ClavStopLPFired == 0) && (m_Clav_Handlers[PLG_CLAV_STOP_LONGPRESS]) )
	{
		m_Clav_Handlers[PLG_CLAV_STOP_LONGPRESS]();
		m_ClavStopLPFired = 1;
	}
	if( (m_Clav_Handlers_times[1] == m_ClavEnterCtr) && (m_ClavEnterLPFired == 0) && (m_Clav_Handlers[PLG_CLAV_ENTER_LONGPRESS]) )
	{
		m_Clav_Handlers[PLG_CLAV_ENTER_LONGPRESS]();
		m_ClavEnterLPFired = 1;
	}
	if( (m_Clav_Handlers_times[2] == m_ClavDownCtr) && (m_ClavDownLPFired == 0) && (m_Clav_Handlers[PLG_CLAV_DOWN_LONGPRESS]) )
	{
		m_Clav_Handlers[PLG_CLAV_DOWN_LONGPRESS]();
		m_ClavDownLPFired = 1;
	}
	if( (m_Clav_Handlers_times[3] == m_ClavUpCtr) && (m_ClavUpLPFired == 0) && (m_Clav_Handlers[PLG_CLAV_UP_LONGPRESS]) )
	{
		m_Clav_Handlers[PLG_CLAV_UP_LONGPRESS]();
		m_ClavUpLPFired = 1;
	}
	
	/* clear all flags */
	m_ClavRising = 0;
	m_ClavFalling = 0;
}

/*=======================================================================*//**
  @brief Handles the long press events. Must be called every 10ms!
*//*========================================================================*/
void PLG_fHandleLongPress (void)
{
	if((PLG_Devices[PLG_CLAV].data & 0x01) ^ 0x01)	m_ClavStopCtr++;
	else { m_ClavStopCtr = 0; m_ClavStopLPFired = 0; }
	if((PLG_Devices[PLG_CLAV].data & 0x02) ^ 0x02)	m_ClavEnterCtr++;
	else { m_ClavEnterCtr = 0; m_ClavEnterLPFired = 0; }
	if((PLG_Devices[PLG_CLAV].data & 0x04) ^ 0x04)	m_ClavDownCtr++;
	else { m_ClavDownCtr = 0; m_ClavDownLPFired = 0; }
	if((PLG_Devices[PLG_CLAV].data & 0x08) ^ 0x08)	m_ClavUpCtr++;
	else { m_ClavUpCtr = 0; m_ClavUpLPFired = 0; }
}


/*****************************************************************************
* Public Functions
*****************************************************************************/ 
