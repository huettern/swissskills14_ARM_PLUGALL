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

#ifndef __PLUGALL_H
#define __PLUGALL_H


/*****************************************************************************
* Defines
*****************************************************************************/ 
#define PLG_INPUT  (GPIOMode_TypeDef)GPIO_Mode_IN
#define PLG_OUTPUT (GPIOMode_TypeDef)GPIO_Mode_OUT

/* Devices and their address*/
#define PLG_NUM_OF_DEVICES 3

#define PLG_SWITCH	0
#define PLG_LED			1
#define PLG_CLAV		2

#define PLG_LCD			7 //doesn't count as device, bcs its not handled in automatic bus controll


/* Clav buttons for callback */
#define PLG_CLAV_STOP_FALL 0
#define PLG_CLAV_STOP_RISE 1
#define PLG_CLAV_ENTER_FALL 2
#define PLG_CLAV_ENTER_RISE 3
#define PLG_CLAV_DOWN_FALL 4
#define PLG_CLAV_DOWN_RISE 5
#define PLG_CLAV_UP_FALL	6
#define PLG_CLAV_UP_RISE	7

#define PLG_CLAV_STOP_LONGPRESS 8
#define PLG_CLAV_ENTER_LONGPRESS 9
#define PLG_CLAV_DOWN_LONGPRESS 10
#define PLG_CLAV_UP_LONGPRESS	11


/*****************************************************************************
* Data Types
*****************************************************************************/
typedef struct {
	uint8_t Stop	 :1;
	uint8_t Enter	 :1;
	uint8_t Down 	 :1;
	uint8_t Up 		 :1;
} PLG_ClavButtonsTypeDef;	

typedef union {
	PLG_ClavButtonsTypeDef Button;
	uint8_t Raw;
} PLG_ClavUnion;

typedef void(*PLG_tpfVoidCallback)(void);

/*****************************************************************************
* Public Functions
*****************************************************************************/ 
void PLG_fInit (void);
void PLG_fSetDataDirection (GPIOMode_TypeDef Mode);

void PLG_fWriteDataBus (uint8_t data);
uint8_t PLG_fReadDataBus (void);

void PLG_fSetAdrBus (uint8_t adr);

void PLG_fWriteDeviceData (uint8_t device, uint8_t data);
uint8_t PLG_fReadDeviceData (uint8_t device);

uint8_t PLG_fReadSwitch (void);
void PLG_fWriteLED (uint8_t data);

uint8_t PLG_fGetClav (void);
void PLG_fGetClavUnion (PLG_ClavUnion* data);

void PLG_StopBusControl (void);
void PLG_StartBusControl (void);

void PLG_SubscribeHandler (uint8_t type, PLG_tpfVoidCallback callback);
void PLG_SubscribeLongPressHandler (uint8_t type, uint16_t time, PLG_tpfVoidCallback callback);
	
void PLG_EnablePWM (void);
void PLG_DisablePWM (void);
	
void PLG_fHandleBus (void);
void PLG_fHandleEvents (void);
void PLG_fHandleLongPress (void);

#endif //#ifndef __PLUGALL_H
