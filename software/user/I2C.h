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
#ifndef __I2C_H
#define __I2C_H




/*****************************************************************************
* Defines
*****************************************************************************/ 



/*****************************************************************************
* Public Functions
*****************************************************************************/ 
void I2C_fInit (void);
void I2C_fInitTempSensor (void);


void I2C_fSendBytes (uint8_t addr, uint8_t* data, uint8_t bytes);
void I2C_fReadBytes (uint8_t addr, uint8_t start, uint8_t* data, uint8_t bytes);

uint8_t I2C_fGetTransmissionComplete (void);

uint8_t I2C_fGetTemp (void);

#endif
