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
#ifndef __SPI_H
#define __SPI_H




/*****************************************************************************
* Defines
*****************************************************************************/ 
#define NSS_HIGH() GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET)
#define NSS_LOW() GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);



/*****************************************************************************
* Public Functions
*****************************************************************************/ 
void SPI_fInit (void);

uint16_t SPI_fSend (uint8_t nbits, uint16_t data);
uint16_t SPI_fSendCont (uint8_t nbits, uint16_t data);
void SPI_fChangeClockSettings (uint16_t cpol, uint16_t cpha);

#endif
