
/******************** (C) COPYRIGHT 2011 Jami***********************************
* File Name		: BH1750FVI.h
* Author		: JamiLiang At Gmail.com
* Date			: 2012/08/01
* Description	: This file provides all the xxx Module functions.
* Version		: V0.1
* ChangeLog	:
* Version		Name       		Date			Description
  0.1			JamiLiang		2012/08/01		Initial Version
   	
*******************************************************************************/
#ifndef _BH1750FVI_H_
#define _BH1750FVI_H_
//-#include "VirtualI2C.h"

/* Includes ------------------------------------------------------------------*/
//#include "Gpio.h"

/* External variables --------------------------------------------------------*/
/* External functions --------------------------------------------------------*/

/* Public typedef ------------------------------------------------------------*/
/* Public define -------------------------------------------------------------
#define BH1750FVI_PORT				Port1
#define BH1750FVI_SDA_PIN			Pin6
#define BH1750FVI_SCL_PIN			Pin7
*/
#define BH1750FVI_ADD				0x46 //ADD line = 0V


#define BH1750FVI_CMD_POWER_OFF				0x00
#define BH1750FVI_CMD_POWER_ON				0x01
#define BH1750FVI_CMD_RESET					0x07

#define BH1750FVI_CMD_CHRM					0x10//1lx		Tm=120ms
#define BH1750FVI_CMD_CHRM2					0x11//0.5lx		Tm=120ms
#define BH1750FVI_CMD_CLRM					0x13//4lx		Tm=16ms

#define BH1750FVI_CMD_OTHRM					0x20
#define BH1750FVI_CMD_OTHRM2				0x21
#define BH1750FVI_CMD_OTLRM					0x23

/* Public macro --------------------------------------------------------------
#define BH_SCLL			setGpiosOuput(BH1750FVI_PORT,BH1750FVI_SCL_PIN,Reset)
#define BH_SCLH			setGpiosOuput(BH1750FVI_PORT,BH1750FVI_SCL_PIN,Set)

#define BH_SDAL			setGpiosOuput(BH1750FVI_PORT,BH1750FVI_SDA_PIN,Reset)
#define BH_SDAH			setGpiosOuput(BH1750FVI_PORT,BH1750FVI_SDA_PIN,Set)

#define BH_SDAIN		setGpioDiretion(BH1750FVI_PORT,BH1750FVI_SDA_PIN,Input)
#define BH_SDAOUT		setGpioDiretion(BH1750FVI_PORT,BH1750FVI_SDA_PIN,Output)

#define BH_SDA			getGpioInput(BH1750FVI_PORT,BH1750FVI_SDA_PIN)
*/
/* Public variables ----------------------------------------------------------*/
/* Public function prototypes ------------------------------------------------*/
void initBH1750FVI(void);

UINT16 getLux(UINT8 rMode);

/* Public functions ----------------------------------------------------------*/


#endif /*_BH1750FVI_H_*/
