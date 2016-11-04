
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
#ifndef _ADUX1020_H_
#define _ADUX1020_H_
//-#include "VirtualI2C.h"

/* Includes ------------------------------------------------------------------*/
//#include "Gpio.h"

/* External variables --------------------------------------------------------*/
/* External functions --------------------------------------------------------*/

/* Public typedef ------------------------------------------------------------*/
//- Public define -------------------------------------------------------------
#define 			ADUX1020_ADD		0xC8

#define PROX_TH1        2000
#define PROX_TH2        20000

//-寄存器定义
#define 			ADUX1020_DEVICE_ID										0x08 //03FC	Chip ID芯片ID号,可以用于识别是否正常操作了芯片 			
#define 			ADUX1020_watchdog 										0x0c //000F 
#define 			ADUX1020_SW_RESET 										0x0f //0000 位0 Software reset; self-resetting bit. Set this bit to 1 to reset the part.		
#define 			ADUX1020_ADC_CTRL											0x10 //1010 STARTUP1 Required start-up write. These bits must be set to 0x0101.
//-#define 			ADUX1020_ADC_REG1											0x11 //004c
                                                           
//-#define 	  	ADUX1020_AFE_TRIM                 		0x12 //5f0c
//-#define 	  	ADUX1020_AFE_TEST                 		0x13 //ada5
//-#define 	  	ADUX1020_REF_CTRL                 		0x14 //0080
//-#define 			ADUX1020_BIAS_PD1                 		0x15 //0000
//-#define 			ADUX1020_TRIM1                    		0x16 //0600
//-#define 			ADUX1020_BTRIM2                   		0x17 //0000
#define 			ADUX1020_OSC32K                   		0x18 //2693	STARTUP2 Required start-up write. These bits must be set to 0x26. 32 kHz oscillator power-down. 32 kHz oscillator calibration value.
//-#define 			ADUX1020_OSC32M                   		0x19 //0004
#define 			ADUX1020_OSC32M_TRIM              		0x1a //4280 OS32M_CALV 32 MHz oscillator calibration value. Reserved Must be set to 0x42.
//-#define 	  	ADUX1020_ADC_POST                 		0x1b //0060
//-#define 	  	ADUX1020_MISC                     		0x1c //2094
#define 	  	ADUX1020_PAD_IO_CTRL              		0x1d //0000 INT pin polarity control. Reserved Must be set to 0.
#define 			ADUX1020_I2CS_CTL                 		0x1e //0001 Reserved Must be set to 1.
#define 			ADUX1020_I2CS_CTL_MATCH           		0x1f //0000 FIFO_TH Minimum number of FIFO words to trigger an interrupt.
#define 			ADUX1020_LED_OFFSET									  0x20 //0320	Gesture mode LED pulse offset (0 μs to 63 μs in 1 μs steps). Gesture mode LED pulse width (0 μs to 31 μs in 1 μs steps).
#define 			ADUX1020_LED_PERIOD               		0x21 //0A13 Gesture mode LED pulse period (0 μs to 255 μs in 1 μs steps). Gesture mode sample period (number of LED pulses).
#define 			ADUX1020_LED_OFFSET_PROX          		0x22 //0320 Proximity mode LED pulse offset (0 μs to 63 μs in 1 μs steps). Proximity mode LED pulse width (0 μs to 31 μs in 1 μs steps).
#define 			ADUX1020_LED_PERIOD_PROX          		0x23 //0113 Proximity mode LED pulse width (0 μs to 31 μs in 1 μs steps). Proximity mode sample period (number of LED pulses).
//-#define 	  	ADUX1020_LED_MASK                 		0x24 //0000
//-#define 	  	ADUX1020_AFE_CTRL                 		0x25 //2412
//-#define 	  	ADUX1020_AFE_CTRL_PROX            		0x26 //2412
//-#define 			ADUX1020_AFE_CTRL2                		0x27 //0022
#define 			ADUX1020_GEST_DI_TH                		0x28 //0000 GEST_DI_TH Gesture detection sensitivity threshold.
#define 			ADUX1020_GEST_DECTION             		0x29 //0300 Orientation control. Gesture Engine Parameter N.
#define 			ADUX1020_PROX_ON_TH1		           		0x2a //0700	Bits[15:0] of proximity ON1 threshold.
#define 			ADUX1020_PROX_OFF_TH1             		0x2b //0600 Bits[15:0] of proximity OFF1 threshold.
#define 			ADUX1020_PROX_ON_TH2              		0x2c //6000 Bits[15:0] of proximity ON2 threshold.
#define 	  	ADUX1020_PROX_OFF_TH2             		0x2d //4000 Bits[15:0] of proximity OFF2 threshold.
#define 	  	ADUX1020_PROX_TH1_HBYTE           		0x2e //0000 Bits[21:16] of proximity OFF1 threshold. Bits[21:16] of proximity ON1 threshold.
#define 	  	ADUX1020_PROX_TH2_HBYTE           		0x2f //0000 Proximity trigger type. Bits[21:16] of proximity OFF2 threshold. Bits[21:16] of proximity ON2 threshold.
#define 			ADUX1020_TEST_MODE                		0x30 //0000 OSC32M_CAL_EN 32 MHz oscillator calibration enable.
//-#define 			ADUX1020_TEST_USE_I2C             		0x31 //0000
//-#define 			ADUX1020_TEST_PD                  		0x32 //0040
//-#define 			ADUX1020_TEST_FORCE_MODE          		0x33 //0000
//-#define 			ADUX1020_TEST_AFE_SEL_MODE        		0x34 //E400
//-#define 			ADUX1020_CH_GAIN12                		0x38 //8080
//-#define 	  	ADUX1020_CH_GAIN34                		0x39 //8080
#define 	  	ADUX1020_CH1_OFFSET               		0x3a //2000 ADC_CH1_OFFSET Offset of the ADC Channel 1 output.
#define 	  	ADUX1020_CH2_OFFSET               		0x3b //2000 Offset of the ADC Channel 2 output.
#define 			ADUX1020_CH3_OFFSET               		0x3c //2000 Offset of the ADC Channel 3 output.
#define 			ADUX1020_CH4_OFFSET               		0x3d //2000 Offset of the ADC Channel 4 output.
#define 			ADUX1020_GEST_SLOPE_TH            		0x3e //0000 Vertical gesture sensitivity threshold.
#define 			ADUX1020_RATE                     		0x40 //806d Gesture Engine Parameter M. Update rate in proximity mode. Update rate in gesture mode.
#define 			ADUX1020_LED_DRV                  		0x41 //1f2f LED driver current for proximity enable. Proximity mode LED driver current. Slew rate control LED driver current.
//-#define 			ADUX1020_OP_TIME1                 		0x42 //4000
//-#define 	  	ADUX1020_OP_TIME2                 		0x43 //0000
#define 	  	ADUX1020_DSAMPLE_TIME              		0x44 //0000 DSAMPLE_TIME System data sampling time.
#define 	  	ADUX1020_OP_MODE                  		0x45 //000F Output of sample mode. Package start in FIFO data. Prevent FIFO overrun enable. FIFO format. Operation mode.
#define 			ADUX1020_DEC_MODE                 		0x46 //0030 Decimation rate for gesture mode. Decimation rate for proximity mode.
#define 			ADUX1020_INT_MASK		            	 	  0x48 //00ef	Interrupt mask (active high).	//-中断屏蔽寄存器
#define 			ADUX1020_INT_STATUS               		0x49 //0000 1 = clear FIFO_STATUS bits. Number of available data bytes in FIFO to be read out. Status of interrupt; each bit is cleared when 1 is written to that bit.
																																						    
//-#define 			ADUX1020_I2CS_STAT   									0x4A //
//-#define 			ADUX1020_SCAN_MODE   									0x4F //
#define 	  	ADUX1020_DATA_BUFFER_OUT							0x60 //Data of next available word (16 bits) in FIFO. 
#define 	  	ADUX1020_READ_X1L    									0x68 
//-#define 	  	ADUX1020_READ_X2L    									0x69
//-#define 	  	ADUX1020_READ_Y1L    									0x6A
//-#define 			ADUX1020_READ_Y2L    									0x6B
//-#define 			ADUX1020_READ_X1H    									0x6C
//-#define 			ADUX1020_READ_X2H    									0x6D
//-#define 			ADUX1020_READ_Y1H    									0x6E
//-#define 			ADUX1020_READ_Y2H    									0x6F
		



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


/* Public functions ----------------------------------------------------------*/



#endif /*_ADUX1020_H_*/
