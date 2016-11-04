/* Clock Calibration Source File                                            */
/* Version 1.3                                                                */

/******************************************************************************
*                                                                             *
* License Agreement                                                           *
*                                                                             *
* Copyright (c) 2013 Analog Devices Inc.  		                              *
* All rights reserved.                                                        *
*                                                                             *
* This source code is intended for the recipient only under the guidelines of *
* the non-disclosure agreement with Analog Devices Inc.                       *
*                                                                             *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR  *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,    *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER      *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING     *
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER         *
* DEALINGS IN THE SOFTWARE.                                                   *
*                                                                             *
*                                                                             *
* This software is intended for use with the ADUX1020 and derivative parts    *
* only                                                                        *
*										                                      *
******************************************************************************/
/*
 * Assumptions : This feature is supported only for ADUX1020 R1 version & multichannel mode set in the configuration file
 */
 
#include <stdint.h>
#include <math.h>
#include "stm32f10x_lib.h"
//-#include "lcd.h"
#include "COMMON.h"
#include "GloblDef.h"
#include "I2c.h"
#include "ADUX1020.h"
#include "channelCalibration.h"
#include "ClockCalibration.h"


#define TARGET_32K_CLOCK	32000
#define DEVIATION_ALLOWED_PERCENT	3

#define DEVIATION_ALLOWED ((TARGET_32K_CLOCK*DEVIATION_ALLOWED_PERCENT)/100)

#define INITIAL_RETRY	3
#define MIN_DATA_RATE_ALLOWED 50

/* MIN_SAMPLES_CNT affects the precision of the 32kHz clock measurement. */
#define MIN_SAMPLES_CNT	600

#define tarval 1280

static ERROR_CODE_t (*regRead)(unsigned short regAddr, unsigned short* regData);
static ERROR_CODE_t (*regWrite)(unsigned short regAddr, unsigned short regData);

//prototype of functions
void enable32kOsc(void);
void disable32kOsc(void);
uint16_t measureOsc(void);
void increaseTrimOsc(int);
void decreaseTrimOsc(int);
void change32MFreq(int);
ERROR_CODE_t calibrate32MClock(void);


typedef enum
{
	INITIAL,
	MEASURE_DATARATE,
	MEASURE_DATARATE_ROLLOVER,
	ADJUST_CLOCK,
	END
}CC_STATE_t;

static CC_STATE_t state = INITIAL;
static uint32_t targetTS;
static uint32_t currentTS;
static uint32_t sampleCntr=0;
static int clockFreq;
static int retry;
static float set_data_rate;
static int decimation_rate;
static unsigned int collect_samples_time_ms = 5000;
static float versionnumber = 1.3;
static int dataRate = 0;



//-static int isRegRWCallbackSet = 0;
float sampleRate[] = {0.1f, 0.2f, 0.5f, 1.0f, 2.0f, 5.0f, 10.0f, 20.0f, 50.0f, 100.0f, 190.0f, 450.0f, 820.0f, 1400.0f, 1400.0f, 1400.0f};




/*
getVersion
*/
float getVersion_CLK()
{
	return versionnumber;
}
/*
setClkCalRegRWCallback
*/
int setClkCalRegRWCallback (ERROR_CODE_t (*regReadPtr)(unsigned short regAddr, unsigned short* regData) , ERROR_CODE_t (*regWritePtr)(unsigned short regAddr, unsigned short regData) )
{
	//-isRegRWCallbackSet = 0;

	if(regReadPtr!=0){
		regRead = regReadPtr;
	}
	else{
		return -1;
	}

	if(regWritePtr!=0){
		regWrite = regWritePtr;
	}
	else{
		return -1;
	}

	//-isRegRWCallbackSet = 1;

	return 0;
}


void enable32kOsc()
{
	regWrite(0x0032, 0x0B0B);
}

void disable32kOsc()
{
	//Give back control to the chip
	regWrite(0x0032, 0x40);
}

uint16_t measureOsc()
{
	uint16_t readreg;
	uint16_t clockCount;
	//Set reg 0x30 bit[5] “r_osc32m_cal_en” to 1.
	regRead(0x0030, &readreg);
	readreg |= 0x20;
	regWrite(0x0030, readreg);

	//After retime, a pulse of 2-cycle @ 32KHz is generated.
	//!!! do we need to wait here???

	//Fast clock counts the clock numbers, and store data in 12-bit register 0x0A.
	regRead(0x000A, &readreg);
	clockCount = readreg & 0x0FFF;

	//Clear reg 0x30 bit[5] “r_osc32m_cal_en” to 0.
	regRead(0x0030, &readreg);
	readreg &= 0xFFDF;
	regWrite(0x0030, readreg);
	return clockCount;
}

void increaseTrimOsc(int step)
{
	uint16_t readreg;
	uint16_t readregtmp;

	regRead(0x001A, &readreg);
	readregtmp = (uint16_t)(readreg & 0xFF);
	if (readregtmp < (255 - step))
	{
		readregtmp = (uint16_t)(readregtmp + step);
		readreg &= 0xFF00;
		readreg |= readregtmp;

		regWrite(0x001A, readreg);
	}
}

void decreaseTrimOsc(int step)
{
	uint16_t readreg;
	uint16_t readregtmp;

	regRead(0x001A, &readreg);
	//readreg = readreg & 0x00FF;

	readregtmp = (uint16_t)(readreg & 0xFF);
	if (readregtmp > (step - 1))
	{
		readregtmp = (uint16_t)(readregtmp - step);
		readreg &= 0xFF00;
		readreg |= readregtmp;

		regWrite(0x001A, readreg);
	}
}

void change32MFreq(int dv)
{
	if (dv > 0)
	{
		//Too high frequency
		increaseTrimOsc(1);
	}
	else
	{
		//Too low frequency
		decreaseTrimOsc(1);
	}
}

ERROR_CODE_t calibrate32MClock(void)
{
	int absdv;
	int dv;
	int prevabsdv;
	int prevdv;
	int targetValue;
	uint16_t currentValue;
	uint16_t prevValue;
	int trycnt;

	enable32kOsc();
	targetValue = 2000;
	currentValue = measureOsc();
	prevValue = currentValue;
	absdv = (int)fabs(targetValue - currentValue);
	dv = (targetValue - currentValue);
	prevabsdv = absdv;
	prevdv = dv;

	while (absdv != 0)
	{
		//Configure OS32M trim register to get register 0x0A close to 2000
		if (dv == 0)
		{
			//We are done.
			break;
		}
		else
		{
			change32MFreq(dv);
		}
		//Because of the clock jitter sometimes we get 
		//same values. That's why we need to repeat the
		//measurement several times.
		trycnt = 18;
		while (trycnt-- > 0)
		{
			//sleep(5);
			//Measure again
			currentValue = measureOsc();
			if ((prevValue == currentValue))
			{
				if (trycnt == 6)
				{
					change32MFreq(dv);
				}
				if (trycnt == 0)
				{
					//If no change - exit with error.
					disable32kOsc();
					return FAILURE_1;
				}
			}
			else
			{
				//OK. Exit the loop.
				trycnt = 0;
			}
		}

		absdv = (int)fabs(targetValue - currentValue);
		dv = (targetValue - currentValue);
		if ((prevdv * dv) < 0)
		{
			//the target crossed
			if (absdv < prevabsdv)
			{
				//the best value is current
				//absdv = 0; below
			}
			else
			{
				//the best value is previous
				if (absdv != 0)
				{
					//Configure OS32M trim register to get register 0x0A close to 2000
					if (dv < 0)
					{
						//Too high frequency
						decreaseTrimOsc(1);
					}
					else
					{
						//Too low frequency
						increaseTrimOsc(1);
					}
				}
			}
			//we are done. Exit the loop.
			absdv = 0;
		}
		prevabsdv = absdv;
		prevdv = dv;
		if (prevValue == currentValue)
		{
			//If no change - exit.
			absdv = 0;
			disable32kOsc();
			return FAILURE_2;
		}
		prevValue = currentValue;
	}
	disable32kOsc();
	return CLOCK_SUCCESS;
}

ERROR_CODE_t clockCalibrationInit(float *dataRate, unsigned int timeCollectMS)
{
	uint16_t readreg;
	//-uint32_t num_samples_second;

	state = INITIAL;
	dataRate = 0;

	regRead(0x0040, &readreg);
	set_data_rate = sampleRate[readreg & 0x000F];
	
	regRead(0x0046, &readreg);
	decimation_rate = (readreg >> 4) & 0x0007;

	decimation_rate = (int)pow(2,decimation_rate);

	set_data_rate = (set_data_rate / decimation_rate);
		
	retry = INITIAL_RETRY;
	dataRate = &set_data_rate; 

/*	num_samples_second = (timeCollectMS*set_data_rate)/1000;

	if(num_samples_second < MIN_SAMPLES_CNT)
	{
		return FAILURE_7;
	}
	*/

	collect_samples_time_ms = timeCollectMS;
	return CLOCK_SUCCESS;
}

ERROR_CODE_t clockCalibration(unsigned short X1, unsigned short X2, unsigned short Y1, unsigned short Y2, uint32_t timeStamp)
{
	if(set_data_rate < MIN_DATA_RATE_ALLOWED)
	{
			return FAILURE_3;
	}
	
	switch(state)
	{
	case INITIAL:
		currentTS = timeStamp;
		targetTS = currentTS;
		targetTS += collect_samples_time_ms;
		sampleCntr=1;
		state = MEASURE_DATARATE;
		if(currentTS > targetTS)
		{
			state = MEASURE_DATARATE_ROLLOVER;
		}

		break;
	case MEASURE_DATARATE_ROLLOVER:
		if(!(timeStamp < targetTS))
		{
			sampleCntr++;
		}
		else
		{
			sampleCntr++;
			state = MEASURE_DATARATE;
		}
		break;
	case MEASURE_DATARATE:
		if(timeStamp < targetTS)
		{
			sampleCntr++;
		}
		else
		{
			dataRate = (sampleCntr*1000/collect_samples_time_ms);
			state = ADJUST_CLOCK;
		}
		break;
	case ADJUST_CLOCK:
		clockFreq = (int)(32000 + 32000 * (dataRate - set_data_rate) / set_data_rate);
		
		if (retry-- > 0)
		{
			if((clockFreq > (TARGET_32K_CLOCK + DEVIATION_ALLOWED)) || (clockFreq < (TARGET_32K_CLOCK - DEVIATION_ALLOWED)))
			{
				//Calibrate 32M
				int diff = clockFreq - TARGET_32K_CLOCK;
				int corr = (int)((double)diff / 545);
				uint16_t trim;
				uint16_t treg;
				regRead(0x0018, &treg);
				trim = treg & 0x3F;
				trim += corr;
				treg &= 0xFFC0;
				treg |= (trim & 0x003F);
				regWrite(0x0018, treg);
				state = INITIAL;
			}
			else
			{
				if ((clockFreq < (TARGET_32K_CLOCK + DEVIATION_ALLOWED)) && (clockFreq > (TARGET_32K_CLOCK - DEVIATION_ALLOWED)))
				{
					return calibrate32MClock();
				}
				
				return FAILURE_6;
			}
		}
		else
		{
		
			return FAILURE_4;
		}
		break;
	default:
		return FAILURE_5;
	}
	return IN_PROGRESS;
}







