/* Clock Calibration Header File                                            */
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
#ifndef __CLOCKCALIBRATION_H__
#define __CLOCKCALIBRATION_H__

typedef enum
{
	CLOCK_SUCCESS,
	FAILURE_1,
	FAILURE_2,
	FAILURE_3,
	FAILURE_4,
	FAILURE_5,
	FAILURE_6,
	FAILURE_7,
	IN_PROGRESS
}ERROR_CODE_t;

int setClkCalRegRWCallback (ERROR_CODE_t (*regReadPtr)(unsigned short regAddr, unsigned short* regData) , ERROR_CODE_t (*regWritePtr)(unsigned short regAddr, unsigned short regData) );

ERROR_CODE_t clockCalibrationInit(float *dataRate, unsigned int timeCollectMS);

ERROR_CODE_t clockCalibration(unsigned short X1, unsigned short X2, unsigned short Y1, unsigned short Y2, uint32_t timeStamp);

float getVersion_CLK(void);

#endif /*__CLOCKCALIBRATION_H__*/
