/* Channel Calibration Header File                                            */
/* Version 1.2                                                                */

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

#ifndef __CHANNELCALIBRATION_H__
#define __CHANNELCALIBRATION_H__

typedef enum
{
	CAL_SUCCESS=0,			// calibration completed successfully

	FEED_MORE_RAWSAMPLES, // Call the offsetCalibration() continuously

	FAIL_NOISY_SIGNAL,	// signal data is noisy hence channel calibration failed
						// Initialize the routine and calibrate again !!!

	RESTART,			// Restart the driver in sample mode again; flush all //intermediate buffers samples and adxu1020 FIFO;

	CAL_FAIL,				// Channel calibration FAILED to calibrate signals within the allowed deviation;
						// Initialize the routine and calibrate again !!!

	RW_REG_TIMEOUT,		//Register read or writes time out occurred

	CALLBACKS_NOT_REGISTERED,// setRegRWCallback () not registered

	MEMORY_ALLOCS_FAIL

} CAL_ERROR_CODE;


int setRegRWCallback (CAL_ERROR_CODE (*regReadPtr)(unsigned short regAddr, unsigned short* regData) , CAL_ERROR_CODE (*regWritePtr)(unsigned short regAddr, unsigned short regData) );

CAL_ERROR_CODE  offsetCalibrationInit(unsigned short tarval);

CAL_ERROR_CODE offsetCalibration(unsigned short X1,  unsigned short X2, unsigned short Y1, unsigned short Y2,unsigned int timeStamp);

float getVersion(void);

#endif /*__CHANNELCALIBRATION_H__*/

