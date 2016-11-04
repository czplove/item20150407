#ifndef __GESTUREANALYSE_H__
#define __GESTUREANALYSE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
	GESTURE_NONE = 0,
	GESTURE_LR,
	GESTURE_RL,
	GESTURE_BT,
	GESTURE_TB,
	GESTURE_CLK,
	GESTURE_UNKNOWN,
	NUMGESTURE
} GestureID;

void ADI_Gesture_Init(void);
void ADI_Gesture_DeInit(void);
void ADI_Gesture_Reset(void);
int ADI_Gesture_ProcessData(int16_t x0, int16_t x1, int16_t y0, int16_t y1);
uint32_t ADI_Gesture_GetResult(void);

void ADI_Gesture_SetTransposeXY(int t);
int ADI_Gesture_GetTransposeXY(void);
void ADI_Gesture_SetInvertX(int t);
int ADI_Gesture_GetInvertX(void);
void ADI_Gesture_SetInvertY(int t);
int ADI_Gesture_GetInvertY(void);

void ADI_Gesture_SetBackgroundCalibration(int en);
int ADI_Gesture_GetBackgroundCalibration(void);
void ADI_Gesture_SetDIDTThreshold(int dt);
int ADI_Gesture_GetDIDTThreshold(void);
int ADI_Gesture_GetThresholdClick(void);
void ADI_Gesture_SetThresholdClick(int32_t tc);

#ifdef __cplusplus
}
#endif

#endif /*__GESTUREANALYSE_H__*/
