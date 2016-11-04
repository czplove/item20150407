
/******************** (C) COPYRIGHT 2011 Jami***********************************
* File Name		: BH1750FVI.c
* Author		: JamiLiang At Gmail.com
* Date			: 2012/08/01
* Description	: This file provides all the xxx Module functions.
* Version		: V0.1
* ChangeLog	:
* Version		Name       		Date			Description
  0.1			JamiLiang		2012/08/01		Initial Version

*******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
//-#include "lcd.h"
#include "COMMON.h"
#include "GloblDef.h"
#include "I2c.h"
#include "ADUX1020.h"
#include "channelCalibration.h"
#include "GestureAnalyse.h"
#include "ClockCalibration.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//ADUX1020 32-bit raw data (will be passed to the algorithm)
/* GESTURE_LR, GESTURE_RL, GESTURE_BT, GESTURE_TB, GESTURE_CLK */
#define GESTURE_MODE_1				0x00000001
#define tarval 1280


UINT32 Gesture_Process_Result = 0xFF;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* External variables ---------------------------------------------------------*/
/* External functions ---------------------------------------------------------*/
extern void led_disp_L_init(void);

#define ADUX1020_Sample_Mode 
//-#define ADUX1020_Gesture_Mode 

void ADUX1020_clock_Calibration(void);



CAL_ERROR_CODE ADUX1020_WriteReg(UINT16 RegName,UINT16 RegValue)
{
	 struct i2c_client client;
	 CAL_ERROR_CODE res = CAL_SUCCESS;
	 UINT8 localReadBuffer[10];
	 
	 client.addr = ADUX1020_ADD;
	 client.num = 2;
	 
	 localReadBuffer[0] = RegName;		//-第一个参数是寄存器首地址
	 localReadBuffer[1] = (RegValue >> 8) & 0xff;		//-高字节
	 localReadBuffer[2] = RegValue & 0xff;		//-低字节
	 res=(CAL_ERROR_CODE)i2c_write_reg_Buffer(&client,localReadBuffer,3);
         if (res )
         {
              res = CAL_SUCCESS; 
         }
         else
         {
              res = CAL_FAIL; 
         }
         return res; 
 }

CAL_ERROR_CODE ADUX1020_ReadReg(UINT16 RegName,UINT16 *RegValue)
{
	 struct i2c_client client;
	 CAL_ERROR_CODE res = CAL_SUCCESS;
	 UINT8 localReadBuffer[10];
	 
	 client.addr = ADUX1020_ADD;
	 client.num = 2;
	 
	 localReadBuffer[0] = RegName;		//-第一个参数是寄存器首地址
	 res = (CAL_ERROR_CODE)i2c_read_reg_Buffer(&client,2,localReadBuffer);

	 *RegValue = (localReadBuffer[0] << 8) + localReadBuffer[1];
         if (res )
         {
              res = CAL_SUCCESS; 
         }
         else
         {
              res = CAL_FAIL; 
         }         
  
	 return res;
}

/**
  * @brief  Switch ADUX1020 to idle mode operation.
  * @param  --
  * @retval None
  *  This function will switch ADUX1020 to idle mode operation
  *
  */
void ADUX1020_IdleMode()
{
  //Set to Idle mode
  ADUX1020_WriteReg(0x33, 0x0008);      /*Force to Active 4 state*/
  ADUX1020_WriteReg(0x45, 0x000e);      /*Force to Active 4 state*/
  ADUX1020_WriteReg(0x45, 0x0000);      /*Set to OFF mode*/
  ADUX1020_WriteReg(0x32, 0x0f4f);      /*Clock control by user*/
  ADUX1020_WriteReg(0x49, 0x8000);      /*Flush FIFO*/
  ADUX1020_WriteReg(0x32, 0x0040);      /*Clock control by chip*/   
}

/**
  * @brief  Switch ADUX1020 to sample mode operation.
  * @param  --
  * @retval None
  *  This function will switch ADUX1020 to sample mode operation
  *
  */
void ADUX1020_SampleMode()
{
  //switch to idle
  ADUX1020_IdleMode();
  
  //Sample Mode setup  
  ADUX1020_WriteReg(0x48, 0x0070);
  ADUX1020_WriteReg(0x45, 0x0148);      /* Sample mode with FIFO overrun prevention */
}

void ADUX1020_GestureMode()
{
  //switch to idle
  ADUX1020_IdleMode();
  
  //Sample Mode setup  
  ADUX1020_WriteReg(0x48, 0x00EF);
  ADUX1020_WriteReg(0x45, 0x0122);      /* Sample mode with FIFO overrun prevention */
  ADUX1020_WriteReg(0x28, 0x0064);
  ADUX1020_WriteReg(0x40, 0x706d);
  ADUX1020_WriteReg(0x29, 0x0300);
  ADUX1020_WriteReg(0x3e, 0x0000);
  
}

/**
  * @brief  Read FIFO data from ADUX1020.
  * @param  RegName: specifies the ADUX1020 register to be read.
  * @retval LM75 register value.
  */
UINT16 ADUX1020_ReadFIFO(void)
{
  //array to store ADUX1020 raw data per byte from FIFO
  UINT8 XY_raw_temp[8] = {0};

  //-UINT32 DataNum = 0;
  UINT16 bufferLen = 0;
  UINT16 retVal = 0;
  struct i2c_client client;
	 //-int Result = 1;
	 UINT8 localReadBuffer[10];
  
  client.addr = ADUX1020_ADD;
	 client.num = 2;
	 
  //reset raw data variables
  ADI_X1 = 0;
  ADI_X2 = 0;
  ADI_Y1 = 0;
  ADI_Y2 = 0;
  
  do
  {
       ADUX1020_ReadReg(0x49, &retVal);
       bufferLen = retVal >> 8;
       if ( bufferLen >= 8)
       {
           break;
       }
  }while(1);
  
    
  //-从传感器中读出数据
  localReadBuffer[0] = 0x60;		//-第一个参数是寄存器首地址
	//-Result=i2c_read_reg_Buffer(&client,8,localReadBuffer);
  i2c_read_reg_Buffer(&client,8,localReadBuffer);
	XY_raw_temp[0] = localReadBuffer[0];
	XY_raw_temp[1] = localReadBuffer[1];
	XY_raw_temp[2] = localReadBuffer[2];
	XY_raw_temp[3] = localReadBuffer[3];
	XY_raw_temp[4] = localReadBuffer[4];
	XY_raw_temp[5] = localReadBuffer[5];
	XY_raw_temp[6] = localReadBuffer[6];
	XY_raw_temp[7] = localReadBuffer[7];
  
  //-下面是对接收到的I2C数据进行整合处理
  ADI_X1 = ((UINT16)XY_raw_temp[0]<<8) + ((UINT16)XY_raw_temp[1]);
  ADI_X2 = ((UINT16)XY_raw_temp[2]<<8) + ((UINT16)XY_raw_temp[3]);
  ADI_Y1 = ((UINT16)XY_raw_temp[4]<<8) + ((UINT16)XY_raw_temp[5]);
  ADI_Y2 = ((UINT16)XY_raw_temp[6]<<8) + ((UINT16)XY_raw_temp[7]);
   
  /* return 0 */
  return 0x0000;    
  
}

/**
  * @brief  Switch ADUX1020 to proximity mode operation.
  * @param  --
  * @retval None
  *  This function will switch ADUX1020 to proximity mode operation
  *
  */
void ADUX1020_ProximityMode()
{
  ADUX1020_IdleMode();
  
  ADUX1020_WriteReg(0x2A, PROX_TH1);      /* ON1 Threshold */
  ADUX1020_WriteReg(0x2C, PROX_TH2);      /* ON2 Threshold */
  ADUX1020_WriteReg(0x48, 0x007A);      /* enable: ON1 INT, ON2 INT, FIFO INT */
  ADUX1020_WriteReg(0x45, 0x0811);      /* Proxmity mode */
  
}

/**
  * @brief  Gesture Analysis using ADUX1020 LSCF algorithm.
  * @param  --
  * @retval Gesture algorithm result
  *  This function will analysis the raw data.
  *
  */
uint32_t ADUX1020_Analysis(void)
{
  int Result = 1;
  uint32_t Gesture_Result = 0;
  
  Result = ADI_Gesture_ProcessData(ADI_X1, ADI_X2, ADI_Y1, ADI_Y2);
  if(Result != 0)
  {
    while(1);		//-这里的死循环值得思考
  }
  
  Gesture_Result = ADI_Gesture_GetResult();


  return Gesture_Result;
}

short data_in[60];

typedef enum {
	G_SIGN_INVALID = 0,
	G_SIGN_WE,
	G_SIGN_EW,
	G_SIGN_SN,
	G_SIGN_NS,
        G_SIGN_HOVER
} ENUM_GESTURE_INDEX;


int sampleCount;
int ADUX1020_GestureResult(void)
{
     short retVal;
     short  DirX, DirY, DirZ;

     int i;
     ENUM_GESTURE_INDEX gest_index;

     ADUX1020_ReadReg(0x49,&retVal);
     
     if( retVal == 0)
     {
          return 0;
     }
     
     sampleCount = retVal >> 8;
     
     if( sampleCount == 0)
     {
         return 0;
     }
     
     ADUX1020_WriteReg(0x49,0x10);
     ADUX1020_WriteReg(0x32,0x0f4f);
     
     
     for( i = 0; i < sampleCount; i++)
     {
        ADUX1020_ReadReg(0x60,&retVal);
        data_in[i] = retVal;
     }    
     
    
    
    i = 0;
    do
    {    
       DirX = (data_in[i] & 0x60) >> 5;
       DirY = (data_in[i] & 0x18) >> 3;
       DirZ = (data_in[i] & 0x80) >> 7;
	
	/** need clarification */
	
	if ((DirX == 1) && (DirY == 0) && (DirZ == 0)) {
		gest_index = G_SIGN_EW;
	} else if ((DirX == 2) && (DirY == 0) && (DirZ == 0)) {
		gest_index = G_SIGN_WE;
	} else if ((DirX == 0) && (DirY == 1) && (DirZ == 0)) {
		gest_index = G_SIGN_SN;
	} else if ((DirX == 0) && (DirY == 2) && (DirZ == 0)) {
		gest_index = G_SIGN_NS;
	} else if ((DirX == 0) && (DirY == 0) && (DirZ == 1)) {
		gest_index = G_SIGN_HOVER;
	} else {
		gest_index = G_SIGN_INVALID;
	}
        if ( gest_index != G_SIGN_INVALID)
        {
            break;
        }
        
        i++;
        if( i >= sampleCount)
        {
            break;
        }
    }while(1);

        ADUX1020_WriteReg(0x32,0x40); 
	return gest_index;
  
}
/**
  * @brief  ADUX1020 Offset Calibration routine.
  * @param  --
  * @retval None
  *
  */
#define MODE_1				0x00000001

void ADUX1020_Algorithm_Init(void)
{
  /* Algorithm Configuration Parameters */
	int DiDtthreshold = 2280;   //-2280常用的参数值
	int Cali = 1;
	int Transpose = 0;
	int InvtX = 0;
	int InvtY = 0;
	int ThresholdClick = 0;
	
	
	/* Initilizing LSCF Algorithm */
#if 0
	ADI_Gesture_Init();
#else
  ADI_Gesture_Reset();
#endif
	
	/* Setting Algorithm Configuration Parameters */
	ADI_Gesture_SetDIDTThreshold(DiDtthreshold);
	ADI_Gesture_SetBackgroundCalibration(Cali);
	ADI_Gesture_SetTransposeXY(Transpose);
	ADI_Gesture_SetInvertX(InvtX);
	ADI_Gesture_SetInvertY(InvtY);
        ADI_Gesture_SetThresholdClick(ThresholdClick);
   
}

unsigned short value_3a,value_3b,value_3c,value_3d;

unsigned short calResult;
void ADUX1020_Offset_Calibration(void)
{
  //-float version = 0.0;
  int status = 0;
  int errorCode;
    
 /* version = getVersion(); */

  status = setRegRWCallback(&ADUX1020_ReadReg,&ADUX1020_WriteReg);
  if(status != CAL_SUCCESS)
    goto exit;
  
  ADUX1020_SampleMode();
 
  errorCode = offsetCalibrationInit(tarval);

  if(errorCode != CAL_SUCCESS)    //-校准到这里没有成功,错误代码是4
    goto exit;
        
  errorCode = FEED_MORE_RAWSAMPLES;
  
  while(errorCode == FEED_MORE_RAWSAMPLES)
  {
    //CLK control by user
    ADUX1020_WriteReg(0x32, 0x0f4f);
    
    ADUX1020_ReadFIFO();

      //CLK control by chip
    ADUX1020_WriteReg(0x32, 0x0040); 
        
    errorCode = offsetCalibration((unsigned short)ADI_X1, (unsigned short)ADI_X2, (unsigned short)ADI_Y1, (unsigned short)ADI_Y2, 0);
    
    if(errorCode == RESTART)
    {
      ADUX1020_IdleMode();
      
      ADUX1020_SampleMode();
      
      errorCode = offsetCalibrationInit(tarval);
      if(errorCode != CAL_SUCCESS)
        goto exit;
      
      errorCode = FEED_MORE_RAWSAMPLES;                  
    }
    else if(errorCode == CAL_SUCCESS)
      goto exit;
    
  }
  
exit:
  calResult = errorCode;
  if (errorCode == CAL_SUCCESS)
  {
      ADUX1020_ReadReg(0x3a, &ADI_CH1_OFFSET);
      ADUX1020_ReadReg(0x3b, &ADI_CH2_OFFSET);
      ADUX1020_ReadReg(0x3c, &ADI_CH3_OFFSET);
      ADUX1020_ReadReg(0x3d, &ADI_CH4_OFFSET);    
  }
  ADUX1020_IdleMode();
  
  //-return errorCode;
}

int initADUX1020(void)	//-也存在第一次操作不成功的情况
{
	 //-struct i2c_client client;
	 //-u8 temp_data8;
	 int Result = 1;
	 //-UINT8 localReadBuffer[10];

	 //-setNextI2cDeviceAddress(BH1750FVI_ADDR);
	 //-initVirtualI2c() ;			//-引脚初始化,已经统一做过

	 //-client.addr = ADUX1020_ADD;
	 //-client.num = 2;

	 STM32_UP_selfT_flag1 |= 0x04;		//-现在的逻辑需要统一清除,先知错,如果全部正确了就消除
	 //-STM32_UP_selfT_flag1 &= 0xfb;
	 //-i2c_read_reg(&client,ADUX1020_DEVICE_ID,&watch_data[5]);		//?这里有个16位和8位的区别,需要注意
	 //-读16位数据
	 //-localReadBuffer[0] = ADUX1020_DEVICE_ID;		//-第一个参数是寄存器首地址
	 //-Result=i2c_read_reg_Buffer(&client,2,localReadBuffer);
	 //-watch_data[3] = localReadBuffer[0];
	 //-watch_data[4] = localReadBuffer[1];
	 //-if(Result <= 0)
	 //-{
	 //-	  STM32_UP_selfT_flag1 |= 0x04;
	 //-}
/*
	 //-写16位数据
	 localReadBuffer[0] = ADUX1020_GEST_DI_TH;		//-第一个参数是寄存器首地址
	 localReadBuffer[1] = 0x26;		//-高字节
	 localReadBuffer[2] = 0x25;		//-低字节
	 res=i2c_write_reg_Buffer(&client,localReadBuffer,3);		  
	 
	 localReadBuffer[0] = ADUX1020_GEST_DI_TH;		//-第一个参数是寄存器首地址
	 res=i2c_read_reg_Buffer(&client,2,localReadBuffer);
	 watch_data[5] = localReadBuffer[0];
	 watch_data[6] = localReadBuffer[1];	
*/	 
	 //-配置手势初始寄存器值,让手势寄存器处于需要的特定状态
	 
	 //-General-Purpose Registers
	 //-写16位数据,软件复位下传感器
	 //-localReadBuffer[0] = ADUX1020_SW_RESET;		//-第一个参数是寄存器首地址
	 //-localReadBuffer[1] = 0x00;		//-高字节
	 //-localReadBuffer[2] = 0x01;		//-低字节
	 //-i2c_write_reg_Buffer(&client,localReadBuffer,3);
	 //-if(res <= 0)
	 //-{
	 //-	  STM32_UP_selfT_flag1 |= 0x04;
	 //-}
	 
	 //-启动必须填写下面的这些数据为什么,启动什么东西?
	 //-localReadBuffer[0] = ADUX1020_ADC_CTRL;		//-第一个参数是寄存器首地址
	 //-localReadBuffer[1] = 0x01;		//-高字节
	 //-localReadBuffer[2] = 0x01;		//-低字节
	 //-res=i2c_write_reg_Buffer(&client,localReadBuffer,3);	 
	 //-if(res <= 0)
	 //-{
	 //-	  STM32_UP_selfT_flag1 |= 0x04;
	 //-}
	 //-Result=ADUX1020_WriteReg(ADUX1020_ADC_CTRL,0x0101);
	 //-if(Result <= 0)
	 //-{
	 //-	  STM32_UP_selfT_flag1 |= 0x04;
	 //-}
	 
	 Result = ADUX1020_WriteReg(0x0C, 0x000F);
	if(Result != 0)
     return Result; 
	 Result = ADUX1020_WriteReg(0x10, 0x1010);
	if(Result != 0)
     return Result; 
	 Result = ADUX1020_WriteReg(0x11, 0x004C);
  if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x12, 0x5f0c);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x13, 0xada5);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x14, 0x0080);
  if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x15, 0x0000);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x16, 0x0600);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x17, 0x0000);
  if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x18, 0x2693);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x19, 0x0004);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x1a, 0x4280);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x1b, 0x0060);
  if(Result != 0)
     return Result;  
  Result = ADUX1020_WriteReg(0x1c, 0x2094);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x1d, 0x0000);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x1e, 0x0001);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x1f, 0x0000);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x20, 0x0320);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x21, 0x0513);
  if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x22, 0x0320);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x23, 0x0113);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x24, 0x0000);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x25, 0x2414);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x26, 0x2414);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x27, 0x0022);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x28, 0x0000);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x29, 0x0300);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x2a, 0x1770);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x2b, 0x157c);
  if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x2c, 0x4268);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x2d, 0x2710);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x2e, 0x0000);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x2f, 0x0000);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x30, 0x0000);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x31, 0x0000);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x32, 0x0040);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x33, 0x0000);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x34, 0xe400);  
  if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x38, 0x8080);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x39, 0x8080);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x3a, 0x1e00);  
 if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x3b, 0x1e00);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x3c, 0x1e00);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x3d, 0x1e00);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x3e, 0x0000);  
   if(Result != 0)
     return Result;
  //sample rate 820Hz; block average point =8; DATA RATE = 102Hz
  Result = ADUX1020_WriteReg(0x40, 0x806c);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x41, 0x1f2f);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x42, 0x4000);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x43, 0x0000);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x44, 0x0005);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x46, 0x0030);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x48, 0x00ef);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x49, 0x0000);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x45, 0x0008);  
   if(Result != 0)
     return Result;
     
  //-return Result;   
	//-ADUX1020_ReadReg(0x45, &ADI_DEC_MODE); 
  //-ADUX1020_ReadReg(0x45, &ADI_DEC_MODE); 
	//-校准 
  ADUX1020_clock_Calibration(); 
  ADUX1020_Offset_Calibration(); 
  
  //-验证是否读写正常
  ADUX1020_ReadReg(0x45, &ADI_DEC_MODE);
	 
	//-设置工作模式
	
//-#ifdef  ADUX1020_Sample_Mode
    //switch ADUX1020 to sample mode
    //-ADUX1020_SampleMode();
    
    //Initialize ADUX1020 LSCF algorithm!
    ADUX1020_Algorithm_Init();        
         
        
//-#endif	
	
//-#ifdef   ADUX1020_Proximity_Mode
    //switch ADUX1020 to Proximity mode
#ifdef ADUX1020_Sample_Mode
    ADUX1020_SampleMode();
#else
   /* ADUX1020_ProximityMode(); */
    ADUX1020_GestureMode();
#endif

    STM32_UP_selfT_flag1 &= 0xfb;	 //-到这里没有提前退出才认为是正常的,没有出错
//-#endif
    return Result;
}

int initADUX1020_re(void)	//-也存在第一次操作不成功的情况
{
	 struct i2c_client client;
	 //-u8 temp_data8;
	 int Result = 1;
	 UINT8 localReadBuffer[10];
	 
	 //-setNextI2cDeviceAddress(BH1750FVI_ADDR);
	 //-initVirtualI2c() ;			//-引脚初始化,已经统一做过
	
	 client.addr = ADUX1020_ADD;
	 client.num = 2;
	 	 
	 STM32_UP_selfT_flag1 &= 0xfb;	 
	 //-i2c_read_reg(&client,ADUX1020_DEVICE_ID,&watch_data[5]);		//?这里有个16位和8位的区别,需要注意
	 //-读16位数据
	 localReadBuffer[0] = ADUX1020_DEVICE_ID;		//-第一个参数是寄存器首地址
	 Result=i2c_read_reg_Buffer(&client,2,localReadBuffer);
	 //-watch_data[3] = localReadBuffer[0];
	 //-watch_data[4] = localReadBuffer[1];
	 if(Result <= 0)
	 {
	 	  STM32_UP_selfT_flag1 |= 0x04;
	 }

	 
	 Result = ADUX1020_WriteReg(0x0C, 0x000F);
	 
	 Result = ADUX1020_WriteReg(0x10, 0x1010);
	 
	 Result = ADUX1020_WriteReg(0x11, 0x004C);
  if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x12, 0x5f0c);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x13, 0xada5);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x14, 0x0080);
  if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x15, 0x0000);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x16, 0x0600);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x17, 0x0000);
  if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x18, ADI_OSC32K);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x19, 0x0004);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x1a, ADI_OSC32M_TRIM);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x1b, 0x0060);
  if(Result != 0)
     return Result;  
  Result = ADUX1020_WriteReg(0x1c, 0x2094);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x1d, 0x0000);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x1e, 0x0001);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x1f, 0x0000);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x20, 0x0320);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x21, 0x0513);
  if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x22, 0x0320);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x23, 0x0113);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x24, 0x0000);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x25, 0x2414);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x26, 0x2414);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x27, 0x0022);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x28, 0x0000);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x29, 0x0300);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x2a, 0x1770);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x2b, 0x157c);
  if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x2c, 0x4268);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x2d, 0x2710);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x2e, 0x0000);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x2f, 0x0000);
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x30, 0x0000);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x31, 0x0000);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x32, 0x0040);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x33, 0x0000);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x34, 0xe400);  
  if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x38, 0x8080);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x39, 0x8080);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x3a, ADI_CH1_OFFSET);  
 if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x3b, ADI_CH2_OFFSET);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x3c, ADI_CH3_OFFSET);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x3d, ADI_CH4_OFFSET);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x3e, 0x0000);  
   if(Result != 0)
     return Result;
  //sample rate 820Hz; block average point =8; DATA RATE = 102Hz
  Result = ADUX1020_WriteReg(0x40, 0x806c);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x41, 0x1f2f);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x42, 0x4000);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x43, 0x0000);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x44, 0x0005);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x46, 0x0030);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x48, 0x00ef);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x49, 0x0000);  
   if(Result != 0)
     return Result;
  Result = ADUX1020_WriteReg(0x45, 0x0008);  
   if(Result != 0)
     return Result;
     
 
	//-校准 
  //-ADUX1020_clock_Calibration(); 	//-同一个环境下校准一次即口.
  //-ADUX1020_Offset_Calibration(); 
  
  //-验证是否读写正常
  //-ADUX1020_ReadReg(0x45, &ADI_DEC_MODE);
	 
	//-设置工作模式
	
//-#ifdef  ADUX1020_Sample_Mode
    //switch ADUX1020 to sample mode
    //-ADUX1020_SampleMode();
    
    //Initialize ADUX1020 LSCF algorithm!
    ADUX1020_Algorithm_Init();        
         
        
//-#endif	
	
//-#ifdef   ADUX1020_Proximity_Mode
    //switch ADUX1020 to Proximity mode
#ifdef ADUX1020_Sample_Mode
    ADUX1020_SampleMode();
#else
   /* ADUX1020_ProximityMode(); */
    ADUX1020_GestureMode();
#endif

//-#endif
    return Result;
}

	 
UINT16 IntensityBuffer[50];
UINT16 bufferCnt = 0;


short LoopCnt;
#define ADI_min 1216      //-1280的正负5%
#define ADI_max 1344

void ADUX1020_sub2(void)
{

  	 UINT16 readVal = 0;
         UINT8 iloop,i;
         UINT16 retVal;
//-     static UINT32 temp_pt=0;

      //-//-测试手势寄存器是否正确
      //-ADUX1020_ReadReg(0x45, &ADI_DEC_MODE);  
      
#ifdef   ADUX1020_Sample_Mode 
      
         //Read FIFO length
         ADUX1020_ReadReg(0x49, &readVal);
         readVal = readVal>>8;

         //CLK control by user
         ADUX1020_WriteReg(0x32, 0x0f4f);
                  
         iloop = readVal / 8;
         LoopCnt = iloop; 
         for(i=0;i<iloop;i++)  
         {
            //read FIFO
            ADUX1020_ReadFIFO();
            
            //-测试用 记录手势四通道值
            //-watch_data[0 + temp_pt*4] = ADI_X1;
            //-watch_data[1 + temp_pt*4] = ADI_X2;
            //-watch_data[2 + temp_pt*4] = ADI_Y1;
            //-watch_data[3 + temp_pt*4] = ADI_Y2;
            
            //-if((watch_data[0 + temp_pt*4] ) > 1408 )  //-1344
            //-  watch_data[0 + temp_pt*4] = 0;
            //-temp_pt++;
            //-if(temp_pt > 127)
            //-  temp_pt = 0;
            //-printf("num%d:ADI_X1= %d,ADI_X2= %d,ADI_Y1= %d,ADI_Y2= %d\n", temp_pt,ADI_X1,ADI_X2,ADI_Y1,ADI_Y2);

            if(((ADI_X1 >= 1344) || (ADI_X1 <= 1216)) && ((ADI_X2 >= 1344) || (ADI_X2 <= 1216))
            	&& ((ADI_Y1 >= 1344) || (ADI_Y1 <= 1216)) && ((ADI_Y2 >= 1344) || (ADI_Y2 <= 1216)))
            {
		            if(ADI_channels_init_flag == 0)
                  ADI_channels_wait_time = Time_2048ms_Counter;
		    				ADI_channels_init_flag = 0x55;    //-需要重新校准    		    				           
    				}
    				else
    				{
    					  ADI_channels_init_flag = 0;
    				}
            
            //Gesture algorithm process!
            retVal = ADUX1020_Analysis();
            
            if( retVal)
            {
                Gesture_Process_Result = retVal;
            }
            switch(Gesture_Process_Result)
            {
              case GESTURE_RL:    //-2从左往右
                //-sprintf((char*)LCDStr, "   RIGHT              ");
                //-LCD_DisplayStringLine(LCD_LINE_6, (uint8_t*)LCDStr); 
                break;
              case GESTURE_LR:    //-1从右往左
                //-sprintf((char*)LCDStr, "   LEFT               ");
                //-LCD_DisplayStringLine(LCD_LINE_6, (uint8_t*)LCDStr);
                break;
              case GESTURE_TB:    //-4从下往上
                if(led_display_start == 0x55)		//-设备ID号:CO2值
                {
                   ps_flag = 2;
                   ps_flag_led = 2;
                }
                else
                {//-不翻页的情况下,需要告知一下主板现在的状态
                	 //-UART1_transmit_flag = YES;		//-可以组织内容发送
                   //-UART1_transmit_control = 9;
                   //-voice_keep_data_flag = 0;
                   ps_flag_led_dis = 0x55;
                }	
                
                ps_flag_led_end = 2;
                ps_flag_led_disp = 2;
                led_disp_L_init();
                ADI_ps_flag = 0x55;		//-识别到了有效手势,暂停手势识别一段时间
                ADI_ps_wait_time = cticks_ms;                
                
                ADI_channels_init_flag = 0;
                //-ADI_channels_wait_time = Time_2048ms_Counter;			//-这是一种没有办法的空闲复位计划,把时间缩短保证不会长时间失灵,把时间加长保证人不会感觉到复位的存在
                																									//-存在的风险就是复位的时候正好碰上人为操作,先这么办,后续再想办法	
                Gesture_Process_Result = 0xff;
                //-sprintf((char*)LCDStr, "   UP                 ");
                //-LCD_DisplayStringLine(LCD_LINE_6, (uint8_t*)LCDStr);
                break;
              case GESTURE_BT:    //-3从上往下
                if(led_display_start == 0x55)		//-设备ID号:CO2值
                {
                   ps_flag = 1;		//-目前对显示没有影响
                   ps_flag_led = 1;
                }
                else
                {
                	 //-UART1_transmit_flag = YES;		//-可以组织内容发送
                   //-UART1_transmit_control = 9;
                   //-voice_keep_data_flag = 0;
                   ps_flag_led_dis = 0x55;
                }
                
                ps_flag_led_end = 1;
                ps_flag_led_disp = 1;
                led_disp_L_init();
                ADI_ps_flag = 0x55;		//-识别到了有效手势,暂停手势识别一段时间
                ADI_ps_wait_time = cticks_ms;
                
                ADI_channels_init_flag = 0;   //-敲击之后如果手势还有效就不需要重新初始化
                //-ADI_channels_wait_time = Time_2048ms_Counter;
                Gesture_Process_Result = 0xff;
                //-sprintf((char*)LCDStr, "   DOWN                ");
                //-LCD_DisplayStringLine(LCD_LINE_6, (uint8_t*)LCDStr);
                break;
              case GESTURE_CLK:
                //-sprintf((char*)LCDStr, "   CLICK               ");
                //-LCD_DisplayStringLine(LCD_LINE_6, (uint8_t*)LCDStr);                
              default:              		
                   
                break;
            }                         	
                      
         }
         //-ADUX1020_ReadReg(0x04, &ADI_PS_Value1);
         //CLK control by chip
         ADUX1020_WriteReg(0x32, 0x0040);
#else

        ADUX1020_ReadReg(0x49,&readVal);
         
         if (  readVal )
         {
              Gesture_Process_Result = ADUX1020_GestureResult();
         }
                  
#endif
}

void readSampleData(int *data,unsigned int *time)
{
      
         //CLK control by user
    ADUX1020_WriteReg(0x32, 0x0f4f);
    
    ADUX1020_ReadFIFO();

    //CLK control by chip
    ADUX1020_WriteReg(0x32, 0x0040);
    
    data[0] = ADI_X1;
    data[1] = ADI_X2;
    data[2] = ADI_Y1;
    data[3] = ADI_Y2;
    *time = cticks_ms;
}

ERROR_CODE_t clock_regWrite(UINT16 RegName,UINT16 RegValue)
{
	 struct i2c_client client;
	 //-ERROR_CODE_t res = 1;
	 UINT8 localReadBuffer[10];
	 
	 client.addr = ADUX1020_ADD;
	 client.num = 2;
	 
	 localReadBuffer[0] = RegName;		//-第一个参数是寄存器首地址
	 localReadBuffer[1] = (RegValue >> 8) & 0xff;		//-高字节
	 localReadBuffer[2] = RegValue & 0xff;		//-低字节
	 //-res=i2c_write_reg_Buffer(&client,localReadBuffer,3);
   i2c_write_reg_Buffer(&client,localReadBuffer,3);
	 return CLOCK_SUCCESS;
}

ERROR_CODE_t clock_regRead(UINT16 RegName,UINT16 *RegValue)
{
	 struct i2c_client client;
	 //-ERROR_CODE_t res = 1;
	 UINT8 localReadBuffer[10];
	 
	 client.addr = ADUX1020_ADD;
	 client.num = 2;
	 
	 localReadBuffer[0] = RegName;		//-第一个参数是寄存器首地址
	 //-res = i2c_read_reg_Buffer(&client,2,localReadBuffer);
   i2c_read_reg_Buffer(&client,2,localReadBuffer);

	 *RegValue = (localReadBuffer[0] << 8) + localReadBuffer[1];
	 return CLOCK_SUCCESS;
}


unsigned short clockcalibrationresult;
unsigned short value_18;
unsigned short value_1a;
//-2015/7/1 19:15:49  by zj
void ADUX1020_clock_Calibration(void)
{
  unsigned int timeCollectMS = 200;
  int status = 0;
  int errorCode;
  float dataRate;
  unsigned int timestamp;
  int data[4];
 

   
  status = setClkCalRegRWCallback( &clock_regRead,&clock_regWrite);
  if(status != 0)
    goto exit;		//-Callback Register for read and write Failed
  
  ADUX1020_SampleMode();		//-这样一个函数没有
  
  errorCode = clockCalibrationInit(&dataRate,timeCollectMS);
  if(errorCode != 0)    //-校准到这里没有成功,错误代码是4
    goto exit;	//-clock Calibration Init Failed
        
  errorCode = IN_PROGRESS;
  
  while(errorCode == IN_PROGRESS)
  {
 
      readSampleData(data,&timestamp);    
      errorCode = clockCalibration((unsigned short)ADI_X1, (unsigned short)ADI_X2, (unsigned short)ADI_Y1, (unsigned short)ADI_Y2, timestamp);
    
      if(errorCode == CLOCK_SUCCESS) 
      {
            goto exit;
      }
      else if(errorCode == IN_PROGRESS) 
      {
			//printf("CLOCK CALBRATION IN PROGRESS !!! \n");
      } 
      else 
      {
           goto exit;
      }

  }
  
exit:
  clockcalibrationresult = errorCode;
  if (errorCode == CLOCK_SUCCESS)
  {
       clock_regRead(0x18, &ADI_OSC32K);		//-读出数据以供后面使用
       clock_regRead(0x1a, &ADI_OSC32M_TRIM);
    
  }
  ADUX1020_IdleMode();
  
  //-return errorCode;
}

