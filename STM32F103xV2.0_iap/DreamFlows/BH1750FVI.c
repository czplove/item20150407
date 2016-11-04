
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
#include "user_conf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BH1750FVI_CMD_CMTHB					0x40
#define BH1750FVI_CMD_CMTLB					0x60

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* External variables ---------------------------------------------------------*/
/* External functions ---------------------------------------------------------*/

void initBH1750FVI(void)	//-对光敏进行一次初始化
{
	 struct i2c_client client;
	 //-u8 temp_data8;
	 int res = 1;
	 
	 //-setNextI2cDeviceAddress(BH1750FVI_ADDR);
	 //-initVirtualI2c() ;			//-引脚初始化,已经统一做过
	
	 client.addr = BH1750FVI_ADD;
	 client.num = 1;
	 	 
	 STM32_UP_selfT_flag1 &= 0xfd;
	 	 
	 res=i2c_write_data(&client,0x00,BH1750FVI_CMD_POWER_ON); 		//-Waiting for measurement command
	 //-while(res <= 0);
	 if(res <= 0)
	 {
	 	  STM32_UP_selfT_flag1 |= 0x02;
	 }
	 
	 res=i2c_write_data(&client,0x00,BH1750FVI_CMD_RESET); 		//-Reset Data register value. Reset command is not acceptable in Power Down mode 
	 if(res <= 0)
	 {
	 	  STM32_UP_selfT_flag1 |= 0x02;
	 }
	 
	 res=i2c_write_data(&client,0x00,BH1750FVI_CMD_CMTHB+2); 		//-Change measurement time
	 if(res <= 0)
	 {
	 	  STM32_UP_selfT_flag1 |= 0x02;
	 }
	 
	 res=i2c_write_data(&client,0x00,BH1750FVI_CMD_CMTLB+5); 		//-Change measurement time 
	 if(res <= 0)
	 {
	 	  STM32_UP_selfT_flag1 |= 0x02;
	 }	
	 
	 res=i2c_write_data(&client,0x00,BH1750FVI_CMD_POWER_OFF); 		//-No active state
	 if(res <= 0)
	 {
	 	  STM32_UP_selfT_flag1 |= 0x02;
	 }
			  	
}




UINT16 getLux(UINT8 rMode)  		//-得到结果.,带人的参数是测量模式
{
	UINT8 rData[2],tdMs;
	UINT16 bh = 0;
	struct i2c_client client;
	 int res = 1;
	 
	 client.addr = BH1750FVI_ADD;
	 client.num = 1;
	
  res=i2c_write_data(&client,0x00,BH1750FVI_CMD_POWER_ON); 		//-Waiting for measurement command
	 //-while(res <= 0);
        
	if((rMode == BH1750FVI_CMD_CHRM) ||(rMode == BH1750FVI_CMD_CHRM2)) 
		tdMs = 100;//100 ms
	else if(rMode == BH1750FVI_CMD_CLRM) 
		tdMs = 16;
	else 
		return 0;
	
	//-第一步
	//-startI2C();							// start Condition
	//-if(i2cSendByte(BH1750FVI_ADDW) == Fail) return 0;
	//-if(i2cSendByte(rMode) == Fail) return 0;
	//-//stopI2C();
	res=i2c_write_data(&client,0x00,rMode); 		//-Waiting for measurement command
	 //-while(res <= 0);		//-为了这个出错一次整个程序挂掉没有必要

  //-第二步
	//-//delayMs(tdMs);// ms by reslutions require!
  //-delayUs(tdMs * 1000);
  Host_LowLevelDelay(tdMs*10);	//-这里的延时需要调整,否则可能数据读不出来
	
	//-第三步
	//-startI2C();						// repeat start condition!
	//-if(i2cSendByte(BH1750FVI_ADDR) == Fail) return 0;
	//-i2cReceiveByte(&rDataH);				// receive a High byte!
	//-mSendAck();
	//-i2cReceiveByte(&rDataL);				// receive a Low byte!
	//-mSendNoack();
	//-stopI2C();							// stop condition
	res=i2c_read_data_Buffer(&client,2,rData);
  
  if(res > 0)
  {  
    //-下面对读出的数据进行解析
    bh=rData[0]*256+rData[1];
    bh= (UINT16)(bh/1.2);
    
    als_data = bh;		//-得到光强数据
  }
  res=i2c_write_data(&client,0x00,BH1750FVI_CMD_POWER_OFF); 		//-No active state
	//- while(res <= 0);
                
	return bh;
}


