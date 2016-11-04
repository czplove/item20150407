#include "platform_config.h"   
#include "mass_mal.h"
#include "stdio.h"
#include "demo.h"

__IO uint32_t Status = 0;  
unsigned char dis_mem=0;

extern void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...);

uint16_t MAL_Write(uint8_t lun, uint32_t Memory_Offset, u8 *Writebuff, uint16_t Transfer_Length)
{
  SST25_W_BLOCK(Memory_Offset, Writebuff, Transfer_Length);
  return MAL_OK;
}


uint16_t MAL_Read(uint8_t lun, uint32_t Memory_Offset, u8 *Readbuff, uint16_t Transfer_Length)
{      
	SST25_R_BLOCK(Memory_Offset, Readbuff, Transfer_Length);
  	return MAL_OK;
}


uint16_t MAL_GetStatus (uint8_t lun)
{
  if (lun == 0)
  {
	  FlashReadID();		

	  //USART_OUT(USART2,"\r\n fac_id %d\n" ,fac_id); 
	   //USART_OUT(USART2,"\r\n  dev_id %d\n" ,dev_id); 
	  if(fac_id==0xbf){					     
	  	Mass_Block_Size[0]=4096;			  
		Mass_Block_Count[0]=512;			 
	  }
	  //      Led_ON();								
	  if(dis_mem==0){						  
		Mass_Memory_Size[0] = Mass_Block_Count[0] * Mass_Block_Size[0];
	//	USART_OUT(USART2,"\r\n SST25VF016B  %d MBytes\n" ,Mass_Memory_Size[0]/1024/1024);        	
		dis_mem=1;
	  }
	  return MAL_OK;
  }					
  //Led_ON();								    
  return MAL_FAIL;
}

