//-简单的实现FLASH的读写,不搞复杂的逻辑,这个尽量少用
#include "user_conf.h"
//-#include "eeprom.h"


/*
以32位为最小单位进行读写操作,实际中需要几个字节,就取几个字节.

?读写的时候是否可以继续进行其它操作
?是否可以被中断打断
?擦除一页的时间是多长

如果有一个定值需要使用FLASH 进行保存,那么可以设置一个标志位判断是否需要修改定值
否则上电直接读FLASH中的值就行

FLASH 0x08000000;到0x0807FFFF;
对于大容量产品，其被划分为 256 页，每页 2K 字节。注意，小容量和中容量产品则每页只有 1K 字节。

模拟FLASH定为16K,0x4000.每页2K,0x800.
0x0807C000;到0x0807FFFF;为设定的模拟FLASH空间.
*/


/*
0x08000000 ~ 0x0807FFFF
对于大容量产品，其被划分为 256 页，每页 2K 字节。注意，小容量和中容量产品则每页只有 1K 字节。

在执行闪存写操作时，任何对闪存的读操作都会锁住总线，在
写操作完成后读操作才能正确地进行；既在进行写或擦除操作
时，不能进行代码或数据的读取操作。所以在每次操作之前，
我们都要等待上一次操作完成这次操作才能开始。
*/
#define FLASH_PAGE_SIZE    ((uint16_t)0x800)


#define BANK2_WRITE_START_ADDR  ((UINT32)0x0807F000)
#define BANK2_WRITE_END_ADDR    ((UINT32)0x0807F7FF)
#define BANK1_WRITE_START_ADDR  ((UINT32)0x0807F800)
#define BANK1_WRITE_END_ADDR    ((UINT32)0x0807FFFF)



#define led_display_long_EEP		(BANK1_WRITE_START_ADDR + 0x000)		//-0x004
//-#define led_display_long_EEP		(BANK1_WRITE_START_ADDR + 0x004)

#define voice_flag_EEP		(BANK2_WRITE_START_ADDR + 0x000)		//-0x004

FLASH_Status FLASHStatus;// = FLASH_COMPLETE;

UINT32 Data = 0x3210ABCD;	
	
void MONI_EEPROM_init(void)
{
	 //uint16_t result = 0;


	/* Unlock the Flash Program Erase controller */
	FLASH_Unlock();		//-第一步
		
	/* Clear All pending flags */
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);		//-这个是后来加的,必要性需要考虑
  
  /* Erase the FLASH pages */
  //-for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
  //{
    FLASHStatus = FLASH_ErasePage(BANK1_WRITE_START_ADDR);//- + (FLASH_PAGE_SIZE * EraseCounter));		//-以页为单位擦除
  //}
  
  //-while((Address < BANK1_WRITE_END_ADDR) && (FLASHStatus == FLASH_COMPLETE))
  //{
    FLASHStatus = FLASH_ProgramWord(BANK1_WRITE_START_ADDR, Data);		//-32 位字写入函数，其他分别为 16 位半字写入和用户选择字节写入函数
    //-Address = Address + 4;
  //}
  
  EEP_Data = (*(UINT32*) BANK1_WRITE_START_ADDR);
		
		/* EEPROM Init */
		//-EE_Init();
		
		/* Write to EEPROM */
		//-result = memcpy_to_eeprom_with_checksum(TITLE_KEY, title, TITLE_SIZE);
		//-result = memcpy_to_eeprom_with_checksum(POINT_KEY, &point, sizeof(point));
		
		/* Read from EEPROM */
		//-result = memcpy_from_eeprom_with_checksum(title, TITLE_KEY, TITLE_SIZE);
		//-result = memcpy_from_eeprom_with_checksum(&point, POINT_KEY, sizeof(point));
		
		FLASH_Lock();		//-结束修改
}

int MONI_EEPROM_read(void)		//-从模拟FLASH中读出定值,供程序使用.
{
	 UINT32 temp_data;
	 //-int res;
	
	 temp_data = (*(UINT32*) led_display_long_EEP);
	 if((temp_data & 0xffff0000) == 0xffff0000)
	 	 return 1;	//-说明第一次上电
	 else	
	   led_display_long = temp_data & 0xff;
	 
	 temp_data = (*(UINT32*) voice_flag_EEP);
	 if((temp_data & 0xffff0000) == 0xffff0000)
	 	 return 1;
	 else
	   voice_flag = temp_data & 0xff;
	   
	 return 0;  
}

void MONI_EEPROM_write(void)
{
	 UINT32 temp_data;


	/* Unlock the Flash Program Erase controller */
	FLASH_Unlock();		//-第一步
		
	/* Clear All pending flags */
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);		//-这个是后来加的,必要性需要考虑
  
  /* Erase the FLASH pages */
  //-for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
  //{
    FLASHStatus = FLASH_ErasePage(BANK1_WRITE_START_ADDR);//- + (FLASH_PAGE_SIZE * EraseCounter));		//-以页为单位擦除
  //}
  
  //-while((Address < BANK1_WRITE_END_ADDR) && (FLASHStatus == FLASH_COMPLETE))
  //{
    FLASHStatus = FLASH_ProgramWord(led_display_long_EEP, led_display_long);		//-32 位字写入函数，其他分别为 16 位半字写入和用户选择字节写入函数
    //-Address = Address + 4;
  //}
  
  temp_data = (*(UINT32*) led_display_long_EEP);
  
  if(temp_data == led_display_long)		//-修改不正确了,会继续修改直到正确为止
  	EEP_Data_flag = 0;
		
				
		FLASH_Lock();		//-结束修改
}

void MONI_EEPROM_write1(void)
{
	 UINT32 temp_data;


	/* Unlock the Flash Program Erase controller */
	FLASH_Unlock();		//-第一步
		
	/* Clear All pending flags */
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);		//-这个是后来加的,必要性需要考虑
  
  /* Erase the FLASH pages */
  //-for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
  //{
    FLASHStatus = FLASH_ErasePage(BANK2_WRITE_START_ADDR);//- + (FLASH_PAGE_SIZE * EraseCounter));		//-以页为单位擦除
  //}
  
  //-while((Address < BANK1_WRITE_END_ADDR) && (FLASHStatus == FLASH_COMPLETE))
  //{
    FLASHStatus = FLASH_ProgramWord(voice_flag_EEP, voice_flag);		//-32 位字写入函数，其他分别为 16 位半字写入和用户选择字节写入函数
    //-Address = Address + 4;
  //}
  
  temp_data = (*(UINT32*) voice_flag_EEP);
  
  if(temp_data == voice_flag)		//-修改不正确了,会继续修改直到正确为止
  	EEP_Data_flag = 0;			//-这里需要修改,不对
		
				
		FLASH_Lock();		//-结束修改
}

/*
把特定的位置定义为特定的变量,然后上电后直接读取变量值进行调用.当需要修改时,首先把变量值记录下来.
然后在主循环中查询,一直等到屏幕熄灭后开始写入FLASH,平时调用还使用RAM中的数值,重新上电后才从FLASH
中读取.
一个数据占用两位,紧接着的一位数据是上个数据的取反,这样可以验证数据是否填写正确,可以简单的读出原数据
后保证数据正确了,就可以,目前不考虑FLASH损坏的情况.

正常情况下通讯随时可以修改RAM里面的数值,并且这个数据是有效的,一但屏幕熄灭之后这个数值就会写入到FLASH
中进行永久保存,如果本身就是熄灭状态,就会立即进行修改保存.

定义的变量:
UINT8		led_display_long;			UINT32 led_display_long_EEP;
*/
void MONI_EEPROM_sub(void)
{
	 MONI_EEPROM_write();
	 if(EEP_Data_flag == 0) //-只有上一个修改成功了才有必要修改下一个
	 {
	 	  EEP_Data_flag = 0x55; 
	 		MONI_EEPROM_write1();
	 }
}
























































