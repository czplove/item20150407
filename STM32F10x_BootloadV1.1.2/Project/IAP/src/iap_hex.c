/*************** (C) Copyright 2015 GreenWay, All rights reserved ***************
* 文件名          : uart2.c
* 作者            : GreenWay
* 版本            : V1.0
* 完成日期        : 2016/1/12 20:29:58
* 功能描述        : 用于IAP升级 处理的是HEX文件
					USART2_TX--PD5;   USART2_RX--PD6
*********************************************************************************/
#include "stm32_eval.h"

#include "common.h"

/*
1.处理hex文件的程序段
初稿不搞那么复杂,先实现功能,就一步一步的解析写代码,后面再考虑一块一块的写代码,速度和用户体验问题.

STM32F103RET6
程序存储器、数据存储器、寄存器和输入输出端口被组织在同一个4GB的线性地址空间内。
数据字节以小端格式存放在存储器中。一个字里的最低地址字节被认为是该字的最低有效字
节，而最高地址字节是最高有效字节。
在hex文件中就相当于数据部分低位在前,高位在后.

目前就使用原来的通讯协议,和处理方式,这样在原来的协议上携带hex报文内容就行,采用查询方式,正确的报文就应答
不正确的不应答,让上位机重发.

2016/1/14 13:28:51
分配FLASH空间
主芯片是STM32F103RET6.		FLASH 512K	0x0800 0000 - 0x0807 FFFF 每页2K
												RAM		64K		0x20000000 - 0x2000FFFF

IAP程序分配:
		FLASH		16K		0x0800 0000 - 0x08003FFF
		RAM

APP0程序分配:
		FLASH		128K		0x08004000 - 0x08023FFF
		RAM

APP1程序分配:
		FLASH		128K		0x08024000 - 0x08043FFF
		RAM

模拟FLASH定为16K,0x4000.每页2K,0x800.
		0x0807C000;到0x0807FFFF;为设定的模拟FLASH空间.

2016/12/15 11:51:51
分配FLASH空间
主芯片是STM32F103RCT6.		FLASH 256K	0x0800 0000 - 0x0807 FFFF 每页2K
												RAM		64K		0x20000000 - 0x2000FFFF

IAP程序分配:
		FLASH		16K		0x0800 0000 - 0x08003FFF
		RAM

APP0程序分配:
		FLASH		112K		0x08004000 - 0x0801FFFF
		RAM

APP1程序分配:
		FLASH		112K		0x08020000 - 0x0803BFFF
		RAM

模拟FLASH定为16K,0x4000.每页2K,0x800.
		0x0804C000;到0x0803FFFF;为设定的模拟FLASH空间.
*/





//从指定地址开始写入指定长度的数据
//WriteAddr:起始地址(此地址必须为2的倍数!!)
//pBuffer:数据指针
//NumToWrite:半字(16位)数(就是要写入的16位数据的个数.)
#if STM32_FLASH_SIZE<256
#define STM_SECTOR_SIZE 1024 //字节
#else
#define STM_SECTOR_SIZE	2048
#endif

u16 iapbuf[1024] = {0}; //用于缓存数据的数组
u16 iapbuf_bak[1024] = {0}; //用于缓存数据的数组
u16 IAP_DataCur = 0;	//当前iapbuffer中已经填充的数据长度,一次填充满了之后写入flash并清零
u32 addrCur = FLASH_APP1_ADDR;			//当前系统写入地址,每次写入之后地址增加2048
u16 IAP_HEX_expectant_pt = 0x4000;

u16 STMFLASH_BUF[STM_SECTOR_SIZE/2];//最多是2K字节

//读取指定地址的半字(16位数据)
//faddr:读地址(此地址必须为2的倍数!!)
//返回值:对应数据.
u16 STMFLASH_ReadHalfWord(u32 faddr)
{
    return *(vu16*)faddr;
}

//不检查的写入
//WriteAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数
void STMFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)
{
    u16 i;
    for(i=0;i<NumToWrite;i++)
    {
        FLASH_ProgramHalfWord(WriteAddr,pBuffer[i]);
        WriteAddr+=2;//地址增加2.
    }
}

//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead)
{
    u16 i;
    for(i=0;i<NumToRead;i++)
    {
        pBuffer[i]=STMFLASH_ReadHalfWord(ReadAddr);//读取2个字节.
        ReadAddr+=2;//偏移2个字节.
    }
}

void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)
{
    u32 secpos;	   //扇区地址
    u16 secoff;	   //扇区内偏移地址(16位字计算)
    u16 secremain; //扇区内剩余地址(16位字计算)
    u16 i;
    u32 offaddr;   //去掉0X08000000后的地址
    if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))return;//非法地址
    FLASH_Unlock();						//解锁
    offaddr=WriteAddr-STM32_FLASH_BASE;		        //实际偏移地址.
    secpos=offaddr/STM_SECTOR_SIZE;			//扇区地址  0~127 for STM32F103RBT6
    secoff=(offaddr%STM_SECTOR_SIZE)/2;		//在扇区内的偏移(2个字节为基本单位.)
    secremain=STM_SECTOR_SIZE/2-secoff;		//扇区剩余空间大小,,最终记录的是可以写入的个数
    if(NumToWrite<=secremain)secremain=NumToWrite;//不大于该扇区范围??可能是这种机制,如果擦除了之后没有写数据,那么不需要再次擦除,直接写即可
    while(1)
    {
        STMFLASH_Read(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//读出整个扇区的内容,备份,这样可以保证不一定需要从第一个字节一次性写入
        for(i=0;i<secremain;i++)//校验数据,,检查的是待写空间的数据,如果有非0XFFFF的就需要擦除
        {
            if(STMFLASH_BUF[secoff+i]!=0XFFFF)break;//需要擦除
        }
        if(i<secremain)//需要擦除
        {
            FLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//擦除这个扇区
            for(i=0;i<secremain;i++)//复制
            {
                STMFLASH_BUF[i+secoff]=pBuffer[i];		//-在RAM中先拼接好全部的内容,然后一次性写入
            }
            STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//写入整个扇区
        }else STMFLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//写已经擦除了的,直接写入扇区剩余区间.

        if(NumToWrite==secremain)break;//写入结束了
        else//写入未结束,这次的内容挂接超过了一个扇区
        {
            secpos++;				//扇区地址增1
            secoff=0;				//偏移位置为0
            pBuffer+=secremain;  	//指针偏移
            WriteAddr+=secremain;	//写地址偏移
            NumToWrite-=secremain;	//字节(16位)数递减
            if(NumToWrite>(STM_SECTOR_SIZE/2))secremain=STM_SECTOR_SIZE/2;//下一个扇区还是写不完
            else secremain=NumToWrite;//下一个扇区可以写完了
        }
    };
    FLASH_Lock();//上锁
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


/*
2016/1/13 9:50:32
计算hex校验和

校验和的计算规则:
以字节(2个HEX字符)为单位，除“:”以外，当前行所有数据的和为00H.注意对和只取低8位.

*/
unsigned short IAP_CRC_HEX(unsigned char *IAP_CRC_HEX_start,unsigned char IAP_CRC_HEX_LEN)
{
	unsigned char i;	//Loop Var
	unsigned short checkdata; 	 // 每一行数据校验

	checkdata = 0;
	//计算校验码
	//文件说明  CRC =  0X100 - sum
// 	printf("\r\nWriteData\r\n");
	for( i=0; i<IAP_CRC_HEX_LEN; i++)  // Compute check data
	{
		//-printf("%02X",hexdata[i]);//2015.6.11调试，确定所有数据接收正确
		checkdata += IAP_CRC_HEX_start[i];//数据相加
	}
//  	printf("\r\n");
	//如果校验没有问题：最低位为00
	checkdata = checkdata & 0x00FF;	// 最低位数据
	return checkdata;
}

/**********************************************************************
函 数 名：hex2binLine(unsigned int linelength, unsigned char *linedata, unsigned long  *flashaddress, unsigned int *datalength, unsigned short *flashdata)
功    能：每一行HEX数据处理，转化成Bin格式，留下有效数据，存入Flash
说    明:
参    数：
		unsigned int linelength： 	需要处理的数据长度
		unsigned char *linedata：	待处理数据内容
		unsigned long  *flashaddress：
		unsigned int *datalength：
		unsigned short *flashdata
返 回 值：
***********************************************************************/
unsigned char  hex2binLine(unsigned int linelength, unsigned char *hexdata, unsigned long  *flashaddress, unsigned int *datalength, unsigned short *flashdata)
{
	//-unsigned long segmentaddress; // Segment Address
	//-unsigned long linearaddress;  // Linear Address
	unsigned char temp8data;
	unsigned short temp16data;
	//-unsigned long temp32data;
	unsigned short checkdata; 	 // 每一行数据校验
	unsigned short tempaddress;
	//-unsigned char hexdata[HexLineNumber/2];
	//unsigned char change to hex data, 2 unsigned char into 1 hex data
	 unsigned char i;	//Loop Var
	/* Initial Variable */
	checkdata = 0;
  temp8data = 0x55;

	checkdata = IAP_CRC_HEX(&hexdata[1],linelength);

  if((hexdata[1] & 0x01) != 0) 	//-必须为偶数否则出错
		hexdata[1]++;

	//数据校验
	if( checkdata != 0x0000)    //不成立，校验错误
	{
		*datalength = 0;
		//-printf("TCP> hex check num error, checkdata =%.2x\r\n",checkdata);
		*flashaddress = 0x00000000;	// set flashaddress = 0, mark the checkdata error
		return 0;
	}
	tempaddress = hexdata[3] + (hexdata[2] << 8);	//-第二块 2个字节表示:数据存储的起始地址(其中高位地址在前，低位地址在后。)

	/* calculate flash address */
	/* calculate data length */
	/* calculate flashdata */
	//判断数据类型
	//0X00: 记录数据
	//0X01: 数据结尾
	//0X02: 扩展段地址
	//0X03: 开始短地址
	//0x04: 扩展线性地址
	//0X05：开始线性地址

	/**************************************************************************************/
	//此处有待完成


	switch(hexdata[4]) 	//-第三块 1个字节表示:数据的类型
	{
//****************************************************************************************************************************
//0X00: 记录数据
		case 0x00://write data
			//**************************************
			//*********地址需要重新计算*************
// 			*flashaddress = segmentaddress + linearaddress + tempaddress + FLASH_UpdateBuffer_MeterAPPVectorTable;

			//-现在的系统中2K空间一次性写入内容,那么这就需要保证这里是准备的2K内容
			//?HEX文件会不会存在断续的情况?
			if(IAP_DataCur == 0)
			{
				if(((addrCur & 0x7FF) != 0) || ((tempaddress & 0x7FF) != 0))	//-规定第一个偏移量必须从0开始
					return 0;
			}
			else
			{
				if(IAP_HEX_expectant_pt != tempaddress)	//-得到的偏移量必须是我所需要的偏移量
					return 0;
			}
			//**************************************
			//**************************************
			//数据包,作用：右移1位，转换成存储所需要的“半字“长度
			*datalength = (hexdata[1] >> 1); // data length 全部以16位为单位组织内容
			IAP_HEX_expectant_pt += hexdata[1];
// 			printf("datalength:%d\r\n",*datalength);

		  //-每接收到一个数据就需要把内容写入到RAM缓冲区进行存储,因为FLASH只能以块为单位擦除
			for( i=0; i<*datalength; i++)  // flash data
			{
					temp16data = hexdata[i*2 + 5] + (hexdata[i*2 + 6] << 8);
					iapbuf[IAP_DataCur] = temp16data;
					IAP_DataCur++;	//-指向的是待存储的地址空间
			}

			//-receiveExpectCount = 0;//清除期望接收模式
			//-serial_Buffer_Length = 0;//清除串口满标志
			//-printf(".");//每次接受一次数据打一个点

			//还没放满,等待下一次数据过来

		break;
//*****************************************************************************************************************************
//0X01: 数据结尾,,文件结束记录
		case 0x01://end of file,,用来标识文件结束，放在文件的最后，标识HEX文件的结尾
			/* No need operation */
			//-接收到结束命令后把剩余没有写入FLASH的内容写入
      temp8data = 0xaa;
		break;
//****************************************************************************************************************************
		case 0x02://Extended segment address,,扩展段地址记录
//0X02: 扩展段地址

		break;
//****************************************************************************************************************************
		case 0x04://Extended linear address,,扩展线性地址记录
//0x04: 扩展线性地址,,0x02 0x04功能码也许可以很灵活的改变很多东西,但是如果对于简单的系统,可以忽略这些内容的处理,因为结果预知

		break;
//****************************************************************************************************************************
		case 0x05:	//-数据类型后2种记录(04，05)都是用来提供地址信息的。每次碰到这2个记录的时候，都可以根据记录计算出一个“基”地址。
								//-对于后面的数据记录，计算地址的时候，都是以这些“基”地址为基础的

			break;

		default:
			break;
	}

	return temp8data;
}


/**********************************************************************
函 数 名：UpdatePackageDataHandle(unsigned char *packagedata, unsigned int datasize)
功    能：数据包处理
说    明:
参    数：unsigned char *packagedata: 数据包数组
		  unsigned int datasize:      数据长度
返 回 值：
***********************************************************************/
unsigned char UpdatePackageDataHandle(unsigned char *packagedata, unsigned int datasize)	//-功能暂定为解析hex一行内容
{
	/* Temporary Variable */
	unsigned long i,j;	//Loop Var
	//-unsigned int  DataIdentityNum[MaxHexLines];//记录冒号在数据包中的位置//record ':(3A)' serial number
	//-unsigned int  IdentityNumber; //对冒号个数计数// record number of ':'
	unsigned int  templength;
	unsigned long FlashAddress; // write flash address
	unsigned int  FlashDataLength;	// write flash data length, unit of 2bytes
	unsigned short FlashData[16]; // flash data to write
	unsigned char temp_status;

	unsigned short iap_recv_pt;
	unsigned short iap_recv_dl;

	iap_recv_pt = datasize - 1;
	iap_recv_dl = 0;
	//初始化变量，即为每一行开头冒号的个数记录
	//-IdentityNumber = 0;
	//进行喂狗，防止写Flash，时间过长
	//-IWDG_Feed();


	if(packagedata[0] != 0x3A)
		return 0x00;	//-认为报文出错直接退出

		//每一行数据分析，HEX转成Bin文件，并进行Falsh存储
		//-规定除最后一帧外别的必须是8行一个数据包
		while(iap_recv_dl < iap_recv_pt)
		{
			if(packagedata[0] != 0x3A)
				return 0x00;

			//-packagedata = packagedata + 1 + templength;	//-指向待处理行的行首
			templength = packagedata[1] + 5;	//-一行中除冒号之外的所有数据个数
			iap_recv_dl = iap_recv_dl + 1 + templength;


			//-以上控制HEX行的选取,选中一行之后进行处理
			temp_status = hex2binLine(templength, packagedata, &FlashAddress, &FlashDataLength, FlashData);	//-仅仅组织报文,不进行FLASH写入
      packagedata = packagedata + 1 + templength;	//-指向待处理行的行首
			//将有效程序数据解析，写入指定地址的Flash
			if(temp_status == 0x55)
			{
			//此时需要检测receiveDataCur的值,要是放满了,就需要写入
				if(IAP_DataCur == 1024)	//-每页现在2K空间,需要一次性写入
				{
						//写入flash中
						STMFLASH_Write(addrCur,iapbuf,1024);	//-对于2K一个扇区的空间,一次性写入希望的数值
						//-写入之后立即读出,进行比较如果正确了就通过,否则认为整个过程失败
	          STMFLASH_Read(addrCur,iapbuf_bak,1024);
	          for(i = 0;i < 1024;i++)
	          {
	              if(iapbuf[i] != iapbuf_bak[i])
	              {//-升级内容出错之后,不能一直停留在这里,这样就不能信息交互了,但是需要可以重来,针对FLASH写出错的情况
	                IAP_HEX_expectant_pt -= 0x10;   //-期望的向上一行
	                IAP_DataCur -= 0x08; //-从新填写一个报文
	                return 0;
	              }
	          }
	          //printf("\r\nwrite addr %x,length 1024\r\n",addrCur);
						addrCur += 2048;//地址+2048
						//写完之后receiveDataCur要清零等待下一次传输
						IAP_DataCur = 0;
				}
				else //有可能最后一包有128个数据但是最终没有2048个数据,此时扩展一个指令用于完成最后一个的写入
				{
						if(IAP_DataCur > 1024)	//-大于出现错误严重,需要特殊考虑
							return 0;
				}

				expect_hex_lines++;
				//-return 0x55;	//-一切处理正常就应答当前帧
			}
			else if(temp_status == 0xaa)
			{//-到这里说明最后一帧了,写入之后就可以开始复制了
					STMFLASH_Write(addrCur,iapbuf,IAP_DataCur); //-完成剩余内容的写入
	        STMFLASH_Read(addrCur,iapbuf_bak,IAP_DataCur);
	          for(i = 0;i < IAP_DataCur;i++)
	          {
	              if(iapbuf[i] != iapbuf_bak[i])
	                return 0;
	          }
	        addrCur = addrCur + IAP_DataCur * 2;//地址

					j = (addrCur - FLASH_APP1_ADDR) / 2;  //-需要复制的半字个数
					FLASH_Unlock();						//解锁
					FLASH_ErasePage(APP_CONFIG_ADDR);//擦除这个扇区
					FLASH_ProgramHalfWord(APP_CONFIG_ADDR,0x0000);	//-不能运行可以运行
					FLASH_ErasePage(APP_CONFIG1_ADDR);//擦除这个扇区
					FLASH_ProgramWord(APP_CONFIG2_ADDR,j);	//-启动复制
          if(*(__IO uint32_t*)(APP_CONFIG2_ADDR) == j)
          {
            FLASH_ProgramHalfWord(APP_CONFIG1_ADDR,0x5555);	//-启动复制
            FLASH_Lock();//上锁
            expect_hex_lines++;
            return 0x55;
          }
          else
          {//-一整填写失败不进行应答让主机重发,重发最后一帧就行没有数据需要填写
            //-IAP_HEX_expectant_pt -= 0x10;   //-期望的向上一行
	          //-IAP_DataCur -= 0x08; //-从新填写一个报文
            FLASH_Lock();//上锁
            return 0x00;	//-一切处理正常就应答当前帧
          }
			}
			else
				return 0;
		}
		return 0;	//-正常退出返回的是0 让行判断应答
}

void STMFLASH_FLAG_clear(void)
{

  FLASH_Unlock();						//解锁
				FLASH_ErasePage(APP_CONFIG_ADDR);//擦除这个扇区
				FLASH_ProgramHalfWord(APP_CONFIG_ADDR,0x0000);	//-不能运行可以运行
				FLASH_ErasePage(APP_CONFIG1_ADDR);//擦除这个扇区
				FLASH_ProgramHalfWord(APP_CONFIG2_ADDR,0x0000);	//-启动复制
        FLASH_ProgramHalfWord(APP_CONFIG1_ADDR,0x0000);	//-启动复制
				FLASH_Lock();//上锁

}










#define APP1_TO_APP0_OFF 	0x020000

void copy_app1_to_app0(unsigned long NumToWrite)
{
    u32 secpos;	   //扇区地址
    u16 secoff;	   //扇区内偏移地址(16位字计算)
    u16 secremain; //扇区内剩余地址(16位字计算)
    u16 i;
    u32 offaddr;   //去掉0X08000000后的地址

    FLASH_Unlock();						//解锁
    offaddr=FLASH_APP0_ADDR-STM32_FLASH_BASE;		        //实际偏移地址.
    secpos=offaddr/STM_SECTOR_SIZE;			//扇区地址  0~127 for STM32F103RBT6
    secoff=(offaddr%STM_SECTOR_SIZE)/2;		//在扇区内的偏移(2个字节为基本单位.)
    secremain=STM_SECTOR_SIZE/2-secoff;		//扇区剩余空间大小,,最终记录的是一次可以写入的个数
    if(NumToWrite<=secremain)secremain=NumToWrite;//不大于该扇区范围??可能是这种机制,如果擦除了之后没有写数据,那么不需要再次擦除,直接写即可
    while(1)
    {
    		STMFLASH_Read(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE + APP1_TO_APP0_OFF,STMFLASH_BUF,STM_SECTOR_SIZE/2);//读出整个扇区的内容,准备写入运行扇区

        FLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//擦除这个扇区

        STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//写入整个扇区
        STMFLASH_Read(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,iapbuf_bak,STM_SECTOR_SIZE/2);
          for(i = 0;i < STM_SECTOR_SIZE/2;i++)
          {
              if(STMFLASH_BUF[i] != iapbuf_bak[i])
                return;
          }


        if(NumToWrite==secremain)
        {//-到这里就可以宣布成功了
        		FLASH_ErasePage(APP_CONFIG_ADDR);//擦除这个扇区
            FLASH_ProgramHalfWord(APP_CONFIG_ADDR,0x5555);	//-可以运行
            if(*(__IO uint16_t*)(APP_CONFIG_ADDR) == 0x5555)
            {
              FLASH_ErasePage(APP_CONFIG1_ADDR);//擦除这个扇区
              FLASH_ProgramHalfWord(APP_CONFIG1_ADDR,0x0000);	//-不再复制
            }
        		break;//写入结束了
        }
        else//写入未结束,这次的内容挂接超过了一个扇区
        {
            secpos++;				//扇区地址增1
            secoff=0;				//偏移位置为0
            //-pBuffer+=secremain;  	//指针偏移
            //-WriteAddr+=secremain;	//写地址偏移
            NumToWrite-=secremain;	//字节(16位)数递减
            if(NumToWrite>(STM_SECTOR_SIZE/2))secremain=STM_SECTOR_SIZE/2;//下一个扇区还是写不完
            else secremain=NumToWrite;//下一个扇区可以写完了
        }
    };
    FLASH_Lock();//上锁


}

























































































