/*************** (C) Copyright 2015 GreenWay, All rights reserved ***************
* �ļ���          : uart2.c
* ����            : GreenWay
* �汾            : V1.0
* �������        : 2016/1/12 20:29:58
* ��������        : ����IAP���� �������HEX�ļ�
					USART2_TX--PD5;   USART2_RX--PD6
*********************************************************************************/
#include "stm32_eval.h"

#include "common.h"

/*
1.����hex�ļ��ĳ����
���岻����ô����,��ʵ�ֹ���,��һ��һ���Ľ���д����,�����ٿ���һ��һ���д����,�ٶȺ��û���������.

STM32F103RET6
����洢�������ݴ洢�����Ĵ�������������˿ڱ���֯��ͬһ��4GB�����Ե�ַ�ռ��ڡ�
�����ֽ���С�˸�ʽ����ڴ洢���С�һ���������͵�ַ�ֽڱ���Ϊ�Ǹ��ֵ������Ч��
�ڣ�����ߵ�ַ�ֽ��������Ч�ֽڡ�
��hex�ļ��о��൱�����ݲ��ֵ�λ��ǰ,��λ�ں�.

Ŀǰ��ʹ��ԭ����ͨѶЭ��,�ʹ���ʽ,������ԭ����Э����Я��hex�������ݾ���,���ò�ѯ��ʽ,��ȷ�ı��ľ�Ӧ��
����ȷ�Ĳ�Ӧ��,����λ���ط�.

2016/1/14 13:28:51
����FLASH�ռ�
��оƬ��STM32F103RET6.		FLASH 512K	0x0800 0000 - 0x0807 FFFF ÿҳ2K
												RAM		64K		0x20000000 - 0x2000FFFF

IAP�������:
		FLASH		16K		0x0800 0000 - 0x08003FFF
		RAM

APP0�������:
		FLASH		128K		0x08004000 - 0x08023FFF
		RAM

APP1�������:
		FLASH		128K		0x08024000 - 0x08043FFF
		RAM

ģ��FLASH��Ϊ16K,0x4000.ÿҳ2K,0x800.
		0x0807C000;��0x0807FFFF;Ϊ�趨��ģ��FLASH�ռ�.

2016/12/15 11:51:51
����FLASH�ռ�
��оƬ��STM32F103RCT6.		FLASH 256K	0x0800 0000 - 0x0807 FFFF ÿҳ2K
												RAM		64K		0x20000000 - 0x2000FFFF

IAP�������:
		FLASH		16K		0x0800 0000 - 0x08003FFF
		RAM

APP0�������:
		FLASH		112K		0x08004000 - 0x0801FFFF
		RAM

APP1�������:
		FLASH		112K		0x08020000 - 0x0803BFFF
		RAM

ģ��FLASH��Ϊ16K,0x4000.ÿҳ2K,0x800.
		0x0804C000;��0x0803FFFF;Ϊ�趨��ģ��FLASH�ռ�.
*/





//��ָ����ַ��ʼд��ָ�����ȵ�����
//WriteAddr:��ʼ��ַ(�˵�ַ����Ϊ2�ı���!!)
//pBuffer:����ָ��
//NumToWrite:����(16λ)��(����Ҫд���16λ���ݵĸ���.)
#if STM32_FLASH_SIZE<256
#define STM_SECTOR_SIZE 1024 //�ֽ�
#else
#define STM_SECTOR_SIZE	2048
#endif

u16 iapbuf[1024] = {0}; //���ڻ������ݵ�����
u16 iapbuf_bak[1024] = {0}; //���ڻ������ݵ�����
u16 IAP_DataCur = 0;	//��ǰiapbuffer���Ѿ��������ݳ���,һ���������֮��д��flash������
u32 addrCur = FLASH_APP1_ADDR;			//��ǰϵͳд���ַ,ÿ��д��֮���ַ����2048
u16 IAP_HEX_expectant_pt = 0x4000;

u16 STMFLASH_BUF[STM_SECTOR_SIZE/2];//�����2K�ֽ�

//��ȡָ����ַ�İ���(16λ����)
//faddr:����ַ(�˵�ַ����Ϊ2�ı���!!)
//����ֵ:��Ӧ����.
u16 STMFLASH_ReadHalfWord(u32 faddr)
{
    return *(vu16*)faddr;
}

//������д��
//WriteAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToWrite:����(16λ)��
void STMFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)
{
    u16 i;
    for(i=0;i<NumToWrite;i++)
    {
        FLASH_ProgramHalfWord(WriteAddr,pBuffer[i]);
        WriteAddr+=2;//��ַ����2.
    }
}

//��ָ����ַ��ʼ����ָ�����ȵ�����
//ReadAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToWrite:����(16λ)��
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead)
{
    u16 i;
    for(i=0;i<NumToRead;i++)
    {
        pBuffer[i]=STMFLASH_ReadHalfWord(ReadAddr);//��ȡ2���ֽ�.
        ReadAddr+=2;//ƫ��2���ֽ�.
    }
}

void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)
{
    u32 secpos;	   //������ַ
    u16 secoff;	   //������ƫ�Ƶ�ַ(16λ�ּ���)
    u16 secremain; //������ʣ���ַ(16λ�ּ���)
    u16 i;
    u32 offaddr;   //ȥ��0X08000000��ĵ�ַ
    if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))return;//�Ƿ���ַ
    FLASH_Unlock();						//����
    offaddr=WriteAddr-STM32_FLASH_BASE;		        //ʵ��ƫ�Ƶ�ַ.
    secpos=offaddr/STM_SECTOR_SIZE;			//������ַ  0~127 for STM32F103RBT6
    secoff=(offaddr%STM_SECTOR_SIZE)/2;		//�������ڵ�ƫ��(2���ֽ�Ϊ������λ.)
    secremain=STM_SECTOR_SIZE/2-secoff;		//����ʣ��ռ��С,,���ռ�¼���ǿ���д��ĸ���
    if(NumToWrite<=secremain)secremain=NumToWrite;//�����ڸ�������Χ??���������ֻ���,���������֮��û��д����,��ô����Ҫ�ٴβ���,ֱ��д����
    while(1)
    {
        STMFLASH_Read(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//������������������,����,�������Ա�֤��һ����Ҫ�ӵ�һ���ֽ�һ����д��
        for(i=0;i<secremain;i++)//У������,,�����Ǵ�д�ռ������,����з�0XFFFF�ľ���Ҫ����
        {
            if(STMFLASH_BUF[secoff+i]!=0XFFFF)break;//��Ҫ����
        }
        if(i<secremain)//��Ҫ����
        {
            FLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//�����������
            for(i=0;i<secremain;i++)//����
            {
                STMFLASH_BUF[i+secoff]=pBuffer[i];		//-��RAM����ƴ�Ӻ�ȫ��������,Ȼ��һ����д��
            }
            STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//д����������
        }else STMFLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//д�Ѿ������˵�,ֱ��д������ʣ������.

        if(NumToWrite==secremain)break;//д�������
        else//д��δ����,��ε����ݹҽӳ�����һ������
        {
            secpos++;				//������ַ��1
            secoff=0;				//ƫ��λ��Ϊ0
            pBuffer+=secremain;  	//ָ��ƫ��
            WriteAddr+=secremain;	//д��ַƫ��
            NumToWrite-=secremain;	//�ֽ�(16λ)���ݼ�
            if(NumToWrite>(STM_SECTOR_SIZE/2))secremain=STM_SECTOR_SIZE/2;//��һ����������д����
            else secremain=NumToWrite;//��һ����������д����
        }
    };
    FLASH_Lock();//����
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


/*
2016/1/13 9:50:32
����hexУ���

У��͵ļ������:
�������ֽ�(2��HEX�ַ�)Ϊ��λ������:�����⣬��ǰ���������ݵĺ�Ϊ00H.ע��Ժ�ֻȡ��8λ.

*/
unsigned short IAP_CRC_HEX(unsigned char *IAP_CRC_HEX_start,unsigned char IAP_CRC_HEX_LEN)
{
	unsigned char i;	//Loop Var
	unsigned short checkdata; 	 // ÿһ������У��

	checkdata = 0;
	//����У����
	//�ļ�˵��  CRC =  0X100 - sum
// 	printf("\r\nWriteData\r\n");
	for( i=0; i<IAP_CRC_HEX_LEN; i++)  // Compute check data
	{
		//-printf("%02X",hexdata[i]);//2015.6.11���ԣ�ȷ���������ݽ�����ȷ
		checkdata += IAP_CRC_HEX_start[i];//�������
	}
//  	printf("\r\n");
	//���У��û�����⣺���λΪ00
	checkdata = checkdata & 0x00FF;	// ���λ����
	return checkdata;
}

/**********************************************************************
�� �� ����hex2binLine(unsigned int linelength, unsigned char *linedata, unsigned long  *flashaddress, unsigned int *datalength, unsigned short *flashdata)
��    �ܣ�ÿһ��HEX���ݴ���ת����Bin��ʽ��������Ч���ݣ�����Flash
˵    ��:
��    ����
		unsigned int linelength�� 	��Ҫ��������ݳ���
		unsigned char *linedata��	��������������
		unsigned long  *flashaddress��
		unsigned int *datalength��
		unsigned short *flashdata
�� �� ֵ��
***********************************************************************/
unsigned char  hex2binLine(unsigned int linelength, unsigned char *hexdata, unsigned long  *flashaddress, unsigned int *datalength, unsigned short *flashdata)
{
	//-unsigned long segmentaddress; // Segment Address
	//-unsigned long linearaddress;  // Linear Address
	unsigned char temp8data;
	unsigned short temp16data;
	//-unsigned long temp32data;
	unsigned short checkdata; 	 // ÿһ������У��
	unsigned short tempaddress;
	//-unsigned char hexdata[HexLineNumber/2];
	//unsigned char change to hex data, 2 unsigned char into 1 hex data
	 unsigned char i;	//Loop Var
	/* Initial Variable */
	checkdata = 0;
  temp8data = 0x55;

	checkdata = IAP_CRC_HEX(&hexdata[1],linelength);

  if((hexdata[1] & 0x01) != 0) 	//-����Ϊż���������
		hexdata[1]++;

	//����У��
	if( checkdata != 0x0000)    //��������У�����
	{
		*datalength = 0;
		//-printf("TCP> hex check num error, checkdata =%.2x\r\n",checkdata);
		*flashaddress = 0x00000000;	// set flashaddress = 0, mark the checkdata error
		return 0;
	}
	tempaddress = hexdata[3] + (hexdata[2] << 8);	//-�ڶ��� 2���ֽڱ�ʾ:���ݴ洢����ʼ��ַ(���и�λ��ַ��ǰ����λ��ַ�ں�)

	/* calculate flash address */
	/* calculate data length */
	/* calculate flashdata */
	//�ж���������
	//0X00: ��¼����
	//0X01: ���ݽ�β
	//0X02: ��չ�ε�ַ
	//0X03: ��ʼ�̵�ַ
	//0x04: ��չ���Ե�ַ
	//0X05����ʼ���Ե�ַ

	/**************************************************************************************/
	//�˴��д����


	switch(hexdata[4]) 	//-������ 1���ֽڱ�ʾ:���ݵ�����
	{
//****************************************************************************************************************************
//0X00: ��¼����
		case 0x00://write data
			//**************************************
			//*********��ַ��Ҫ���¼���*************
// 			*flashaddress = segmentaddress + linearaddress + tempaddress + FLASH_UpdateBuffer_MeterAPPVectorTable;

			//-���ڵ�ϵͳ��2K�ռ�һ����д������,��ô�����Ҫ��֤������׼����2K����
			//?HEX�ļ��᲻����ڶ��������?
			if(IAP_DataCur == 0)
			{
				if(((addrCur & 0x7FF) != 0) || ((tempaddress & 0x7FF) != 0))	//-�涨��һ��ƫ���������0��ʼ
					return 0;
			}
			else
			{
				if(IAP_HEX_expectant_pt != tempaddress)	//-�õ���ƫ����������������Ҫ��ƫ����
					return 0;
			}
			//**************************************
			//**************************************
			//���ݰ�,���ã�����1λ��ת���ɴ洢����Ҫ�ġ����֡�����
			*datalength = (hexdata[1] >> 1); // data length ȫ����16λΪ��λ��֯����
			IAP_HEX_expectant_pt += hexdata[1];
// 			printf("datalength:%d\r\n",*datalength);

		  //-ÿ���յ�һ�����ݾ���Ҫ������д�뵽RAM���������д洢,��ΪFLASHֻ���Կ�Ϊ��λ����
			for( i=0; i<*datalength; i++)  // flash data
			{
					temp16data = hexdata[i*2 + 5] + (hexdata[i*2 + 6] << 8);
					iapbuf[IAP_DataCur] = temp16data;
					IAP_DataCur++;	//-ָ����Ǵ��洢�ĵ�ַ�ռ�
			}

			//-receiveExpectCount = 0;//�����������ģʽ
			//-serial_Buffer_Length = 0;//�����������־
			//-printf(".");//ÿ�ν���һ�����ݴ�һ����

			//��û����,�ȴ���һ�����ݹ���

		break;
//*****************************************************************************************************************************
//0X01: ���ݽ�β,,�ļ�������¼
		case 0x01://end of file,,������ʶ�ļ������������ļ�����󣬱�ʶHEX�ļ��Ľ�β
			/* No need operation */
			//-���յ�����������ʣ��û��д��FLASH������д��
      temp8data = 0xaa;
		break;
//****************************************************************************************************************************
		case 0x02://Extended segment address,,��չ�ε�ַ��¼
//0X02: ��չ�ε�ַ

		break;
//****************************************************************************************************************************
		case 0x04://Extended linear address,,��չ���Ե�ַ��¼
//0x04: ��չ���Ե�ַ,,0x02 0x04������Ҳ����Ժ����ĸı�ܶණ��,����������ڼ򵥵�ϵͳ,���Ժ�����Щ���ݵĴ���,��Ϊ���Ԥ֪

		break;
//****************************************************************************************************************************
		case 0x05:	//-�������ͺ�2�ּ�¼(04��05)���������ṩ��ַ��Ϣ�ġ�ÿ��������2����¼��ʱ�򣬶����Ը��ݼ�¼�����һ����������ַ��
								//-���ں�������ݼ�¼�������ַ��ʱ�򣬶�������Щ��������ַΪ������

			break;

		default:
			break;
	}

	return temp8data;
}


/**********************************************************************
�� �� ����UpdatePackageDataHandle(unsigned char *packagedata, unsigned int datasize)
��    �ܣ����ݰ�����
˵    ��:
��    ����unsigned char *packagedata: ���ݰ�����
		  unsigned int datasize:      ���ݳ���
�� �� ֵ��
***********************************************************************/
unsigned char UpdatePackageDataHandle(unsigned char *packagedata, unsigned int datasize)	//-�����ݶ�Ϊ����hexһ������
{
	/* Temporary Variable */
	unsigned long i,j;	//Loop Var
	//-unsigned int  DataIdentityNum[MaxHexLines];//��¼ð�������ݰ��е�λ��//record ':(3A)' serial number
	//-unsigned int  IdentityNumber; //��ð�Ÿ�������// record number of ':'
	unsigned int  templength;
	unsigned long FlashAddress; // write flash address
	unsigned int  FlashDataLength;	// write flash data length, unit of 2bytes
	unsigned short FlashData[16]; // flash data to write
	unsigned char temp_status;

	unsigned short iap_recv_pt;
	unsigned short iap_recv_dl;

	iap_recv_pt = datasize - 1;
	iap_recv_dl = 0;
	//��ʼ����������Ϊÿһ�п�ͷð�ŵĸ�����¼
	//-IdentityNumber = 0;
	//����ι������ֹдFlash��ʱ�����
	//-IWDG_Feed();


	if(packagedata[0] != 0x3A)
		return 0x00;	//-��Ϊ���ĳ���ֱ���˳�

		//ÿһ�����ݷ�����HEXת��Bin�ļ���������Falsh�洢
		//-�涨�����һ֡���ı�����8��һ�����ݰ�
		while(iap_recv_dl < iap_recv_pt)
		{
			if(packagedata[0] != 0x3A)
				return 0x00;

			//-packagedata = packagedata + 1 + templength;	//-ָ��������е�����
			templength = packagedata[1] + 5;	//-һ���г�ð��֮����������ݸ���
			iap_recv_dl = iap_recv_dl + 1 + templength;


			//-���Ͽ���HEX�е�ѡȡ,ѡ��һ��֮����д���
			temp_status = hex2binLine(templength, packagedata, &FlashAddress, &FlashDataLength, FlashData);	//-������֯����,������FLASHд��
      packagedata = packagedata + 1 + templength;	//-ָ��������е�����
			//����Ч�������ݽ�����д��ָ����ַ��Flash
			if(temp_status == 0x55)
			{
			//��ʱ��Ҫ���receiveDataCur��ֵ,Ҫ�Ƿ�����,����Ҫд��
				if(IAP_DataCur == 1024)	//-ÿҳ����2K�ռ�,��Ҫһ����д��
				{
						//д��flash��
						STMFLASH_Write(addrCur,iapbuf,1024);	//-����2Kһ�������Ŀռ�,һ����д��ϣ������ֵ
						//-д��֮����������,���бȽ������ȷ�˾�ͨ��,������Ϊ��������ʧ��
	          STMFLASH_Read(addrCur,iapbuf_bak,1024);
	          for(i = 0;i < 1024;i++)
	          {
	              if(iapbuf[i] != iapbuf_bak[i])
	              {//-�������ݳ���֮��,����һֱͣ��������,�����Ͳ�����Ϣ������,������Ҫ��������,���FLASHд��������
	                IAP_HEX_expectant_pt -= 0x10;   //-����������һ��
	                IAP_DataCur -= 0x08; //-������дһ������
	                return 0;
	              }
	          }
	          //printf("\r\nwrite addr %x,length 1024\r\n",addrCur);
						addrCur += 2048;//��ַ+2048
						//д��֮��receiveDataCurҪ����ȴ���һ�δ���
						IAP_DataCur = 0;
				}
				else //�п������һ����128�����ݵ�������û��2048������,��ʱ��չһ��ָ������������һ����д��
				{
						if(IAP_DataCur > 1024)	//-���ڳ��ִ�������,��Ҫ���⿼��
							return 0;
				}

				expect_hex_lines++;
				//-return 0x55;	//-һ�д���������Ӧ��ǰ֡
			}
			else if(temp_status == 0xaa)
			{//-������˵�����һ֡��,д��֮��Ϳ��Կ�ʼ������
					STMFLASH_Write(addrCur,iapbuf,IAP_DataCur); //-���ʣ�����ݵ�д��
	        STMFLASH_Read(addrCur,iapbuf_bak,IAP_DataCur);
	          for(i = 0;i < IAP_DataCur;i++)
	          {
	              if(iapbuf[i] != iapbuf_bak[i])
	                return 0;
	          }
	        addrCur = addrCur + IAP_DataCur * 2;//��ַ

					j = (addrCur - FLASH_APP1_ADDR) / 2;  //-��Ҫ���Ƶİ��ָ���
					FLASH_Unlock();						//����
					FLASH_ErasePage(APP_CONFIG_ADDR);//�����������
					FLASH_ProgramHalfWord(APP_CONFIG_ADDR,0x0000);	//-�������п�������
					FLASH_ErasePage(APP_CONFIG1_ADDR);//�����������
					FLASH_ProgramWord(APP_CONFIG2_ADDR,j);	//-��������
          if(*(__IO uint32_t*)(APP_CONFIG2_ADDR) == j)
          {
            FLASH_ProgramHalfWord(APP_CONFIG1_ADDR,0x5555);	//-��������
            FLASH_Lock();//����
            expect_hex_lines++;
            return 0x55;
          }
          else
          {//-һ����дʧ�ܲ�����Ӧ���������ط�,�ط����һ֡����û��������Ҫ��д
            //-IAP_HEX_expectant_pt -= 0x10;   //-����������һ��
	          //-IAP_DataCur -= 0x08; //-������дһ������
            FLASH_Lock();//����
            return 0x00;	//-һ�д���������Ӧ��ǰ֡
          }
			}
			else
				return 0;
		}
		return 0;	//-�����˳����ص���0 �����ж�Ӧ��
}

void STMFLASH_FLAG_clear(void)
{

  FLASH_Unlock();						//����
				FLASH_ErasePage(APP_CONFIG_ADDR);//�����������
				FLASH_ProgramHalfWord(APP_CONFIG_ADDR,0x0000);	//-�������п�������
				FLASH_ErasePage(APP_CONFIG1_ADDR);//�����������
				FLASH_ProgramHalfWord(APP_CONFIG2_ADDR,0x0000);	//-��������
        FLASH_ProgramHalfWord(APP_CONFIG1_ADDR,0x0000);	//-��������
				FLASH_Lock();//����

}










#define APP1_TO_APP0_OFF 	0x020000

void copy_app1_to_app0(unsigned long NumToWrite)
{
    u32 secpos;	   //������ַ
    u16 secoff;	   //������ƫ�Ƶ�ַ(16λ�ּ���)
    u16 secremain; //������ʣ���ַ(16λ�ּ���)
    u16 i;
    u32 offaddr;   //ȥ��0X08000000��ĵ�ַ

    FLASH_Unlock();						//����
    offaddr=FLASH_APP0_ADDR-STM32_FLASH_BASE;		        //ʵ��ƫ�Ƶ�ַ.
    secpos=offaddr/STM_SECTOR_SIZE;			//������ַ  0~127 for STM32F103RBT6
    secoff=(offaddr%STM_SECTOR_SIZE)/2;		//�������ڵ�ƫ��(2���ֽ�Ϊ������λ.)
    secremain=STM_SECTOR_SIZE/2-secoff;		//����ʣ��ռ��С,,���ռ�¼����һ�ο���д��ĸ���
    if(NumToWrite<=secremain)secremain=NumToWrite;//�����ڸ�������Χ??���������ֻ���,���������֮��û��д����,��ô����Ҫ�ٴβ���,ֱ��д����
    while(1)
    {
    		STMFLASH_Read(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE + APP1_TO_APP0_OFF,STMFLASH_BUF,STM_SECTOR_SIZE/2);//������������������,׼��д����������

        FLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//�����������

        STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//д����������
        STMFLASH_Read(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,iapbuf_bak,STM_SECTOR_SIZE/2);
          for(i = 0;i < STM_SECTOR_SIZE/2;i++)
          {
              if(STMFLASH_BUF[i] != iapbuf_bak[i])
                return;
          }


        if(NumToWrite==secremain)
        {//-������Ϳ��������ɹ���
        		FLASH_ErasePage(APP_CONFIG_ADDR);//�����������
            FLASH_ProgramHalfWord(APP_CONFIG_ADDR,0x5555);	//-��������
            if(*(__IO uint16_t*)(APP_CONFIG_ADDR) == 0x5555)
            {
              FLASH_ErasePage(APP_CONFIG1_ADDR);//�����������
              FLASH_ProgramHalfWord(APP_CONFIG1_ADDR,0x0000);	//-���ٸ���
            }
        		break;//д�������
        }
        else//д��δ����,��ε����ݹҽӳ�����һ������
        {
            secpos++;				//������ַ��1
            secoff=0;				//ƫ��λ��Ϊ0
            //-pBuffer+=secremain;  	//ָ��ƫ��
            //-WriteAddr+=secremain;	//д��ַƫ��
            NumToWrite-=secremain;	//�ֽ�(16λ)���ݼ�
            if(NumToWrite>(STM_SECTOR_SIZE/2))secremain=STM_SECTOR_SIZE/2;//��һ����������д����
            else secremain=NumToWrite;//��һ����������д����
        }
    };
    FLASH_Lock();//����


}

























































































