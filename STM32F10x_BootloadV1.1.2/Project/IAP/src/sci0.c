/*
����:ʹ��һ�������߼�,����һ��512�ֽڵĻ�����,Ȼ��ѭ���洢
����:Ҳʹ��512�ֽڵĿռ䱣������,Ȼ����һ�����ͱ�־,ֱ�����ݷ��ͳ�ȥΪֹ
*/
//-#include "user_conf.h"
#include "stm32_eval.h"

#include "common.h"



#define   MOD_KG_WAIT_TIME_VALUE                   250

#define   MOD_KG_polling_ID_FRAME                  0x01
#define   MOD_KG_polling_data_FRAME                0x02
#define   MOD_KG_LED_display_FRAME                 0x03
#define   MOD_KG_receive_data_FRAME                0x04
#define   MOD_KG_receive_set_FRAME                 0x05
#define   MOD_KG_SET_screen_FRAME									 0x06
#define   MOD_KG_mode_set_FRAME										 0x07
#define   MOD_KG_chage_page_FRAME									 0x08
#define   MOD_KG_chage_pic_FRAME									 0x09
#define   MOD_KG_SET_loop_FRAME		  							 0x0A
#define   MOD_KG_send_state_FRAME		  						 0x0B
#define   MOD_KG_re_Weather_FRAME		  						 0x0C
#define   MOD_KG_re_show_FRAME		  						   0x0D
#define   MOD_KG_polling_state_FRAME		  				 0x0E
#define   MOD_KG_polling_PM25_FRAME		  				   0x0F
#define   MOD_KG_end_PM25_FRAME		  	    			   0x10
#define   MOD_KG_polling_hl_FRAME		  	    			 0x11
#define   MOD_KG_sys_state_FRAME		  	    			 0x12
#define   MOD_KG_SET_hl_time_FRAME		  	    		 0x13
#define   MOD_KG_send_leddis_flag_FRAME		     		 0x14
#define   MOD_KG_SET_voice_flag_FRAME		     		   0x15
#define   MOD_KG_polling_voice_flag_FRAME		   	   0x16
#define   MOD_KG_re_gateway_flag_FRAME		    	   0x17
#define   MOD_KG_re_ZigBee_flag_FRAME		    	     0x19
#define   MOD_KG_send_HRL_mode_FRAME		      	   0x1A			//-���ֵ�Ƚ�����.�������ư�ʹ�õĲ�ͬ
#define   MOD_KG_send_voice_mode_FRAME	      	   0x1B
#define   MOD_KG_send_es705_mode_FRAME	      	   0x1C
#define   MOD_KG_send_es705_event_type	      	   0x1E
#define   MOD_KG_send_HRL_ONOFF_FRAME 	      	   0x1F
#define   MOD_KG_send_inquire_state_FRAME 	   	   0x20
#define   MOD_KG_set_hl_set_FRAME	     	           0x21


extern unsigned char UpdatePackageDataHandle(unsigned char *packagedata, unsigned int datasize);

extern u32 addrCur;
extern u16 IAP_DataCur;
extern u16 IAP_HEX_expectant_pt;

//-ΪЭ�鴦����ʱ����ı���
BYTE  MOD_KG_rxd_head_flag;
BYTE  MOD_KG_rec_OK;
BYTE  MOD_KG_rec_frame_type;
WORD  MOD_KG_rxd_wait_time;
BYTE  MOD_KG_transmit_flag;
BYTE  MOD_KG_transmit_control;
WORD  MOD_KG_transmit_wait_time;
WORD  MOD_KG_Uto7620_wait_time;
WORD  MOD_KG_es705_wait_time;


void UART1_Init(void)
{
  port_recv_pt[0] = 0;
  port_send_pt[0] = 0;
  port_recv_dl[0] = 0;

  //-Э���ʼ������
  port_deal_flag[0] = 0;
  MOD_KG_transmit_control = 0;	//-�ȴ��������,Ȼ��ʼ�������ڱ���
  MOD_KG_rxd_head_flag=NO;
  //-UART0_transmit_flag = 0x55;
  //-UART0_start_tx_time = Time_2048ms_Counter;

  //-UART1_sloop_flag = 0;

}

void UART1_start(void)
{
  //-Э���ʼ������
  MOD_KG_transmit_control = 1;	//-�ȴ��������,Ȼ��ʼ�������ڱ���

}


void UART_Rx_Deal(void)
{
#ifndef STM3210E_UP
  port_recv_pt[0] = 512 - DMA_GetCurrDataCounter(DMA1_Channel6);	//-�õ����ո���,�����Ϊ����ƫ�����Ļ�,���ڵ���ֵ����ָ�����ռ��
  if(port_recv_pt[0] >= 512)
  	port_recv_pt[0] = 0;
#else
  port_recv_pt[0] = 512 - DMA_GetCurrDataCounter(DMA1_Channel5);	//-�õ����ո���,�����Ϊ����ƫ�����Ļ�,���ڵ���ֵ����ָ�����ռ��
  if(port_recv_pt[0] >= 512)
  	port_recv_pt[0] = 0;
#endif
}

void UART_Tx_Deal(void)
{
  /* Enable USARTy DMA TX request */
  //-USART_DMACmd(USARTy, USART_DMAReq_Tx, ENABLE);			//-����DMA����
#ifndef STM3210E_UP
  if(port_send_len[0] != 0)	//-һ��Ҫ��֤���ݷ��ͳ�ȥ��,��ʹ��DMA��������
  {
		  DMA_Cmd(DMA1_Channel7, DISABLE);
		  DMA1_Channel7->CNDTR = port_send_len[0]; //-���������Ĵ���,ָʾʣ��Ĵ������ֽ���Ŀ
		  DMA_Cmd(DMA1_Channel7, ENABLE);
      //-if(port_send_len[0] != 42)		//-������
      //-  port_send_len[0] = 0;   //- 04 13 8B 00 01 46 70���������ڳ�������������,����32��û�з��ͳ�ȥ

      port_send_len[0] = 0;
	}
#else
  if(port_send_len[0] != 0)	//-һ��Ҫ��֤���ݷ��ͳ�ȥ��,��ʹ��DMA��������
  {
		  DMA_Cmd(DMA1_Channel4, DISABLE);
		  DMA1_Channel4->CNDTR = port_send_len[0]; //-���������Ĵ���,ָʾʣ��Ĵ������ֽ���Ŀ
		  DMA_Cmd(DMA1_Channel4, ENABLE);
		  port_send_len[0] = 0;
	}
#endif
}


////////////////////////////////////////////////////////////////////////////////
/*
����1:��ѯ�������豸��ID��
7620���ͣ�
0xaa 0x55 0x05 0x02 0x01 CRC
STM32��Ӧ��
0xaa 0x55 0x0b 0x02 0x01 0x01 0x02 0x03 0x04 0x05 0x06 CRC






*/
unsigned int MOD_KG_CRC16(unsigned char *MOD_KG_CRC16_start,unsigned char MOD_KG_CRC16_bytes)    //*xΪָ��ÿ��ǰ5�����ݵ�ָ��
{	//-��������У��CRC
unsigned int bx, cx, i, j;

    bx = 0xffff;
    cx = 0xa001;
    for(i=0;i<MOD_KG_CRC16_bytes;i++)
     {
      bx=bx^MOD_KG_CRC16_start[i];
      for(j=0;j<8;j++)
       {
        if ((bx&0x0001)==1)
         {
          bx=bx>>1;
          bx=bx&0x7fff;
          bx=bx^cx;
         }
        else
         {
          bx=bx>>1;
          bx=bx&0x7fff;
         }
       }
     }
    return(bx);
}

void MOD_KG_clear_port_report_deal(void)
{
	 WORD i,len,temp_loop;

	 len = port_report[2] + 2;
	 temp_loop = port_recv_dl[0];

	 for(i=0;i<len;i++)
	 {
	 	  if(temp_loop > 0)
	 	  	temp_loop = temp_loop - 1;
	 	  else
	 	  	temp_loop = 511;
	    port_recv[temp_loop] = 0;
	 }

	 		//-ʹ�õ���ʱ�������
	 for(i=0;i<len;i++)
	   port_report[i] = 0;
}

//ϵͳ��λ

void Sys_Soft_Reset(void)
{
    SCB->AIRCR =0X05FA0000|(u32)0x04;
}

void Sys_run_old(void)
{
  WORD the_ram_ax;

  if(*(__IO uint16_t*)(APP_CONFIG1_ADDR) != 0x5555)
  {//-ǿ��ʹ���ϳ����ǰ�����ϳ���û�б��޸�
    FLASH_Unlock();						//����
		FLASH_ErasePage(APP_CONFIG_ADDR);//�����������
		FLASH_ProgramHalfWord(APP_CONFIG_ADDR,0x5555);	//-����ʧ�ܿ��������ϳ���
    if(*(__IO uint16_t*)(APP_CONFIG_ADDR) == 0x5555)
    {
        port_send[0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[1] = 0x55;

			  port_send[2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[3] = 0x9c;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[4] = 0x04;		//-���ݳ���
			  port_send[5] = 0x55;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0],port_send[2]);
			  port_send[6] =LOBYTE(the_ram_ax);
			  port_send[7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[2] + 2;
			  port_deal_flag[0] = 0xaa;
        UART_Tx_Deal();
#ifndef STM3210E_UP
        while(DMA_GetFlagStatus(DMA1_FLAG_TC7) != 1);  //-�ȴ����ͽ���
#else
        while(DMA_GetFlagStatus(DMA1_FLAG_TC4) != 1);
#endif
    }
	  FLASH_Lock();//����
  }
}


void MOD_KG_LED_display_deal(void)		//-��7620�´�����ݽ��н���
{
	  WORD the_ram_ax;


       //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[1] = 0x55;

			  port_send[2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[3] = UART3_TO_UART2_FLAG;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[4] = 0xaa;		//-���ݳ���
			  port_send[5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0],port_send[2]);
			  port_send[6] =LOBYTE(the_ram_ax);
			  port_send[7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[2] + 2;
			  port_deal_flag[0] = 0xaa;

}


void MOD_KG_HEX_package_deal(void)		//-��7620�´�����ݽ��н���
{
	  WORD the_ram_ax;
	  unsigned char temp_flag;

    //-�Խ��յ��ı��Ľ���ǰ�ڰ���
    recv_hex_lines = (port_report[4] << 8) + port_report[5];
    expect_hex_lines = recv_hex_lines;

    the_ram_ax = port_report[2] - 6;	//-��Ҫ�����ĵĳ���
    //-��ת���ڲ�����HEX����,�����ȷ�Ļ���Ӧ�������´α��ķ���,�м�Ķ�ʧ�ط���λ�ȵ�,����λ������
    temp_flag = UpdatePackageDataHandle(&port_report[6], the_ram_ax);
    if((temp_flag == 0x55) || (expect_hex_lines > recv_hex_lines))	//-���ԵĻ��ͽ���Ӧ��,��������λ�������ط�
    {
    	//-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[1] = 0x55;

			  port_send[2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[3] = 0xA2;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  expect_hex_lines -= 1;
			  port_send[4] = HIBYTE(expect_hex_lines);		//-���ݳ���
			  port_send[5] = LOBYTE(expect_hex_lines);

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0],port_send[2]);
			  port_send[6] =LOBYTE(the_ram_ax);
			  port_send[7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[2] + 2;
			  port_deal_flag[0] = 0xaa;
    }

}


void MOD_KG_inquire_version_deal(void)		//-��7620�´�����ݽ��н���
{
	  WORD the_ram_ax;
	  //-unsigned char temp_flag;


    //-if(temp_flag == 0x55)	//-���ԵĻ��ͽ���Ӧ��,��������λ�������ط�
    {
    	//-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[1] = 0x55;

			  port_send[2] = 8;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[3] = 0xA4;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[4] = 0;		//-���ݳ���
			  port_send[5] = 0;

        port_send[6] = 0;		//-���ݳ���
			  port_send[7] = 0;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0],port_send[2]);
			  port_send[8] =LOBYTE(the_ram_ax);
			  port_send[9] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[2] + 2;
			  port_deal_flag[0] = 0xaa;
    }

}

void MOD_KG_inquire_update_deal(void)		//-�ѽ��յ������ݽ���͸������
{
	  WORD the_ram_ax;

    if((port_report[4] == 1) && (port_report[5] == 0x55))
    {//-������յ���������
        //-run_in_iap();   //-�˳���������,����IAP����

        //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[1] = 0x55;

			  port_send[2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[3] = 0xa3;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[4] = 0x01;		//-���ݳ���
			  port_send[5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0],port_send[2]);
			  port_send[6] =LOBYTE(the_ram_ax);
			  port_send[7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[2] + 2;
			  port_deal_flag[0] = 0xaa;

        //-while(1);   //-�ȴ���λ����,����IAP����
    }

    if((port_report[4] == 2) && (port_report[5] == 0x55))
    {//-������յ���������
        //-run_in_iap();   //-�˳���������,����IAP����

        //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[1] = 0x55;

			  port_send[2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[3] = 0xa3;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[4] = 0x02;		//-���ݳ���
			  port_send[5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0],port_send[2]);
			  port_send[6] =LOBYTE(the_ram_ax);
			  port_send[7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[2] + 2;
			  port_deal_flag[0] = 0xaa;

        //-while(1);   //-�ȴ���λ����,����IAP����
    }

    //-��λΪ��ʼ״̬�ȴ�����
    addrCur = FLASH_APP1_ADDR;
    IAP_DataCur = 0;
    IAP_HEX_expectant_pt = 0x4000;
}



//-���洦��ͨѶ���յ�������,��ʵ����һ���������ϲ�Э��Ĵ���,���������������ڵ���
//-������һ����Ҫ�������Ǵ�����Ϣ
void CommandScan()
{
	 WORD the_ram_ax,the_ram_bx;
	 WORD temp_data,temp_data1,delta_len,temp_loop,temp_int;


   UART_Rx_Deal();
   UART_Tx_Deal();

#ifndef STM3210E_UP
	 //-�жϷ����Ƿ������,��������˿��������߼�����,���򲻴���
	 if(DMA_GetFlagStatus(DMA1_FLAG_TC7))		//-����˵�����������
   {//-ֻ�е���������˲ſ������¸�DMA���ʹ�������
   	  //-for(temp_loop = 0;temp_loop < 43;temp_loop++)
   	  //-	port_send[temp_loop] = 0;
   	  port_deal_flag[0] = 0;		//-��һ����0ֵ�Ϳ������´μ�鷢��ʱ������������
   	  //-MOD_KG_transmit_flag=YES;
   	  DMA_ClearFlag(DMA1_FLAG_TC7);
   }
#else
    //-�жϷ����Ƿ������,��������˿��������߼�����,���򲻴���
	 if(DMA_GetFlagStatus(DMA1_FLAG_TC4))		//-����˵�����������
   {//-ֻ�е���������˲ſ������¸�DMA���ʹ�������
   	  //-���ݷ������֮��ȫ������,�������ڲ�֪��ÿ���Ŀռ��С��,��ôѡȡһ��������
   	  //-for(temp_loop = 0;temp_loop < 43;temp_loop++)
   	  //-	port_send[0][temp_loop] = 0;
   	  port_deal_flag[0] = 0;		//-��һ����0ֵ�Ϳ������´μ�鷢��ʱ������������
   	  //-MOD_KG_transmit_flag=YES;		//-���ҵ����ݷ��ͳ�ȥ��Ҳ������������������??����Ӧ�ò���������֯���ݵķ���,������������ݻᱻ����,���յı�־λ
   	  DMA_ClearFlag(DMA1_FLAG_TC4);
   }
#endif

   if(port_deal_flag[0] == 0)
   {
      if((port_recv_pt[0]!=port_recv_dl[0]))
      {
           if(MOD_KG_rxd_head_flag==NO)	//-���յ������ݻ�û�д����ʱ�����NO
           {
           	   //-MOD_KG_rxd_wait_time=cticks_ms;		//-��ӵ��д�����
		   	   	   temp_data = port_recv_pt[0];
		   	   	   temp_data1 = port_recv_dl[0];
		           if(temp_data1>temp_data)	//-ǰ����Ǵ���ָ���ʵ�ʵĽ���ָ����бȽ�
		               delta_len=(temp_data+512)-temp_data1;
		           else
		               delta_len=temp_data-temp_data1;	//-һ���ĳ���
		           for(temp_loop=temp_data1;temp_loop<(delta_len+temp_data1);temp_loop++)
		           {
		        	   if(port_recv[port_recv_dl[0]]==0xaa)	//-����ط��Ƚϵ��Ǵ�վ��ַ,�����Ҿ���û���κι��ɾ���ͨѶ
		        	   {	//-����һ�п������õ�
		        	     the_ram_ax=(port_recv_dl[0]+1)&0x1ff;
		        	     if(temp_data == the_ram_ax)
		        	     	 break;
		        	     if(port_recv[the_ram_ax]==0x55)	//-�Ƚϵ��ǹ�����
		        	     {
		        	         MOD_KG_rxd_head_flag=YES;	//-��ʾ�Ѿ��ɹ�ʶ����յ����±��ĵ�ͷ��
		        	         break;
		        	     }
		        	   }
		        	   port_recv_dl[0]++;	//-����һ���ֵı���
		        	   port_recv_dl[0]&=0x1ff;
		           }
   	   	   }
   	   	   if(MOD_KG_rxd_head_flag==YES)	//-���յ������ݻ�û�д����ʱ�����NO
       		 {
       		 	   temp_data = port_recv_pt[0];
       		 	   if(port_recv_dl[0]>temp_data)
               		delta_len=(temp_data+512)-port_recv_dl[0];
               else
               		delta_len=temp_data-port_recv_dl[0];

               if(delta_len>6)	//-���ٻ���4���ֽڲ�����֯һ������
		           {
		               temp_int=(port_recv_dl[0]+2)&0x1ff;
		               if(delta_len>=(unsigned short)(port_recv[temp_int]+2))	//-�õ��ı��ĳ��Ⱥ������ϵı��ĳ��Ƚ��бȽ�
		               {
		                  MOD_KG_rxd_head_flag=NO;
		                  MOD_KG_rec_OK=YES;

		               }

		           }
               else
                 return;

     			 }

           if(MOD_KG_rec_OK==YES)
           {
              //-������Ϳ���˵��Ӧ�����Ѿ�������
              MOD_KG_rxd_head_flag=NO;
              MOD_KG_rec_OK=NO;	//-�ɹ����յ������ݿ�ʼ������֮��,�ͻָ�0
              MOD_KG_transmit_flag=YES;
              //-test_cn_wait_time = cticks_ms;	//-������
              //-MOD_KG_transmit_wait_time=Time_1ms_Counter;	//-��Ȼ����˵���Է���������,���ǻ���Ҫ��ʱһ��ʱ��,��Ϊ���յ���������Ҫ����
              //-�����Ƕ�ʵ�����ݵĴ���,������Ǻ��Ĳ���
              the_ram_bx=(port_recv_dl[0]+3)&0x1ff;;
              if(port_recv[the_ram_bx]!=0xFF)	//-����ǶԹ�������ж�,�����벻ͬ�жϵ�����Ҳ��һ��
              {	//-����������һ�ִ������ڿ��Բ���
                  the_ram_ax=(port_recv_dl[0]+2)&0x1ff;
                  temp_int=port_recv[the_ram_ax]+2+port_recv_dl[0];
                  for(temp_loop=port_recv_dl[0];temp_loop<temp_int;temp_loop++)	//-���������ɵ����ܾ��Ǳ�֤��λ����Ҫ����ı����ֽ�
                  {	//-�򵥵Ĳ���Ҫ���������Ǹ��ӵĻ�����Ҫ��,��ô�������˵û������Ծͻ�ܺ�
                       if(temp_loop<=511)
                         port_report[temp_loop-port_recv_dl[0]]=port_recv[temp_loop];
                       else
                         port_report[temp_loop-port_recv_dl[0]]=port_recv[temp_loop-512];	//-�ѵ��Ǹ��ٸ��µ�Ե����Ҫ��ǰ���Ƴ���
                  }	//-���߻���һ�ֿ����Ծ���ͳһ����
                //-port_recv_dl[0]+=delta_len;	//-����ط��������������Ĵ�����
                port_recv_dl[0]+=(port_report[2] + 2);
                port_recv_dl[0]&=0x1ff;
                temp_int=MOD_KG_CRC16(&port_report[0],port_report[2]);
                if((LOBYTE(temp_int)!=port_report[port_report[2]+0])||(HIBYTE(temp_int)!=port_report[port_report[2]+1]))	//-����CRC���
                {	//-������Ĳ�ȷ����,���У�鲻��ȷ�Ļ���ֱ�Ӱ�ͷ���,����Ѱ��
                    return;	//-������˵���ɹ����յ��ı���CRCУ��û��ͨ��
                }
              }
              else
              {
                  //-port_recv_dl[0]+=delta_len;	//-������Ϳ��԰ѽ��ջ������е�����������,�Ѿ�����������
                  port_recv_dl[0]+=7;
                  port_recv_dl[0]&=0x1ff;
                  return;
              }

              if(port_report[3] == 0x1C)	//-�жϹ�����
              {
                 if((port_report[4] == 0x03) && (port_report[5] == 0x55))
                 {//-������λ
                     MOD_KG_rec_frame_type = MOD_KG_polling_ID_FRAME;
                     Sys_Soft_Reset();
                 }
                 else
                 {
                   if((port_report[4] == 0x04) && (port_report[5] == 0x55))
                   {//-����ʧ��ǿ��ʹ���ϳ���
                       MOD_KG_rec_frame_type = MOD_KG_polling_ID_FRAME;
                       Sys_run_old();
                   }

                 }
              }
              else
              {
                  if(port_report[3] == 0x0d)
                  {
                     MOD_KG_LED_display_deal();

                  }
                  else
                  {
                      MOD_KG_rec_frame_type = MOD_KG_polling_data_FRAME;
										  if(port_report[3] == 0x22)
		                  {
		                     MOD_KG_HEX_package_deal();

		                  }
		                  else
		                  {
                        if(port_report[3] == 0x24)
                        {//-��ѯ�汾��
                           MOD_KG_inquire_version_deal();

                        }
                        else
                        {
                          if(port_report[3] == 0x23)
                          {//-����֪ͨSTM32���г�������
                             MOD_KG_inquire_update_deal();

                          }
                          else
                              MOD_KG_rec_frame_type = MOD_KG_polling_data_FRAME;
                        }

		                  }
                  }
              }

              //-MOD_KG_clear_port_report_deal();		//-ÿ�δ������֮�������������ֹ����ʹ��
           }
      }
   }

}

//-��һ��STM32��7620����ư巢������		����ҪӦ��
//-�ڶ���7620��STM32��������		����Ϊ�ӻ��ǿ�������Ӧ���
//-������7620����ư巢������,����ֱ��͸������ư�

//-��ư���7620��������

//-��ֻ֮Ҫû������Ҫ�ȴ�Ӧ���,�Ҷ�������������
//-���ﲻ��Ҫ���κ��豸�´��κ�����,������Ҳ�Ǵӻ�,����Ҫ�ȴ�����

//-2016.1.7 19:46
//-ͨ��ץ����������7620���ͳ�����(�ҽӴ���PC�˶���������),ͨ���۲�
//-����ָ��STM32Ҳ��ȷ���յ�������,���Ǿ���û��Ӧ��.

//?Ŀǰ���ɿ��ܴ���1�ķ��ͷ�ʽ,�ͱ������д��ڵ�ɾ����ʽ�г�ͻ,���ܵ����������ĵĶ�ʧ.


