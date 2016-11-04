/*
����:ʹ��һ�������߼�,����һ��512�ֽڵĻ�����,Ȼ��ѭ���洢
����:Ҳʹ��512�ֽڵĿռ䱣������,Ȼ����һ�����ͱ�־,ֱ�����ݷ��ͳ�ȥΪֹ
*/
#include "user_conf.h"




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

extern void RunLed_stata_judge(void);
extern void RunLed_stata_judge_new(BYTE page,BYTE *data);

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

  UART0_transmit_flag = 0x55;
  UART0_start_tx_time = Time_2048ms_Counter;

  UART1_sloop_flag = 0;

}

void UART1_start(void)
{
  //-Э���ʼ������
  MOD_KG_transmit_control = 1;	//-�ȴ��������,Ȼ��ʼ�������ڱ���

}


void UART_Rx_Deal(void)
{
  port_recv_pt[0] = 512 - DMA_GetCurrDataCounter(DMA1_Channel6);	//-�õ����ո���,�����Ϊ����ƫ�����Ļ�,���ڵ���ֵ����ָ�����ռ��
  if(port_recv_pt[0] >= 512)
  	port_recv_pt[0] = 0;
}

void UART_Tx_Deal(void)
{
  /* Enable USARTy DMA TX request */
  //-USART_DMACmd(USARTy, USART_DMAReq_Tx, ENABLE);			//-����DMA����

  if(port_send_len[0] != 0)	//-һ��Ҫ��֤���ݷ��ͳ�ȥ��,��ʹ��DMA��������
  {
		  DMA_Cmd(DMA1_Channel7, DISABLE);
		  DMA1_Channel7->CNDTR = port_send_len[0]; //-���������Ĵ���,ָʾʣ��Ĵ������ֽ���Ŀ
		  DMA_Cmd(DMA1_Channel7, ENABLE);
      //-if(port_send_len[0] != 42)		//-������
      //-  port_send_len[0] = 0;   //- 04 13 8B 00 01 46 70���������ڳ�������������,����32��û�з��ͳ�ȥ

      port_send_len[0] = 0;
	}
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
	    port_recv[0][temp_loop] = 0;
	 }

	 		//-ʹ�õ���ʱ�������
	 for(i=0;i<len;i++)
	   port_report[i] = 0;
}

void MOD_KG_polling_ID_cmd(void)
{
	  WORD the_ram_ax;

	  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
	  port_send[0][1] = 0x55;

	  port_send[0][2] = 0x05;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

	  port_send[0][3] = 0x02;		//-������:01h �����������ϱ�

	  //-��Ч����
	  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
	  port_send[0][4] = 0x01;		//-���ݳ���

	  //-CRC16
	  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
	  port_send[0][5] =LOBYTE(the_ram_ax);
	  port_send[0][6] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

	  //-���ͳ���
	  port_send_len[0] = port_send[0][2] + 2;
	  port_deal_flag[0] = 0xaa;

	  cticks_500ms = 0;
}

void MOD_KG_send_sense_data(void)
{
	  WORD the_ram_ax,i;

	  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
	  port_send[0][1] = 0x55;

	  port_send[0][2] = 40;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

	  port_send[0][3] = 0x01;		//-������:01h �����������ϱ�

	  //-��Ч����
	  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
	  port_send[0][4] = 0x0A;		//-���ݳ���		,,CO2
	  port_send[0][5] = 0xA0;		//-�豸ID
	  port_send[0][6] = 0x04;		//-������
	  port_send[0][7] = 0x01;		//-��������
	  port_send[0][8] = HIBYTE(port_send_sense_data[1]);		//-����λ
	  port_send[0][9] = LOBYTE(port_send_sense_data[1]);

	  port_send[0][10] = 0x0A;		//-���ݳ���		,,�¶�
	  port_send[0][11] = 0xD1;		//-�豸ID
	  port_send[0][12] = 0x04;		//-������
	  port_send[0][13] = 0x02;		//-��������
	  port_send[0][14] = HIBYTE(port_send_sense_data[2]);		//-����λ
	  port_send[0][15] = LOBYTE(port_send_sense_data[2]);

	  port_send[0][16] = 0x0A;		//-���ݳ���		,,VOC
	  port_send[0][17] = 0xD6;		//-�豸ID
	  port_send[0][18] = 0x04;		//-������
	  port_send[0][19] = 0x02;		//-��������
	  port_send[0][20] = HIBYTE(port_send_sense_data[3]);		//-����λ
	  port_send[0][21] = LOBYTE(port_send_sense_data[3]);

	  port_send[0][22] = 0x0A;		//-���ݳ���		,,PM2.5
	  port_send[0][23] = 0xD5;		//-�豸ID
	  port_send[0][24] = 0x04;		//-������
	  port_send[0][25] = 0x01;		//-��������
	  port_send[0][26] = HIBYTE(port_send_sense_data[4]);		//-����λ
	  port_send[0][27] = LOBYTE(port_send_sense_data[4]);

	  port_send[0][28] = 0x0A;		//-���ݳ���		,,����
	  port_send[0][29] = 0xD4;		//-�豸ID
	  port_send[0][30] = 0x04;		//-������
	  port_send[0][31] = 0x01;		//-��������
	  port_send[0][32] = HIBYTE(port_send_sense_data[5]);		//-����λ
	  port_send[0][33] = LOBYTE(port_send_sense_data[5]);

	  port_send[0][34] = 0x01;		//-���ݳ���		,,Ԥ��
	  port_send[0][35] = 0x02;		//-�豸ID
	  port_send[0][36] = HIBYTE(NTC_data);		//-������			//-�����¶�
	  port_send[0][37] = LOBYTE(NTC_data);		//-��������
	  port_send[0][38] = HIBYTE(port_send_sense_data[6]);		//-�¶�2
	  port_send[0][39] = LOBYTE(port_send_sense_data[6]);

	  //-CRC16
	  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
	  port_send[0][40] =LOBYTE(the_ram_ax);
	  port_send[0][41] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

	  if((Judge_LongTime_In_MainLoop(MOD_KG_Uto7620_wait_time,146) == YES) || (port_send_sense_data[0] == 0x55))		//-��Լ5���ӷ�һ��
	  {
	  	MOD_KG_Uto7620_wait_time = Time_2048ms_Counter;
	  	port_send_sense_data[0] = 0;
		  //-���ͳ���,���������͵Ĺ���
		  port_send_len[0] = port_send[0][2] + 2;
		  port_deal_flag[0] = 0xaa;
		}

	  //-ͬʱ����ư�ת��,ֻҪ������û�з��ͳ�ȥ�����ݾͿ���
	  for(i=0;i<(port_send[0][2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_send[0][i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }

	  //-������������ʾ������ʾ,���ʱ����б�Ҫ�ж��л���ʾ״̬
	  RunLed_stata_judge();
}

void MOD_KG_send_inquire_state(void)
{
	  WORD the_ram_ax;

	  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
	  port_send[0][1] = 0x55;

	  port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

	  port_send[0][3] = 0x20;		//-������:01h �����������ϱ�

	  //-��Ч����
	  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
	  port_send[0][4] = 0x00;		//-���ݳ���		,,CO2
	  port_send[0][5] = 0x00;		//-�豸ID

	  //-CRC16
	  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
	  port_send[0][6] =LOBYTE(the_ram_ax);
	  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

		  //-���ͳ���,���������͵Ĺ���
		  port_send_len[0] = port_send[0][2] + 2;
		  port_deal_flag[0] = 0xaa;

}

void MOD_KG_polling_ID_ack(void)		//-��7620��ѯ��Ӧ��
{
	  WORD the_ram_ax;

	  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
	  port_send[0][1] = 0x55;

	  port_send[0][2] = 0x0b;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

	  port_send[0][3] = 0x02;		//-������:01h �����������ϱ�

	  //-��Ч����
	  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
	  port_send[0][4] = 0x01;		//-���ݳ���
	  port_send[0][5] = 0xA0;
	  port_send[0][6] = 0xD4;
	  port_send[0][7] = 0xD5;
	  port_send[0][8] = 0xD6;
	  port_send[0][9] = 0x05;
	  port_send[0][10] = 0x06;

	  //-CRC16
	  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
	  port_send[0][11] =LOBYTE(the_ram_ax);
	  port_send[0][12] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

	  //-���ͳ���
	  port_send_len[0] = port_send[0][2] + 2;
	  port_deal_flag[0] = 0xaa;
}

//-rece:0xaa 0x55 0x05 0x02 0x02 CRC
//-send:0xaa 0x55 0x05 0x02 0x02 ���� CRC
void MOD_KG_polling_data_ack(void)		//-��7620��ѯ��Ӧ��
{
	  WORD the_ram_ax;

	  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
	  port_send[0][1] = 0x55;

	  port_send[0][2] = 40;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

	  port_send[0][3] = 0x01;		//-������:01h �����������ϱ�

	  //-��Ч����
	  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
	  port_send[0][4] = 0x0A;		//-���ݳ���		,,CO2
	  port_send[0][5] = 0xA0;		//-�豸ID
	  port_send[0][6] = 0x04;		//-������
	  port_send[0][7] = 0x01;		//-��������
	  port_send[0][8] = HIBYTE(port_send_sense_data[1]);		//-����λ
	  port_send[0][9] = LOBYTE(port_send_sense_data[1]);

	  port_send[0][10] = 0x0A;		//-���ݳ���		,,�¶�
	  port_send[0][11] = 0xD1;		//-�豸ID
	  port_send[0][12] = 0x04;		//-������
	  port_send[0][13] = 0x02;		//-��������
	  port_send[0][14] = HIBYTE(port_send_sense_data[2]);		//-����λ
	  port_send[0][15] = LOBYTE(port_send_sense_data[2]);

	  port_send[0][16] = 0x0A;		//-���ݳ���		,,VOC
	  port_send[0][17] = 0xD6;		//-�豸ID
	  port_send[0][18] = 0x04;		//-������
	  port_send[0][19] = 0x02;		//-��������
	  port_send[0][20] = HIBYTE(port_send_sense_data[3]);		//-����λ
	  port_send[0][21] = LOBYTE(port_send_sense_data[3]);

	  port_send[0][22] = 0x0A;		//-���ݳ���		,,PM2.5
	  port_send[0][23] = 0xD5;		//-�豸ID
	  port_send[0][24] = 0x04;		//-������
	  port_send[0][25] = 0x01;		//-��������
	  port_send[0][26] = HIBYTE(port_send_sense_data[4]);		//-����λ
	  port_send[0][27] = LOBYTE(port_send_sense_data[4]);

	  port_send[0][28] = 0x0A;		//-���ݳ���		,,����
	  port_send[0][29] = 0xD4;		//-�豸ID
	  port_send[0][30] = 0x04;		//-������
	  port_send[0][31] = 0x01;		//-��������
	  port_send[0][32] = HIBYTE(port_send_sense_data[5]);		//-����λ
	  port_send[0][33] = LOBYTE(port_send_sense_data[5]);

	  port_send[0][34] = 0x01;		//-���ݳ���		,,Ԥ�� �����¶�2
	  port_send[0][35] = 0x02;		//-�豸ID
	  port_send[0][36] = 0x03;		//-������
	  port_send[0][37] = 0x04;		//-��������
	  port_send[0][38] = HIBYTE(port_send_sense_data[6]);		//-����λ
	  port_send[0][39] = LOBYTE(port_send_sense_data[6]);

	  //-CRC16
	  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
	  port_send[0][40] =LOBYTE(the_ram_ax);
	  port_send[0][41] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

	  //-���ͳ���
	  port_send_len[0] = port_send[0][2] + 2;
	  port_deal_flag[0] = 0xaa;
}

void MOD_KG_LED_display_deal(void)		//-��7620�´�����ݽ��н���
{
	  //-WORD the_ram_ax;

	  //-�ر���һ���������洢����,�ñ�־λ,Ȼ��ˢ�µ�ʱ��������������Ƿ���Ч
	  //-����еĻ����л���ʾ

}

void MOD_KG_SET_screen_deal_ack(void)		//-��������ʱ��
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-����ʱ��
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = 0xaa;		//-���ݳ���
			  port_send[0][5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }



}

void MOD_KG_receive_set_deal_ack(void)		//-�����Ųʵ���ɫ����
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-����ʱ��
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 8;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = 1;		//-���ݳ���
			  port_send[0][5] = HL_ld_R_user[0];
			  port_send[0][6] = HL_ld_G_user[0];
			  port_send[0][7] = HL_ld_B_user[0];


			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][8] =LOBYTE(the_ram_ax);
			  port_send[0][9] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }



}

void MOD_KG_mode_set_deal_ack(void)		//-��������ʱ��
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-����ʱ��
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = HL_flag;		//-���ݳ���
			  port_send[0][5] = 0xaa;

        if(HL_flag == 3)  //-������
			 		HL_flag = 3;
			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }



}

void MOD_KG_chage_pic_deal_ack(void)		//-��������ʱ��
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-����ʱ��
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = 0xaa;		//-���ݳ���
			  port_send[0][5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }



}

void MOD_KG_SET_loop_deal_ack(void)		//-��������ʱ��
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-����ʱ��
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = 0xaa;		//-���ݳ���
			  port_send[0][5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }



}

void MOD_KG_send_state_deal_ack(void)		//-��������ʱ��
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-����ʱ��
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = 0xaa;		//-���ݳ���
			  port_send[0][5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }



}

void MOD_KG_re_Weather_deal_ack(void)		//-��������ʱ��
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-����ʱ��
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = 0xaa;		//-���ݳ���
			  port_send[0][5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }



}

void MOD_KG_re_show_deal_ack(void)		//-��������ʱ��
{
	  WORD the_ram_ax;
	  static BYTE i=0;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-����ʱ��
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = 0xaa;		//-���ݳ���
			  port_send[0][5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }

    //-test_cn[3]++;
    //-test_cn[4] = 4;
    //-if(Judge_Time_In_MainLoop(test_cn_wait_time,400)==YES)

    if(test_cn_wait_time < cticks_ms)
    	test_cn[i] = cticks_ms - test_cn_wait_time;
    else
    	test_cn[i] = 65536 + cticks_ms - test_cn_wait_time;

    if(test_cn[i] > 400)
      test_cn[7]++;

    if(i<6)
	  {
    	i++;
    }
    else
    	i = 0;
}

void MOD_KG_polling_state_deal_ack(void)		//-��������ʱ��
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-����ʱ��
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = STM32_shedeng_selfT_flag1;		//-���ݳ���
			  port_send[0][5] = 0;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }



}

void MOD_KG_polling_hl_deal_ack(void)		//-��������ʱ��
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-����ʱ��
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = HL_flag;		//-���ݳ���
			  port_send[0][5] = 0x00;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }



}

void MOD_KG_SET_hl_time_deal_ack(void)		//-��������ʱ��
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-����ʱ��
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = 0xaa;		//-���ݳ���
			  port_send[0][5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }



}

void MOD_KG_control_HRL_deal_ack(void)		//-������֯Ӧ�����ݽ�����Ϊ�˿��ƴ��ڷ���,����7620������
{
	  WORD the_ram_ax,i;
	  BYTE	temp_report[32];

	  //-if(port_report[4] == 0x01)		//-
	  {


       //-������ݳɹ����յ�֮���Ӧ��һ��
			  temp_report[0] = 0xaa;		//-���ֽڰ�ͷ
			  temp_report[1] = 0x55;

			  temp_report[2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  temp_report[3] = 0xC0;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  temp_report[4] = 0xaa;		//-���ݳ���
			  temp_report[5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&temp_report[0],temp_report[2]);
			  temp_report[6] =LOBYTE(the_ram_ax);
			  temp_report[7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  for(i=0;i<(temp_report[2] + 2);i++)
			  {
			  	port_send[1][port_send_pt[1]] = temp_report[i];
			  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
			  }

	  }



}

void MOD_KG_leddis_end_ack(void)		//-������֯Ӧ�����ݽ�����Ϊ�˿��ƴ��ڷ���,����7620������
{
	  WORD the_ram_ax,i;
	  BYTE	temp_report[32];

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-����ʱ��
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-������ݳɹ����յ�֮���Ӧ��һ��
			  temp_report[0] = 0xaa;		//-���ֽڰ�ͷ
			  temp_report[1] = 0x55;

			  temp_report[2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  temp_report[3] = 0xC1;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  temp_report[4] = 0xaa;		//-���ݳ���
			  temp_report[5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&temp_report[0],temp_report[2]);
			  temp_report[6] =LOBYTE(the_ram_ax);
			  temp_report[7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  for(i=0;i<(temp_report[2] + 2);i++)
			  {
			  	port_send[1][port_send_pt[1]] = temp_report[i];
			  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
			  }

	  }



}

void MOD_KG_send_HRL_mode_ack(void)		//-������֯Ӧ�����ݽ�����Ϊ�˿��ƴ��ڷ���,����7620������
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-����ʱ��
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 8;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = 0x1A;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = HL_flag;		//-���ݳ���
			  port_send[0][5] = HL_ld_R_user[0];
			  port_send[0][6] = HL_ld_G_user[0];		//-���ݳ���
			  port_send[0][7] = HL_ld_B_user[0];

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][8] =LOBYTE(the_ram_ax);
			  port_send[0][9] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;

	  }



}

void MOD_KG_send_leddis_flag_ack(void)		//-
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-����ʱ��
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 7;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = 0x14;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = UART1_led_disdata00;		//-���ݳ���
			  port_send[0][5] = UART1_led_disdata01;
			  port_send[0][6] = UART1_led_disdata02;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][7] =LOBYTE(the_ram_ax);
			  port_send[0][8] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;

	  }

//-				test_cn[6]++;		//-������

}

void MOD_KG_SET_voice_flag_ack(void)		//-��������ʱ��
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-����ʱ��
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = 0xaa;		//-���ݳ���
			  port_send[0][5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }



}

void MOD_KG_polling_voice_flag_ack(void)		//-��������ʱ��
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-����ʱ��
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = voice_flag;		//-���ݳ���
			  port_send[0][5] = 0;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }



}

void MOD_KG_re_gateway_flag_ack(void)		//-��������ʱ��
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-����ʱ��
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = 0xaa;		//-���ݳ���
			  port_send[0][5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }



}

void MOD_KG_send_voice_mode_ack(void)		//-��������ʱ��
{
	  WORD the_ram_ax;
	  //-static BYTE i;		//?ͬһ���������治��ʹ�ü���һ���ľֲ���������

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-����ʱ��
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = 0xaa;		//-���ݳ���
			  port_send[0][5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }
		//-TWI_Delay();
		/*
		if(test_cn_wait_time < cticks_ms)
    	test_cn[i] = cticks_ms - test_cn_wait_time;
    else
    	test_cn[i] = 65536 + cticks_ms - test_cn_wait_time;

    if(test_cn[i] > 400)
      test_cn[7]++;

    if(i<6)
	  {
    	i++;
    }
    else
    	i = 0;
		*/
}

void MOD_KG_set_hl_set_ack(void)		//-��������ʱ��
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-����ʱ��
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = HL_ld_brightness;		//-���ݳ���
			  port_send[0][5] = 0x00;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }



}

void MOD_KG_polling_PM25_ack(void)		//-������֯Ӧ�����ݽ�����Ϊ�˿��ƴ��ڷ���,����7620������
{
	  WORD the_ram_ax,i;
	  BYTE	temp_report[32];

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-����ʱ��
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-������ݳɹ����յ�֮���Ӧ��һ��
			  temp_report[0] = 0xaa;		//-���ֽڰ�ͷ
			  temp_report[1] = 0x55;

			  temp_report[2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  temp_report[3] = 0x8F;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  temp_report[4] = 0xaa;		//-���ݳ���
			  temp_report[5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&temp_report[0],temp_report[2]);
			  temp_report[6] =LOBYTE(the_ram_ax);
			  temp_report[7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  for(i=0;i<(temp_report[2] + 2);i++)
			  {
			  	port_send[1][port_send_pt[1]] = temp_report[i];
			  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
			  }

	  }



}



void MOD_KG_receive_set_deal(void)		//-�û����õƵ���ɫ,Ŀǰ֧������������ɫ
{
	  WORD i;


	  for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }

    HRL_RUN_flag = 0;

}

void MOD_KG_mode_set_deal(void)		//-�Ų�ģʽ����
{
	  WORD i;


	  for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;

      //-if(port_report[4] == 2)   //-������
      //-  port_report[4] = 2;
      //-if(port_report[4] == 3)   //-������
      //-  port_report[4] = 3;
      //-if(port_report[4] == 4)   //-������
      //-  port_report[4] = 4;
      //-if(port_report[4] == 5)   //-������
      //-  port_report[4] = 5;

	  }

    if(port_report[4] != 0)
        HRL_RUN_flag = 0;
      else
      {
        if(led_display_start == 0x55)
          HRL_RUN_flag = 0xaa;
      }

}

void MOD_KG_chage_page_deal(void)		//-��ҳ
{
	  WORD i;


	  for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }

}

void MOD_KG_SET_screen_deal(void)		//-��������ʱ��
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }

}

void MOD_KG_chage_pic_deal(void)		//-��������ʱ��
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }

}

void MOD_KG_SET_loop_deal(void)		//-��������ʱ��
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }

}

void MOD_KG_send_state_deal(void)		//-��������ʱ��
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }

}

void MOD_KG_re_Weather_deal(void)		//-��������ʱ��
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }

}

void MOD_KG_re_show_deal(void)		//-��������ʱ��
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }
    //-test_cn[0]++;    //-���Լ�����
    //-test_cn_wait_time = cticks_ms;
	  //-test_cn[4] = 1;

    led_display_start = 0x55;
    if(HL_flag == 0)
      HRL_RUN_flag = 0xaa;

    RunLed_stata_num = port_report[4];

    RunLed_stata_judge_new(RunLed_stata_num,&port_report[5]);
}

void MOD_KG_polling_state_deal(void)		//-��������ʱ��
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }

}

void MOD_KG_polling_PM25_deal(void)		//-��������ʱ��
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }

}

void MOD_KG_end_PM25_deal(void)		//-��������ʱ��
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }

}

void MOD_KG_polling_hl_deal(void)
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }
}

void MOD_KG_sys_state_deal(void)
{
	   WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-����ʱ��
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = 0x92;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = STM32_UP_selfT_flag1;		//-���ݳ���
			  port_send[0][5] = 0x00;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }
}

void MOD_KG_SET_hl_time_deal(void)
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }
}

void MOD_KG_send_leddis_flag_deal(void)
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }
}

void MOD_KG_SET_voice_flag_deal(void)		//-�ѽ��յ������ݽ���͸������
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }
}

void MOD_KG_polling_voice_flag_deal(void)		//-�ѽ��յ������ݽ���͸������
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }
}

void MOD_KG_re_gateway_flag_deal(void)		//-�ѽ��յ������ݽ���͸������
{
	  WORD i;


    HRL_RUN_flag = 0;		//-�����������Ϩ�������
    UART0_transmit_flag = 0;
    HRL_RUN_ONOFF = port_report[4];

    MOD_KG_transmit_control = 1;	//-�����������,��ʼ�������ĵĴ���
    MOD_KG_transmit_wait_time=cticks_ms;  //-һ�����Ƿ�������������ʱ��������Ӧ��
    //-������ݳɹ����յ�֮���Ӧ��һ��
			  /*port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = 0x97;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = 0xaa;		//-���ݳ���
			  port_send[0][5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;*/

			  for(i=0;i<(port_report[2] + 2);i++)		//-��������̻���Ҫת������ư�
			  {
			  	port_send[1][port_send_pt[1]] = port_report[i];
			  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
			  }
}

void MOD_KG_re_ZigBee_flag_deal(void)		//-�ѽ��յ������ݽ���͸������
{
	  WORD the_ram_ax;


    if(port_report[4] == 1)
    {//-zigbee�������
    	 zigbee_flag = 1;
    	 zigbee_wait_time = Time_2048ms_Counter;		//-�����ʱ��ʼ
    }
    else if(port_report[4] == 0)
    {//-zigbee��ֹ����
    	 zigbee_flag = 0;
    }

    //-HRL_RUN_flag = 0;		//-

    //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = port_report[3] | 0x80;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = port_report[4];		//-���ݳ���
			  port_send[0][5] = 0;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
}


void MOD_KG_send_HRL_mode_deal(void)		//-�ѽ��յ������ݽ���͸������
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }
}

void MOD_KG_send_voice_mode_deal(void)		//-�ѽ��յ������ݽ���͸������
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }
}

void MOD_KG_send_es705_mode_deal(void)		//-�ѽ��յ������ݽ���͸������
{
	  WORD i;

		/*
	  if(es705_mode == 0)
    {
      if(port_report[4] == 1)
      {//-zigbee�������
         es705_mode_file = port_report[5];
         f_rec = 1;
         es705_mode = 1;
         es705_training_count = 0;
      }
      else if(port_report[4] == 2)
      {//-zigbee��ֹ����
         f_rec = 1;
         es705_mode = 2;
         es705_training_count = 0;
      }
      else
      {
         return;
      }


    //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = port_report[3] | 0x80;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = port_report[4];		//-���ݳ���
			  port_send[0][5] = port_report[5];

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
    }
    else if(es705_mode == 1)
    {
        if((port_report[4] == 0) && (port_report[5] == 0))
        {
            es705_mode_file = 0;
            es705_mode = 0;
            es705_training_count = 0;

            port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
            port_send[0][1] = 0x55;

            port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

            port_send[0][3] = port_report[3] | 0x80;		//-������:01h �����������ϱ�

            //-��Ч����
            //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
            port_send[0][4] = port_report[4];		//-���ݳ���
            port_send[0][5] = port_report[5];

            //-CRC16
            the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
            port_send[0][6] =LOBYTE(the_ram_ax);
            port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

            //-���ͳ���
            port_send_len[0] = port_send[0][2] + 2;
            port_deal_flag[0] = 0xaa;
        }
    }
    */

    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }
}

//-STM32��������������Ϣ
void MOD_KG_send_es705_deal(void)
{
	   WORD the_ram_ax,i;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-����ʱ��
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 7;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = 0x1d;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = es705_training_status;		//-���ݳ���
			  port_send[0][5] = es705_training_count;
        port_send[0][6] = es705_mode_file;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][7] =LOBYTE(the_ram_ax);
			  port_send[0][8] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }

    for(i=0;i<(port_send[0][2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_send[0][i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }
}

void MOD_KG_send_es705_event_type_deal(void)
{
	   WORD the_ram_ax;
     //-static BYTE i=0;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-����ʱ��
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = 0x1e;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = es705_event_type >> 8;		//-���ݳ���
			  port_send[0][5] = es705_event_type & 0xFF;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;

        //-if(i > 3)
        //-{
        //-   es705_TO_UART2_FLAG = 0;
        //-   i = 0;
        //-}
        //-else
        //-   i++;
	  }
}

void MOD_KG_send_es705_event_type_ack(void)
{
   es705_TO_UART2_FLAG = 0;

}

void MOD_KG_send_HRL_ONOFF_ack(void)		//-�ѽ��յ������ݽ���͸������
{
	  WORD the_ram_ax;


    HRL_RUN_ONOFF = port_report[4];


    //-HRL_RUN_flag = 0;		//-

    //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = port_report[3] | 0x80;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = HRL_RUN_ONOFF;		//-���ݳ���
			  port_send[0][5] = 0;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
}

void MOD_KG_send_inquire_state_ack(void)		//-�ѽ��յ������ݽ���͸������
{
	  //-WORD the_ram_ax;

	  HRL_RUN_flag = 0;		//-�����������Ϩ�������
    UART0_transmit_flag = 0;
    MOD_KG_transmit_control = 1;
}

void MOD_KG_set_hl_set_deal(void)		//-�ѽ��յ������ݽ���͸������
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }
}







//-���洦��ͨѶ���յ�������,��ʵ����һ���������ϲ�Э��Ĵ���,���������������ڵ���
//-������һ����Ҫ�������Ǵ�����Ϣ
void CDT9702_Main()
{
	 WORD the_ram_ax,the_ram_bx;
	 WORD temp_data,temp_data1,delta_len,temp_loop,temp_int;


	 //-�жϷ����Ƿ������,��������˿��������߼�����,���򲻴���
	 if(DMA_GetFlagStatus(DMA1_FLAG_TC7))		//-����˵�����������
   {//-ֻ�е���������˲ſ������¸�DMA���ʹ�������
   	  for(temp_loop = 0;temp_loop < 43;temp_loop++)
   	  	port_send[0][temp_loop] = 0;
   	  port_deal_flag[0] = 0;		//-��һ����0ֵ�Ϳ������´μ�鷢��ʱ������������
   	  //-MOD_KG_transmit_flag=YES;
   	  DMA_ClearFlag(DMA1_FLAG_TC7);
   }

	 if(port_deal_flag[0] == 0)	//-����᲻������һ�η���ʧ�ܶ�������Ҳ���ܷ���
   {
   	   //-���ȴ�����յ�������,���û�����ݽ��յ�,����֯������Ҫ���͵�����
   	   if((port_recv_pt[0]!=port_recv_dl[0]) && (MOD_KG_transmit_flag == YES))
   	   {

   	   	   if(MOD_KG_rxd_head_flag==NO)	//-���յ������ݻ�û�д����ʱ�����NO
           {
           	   MOD_KG_rxd_wait_time=cticks_ms;		//-��ӵ��д�����
		   	   	   temp_data = port_recv_pt[0];
		   	   	   temp_data1 = port_recv_dl[0];
		           if(temp_data1>temp_data)	//-ǰ����Ǵ���ָ���ʵ�ʵĽ���ָ����бȽ�
		               delta_len=(temp_data+512)-temp_data1;
		           else
		               delta_len=temp_data-temp_data1;	//-һ���ĳ���
		           for(temp_loop=temp_data1;temp_loop<(delta_len+temp_data1);temp_loop++)
		           {
		        	   if(port_recv[0][port_recv_dl[0]]==0xaa)	//-����ط��Ƚϵ��Ǵ�վ��ַ,�����Ҿ���û���κι��ɾ���ͨѶ
		        	   {	//-����һ�п������õ�
		        	     the_ram_ax=(port_recv_dl[0]+1)&0x1ff;
		        	     if(temp_data == the_ram_ax)
		        	     	 break;
		        	     if(port_recv[0][the_ram_ax]==0x55)	//-�Ƚϵ��ǹ�����
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
		               if(delta_len>=(unsigned short)(port_recv[0][temp_int]+2))	//-�õ��ı��ĳ��Ⱥ������ϵı��ĳ��Ƚ��бȽ�
		               {
		                  MOD_KG_rxd_head_flag=NO;
		                  MOD_KG_rec_OK=YES;
                      goto rec_ok_deal;	//-�������ؿ���,���������Ϊ�ǳɹ����յ�һ�����ر�����
		               }

		           }

     			 }
   	   }
   	   goto rxd_out_time;		//?����������Ǽ򵥵�����ģʽ,����ֱ�Ӳ�ѯ����
rec_ok_deal:
	    if(MOD_KG_rec_OK==YES)	//-�϶��Ƿ�ֹ�ܷɵ�.
	    {	//-������Ϳ���˵��Ӧ�����Ѿ�������
	        MOD_KG_rec_OK=NO;	//-�ɹ����յ������ݿ�ʼ������֮��,�ͻָ�0
	        MOD_KG_transmit_flag=YES;
	        test_cn_wait_time = cticks_ms;	//-������
	        //-MOD_KG_transmit_wait_time=Time_1ms_Counter;	//-��Ȼ����˵���Է���������,���ǻ���Ҫ��ʱһ��ʱ��,��Ϊ���յ���������Ҫ����
	        //-�����Ƕ�ʵ�����ݵĴ���,������Ǻ��Ĳ���
	        the_ram_bx=(port_recv_dl[0]+3)&0x1ff;;
	        if(port_recv[0][the_ram_bx]!=0xFF)	//-����ǶԹ�������ж�,�����벻ͬ�жϵ�����Ҳ��һ��
	        {	//-����������һ�ִ������ڿ��Բ���
	          	the_ram_ax=(port_recv_dl[0]+2)&0x1ff;
	          	temp_int=port_recv[0][the_ram_ax]+2+port_recv_dl[0];
	          	for(temp_loop=port_recv_dl[0];temp_loop<temp_int;temp_loop++)	//-���������ɵ����ܾ��Ǳ�֤��λ����Ҫ����ı����ֽ�
	          	{	//-�򵥵Ĳ���Ҫ���������Ǹ��ӵĻ�����Ҫ��,��ô�������˵û������Ծͻ�ܺ�
	                 if(temp_loop<=511)
	           	       port_report[temp_loop-port_recv_dl[0]]=port_recv[0][temp_loop];
	                 else
	           	       port_report[temp_loop-port_recv_dl[0]]=port_recv[0][temp_loop-512];	//-�ѵ��Ǹ��ٸ��µ�Ե����Ҫ��ǰ���Ƴ���
	          	}	//-���߻���һ�ֿ����Ծ���ͳһ����
	         	//-port_recv_dl[0]+=delta_len;	//-����ط��������������Ĵ�����
            port_recv_dl[0]+=(port_report[2] + 2);
	         	port_recv_dl[0]&=0x1ff;
	         	temp_int=MOD_KG_CRC16(&port_report[0],port_report[2]);
	         	if((LOBYTE(temp_int)!=port_report[port_report[2]+0])||(HIBYTE(temp_int)!=port_report[port_report[2]+1]))	//-����CRC���
	         	{	//-������Ĳ�ȷ����,���У�鲻��ȷ�Ļ���ֱ�Ӱ�ͷ���,����Ѱ��
	          		goto inrda;	//-������˵���ɹ����յ��ı���CRCУ��û��ͨ��
	          }
	        }
	        else
	        {
	            //-port_recv_dl[0]+=delta_len;	//-������Ϳ��԰ѽ��ջ������е�����������,�Ѿ�����������
              port_recv_dl[0]+=7;
	            port_recv_dl[0]&=0x1ff;
	            goto inrda;
	        }




      //-��ʱ�ȸ�ֵ,��������������Դ����߼�
      //-��������ж��½��ձ���,Ȼ�������������

////////////////////////////////////////////////////////////////////////////////
				if(port_report[3] == 2)	//-�жϹ�����
	      {
	      	 if(port_report[4] == 1)
	      	 {
	      	 	   MOD_KG_rec_frame_type = MOD_KG_polling_ID_FRAME;
	      	 	   //-�򵥵Ķ���������ô����,ֱ�ӷ���
                  MOD_KG_polling_ID_ack();
                  MOD_KG_transmit_wait_time=cticks_ms;  //-һ��������ͨѶ��,��ô���������ӳ�2S
						    	MOD_KG_transmit_flag=NO;
	      	 }
	      	 else
	      	 {
	      	 	   MOD_KG_rec_frame_type = MOD_KG_polling_data_FRAME;
	      	 	   //-�򵥵Ķ���������ô����,ֱ�ӷ���
                  MOD_KG_polling_data_ack();
                  MOD_KG_transmit_wait_time=cticks_ms;
						    	MOD_KG_transmit_flag=NO;
	      	 }
	      }
	      else
	      {//-����LED������ͼƬ
	      	 if(port_report[3] == 3)
	      	 {
	      	  	MOD_KG_rec_frame_type = MOD_KG_LED_display_FRAME;
	      	  	//-����ҪӦ��
                  MOD_KG_LED_display_deal();
                  MOD_KG_transmit_wait_time=cticks_ms;
						    	//-MOD_KG_transmit_flag=NO;
	      	 }
	      	 else
	      	 {
	      	 	  if(port_report[3] == 4)
	      	 	  {
	      	 	  	MOD_KG_rec_frame_type = MOD_KG_SET_screen_FRAME;
	      	 	  	//-
                  MOD_KG_SET_screen_deal();
                  MOD_KG_transmit_wait_time=cticks_ms;
	      	 	  }
	      	 	  else
	      	 	  {
		      	 		if(port_report[3] == 5)		//-���յ��Ųʵ�����,�������õĶ���һ����ֵ��Ķ���,������Ҫ��֤���ͳɹ�
					      {
					      	  MOD_KG_rec_frame_type = MOD_KG_receive_set_FRAME;
					      	  MOD_KG_receive_set_deal();
	                  MOD_KG_transmit_wait_time=cticks_ms;
					      }
					      else
					      {
					      	if(port_report[3] == 6)		//-���յ��Ų�ģʽ����,�������õĶ���һ����ֵ��Ķ���,������Ҫ��֤���ͳɹ�
						      {
						      	  MOD_KG_rec_frame_type = MOD_KG_mode_set_FRAME;
						      	  //-7620�����Ųʵ�ģʽ,�˴����յ�֮��,����Ҫ����,����ֱ��ͨ����һ������͸����ȥ
	                  MOD_KG_mode_set_deal();
	                  MOD_KG_transmit_wait_time=cticks_ms;
						      }
						      else
						      {
						      	if(port_report[3] == 8)		//-���յ��Ų�ģʽ����,�������õĶ���һ����ֵ��Ķ���,������Ҫ��֤���ͳɹ�
							      {
							      	  MOD_KG_rec_frame_type = MOD_KG_chage_page_FRAME;
							      	  MOD_KG_chage_page_deal();
			          	      //-MOD_KG_transmit_flag=NO;
			          	      MOD_KG_transmit_wait_time=cticks_ms;
							      }
							      else
							      {
							      	if(port_report[3] == 9)		//-���յ��Ų�ģʽ����,�������õĶ���һ����ֵ��Ķ���,������Ҫ��֤���ͳɹ�
								      {
								      	  MOD_KG_rec_frame_type = MOD_KG_chage_pic_FRAME;
								      	  MOD_KG_chage_pic_deal();
          	      				MOD_KG_transmit_wait_time=cticks_ms;
								      }
								      else
								      {
								      	if(port_report[3] == 0x0A)		//-���յ��Ų�ģʽ����,�������õĶ���һ����ֵ��Ķ���,������Ҫ��֤���ͳɹ�
									      {
									      	  MOD_KG_rec_frame_type = MOD_KG_SET_loop_FRAME;
									      	  MOD_KG_SET_loop_deal();
          	     						MOD_KG_transmit_wait_time=cticks_ms;
									      }
									      else
									      {
									      	if(port_report[3] == 0x0B)		//-���յ��Ų�ģʽ����,�������õĶ���һ����ֵ��Ķ���,������Ҫ��֤���ͳɹ�
										      {
										      	  MOD_KG_rec_frame_type = MOD_KG_send_state_FRAME;
										      	  MOD_KG_send_state_deal();
          	      						MOD_KG_transmit_wait_time=cticks_ms;
										      }
										      else
										      {
										      	if(port_report[3] == 0x0C)		//-���յ��Ų�ģʽ����,�������õĶ���һ����ֵ��Ķ���,������Ҫ��֤���ͳɹ�
											      {
											      	  MOD_KG_rec_frame_type = MOD_KG_re_Weather_FRAME;
											      	  MOD_KG_re_Weather_deal();
          	      							MOD_KG_transmit_wait_time=cticks_ms;
											      }
											      else
											      {
											      	if(port_report[3] == 0x0D)		//-ָ����ʾ��һҳ����������
												      {
												      	  MOD_KG_rec_frame_type = MOD_KG_re_show_FRAME;
												      	  MOD_KG_re_show_deal();
								          	      MOD_KG_transmit_wait_time=cticks_ms;  //-��Բ��������,�������ݶ��ӳ�15S
								                  UART1_sloop_flag = 1;
								                  UART1_sloop_wait_time = cticks_ms;
												      }
												      else
												      {
												      	if(port_report[3] == 0x0E)		//-ָ����ʾ��һҳ����������
													      {
													      	  MOD_KG_rec_frame_type = MOD_KG_polling_state_FRAME;
													      	  MOD_KG_polling_state_deal();
          	      									MOD_KG_transmit_wait_time=cticks_ms;
													      }
													      else
													      {
													      	if(port_report[3] == 0x0F)		//?ָ����ʾ��һҳ����������
														      {
														      	  MOD_KG_rec_frame_type = MOD_KG_polling_PM25_FRAME;
														      	  MOD_KG_polling_PM25_deal();
          	      										MOD_KG_transmit_wait_time=cticks_ms;
														      }
														      else
														      {
														      	if(port_report[3] == 0x10)		//-ָ����ʾ��һҳ����������
															      {
															      	  MOD_KG_rec_frame_type = MOD_KG_end_PM25_FRAME;
															      	  MOD_KG_end_PM25_deal();
          	      											MOD_KG_transmit_wait_time=cticks_ms;
															      }
															      else
															      {
															      	if(port_report[3] == 0x11)		//-ָ����ʾ��һҳ����������
																      {
																      	  MOD_KG_rec_frame_type = MOD_KG_polling_hl_FRAME;
																      	  MOD_KG_polling_hl_deal();
          	      												MOD_KG_transmit_wait_time=cticks_ms;
																      }
																      else
																      {
																      	if(port_report[3] == 0x12)		//-ָ����ʾ��һҳ����������
																	      {
																	      	  MOD_KG_rec_frame_type = MOD_KG_sys_state_FRAME;
																	      	  MOD_KG_sys_state_deal();
													                  MOD_KG_transmit_wait_time=cticks_ms;
													          	      MOD_KG_transmit_flag=NO;
																	      }
																	      else
																	      {
																	      	if(port_report[3] == 0x13)		//-ָ����ʾ��һҳ����������
																		      {
																		      	  MOD_KG_rec_frame_type = MOD_KG_SET_hl_time_FRAME;
																		      	  MOD_KG_SET_hl_time_deal();
          	      														MOD_KG_transmit_wait_time=cticks_ms;
																		      }
																		      else
																		      {
																		      	if(port_report[3] == 0x94)		//-7620����Ӧ��
																			      {
																			      	  MOD_KG_rec_frame_type = MOD_KG_send_leddis_flag_FRAME;
																			      	  MOD_KG_send_leddis_flag_deal();
          	      															MOD_KG_transmit_wait_time=cticks_ms;
																			      }
																			      else
																			      {
																			      	if(port_report[3] == 0x15)		//?ָ����ʾ��һҳ����������
																				      {
																				      	  MOD_KG_rec_frame_type = MOD_KG_SET_voice_flag_FRAME;
																				      	  MOD_KG_SET_voice_flag_deal();
          	      																MOD_KG_transmit_wait_time=cticks_ms;
																				      }
																				      else
																				      {
																				      	if(port_report[3] == 0x16)		//-ָ����ʾ��һҳ����������
																					      {
																					      	  MOD_KG_rec_frame_type = MOD_KG_polling_voice_flag_FRAME;
																					      	  MOD_KG_polling_voice_flag_deal();
          	      																	MOD_KG_transmit_wait_time=cticks_ms;
																					      }
																					      else
																					      {
																					      	if(port_report[3] == 0x17)		//-�������ص���������
																						      {
																						      	  MOD_KG_rec_frame_type = MOD_KG_re_gateway_flag_FRAME;
																						      	  MOD_KG_re_gateway_flag_deal();
          	      																		MOD_KG_transmit_wait_time=cticks_ms;
																						      }
																						      else
																						      {
																						      	if(port_report[3] == 0x19)		//-�������ص���������
																							      {
																							      	  MOD_KG_rec_frame_type = MOD_KG_re_ZigBee_flag_FRAME;
																							      	  MOD_KG_re_ZigBee_flag_deal();
																			                  MOD_KG_transmit_wait_time=cticks_ms;
																			          	      MOD_KG_transmit_flag=NO;
																							      }
																							      else
																							      {
																							      	if(port_report[3] == 0x9A)		//-�������ص���������
																								      {
																								      	  MOD_KG_rec_frame_type = MOD_KG_send_HRL_mode_FRAME;
																								      	  MOD_KG_send_HRL_mode_deal();
          	      																				MOD_KG_transmit_wait_time=cticks_ms;
																								      }
																								      else
																								      {
																								      	if(port_report[3] == 0x1B)		//-�������ص���������
																									      {
																									      	  MOD_KG_rec_frame_type = MOD_KG_send_voice_mode_FRAME;
																									      	  MOD_KG_send_voice_mode_deal();
          	     																						MOD_KG_transmit_wait_time=cticks_ms;
																									      }
																									      else
																									      {
																									      	if(port_report[3] == 0x1C)		//-�������ص���������
																										      {
																										      	  MOD_KG_rec_frame_type = MOD_KG_send_es705_mode_FRAME;
																										      	  MOD_KG_send_es705_mode_deal();
																						                  MOD_KG_transmit_wait_time=cticks_ms;
																						          	      MOD_KG_transmit_flag=NO;
																										      }
																										      else
																										      {
																										      	if(port_report[3] == 0x9e)		//-�������ص���������
																											      {
																											      	  MOD_KG_rec_frame_type = MOD_KG_send_es705_event_type;
																											      	  MOD_KG_send_es705_event_type_ack();
          	      																							MOD_KG_transmit_wait_time=cticks_ms;
																											      }
																											      else
																											      {
																											      	if(port_report[3] == 0x1F)		//-�������ص���������
																												      {
																												      	  MOD_KG_rec_frame_type = MOD_KG_send_HRL_ONOFF_FRAME;
																												      	  MOD_KG_send_HRL_ONOFF_ack();
																								                  MOD_KG_transmit_wait_time=cticks_ms;
																								          	      MOD_KG_transmit_flag=NO;
																												      }
																												      else
																												      {
																												      	if(port_report[3] == 0xA0)		//-�������ص���������
																													      {
																													      	  MOD_KG_rec_frame_type = MOD_KG_send_inquire_state_FRAME;
																													      	  MOD_KG_send_inquire_state_ack();
          	      																									MOD_KG_transmit_wait_time=cticks_ms;
																													      }
																													      else
																													      {
																													      		if(port_report[3] == 0x21)		//-�������ص���������
																															      {
																															      	  MOD_KG_rec_frame_type = MOD_KG_set_hl_set_FRAME;
																															      	  MOD_KG_set_hl_set_deal();
          	      																											MOD_KG_transmit_wait_time=cticks_ms;
																															      }
																															      else
																															      {
																															      	MOD_KG_rec_frame_type = 255;
																															      }
																													      }
																												      }
																											      }
																										      }
																									      }
																								      }
																							      }
																						      }
																					      }
																				      }
																			      }
																		      }
																	      }
																      }
															      }
														      }
													      }
												      }
											      }
										      }
									      }
								      }
							      }
						      }
					      }
				    	}
	      	 }
	      }

////////////////////////////////////////////////////////////////////////////////

    			MOD_KG_clear_port_report_deal();		//-ÿ�δ������֮�������������ֹ����ʹ��

    	}
rxd_out_time:	//-ִ�е�����˵�����ճ�ʱ,����ɹ�,�������ڿ��Լ���������,,�����������ִ�������,����
	    if(Judge_Time_In_MainLoop(MOD_KG_rxd_wait_time,MOD_KG_WAIT_TIME_VALUE)==YES)	//-����ͨ�ŵĻ��ǲ�Ӧ�ó�ʱ��,����ʱ����Ϊ����,�������³�ʼ��
	    {	//-������һ������֮��,�����ȴ��ش�ʱ��֮��,�Ϳ��Դ��·���һ��
	      	MOD_KG_rec_OK=NO;
	      	MOD_KG_rxd_head_flag=NO;
	      	MOD_KG_rxd_wait_time=cticks_ms;
	      	//-MOD_KG_transmit_flag=YES;	//-��ʾ���ڿ�����֯��������
	      	//-MOD_KG_wait_replay=NO;	//-��ʾ���ڻ�û�еȴ��ظ�
	      	//-MOD_KG_transmit_wait_time=Time_1ms_Counter;

	       	//-MOD_KG_comm_err_counter[port_send[0][0]]++;	//-��ʱ�������


	    }
	    if(Judge_Time_In_MainLoop(MOD_KG_transmit_wait_time,80)==YES)
	    	MOD_KG_transmit_flag=YES;	//-һ�����յ���Ч����,��ô80msʱ���ſ��Դ����´ο��ܵı���
inrda:
		   //-�������� ,,�������һ������,�೤ʱ�䷢��һ��,����˵������ʹ���������ʲô
		   if(MOD_KG_transmit_flag==YES)		//-Ŀǰ����ν����˫���,���е���������֮����Ҫ��ֹ����,ֱ���ڴ��ڵĶ��������ͳ�ȥ
		   {
		   	  //-if((Judge_Time_In_MainLoop(MOD_KG_transmit_wait_time,10000)==YES) || (port_send_sense_data[0] == 0x55))	//-ֻ�й��˵ȴ�ʱ��Żᷢ��
		   	  if(Judge_Time_In_MainLoop(MOD_KG_transmit_wait_time,10)==YES)	//-һ�������ݷ��ͳ�ȥ,��Ҫ�����ն��㹻�Ĵ���ʱ��Ȼ���ٷ���
			   	  switch(MOD_KG_transmit_control)
	          {
	          	    case 0:
                       if(Judge_Time_In_MainLoop(MOD_KG_transmit_wait_time,1000)==YES)
                       {
                          MOD_KG_send_inquire_state();		//-����֮��������ѯ�����Ƿ��������
                          MOD_KG_transmit_flag=NO;
                          MOD_KG_transmit_wait_time=cticks_ms;	//-Ӧ����Ҳ����ʱЧ�Ե�
                       }
	          	    	   break;
	                case 1:
	                     //-MOD_KG_rec_frame_type=MOD_KG_YX_FRAME;	//-��ʾ��Ҫ���յ�֡�����������ֵ,���յ�������ֵ������ȷ��
	                     //-MOD_KG_wait_replay=YES;
	                     //-MOD_KG_polling_ID_cmd();	//-����Լ�����������,�ʴ��͵�,����˿��ϵ�����װ�ö��ǽ��ղ�ѯ
	                     if((Judge_Time_In_MainLoop(MOD_KG_transmit_wait_time,1900)==YES) && (UART1_sloop_flag == 0))	//-�������͵����ȼ����,ֻ�п���ʱ��������,һ�����κ����ݶ��˳�����
	                     {
	                     		MOD_KG_send_sense_data();  //-��������
	                     		Device_communication_cn++;
	                     		if(Device_communication_cn > 60)
	                     		{//-����30�ζ�û��Ӧ����Ϊ������ͨѶ��ֹ��,��Ҫ����Щ�������
	                     			HRL_RUN_flag = 0;
	                     		}
	                     		//-port_send_sense_data[0] = 0;
	                     		//-MOD_KG_now_poll_addr++;	//-�ӻ���ַ

	                     		MOD_KG_transmit_flag=NO;	//-���ڿ�ʼ���ܷ��ͱ�����
	                     		MOD_KG_rxd_head_flag=NO;	//-Ϊ��ͬ��׼����,���ǲ��������еĶ�����������
	                     		MOD_KG_transmit_wait_time=cticks_ms;	//-Ӧ����Ҳ����ʱЧ�Ե�
	                   	 }
	                     MOD_KG_transmit_control=2;	//-���еĶ˿�װ�ò�ѯһ��֮��,�ٻ���һ�౨�Ĳ�ѯ

	                     break;
	                case 2:
	                     //-MOD_KG_rec_frame_type=MOD_KG_YX_FRAME;	//-��ʾ��Ҫ���յ�֡�����������ֵ,���յ�������ֵ������ȷ��
	                     //-MOD_KG_wait_replay=YES;
	                     //-MOD_KG_polling_ID_cmd();	//-����Լ�����������,�ʴ��͵�,����˿��ϵ�����װ�ö��ǽ��ղ�ѯ

                       if(UART3_TO_UART2_FLAG == 0)
                       {

                       }
                       else
                       {
                         if(UART3_TO_UART2_FLAG == 0x84)	//-��Щ��־λ�����ڴ���3����λ��,���ݱ������Լ���֯��7620����
                         {
                           MOD_KG_SET_screen_deal_ack();
                           MOD_KG_transmit_flag=NO;
                           MOD_KG_transmit_wait_time=cticks_ms;
                         }
                         else
                         {
                         		if(UART3_TO_UART2_FLAG == 0x85)	//-��Щ��־λ�����ڴ���3����λ��,���ݱ������Լ���֯��7620����
                         		{
	                            MOD_KG_receive_set_deal_ack();
	                            MOD_KG_transmit_flag=NO;
	                            MOD_KG_transmit_wait_time=cticks_ms;
                          	}
                          	else
                          	{
                          		if(UART3_TO_UART2_FLAG == 0x86)	//-��Щ��־λ�����ڴ���3����λ��,���ݱ������Լ���֯��7620����
		                          {
		                            MOD_KG_mode_set_deal_ack();
		                            MOD_KG_transmit_flag=NO;
		                            MOD_KG_transmit_wait_time=cticks_ms;
		                          }
		                          else
		                          {
		                          	 if(UART3_TO_UART2_FLAG == 0x89)	//-��Щ��־λ�����ڴ���3����λ��,���ݱ������Լ���֯��7620����
				                         {
				                            MOD_KG_chage_pic_deal_ack();
				                            MOD_KG_transmit_flag=NO;
				                            MOD_KG_transmit_wait_time=cticks_ms;
				                         }
				                         else
				                         {
				                         	 if(UART3_TO_UART2_FLAG == 0x8A)	//-��Щ��־λ�����ڴ���3����λ��,���ݱ������Լ���֯��7620����
					                         {
					                            MOD_KG_SET_loop_deal_ack();
					                            MOD_KG_transmit_flag=NO;
					                            MOD_KG_transmit_wait_time=cticks_ms;
					                         }
					                         else
					                         {
					                         	 if(UART3_TO_UART2_FLAG == 0x8B)	//-��Щ��־λ�����ڴ���3����λ��,���ݱ������Լ���֯��7620����
						                         {
						                            MOD_KG_send_state_deal_ack();
						                            MOD_KG_transmit_flag=NO;
						                            MOD_KG_transmit_wait_time=cticks_ms;
						                         }
						                         else
						                         {
						                         	 if(UART3_TO_UART2_FLAG == 0x8C)	//-��Щ��־λ�����ڴ���3����λ��,���ݱ������Լ���֯��7620����
							                         {
							                            MOD_KG_re_Weather_deal_ack();
							                            MOD_KG_transmit_flag=NO;
							                            MOD_KG_transmit_wait_time=cticks_ms;
							                         }
							                         else
							                         {
							                         	 if(UART3_TO_UART2_FLAG == 0x8D)	//-��Щ��־λ�����ڴ���3����λ��,���ݱ������Լ���֯��7620����
								                         {
								                            MOD_KG_re_show_deal_ack();
								                            MOD_KG_transmit_flag=NO;
								                            MOD_KG_transmit_wait_time=cticks_ms;
								                         }
								                         else
								                         {
								                         	 if(UART3_TO_UART2_FLAG == 0x8E)	//-��Щ��־λ�����ڴ���3����λ��,���ݱ������Լ���֯��7620����
									                         {
									                            MOD_KG_polling_state_deal_ack();
									                            MOD_KG_transmit_flag=NO;
									                            MOD_KG_transmit_wait_time=cticks_ms;
									                         }
									                         else
									                         {
									                         	 if(UART3_TO_UART2_FLAG == 0x91)	//-��Щ��־λ�����ڴ���3����λ��,���ݱ������Լ���֯��7620����
										                         {
										                            MOD_KG_polling_hl_deal_ack();
										                            MOD_KG_transmit_flag=NO;
										                            MOD_KG_transmit_wait_time=cticks_ms;
										                         }
										                         else
										                         {
										                         	 if(UART3_TO_UART2_FLAG == 0x93)	//-��Щ��־λ�����ڴ���3����λ��,���ݱ������Լ���֯��7620����
											                         {
											                            MOD_KG_SET_hl_time_deal_ack();
											                            MOD_KG_transmit_flag=NO;
											                            MOD_KG_transmit_wait_time=cticks_ms;
											                         }
											                         else
											                         {
											                         	 if(UART3_TO_UART2_FLAG == 0xC0)	//-��Щ��־λ�����ڴ���3����λ��,���ݱ������Լ���֯��7620����
												                         {
												                            MOD_KG_control_HRL_deal_ack();
												                            MOD_KG_transmit_flag=NO;
												                            MOD_KG_transmit_wait_time=cticks_ms;
												                         }
												                         else
												                         {
												                         	 if(UART3_TO_UART2_FLAG == 0x94)	//-��Щ��־λ�����ڴ���3����λ��,���ݱ������Լ���֯��7620����
													                         {
													                            MOD_KG_send_leddis_flag_ack();
													                            MOD_KG_transmit_flag=NO;
													                            MOD_KG_transmit_wait_time=cticks_ms;
													                         }
													                         else
													                         {
													                         	 if(UART3_TO_UART2_FLAG == 0x95)	//-��Щ��־λ�����ڴ���3����λ��,���ݱ������Լ���֯��7620����
														                         {
														                            MOD_KG_SET_voice_flag_ack();
														                            MOD_KG_transmit_flag=NO;
														                            MOD_KG_transmit_wait_time=cticks_ms;
														                         }
														                         else
														                         {
														                         	 if(UART3_TO_UART2_FLAG == 0x96)	//-��Щ��־λ�����ڴ���3����λ��,���ݱ������Լ���֯��7620����
															                         {
															                            MOD_KG_polling_voice_flag_ack();
															                            MOD_KG_transmit_flag=NO;
															                            MOD_KG_transmit_wait_time=cticks_ms;
															                         }
															                         else
															                         {
															                         	 if(UART3_TO_UART2_FLAG == 0xC1)	//-��Щ��־λ�����ڴ���3����λ��,���ݱ������Լ���֯��7620����
																                         {
																                            MOD_KG_leddis_end_ack();
																                            MOD_KG_transmit_flag=NO;
																                            MOD_KG_transmit_wait_time=cticks_ms;
																                         }
																                         else
																                         {
																                         	 if(UART3_TO_UART2_FLAG == 0xC2)	//-��Щ��־λ�����ڴ���3����λ��,���ݱ������Լ���֯��7620����
																	                         {
																	                            MOD_KG_send_HRL_mode_ack();
																	                            MOD_KG_transmit_flag=NO;
																	                            MOD_KG_transmit_wait_time=cticks_ms;
																	                         }
																	                         else
																	                         {
																	                         	 if(UART3_TO_UART2_FLAG == 0x97)	//-��Щ��־λ�����ڴ���3����λ��,���ݱ������Լ���֯��7620����
																		                         {
																		                            MOD_KG_re_gateway_flag_ack();
																		                            MOD_KG_transmit_flag=NO;
																		                            MOD_KG_transmit_wait_time=cticks_ms;
																		                         }
																		                         else
																		                         {
																		                         	 if(UART3_TO_UART2_FLAG == 0x9B)	//-��Щ��־λ�����ڴ���3����λ��,���ݱ������Լ���֯��7620����
																			                         {
																			                            MOD_KG_send_voice_mode_ack();
																			                            MOD_KG_transmit_flag=NO;
																			                            MOD_KG_transmit_wait_time=cticks_ms;
																			                         }
																			                         else
																			                         {
																			                         	 if(UART3_TO_UART2_FLAG == 0xA1)	//-��Щ��־λ�����ڴ���3����λ��,���ݱ������Լ���֯��7620����
																				                         {
																				                            MOD_KG_set_hl_set_ack();
																				                            MOD_KG_transmit_flag=NO;
																				                            MOD_KG_transmit_wait_time=cticks_ms;
																				                         }
																			                         }
																		                         }
																	                         }
																                         }
															                         }
														                         }
													                         }
												                         }
											                         }
										                         }
									                         }
								                         }
							                         }
						                         }
					                         }
				                         }
		                          }

                          	}
                         }
                       }

	                     UART3_TO_UART2_FLAG = 0;
	                     //-MOD_KG_transmit_flag=NO;	//-���ڿ�ʼ���ܷ��ͱ�����
	                     MOD_KG_rxd_head_flag=NO;	//-Ϊ��ͬ��׼����,���ǲ��������еĶ�����������
	                     //-MOD_KG_transmit_wait_time=cticks_ms;	//-Ӧ����Ҳ����ʱЧ�Ե�

	                     MOD_KG_transmit_control=1;	//-���еĶ˿�װ�ò�ѯһ��֮��,�ٻ���һ�౨�Ĳ�ѯ

	                     break;
                  case 3:

	                     if(es705_TO_UART2_FLAG == 1)	//-��Щ��־λ�����ڴ���3����λ��,���ݱ������Լ���֯��7620����
                       {
	                     		MOD_KG_send_es705_deal();
                          if(es705_training_count == 4)
                             es705_training_count = 0;   //-���ͽ����˲ſ�������
                          es705_TO_UART2_FLAG = 0;
                       }
                       else if((es705_TO_UART2_FLAG & 0x7f) == 2)
                       {
                          if(es705_TO_UART2_FLAG == 0x02)
                          {
                             MOD_KG_send_es705_event_type_deal();  //-����ط��ȴ�Ӧ����Ҫ����һ��ʱ���
                             MOD_KG_es705_wait_time=cticks_ms;
                             es705_TO_UART2_FLAG = 0x82;
                          }
                          else
                          {
                            if(Judge_Time_In_MainLoop(MOD_KG_es705_wait_time,1000)==YES)
                            {
                              MOD_KG_send_es705_event_type_deal();
                              MOD_KG_es705_wait_time=cticks_ms;
                            }
                          }
                       }
                       //-MOD_KG_transmit_flag=NO;
                       //-es705_TO_UART2_FLAG = 0;
                       //-MOD_KG_rxd_head_flag=NO;    //-�о����ﲻ��Ҫ���,��Ϊ��˫����,������ܻ��ж�ʧ
                       MOD_KG_transmit_control=1;	//-���еĶ˿�װ�ò�ѯһ��֮��,�ٻ���һ�౨�Ĳ�ѯ
                       break;
	                default:
	                     break;
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


