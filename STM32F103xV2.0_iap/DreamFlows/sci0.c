/*
����:ʹ��һ�������߼�,����һ��512�ֽڵĻ�����,Ȼ��ѭ���洢
����:Ҳʹ��512�ֽڵĿռ䱣������,Ȼ����һ�����ͱ�־,ֱ�����ݷ��ͳ�ȥΪֹ

����:�������ϵ�STM32ͨѶ.
�Ƿ���ò�ѯ�ķ�ʽ��������?�Ҿ�����Ϊһ���ն���ʾ�豸,û�б�Ҫȥ�صز�ѯ
�ܶ�����,���������ˢ�µĻ�,�;���ˢ�¶������ò�ѯ��ʽ.��Ҫ����Ϣ�Ų�ѯ.
*/
#include "user_conf.h"


extern void led_disp_L_init(void);
extern int ADXL345_init_re(u8 data);
extern void ADXL345_init(void);


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
#define   MOD_KG_SET_hl_time_FRAME		  	    		 0x13
#define   MOD_KG_send_leddis_flag_FRAME		     		 0x14
#define   MOD_KG_SET_voice_flag_FRAME		     		   0x15
#define   MOD_KG_polling_voice_flag_FRAME		   	   0x16
#define   MOD_KG_re_gateway_flag_FRAME		    	   0x17

#define   MOD_KG_send_voice_mode_FRAME	      	   0x1B
#define   MOD_KG_send_es705_mode_FRAME	      	   0x1C
#define   MOD_KG_send_es705_status_FRAME	     	   0x1D

#define   MOD_KG_set_hl_set_FRAME	     	           0x21

#define   MOD_KG_7620_inquire_upSDB_FRAME          0x22
#define   MOD_KG_7620_inquire_update_FRAME	       0x23
#define   MOD_KG_7620_inquire_version_FRAME	       0x24



#define   MOD_KG_control_HRL_FRAME		  	    	   0x40
#define   MOD_KG_leddis_end_FRAME	  	  	    	   0x41
#define   MOD_KG_send_HRL_mode_FRAME	  	     	   0x42


#define   MOD_KG_send_max	  	     	   5		//-����һ�������ظ����͵Ĵ���

//-ΪЭ�鴦����ʱ����ı���
BYTE  MOD_KG_rxd_head_flag;
BYTE  MOD_KG_rec_OK;
BYTE  MOD_KG_rec_frame_type;
WORD  MOD_KG_rxd_wait_time;
BYTE  MOD_KG_transmit_flag;
BYTE  MOD_KG_transmit_control;
WORD  MOD_KG_transmit_wait_time;



void UART1_Init(void)
{
  port_recv_pt[0] = 0;
  port_send_pt[0] = 0;
  port_recv_dl[0] = 0;

  //-Э���ʼ������
  port_deal_flag = 0;
  MOD_KG_transmit_control = 1;

  UART1_transmit_flag = 0;
  UART1_transmit_control = 0;

  MOD_KG_rxd_head_flag=NO;
}


void UART_Rx_Deal(void)
{
	/*
  if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET) //-1: Received data is ready to be read.
  {
    port_recv[0][port_recv_pt[0]] = (USART_ReceiveData(USART1) & 0xFF);
    if(port_recv[0][port_recv_pt[0]] == 0xaa)
      Temp_Flag = 0xaa;

    port_recv_pt[0] = (port_recv_pt[0] + 1)&0x1ff;
  }
  */
  port_recv_pt[0] = 512 - DMA_GetCurrDataCounter(DMA1_Channel5);	//-�õ����ո���,�����Ϊ����ƫ�����Ļ�,���ڵ���ֵ����ָ�����ռ��
  if(port_recv_pt[0] >= 512)
  	port_recv_pt[0] = 0;
}

void UART_Tx_Deal(void)
{
	/*
  if(USART_GetFlagStatus(USART1, USART_FLAG_TC) == SET && Temp_Flag == 0xaa)
  {
    //-USART_SendData(USART1, port_send[0][port_send_pt[0]++]);
    USART_SendData(USART1, port_recv[0][(port_recv_pt[0] - 2)&0x1ff]);
    //-USART_SendData(USART1, 0xa8);
    Temp_Flag = 0;
  }
  */
  /* Enable USARTy DMA TX request */
  //-USART_DMACmd(USARTy, USART_DMAReq_Tx, ENABLE);			//-����DMA����

  if(port_send_len[0] != 0)	//-һ��Ҫ��֤���ݷ��ͳ�ȥ��,��ʹ��DMA��������
  {
		  DMA_Cmd(DMA1_Channel4, DISABLE);
		  DMA1_Channel4->CNDTR = port_send_len[0]; //-���������Ĵ���,ָʾʣ��Ĵ������ֽ���Ŀ
		  DMA_Cmd(DMA1_Channel4, ENABLE);
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
	  port_deal_flag = 0xaa;

	  cticks_500ms = 0;
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
	  port_send[0][5] = 0x01;
	  port_send[0][6] = 0x02;
	  port_send[0][7] = 0x03;
	  port_send[0][8] = 0x04;
	  port_send[0][9] = 0x05;
	  port_send[0][10] = 0x06;

	  //-CRC16
	  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
	  port_send[0][11] =LOBYTE(the_ram_ax);
	  port_send[0][12] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

	  //-���ͳ���
	  port_send_len[0] = port_send[0][2] + 2;
	  port_deal_flag = 0xaa;
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
	  port_send[0][8] = LOBYTE(port_send_sense_data[1]);		//-����λ
	  port_send[0][9] = HIBYTE(port_send_sense_data[1]);

	  port_send[0][10] = 0x0A;		//-���ݳ���		,,�¶�
	  port_send[0][11] = 0xD1;		//-�豸ID
	  port_send[0][12] = 0x04;		//-������
	  port_send[0][13] = 0x02;		//-��������
	  port_send[0][14] = LOBYTE(port_send_sense_data[2]);		//-����λ
	  port_send[0][15] = HIBYTE(port_send_sense_data[2]);

	  port_send[0][16] = 0x0A;		//-���ݳ���		,,ʪ��
	  port_send[0][17] = 0xD2;		//-�豸ID
	  port_send[0][18] = 0x04;		//-������
	  port_send[0][19] = 0x02;		//-��������
	  port_send[0][20] = LOBYTE(port_send_sense_data[3]);		//-����λ
	  port_send[0][21] = HIBYTE(port_send_sense_data[3]);

	  port_send[0][22] = 0x0A;		//-���ݳ���		,,PM2.5
	  port_send[0][23] = 0x44;		//-�豸ID
	  port_send[0][24] = 0x04;		//-������
	  port_send[0][25] = 0x01;		//-��������
	  port_send[0][26] = LOBYTE(port_send_sense_data[4]);		//-����λ
	  port_send[0][27] = HIBYTE(port_send_sense_data[4]);

	  port_send[0][28] = 0x0A;		//-���ݳ���		,,����
	  port_send[0][29] = 0xD4;		//-�豸ID
	  port_send[0][30] = 0x04;		//-������
	  port_send[0][31] = 0x01;		//-��������
	  port_send[0][32] = LOBYTE(port_send_sense_data[5]);		//-����λ
	  port_send[0][33] = HIBYTE(port_send_sense_data[5]);

	  port_send[0][34] = 0x01;		//-���ݳ���		,,Ԥ��
	  port_send[0][35] = 0x02;		//-�豸ID
	  port_send[0][36] = 0x03;		//-������
	  port_send[0][37] = 0x04;		//-��������
	  port_send[0][38] = LOBYTE(port_send_sense_data[6]);		//-����λ
	  port_send[0][39] = HIBYTE(port_send_sense_data[6]);

	  //-CRC16
	  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
	  port_send[0][40] =LOBYTE(the_ram_ax);
	  port_send[0][41] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

	  //-���ͳ���
	  port_send_len[0] = port_send[0][2] + 2;
	  port_deal_flag = 0xaa;
}
/*
����ͨѶͼƬ���Ƚ�����ʾ�Ľṹ,Ȼ��ϵͳ�д���������,���л���ʾ,�������ƻ���
0xaa 0x55 0x03 ���� CRC
*/
void MOD_KG_LED_display_deal(void)		//-������STM32�´�����ݽ��н���,�ڱ�ϵͳ��ͨѶͼƬ֮������л�?
{
    u8 i;
	  //-WORD the_ram_ax;
    UINT32	*led_display_data_pt;

	  if(led_display_txpic_flag != 0x55)
	  {
			  if(led_display_num == 5)		//-������������Ϊ����,��������ʾ�Ŀ�����֯����ʾ����
				{//-led_display_data2[16]
					 led_display_data_pt = &led_display_data6[0];

			  }
			  else
			  {//-led_display_data1[16]
					 led_display_data_pt = &led_display_data5[0];

			  }

			  for(i = 0;i < 16;i++)	//-�������һ��ͼƬ
				{
					 led_display_data_pt[i] = (port_report[i*3+3] << 16) + (port_report[i*3+4] << 8) + port_report[i*3+5];
				}

				led_display_txpic_flag = 0x55;
		}
	  //-�ر���һ���������洢����,�ñ�־λ,Ȼ��ˢ�µ�ʱ��������������Ƿ���Ч
	  //-����еĻ����л���ʾ

}

void MOD_KG_receive_data_deal(void)		//-������������STM32�Ĵ���������
{
	  WORD temp_data,the_ram_ax;

	  //-2016/2/3 9:25:22	���ӱ���Ӧ������,�����������ư�״̬
				port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = 0x81;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = 0;		//-���ݳ���
			  port_send[0][5] = 0;		//-���ݳ���

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

	  if(UART1_renew_flag != 0x55)
		{//-����˵�����Ը������ݿ�����
		  if(port_report[5] == 0xA0)		//-�豸ID��:CO2ֵ
		  {
		  	 temp_data = port_report[9] + (port_report[8] << 8);
		  	 if(temp_data > 0)
		  	 	co2_data = temp_data;
		  }

	  if(port_report[11] == 0xD1)		//-�豸ID��:�¶�
	  {
	  	  temp_data = port_report[15] + (port_report[14] << 8);
	  	  //-if(temp_data > 0)
	  	  	temperature_data = temp_data;
	  }

	  //-if(port_report[17] == 0xD2)		//-�豸ID��:ʪ��
	  //-{
	  //-	 humidity_data = port_report[20] + (port_report[21] << 8);
	  //-}

	  if(port_report[23] == 0xD5)		//-�豸ID��:PM2.5
	  {
	  	 temp_data = port_report[27] + (port_report[26] << 8);
	  	 if(temp_data > 0)
	  	 		pm_data =	temp_data;
	  }

	  if(port_report[29] == 0xD4)		//-�豸ID��:����
	  {
	  	 temp_data = port_report[33] + (port_report[32] << 8);
	  	 if(temp_data > 0)
	  	 		Noise_Value = temp_data;
	  }

	  if(port_report[17] == 0xD6)		//-�豸ID��:VOC
	  {
	  	 temp_data = port_report[21] + (port_report[20] << 8);
	  	 if(temp_data > 0)
	  	 		VOC_data = temp_data;
	  }

	  if(led_display_new != 0xa5)
	  	 led_display_new = 0x55;		//-ˢ�����е���ʾ����,���ǹ��ı䶨ֵ����,��Ҫ��ʾ����

	  //-�ر���һ���������洢����,�ñ�־λ,Ȼ��ˢ�µ�ʱ��������������Ƿ���Ч
	  //-����еĻ����л���ʾ

	    //-һ�����յ��������ľ���Ϊϵͳ�Ѿ�����������,��ô������������,�ָ�������ʾ
	    led_display_flag = 3;
		}
}

void MOD_KG_SET_screen_deal(void)		//-��������ʱ��
{
	  WORD the_ram_ax;

	  if(port_report[4] == 0x01)		//-
	  {
	  	 led_display_long = port_report[5];	//-����ʱ��
	  	 EEP_Data_flag = 0x55;		//-��ֵ���޸���
	  	 cticks_s_page = 0;
       led_display_start = 0x55;

       //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

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
			  port_deal_flag = 0xaa;
	  }



}

void MOD_KG_mode_set_deal(void)		//-�Ų�ģʽ����
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 HL_flag = port_report[4];	//-�Ų�ģʽ����,00h ���ر� �� 01h �Զ���  02 �û�ģʽ
	  	 if(HL_flag == 0)
	  	 {
	  	 	  HL_RED_pwmval = 0;
		 	  	HL_GREEN_pwmval = 0;
		 	  	HL_BLUE_pwmval = 0;
	  	 }
			 HL_new_value_flag = 1;
			 HL_run_time = Time_2048ms_Counter;

       //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = port_report[3] | 0x80;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = port_report[4];		//-���ݳ���

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][5] =LOBYTE(the_ram_ax);
			  port_send[0][6] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;
	  }



}

void MOD_KG_receive_set_deal(void)		//-�Ųʵ�����,Ŀǰ֧������������ɫ
{
	  WORD the_ram_ax;


	  //-aa5515051047DD22000080ff002500ff0000ff8000
    if(port_report[4] == 0x10)		//-���ù̶���ɫ��RGBֵ
	  {
	  	  HL_ld_R_user[0] = port_report[5];
		    HL_ld_G_user[0] = port_report[6];
		    HL_ld_B_user[0] = port_report[7];

		    //-if(((HL_GREEN_pwmval + HL_BLUE_pwmval) >250) && (HL_RED_pwmval < 127))
		    //-	HL_RED_pwmval = 0;
		    //-else if(((HL_GREEN_pwmval + HL_BLUE_pwmval) >250) && (HL_RED_pwmval >= 127) && (HL_RED_pwmval <= 180))
		    //-	HL_RED_pwmval = 9;

		    //-HL_RED_pwmval = (PWM_Period_Value * HL_RED_pwmval) / 255;   //-��
        //-HL_GREEN_pwmval = (PWM_Period_Value * HL_GREEN_pwmval) / 255;		//-��
        //-HL_BLUE_pwmval = (PWM_Period_Value * HL_BLUE_pwmval) / 255;	//-��

        HL_run_time = Time_2048ms_Counter;		//-�����˵�ɫ���Ҳ������,������Ҫ���¼�ʱ�ر�ʱ��

        HL_new_value_flag = 1;
        HL_flag = 1;


		    //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = port_report[3] | 0x80;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = port_report[4];		//-����λ0
			  port_send[0][5] = port_report[5];
			  port_send[0][6] = port_report[6];
			  port_send[0][7] = port_report[7];
			  port_send[0][8] = port_report[8];
			  //-port_send[0][9] = port_report[9];
			  //-port_send[0][10] = port_report[10];
			  //-port_send[0][11] = port_report[11];
			  //-port_send[0][12] = port_report[12];
			  //-port_send[0][13] = port_report[13];
			  //-port_send[0][14] = port_report[14];
			  //-port_send[0][15] = port_report[15];
			  //-port_send[0][16] = port_report[16];
			  //-port_send[0][17] = port_report[17];
			  //-port_send[0][18] = port_report[18];
			  //-port_send[0][19] = port_report[19];
			  //-port_send[0][20] = port_report[20];

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][9] =LOBYTE(the_ram_ax);
			  port_send[0][10] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;
	  }

}

void MOD_KG_chage_page_deal(void)		//-���շ�ҳ����,���ڽ�����������ʱ����Ӧ��
{
	  //-WORD the_ram_ax;

	  if(led_display_start == 0x55)		//-�豸ID��:CO2ֵ
	  {
	  	 ps_flag = port_report[4];
	  	 ps_flag_led = port_report[4];
	  }
	  ps_flag_led_end = port_report[4];
	  ps_flag_led_disp = port_report[4];	  //-������Ҫ�ر�ע��,������ʾ����,����ˢ���Ĺ�����һ��Ҫ��
	  led_disp_L_init();


	  //-�ر���һ���������洢����,�ñ�־λ,Ȼ��ˢ�µ�ʱ��������������Ƿ���Ч
	  //-����еĻ����л���ʾ

}

void MOD_KG_chage_pic_deal(void)		//-���շ�ҳ����,���ڽ�����������ʱ����Ӧ��
{
	  WORD the_ram_ax;

	  if(led_display_txpic_flag != 0x55)
	  {
	  	 led_display_txpic_num = port_report[4];

       led_display_txpic_flag = 0x55;

       port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = port_report[3] | 0x80;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = port_report[4];		//-���ݳ���
			  port_send[0][5] = port_report[5];		//-���ݳ���

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;
	  }

	  cticks_s_page = 0;
    led_display_start = 0x55;

	  //-�ر���һ���������洢����,�ñ�־λ,Ȼ��ˢ�µ�ʱ��������������Ƿ���Ч
	  //-����еĻ����л���ʾ

}

void MOD_KG_SET_loop_deal(void)		//-�ı��������
{
	  WORD the_ram_ax;

	  cticks_ms_pwm_loop = port_report[4] + (port_report[5] << 8);

	  //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = port_report[3] | 0x80;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = port_report[4];		//-���ݳ���
			  port_send[0][5] = port_report[5];		//-���ݳ���

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;
}

void MOD_KG_send_state_deal(void)		//-�����������ڵ�״̬,���ж�Ӧ��״̬��ʾ
{
	  WORD the_ram_ax;

	  if(port_report[4] == 1)
	  {
	  	 led_display_txpic_flag = 0x55;		//-�ô������׼��ͼƬ
	  	 led_display_txpic_num = 0;
	  	 cartoon_end_wait_time = cticks_ms;
	  	 //-led_display_new = 0xa5;	//-��������
	  	 //-led_display_flag = 7;
	  }
	  else
	  {
	  	 led_display_new = 0;
	  	 led_display_flag = 3;
	  	 led_display_txpic_flag = 0x55;		//-�ô������׼��ͼƬ
	  	 led_display_txpic_num = 9;		//-ָ���̶���ͼƬ
	  }


	  //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = port_report[3] | 0x80;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = port_report[4];		//-���ݳ���
			  port_send[0][5] = port_report[5];		//-���ݳ���

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;
}

void MOD_KG_re_Weather_deal(void)		//-
{
	  WORD the_ram_ax;


	  	 Weather_flag = port_report[4];		//-������������
	  	 led_display_page = 4;
	  	 led_display_page_end = 4;		//-��������������һҳ��ʾ
	  	 led_display_new = 0x55;
	  	 led_display_txpic_flag = 0;

	  	 cticks_s_page = 0;			//-����
       led_display_start = 0x55;




	  //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = port_report[3] | 0x80;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = port_report[4];		//-���ݳ���
			  port_send[0][5] = port_report[5];		//-���ݳ���

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;
}

void MOD_KG_re_show_deal(void)		//-������ʾ�ض�ҳ������
{
	  WORD the_ram_ax;
	  //-test_cn++;    //-���Լ�����
	  led_display_page = port_report[4];
	  voice_keep_data_flag = 0;	//-��Ȼ��ʾҳ������,��ô����Ҫ�ⶳ���ܵĶ�������
	  UART1_renew_flag = 0x55;		//-һ�����յ�ָ����������ʾ,��һ��ʱ���ڲ����д��ڸ������ݿ�
	  UART1_renew_wait_time = Time_2048ms_Counter;

	  cticks_s_page = 0;
    led_display_start = 0x55;
	  led_display_new = 0x55;		//-ˢ�����е���ʾ����,���ǹ��ı䶨ֵ����,��Ҫ��ʾ����
	  led_display_deal_flag = 0xaa;	//-�������ֵ��Ŀ����Ϊ�˱�֤��׼������,����ʾ
	  led_display_ye_ok = 0;

	  the_ram_ax = port_report[6] + (port_report[5] << 8);
	  if(led_display_page == 0)
    {//-��ʾPM2.5
    	 pm_data = the_ram_ax;		//-���Կ��Ǹ�����Чֵ��Χ,�������Գ�����,Ҳ����������
    }
    else if(led_display_page == 1)
    {//-����
    	 Noise_Value = the_ram_ax;
    }
    else if(led_display_page == 2)
    {//-VOC
    	 VOC_data = the_ram_ax;
    }
    else if(led_display_page == 3)
    {//-CO2
    	 co2_data = the_ram_ax;
    }
    else if(led_display_page == 4)
    {//-����
    	 Weather_flag = the_ram_ax;		//-
    }
    else if(led_display_page == 20)
    {//-��ǿ
    	 als_data = the_ram_ax;
    }
    else if(led_display_page == 21)
    {//-����ֵ
    	 CSKEY_DATA = the_ram_ax;
    }

	  //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = port_report[3] | 0x80;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = port_report[4];		//-���ݳ���
			  port_send[0][5] = port_report[5];		//-���ݳ���
			  port_send[0][6] = port_report[6];		//-���ݳ���

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][7] =LOBYTE(the_ram_ax);
			  port_send[0][8] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

}

void MOD_KG_polling_state_deal(void)		//-���յ���ѯ����״̬����֮��,ֱ�ӷ���״ֵ̬
{
	  WORD the_ram_ax;



	  //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = port_report[3] | 0x80;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = STM32_UP_selfT_flag1;		//-���ݳ���
			  port_send[0][5] = 0;		//-���ݳ���

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

}

void MOD_KG_polling_PM25_deal(void)		//-���յ�Ӧ��֮��Ͳ���,��������
{
	  //-WORD the_ram_ax;

	  UART1_transmit_flag = NO;		//-������֯���ݷ���
    UART1_transmit_control = 0;	//-�������������͵�����

}

void MOD_KG_end_PM25_deal(void)		//-���յ�Ӧ��֮��Ͳ���,��������
{
	  //-WORD the_ram_ax;

	  UART1_transmit_flag = NO;		//-������֯���ݷ���
    UART1_transmit_control = 0;	//-�������������͵�����

}

void MOD_KG_control_HRL_deal(void)		//-��������,ֻҪ���յ�Ӧ�����Ϊ������Ч,�����ٴη���
{
	  //-WORD the_ram_ax;

	  UART1_transmit_flag = NO;		//-������֯���ݷ���
    UART1_transmit_control = 0;	//-�������������͵�����


}

void MOD_KG_leddis_end_deal(void)		//-��������,ֻҪ���յ�Ӧ�����Ϊ������Ч,�����ٴη���
{
	  //-WORD the_ram_ax;

	  UART1_transmit_flag = NO;		//-������֯���ݷ���
    UART1_transmit_control = 0;	//-�������������͵�����


}

void MOD_KG_send_HRL_mode_deal(void)		//-��������,ֻҪ���յ�Ӧ�����Ϊ������Ч,�����ٴη���
{
	  //-WORD the_ram_ax;

	  UART1_transmit_flag = NO;		//-������֯���ݷ���
    UART1_transmit_control = 0;	//-�������������͵�����


}

void MOD_KG_polling_hl_deal(void)		//-���յ�Ӧ��֮��Ͳ���,��������
{
	  WORD the_ram_ax;



	  //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = port_report[3] | 0x80;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = HL_flag;		//-���ݳ���
			  port_send[0][5] = 0;		//-���ݳ���

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

}

//-�������������͵ı���
void MOD_KG_polling_PM25_cmd(void)		//-���յ���ѯ����״̬����֮��,ֱ�ӷ���״ֵ̬
{
	  WORD the_ram_ax;



	  //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 8;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = 0x0F;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  if(voice_flag == 0x55)	//-����һ����������
			  	port_send[0][4] = led_display_page | 0x80;		//-���ݳ���
			  else
		      port_send[0][4] = led_display_page;		//-���ݳ���

			  port_send[0][5] = HIBYTE(led_display_data);		//-���ݳ���
			  port_send[0][6] = LOBYTE(led_display_data);		//-���ݳ���
			  if(led_display_page == 4)	//-�����,���뱣֤ҳ��仯����ֵͬ����Ӧ
			  {
			  	 port_send[0][5] = 0;		//-���ݳ���
           if(Weather_flag > 5)
             Weather_flag = 1;  //-ʵ��֤���б��������ݵ�ʱ��,���Ե��ж�Ϊ��������ʱ�͹̶�����1,���������ʱʹ��,��Ҫ�ҵ�����ԭ��
			     port_send[0][6] = Weather_flag;		//-���ݳ���
			  }

			  port_send[0][7] = HL_flag;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][8] =LOBYTE(the_ram_ax);
			  port_send[0][9] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

}

void MOD_KG_end_PM25_cmd(void)		//-���յ���ѯ����״̬����֮��,ֱ�ӷ���״ֵ̬
{
	  WORD the_ram_ax;



	  //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = 0x10;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = 0;		//-���ݳ���
			  port_send[0][5] = 0;		//-���ݳ���

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

}

void MOD_KG_control_FUN_cmd(void)		//-���յ���ѯ����״̬����֮��,ֱ�ӷ���״ֵ̬
{
	  WORD the_ram_ax;



	  //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = 0x43;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = 0x55;		//-���ݳ���
			  port_send[0][5] = led_display_page;		//-���ݳ���

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

}

void MOD_KG_update_status_cmd(void)		//-���յ���ѯ����״̬����֮��,ֱ�ӷ���״ֵ̬
{
	  WORD the_ram_ax;



	  //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 10;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = 0x44;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = led_display_start;		//-����������	0x55˵������
			  port_send[0][5] = led_display_page;		  //-��������ʾҳ
			  port_send[0][6] = HL_flag;      //-�ŲʵƱ��
			  port_send[0][7] = HIBYTE(led_display_data);      //-��ǰ��ʾ����Ҳ������
			  port_send[0][8] = LOBYTE(led_display_data);
			  if(led_display_page == 4)	//-�����Ǻ�ӵ���Ҫ���⴦����
			  {
			  	 port_send[0][7] = 0;		//-���ݳ���
           if(Weather_flag > 5)
             Weather_flag = 1;
			     port_send[0][8] = Weather_flag;		//-���ݳ���
			  }
			  port_send[0][9] = voice_flag;		//-��ʾ�Ƿ������������,�������Ļ��Ͳ���7620ת��

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][10] =LOBYTE(the_ram_ax);
			  port_send[0][11] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

}

void MOD_KG_end_HRL_cmd(void)		//-���յ���ѯ����״̬����֮��,ֱ�ӷ���״ֵ̬
{
	  WORD the_ram_ax;
	  static BYTE end_HRL_cn=0;


	  //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = 0x40;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = 0x55;		//-���ݳ���
			  port_send[0][5] = 0;		//-���ݳ���

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

	   end_HRL_cn++;
	   if(end_HRL_cn >= MOD_KG_send_max)
	   {
	   	  end_HRL_cn = 0;
	   	  UART1_transmit_flag = NO;		//-������֯���ݷ���
    		UART1_transmit_control = 0;	//-�������������͵�����
	   }
}

void MOD_KG_can_HRL_cmd(void)		//-���յ���ѯ����״̬����֮��,ֱ�ӷ���״ֵ̬
{
	  WORD the_ram_ax;
	  static BYTE can_HRL_cn=0;


	  //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = 0x40;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = 0xaa;		//-���ݳ���
			  port_send[0][5] = 0;		//-���ݳ���

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

		 can_HRL_cn++;
	   if(can_HRL_cn >= MOD_KG_send_max)
	   {
	   	  can_HRL_cn = 0;
	   	  UART1_transmit_flag = NO;		//-������֯���ݷ���
    		UART1_transmit_control = 0;	//-�������������͵�����
	   }

}

void MOD_KG_send_HRL_mode_cmd(void)		//-���յ���ѯ����״̬����֮��,ֱ�ӷ���״ֵ̬
{
	  WORD the_ram_ax;
	  static BYTE send_HRL_mode_cn=0;


	  //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 9;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = 0x42;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  if(HL_flag == 0)
			  	port_send[0][4] = 0xaa;		//-����������ƵĿ���λ,���������
			  else
			  	port_send[0][4] = 0x55;		//-����������ƵĿ���λ	,�ر������
			  port_send[0][5] = HL_flag;		//-�Ųʵ�ģʽ
			  port_send[0][6] = HL_ld_R_user[0];		//-��ǰ�û�RGBֵ
			  port_send[0][7] = HL_ld_G_user[0];
			  port_send[0][8] = HL_ld_B_user[0];

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][9] =LOBYTE(the_ram_ax);
			  port_send[0][10] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

	   send_HRL_mode_cn++;
	   if(send_HRL_mode_cn >= MOD_KG_send_max)
	   {
	   	  send_HRL_mode_cn = 0;
	   	  UART1_transmit_flag = NO;		//-������֯���ݷ���
    		UART1_transmit_control = 0;	//-�������������͵�����
	   }
}

void MOD_KG_send_leddis_flag_cmd(void)		//-���յ���ѯ����״̬����֮��,ֱ�ӷ���״ֵ̬
{
	  WORD the_ram_ax;
	  static BYTE send_leddis_flag_cn=0;
	  //-0xaa 0x55

	  //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 9;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = 0x14;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = led_display_page;		//-���ݳ���
			  port_send[0][5] = HIBYTE(led_display_data);		//-���ݳ���
			  port_send[0][6] = LOBYTE(led_display_data);		//-���ݳ���
			 	if(led_display_page == 4)	//-�����Ǻ�ӵ���Ҫ���⴦����
			  {
			  	 port_send[0][5] = 0;		//-���ݳ���
           if(Weather_flag > 5)
             Weather_flag = 1;
			     port_send[0][6] = Weather_flag;		//-���ݳ���
			  }

			 	port_send[0][7] = voice_flag;		//-��ʾ�Ƿ������������,�������Ļ��Ͳ���7620ת��
			 	port_send[0][8] = HL_flag;
			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][9] =LOBYTE(the_ram_ax);
			  port_send[0][10] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

        //-test_cn++;    //-���Լ�����
			  send_leddis_flag_cn++;
	   if(send_leddis_flag_cn >= MOD_KG_send_max)
	   {
	   	  send_leddis_flag_cn = 0;
	   	  UART1_transmit_flag = NO;		//-������֯���ݷ���
    		UART1_transmit_control = 0;	//-�������������͵�����
	   }

}

void MOD_KG_send_leddis_end_cmd(void)		//-���������巢�͵�����Ϩ��״̬
{
	  WORD the_ram_ax;
	  static BYTE send_leddis_end_cn=0;
	  //-0xaa 0x55

	  //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = 0x41;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = 0;		//-���ݳ���
			  port_send[0][5] = 0;		//-���ݳ���

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

			  send_leddis_end_cn++;
	   if(send_leddis_end_cn >= MOD_KG_send_max)
	   {
	   	  send_leddis_end_cn = 0;
	   	  UART1_transmit_flag = NO;		//-������֯���ݷ���
    		UART1_transmit_control = 0;	//-�������������͵�����
	   }

}

void MOD_KG_SET_hl_time_deal(void)		//-�ı��������
{
	  WORD the_ram_ax;

	  HL_run_time_delay = port_report[4] + (port_report[5] << 8);

	  //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = port_report[3] | 0x80;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = port_report[4];		//-���ݳ���
			  port_send[0][5] = port_report[5];		//-���ݳ���

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;
}

void MOD_KG_send_leddis_flag_deal(void)		//-˵���Ѿ��յ���,����Ҫ�ٷ�����
{
	  //-WORD the_ram_ax;

		UART1_transmit_flag = NO;		//-������֯���ݷ���
    UART1_transmit_control = 0;	//-�������������͵�����

}

void MOD_KG_SET_voice_flag_deal(void)		//-�ı��������
{
	  WORD the_ram_ax;

	  voice_flag = port_report[4];
	  EEP_Data_flag = 0x55;		//-��ֵ���޸���

	  //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = port_report[3] | 0x80;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = port_report[4];		//-���ݳ���
			  port_send[0][5] = port_report[5];		//-���ݳ���

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;
}

void MOD_KG_polling_voice_flag_deal(void)		//-�ı��������
{
	  WORD the_ram_ax;



	  //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = port_report[3] | 0x80;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = voice_flag;		//-���ݳ���
			  port_send[0][5] = 0;		//-���ݳ���

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;
}

void MOD_KG_re_gateway_flag_deal(void)		//-�������֮�������ʼ����
{
	  WORD the_ram_ax;

	  //-���̴���
	  led_display_new = 0x55;
	  led_display_flag = 3;

    //-�����Ųʵ����ȶ�ֵ
    HL_ld_brightness[0] = port_report[5];

	  //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = port_report[3] | 0x80;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = 0xaa;		//-���ݳ���
			  port_send[0][5] = 0xaa;		//-���ݳ���

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;
}

void MOD_KG_send_voice_mode_deal(void)		//-�������֮�������ʼ����
{
	  BYTE temp_data8;
	  WORD the_ram_ax;
	  int res;

	  //-���̴���
	  if(port_report[4] == 0)
	  {	//-�ر��û�
	  	//-ADXL_TAP_off_flag = 0x55;		//?����Ӧ����Ҫ���Ƕ�ʧ���Ĳ��ָܻ���̬�����
	  	//-ADXL_TAP_off_time = cticks_ms;
	  	//-���ٹرն��ǵ���������
      //-�ϲ���㹫ʽ��V/3+57,��ôV�ķ�Χ��0~100,�����ҽ��յ��ķ�Χ��57~90,����ֵ33
	  	temp_data8 = port_report[5]/3;//- + 57;
      temp_data8 = temp_data8 * 7;
	  	res = ADXL345_init_re(temp_data8);
	  	if(res != 1)
	  		ADXL345_init_re(temp_data8);
	  }
	  else
	  {//-�ָ�������ֵ
	  	ADXL_TAP_off_flag = 0;
	  	res = ADXL345_init_re(0);
	  	if(res != 1)
	  		ADXL345_init();
	  }

	  //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = port_report[3] | 0x80;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = 0xaa;		//-���ݳ���
			  port_send[0][5] = 0xaa;		//-���ݳ���

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;
        //-test_cn++;    //-���Լ�����
}

void MOD_KG_send_es705_status_deal(void)	//-��Ҫ�ر����Ƶ�
{
	 //-WORD the_ram_ax;

	 if(port_report[5] == (es705_training_count + 1))
	    es705_training_count = port_report[5];		//-ѧϰ�ɹ��Ĵ���
	 es705_training_flag = 0x55;
	 es705_training_wait_time = Time_2048ms_Counter;


	 led_display_txpic_num = 20 +es705_training_count;		//-ָʾ�ĸ�ͼƬ���ڱ���ʾ
	 led_display_txpic_flag = 0x55;
	 cticks_s_page = 0;
   led_display_start = 0x55;

   if(es705_training_count == 4)
   {
   	  led_display_txpic_flag = 0;
   	  es705_training_count = 0;
   }
}

void MOD_KG_set_hl_set_deal(void)		//-�ı��������
{
	  WORD the_ram_ax;

	  if(port_report[4] > 100)
	  	 HL_ld_brightness[1] = 100;
	  else
	  	 HL_ld_brightness[1] = port_report[4];

    HL_ld_brightness_flag = 0x55;
    HL_new_value_flag = 1;

	  //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = port_report[3] | 0x80;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = port_report[4];		//-���ݳ���
			  port_send[0][5] = port_report[5];		//-���ݳ���

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;
}

void MOD_KG_inquire_version_deal(void)		//-
{
	  WORD the_ram_ax;

	  //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = port_report[3] | 0x80;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = HIBYTE(STM32_SYS_VERSION_NUM);		//-���ݳ���
			  port_send[0][5] = LOBYTE(STM32_SYS_VERSION_NUM);		//-���ݳ���

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;
}


void MOD_KG_inquire_update_deal(void)		//-
{
	  WORD the_ram_ax;

    if((port_report[4] == 2) && (port_report[5] == 0x55))
    {
        FLASH_Unlock();						//����
				FLASH_ErasePage(APP_CONFIG_ADDR);//�����������
				FLASH_ProgramHalfWord(APP_CONFIG_ADDR,0x0000);	//-�������п�������
				FLASH_ErasePage(APP_CONFIG1_ADDR);//�����������
				FLASH_ProgramHalfWord(APP_CONFIG2_ADDR,0x0000);	//-��������
        FLASH_ProgramHalfWord(APP_CONFIG1_ADDR,0x0000);	//-��������
				FLASH_Lock();//����

	  //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = port_report[3] | 0x80;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = 0x02;		//-���ݳ���
			  port_send[0][5] = 0xaa;		//-���ݳ���

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

        while(1);   //-�ȴ���λ����,����IAP����
    }
}

void MOD_KG_set_reset_deal(void)		//-�ı��������
{
	  WORD the_ram_ax;

	  if((port_report[4] == 0x03) && (port_report[5] == 0x55))
	  {


	  //-������ݳɹ����յ�֮���Ӧ��һ��
			  port_send[0][0] = 0xaa;		//-���ֽڰ�ͷ
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-�������� 1���ֽڣ��Ӱ�ͷ��ʼ����CRC16֮ǰ�����ݡ�

			  port_send[0][3] = port_report[3] | 0x80;		//-������:01h �����������ϱ�

			  //-��Ч����
			  //-�������Ч���������Կ�Ϊ��λ��"���ݳ���+�豸ID+������+��������+����λ"
			  port_send[0][4] = port_report[4];		//-���ݳ���
			  port_send[0][5] = port_report[5];		//-���ݳ���

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-ֱ���������еı������ݶ��Ѿ�׼������

			  //-���ͳ���
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

			  while(1);
		}
}








//-���洦��ͨѶ���յ�������,��ʵ����һ���������ϲ�Э��Ĵ���,���������������ڵ���
void CDT9702_Main()
{
	 WORD the_ram_ax,the_ram_bx;
	 WORD temp_data,temp_data1,delta_len,temp_loop,temp_int;


	 //-�жϷ����Ƿ������,��������˿��������߼�����,���򲻴���
	 if(DMA_GetFlagStatus(DMA1_FLAG_TC4))		//-����˵�����������
   {//-ֻ�е���������˲ſ������¸�DMA���ʹ�������
   	  //-���ݷ������֮��ȫ������,�������ڲ�֪��ÿ���Ŀռ��С��,��ôѡȡһ��������
   	  for(temp_loop = 0;temp_loop < 43;temp_loop++)
   	  	port_send[0][temp_loop] = 0;
   	  port_deal_flag = 0;		//-��һ����0ֵ�Ϳ������´μ�鷢��ʱ������������
   	  //-MOD_KG_transmit_flag=YES;		//-���ҵ����ݷ��ͳ�ȥ��Ҳ������������������??����Ӧ�ò���������֯���ݵķ���,������������ݻᱻ����,���յı�־λ
   	  DMA_ClearFlag(DMA1_FLAG_TC4);
   }

	 if(port_deal_flag == 0)	//-����᲻������һ�η���ʧ�ܶ�������Ҳ���ܷ���
   {
   	   //-���ȴ�����յ�������,���û�����ݽ��յ�,����֯������Ҫ���͵�����
   	   if((port_recv_pt[0]!=port_recv_dl[0]) && (MOD_KG_transmit_flag == YES))	//-����һ���ж�����,������һ��֮��һ��ʱ��֮��ſ��Դ�����������
   	   {

   	   	   if(MOD_KG_rxd_head_flag!=YES)	//-���յ������ݻ�û�д����ʱ�����NO
           {
           		 MOD_KG_rxd_wait_time=cticks_ms;	//-�����Ŀ����Ϊ��һ�����ղ��������ݾʹ�ͷ��ʼ��ʱ,Ҳ����һ����Ч֡���ܶϿ�̫��ʱ��
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
	        MOD_KG_transmit_flag=YES;	//-���յ�Ӧ��Ϳ��Է���������
	        //-MOD_KG_transmit_wait_time=Time_1ms_Counter;	//-��Ȼ����˵���Է���������,���ǻ���Ҫ��ʱһ��ʱ��,��Ϊ���յ���������Ҫ����
	        //-�����Ƕ�ʵ�����ݵĴ���,������Ǻ��Ĳ���
	        the_ram_bx=(port_recv_dl[0]+3)&0x1ff;;
	        if(port_recv[0][the_ram_bx]!=0xff)	//-����ǶԹ�������ж�,�����벻ͬ�жϵ�����Ҳ��һ��
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
	         	//-port_recv_dl[0]+=delta_len;	//-�����������������ı���
	         	port_recv_dl[0]+=(port_report[2] + 2); //-������⵽����Ч����
            port_recv_dl[0]&=0x1ff;
	         	temp_int=MOD_KG_CRC16(&port_report[0],port_report[2]);
	         	if((LOBYTE(temp_int)!=port_report[port_report[2]+0])||(HIBYTE(temp_int)!=port_report[port_report[2]+1]))	//-����CRC���
	         	{	//-������Ĳ�ȷ����,���У�鲻��ȷ�Ļ���ֱ�Ӱ�ͷ���,����Ѱ��
	          		goto inrda;	//-������˵���ɹ����յ��ı���CRCУ��û��ͨ��
	          }
	        }
	        else
	        {
	            //-port_recv_dl[0]+=delta_len;	//-����ط��������������Ĵ�����
	         	  port_recv_dl[0]+=7; //-������⵽����Ч����
              port_recv_dl[0]&=0x1ff;
	            goto inrda;
	        }


////////////////////////////////////////////////////////////////////////////////
					if(port_report[3] == 1)
					{
							MOD_KG_rec_frame_type = MOD_KG_receive_data_FRAME;
							MOD_KG_receive_data_deal();
							MOD_KG_transmit_wait_time=cticks_ms;	//-�������ʱ���Ŀ���Ǹ���������ն˴���ʱ��
					}
					else
					{
							if(port_report[3] == 2)
							{
								 if(port_report[4] == 1)
				      	 {
				      	 	   MOD_KG_rec_frame_type = MOD_KG_polling_ID_FRAME;
				      	 	   MOD_KG_polling_ID_ack();
				      	 	   MOD_KG_transmit_wait_time=cticks_ms;	//-�������ʱ���Ŀ���Ǹ���������ն˴���ʱ��
						    		 MOD_KG_transmit_flag=NO;
				      	 }
				      	 else
				      	 {
				      	 	   MOD_KG_rec_frame_type = MOD_KG_polling_data_FRAME;
				      	 	   MOD_KG_polling_data_ack();
				      	 	   MOD_KG_transmit_wait_time=cticks_ms;	//-�������ʱ���Ŀ���Ǹ���������ն˴���ʱ��
		  					     MOD_KG_transmit_flag=NO;
				      	 }
							}
							else
							{
									if(port_report[3] == 3)
						      {//-����LED������ͼƬ
						      	  MOD_KG_rec_frame_type = MOD_KG_LED_display_FRAME;
						      	  MOD_KG_LED_display_deal();
						      	  MOD_KG_transmit_wait_time=cticks_ms;	//-�������ʱ���Ŀ���Ǹ���������ն˴���ʱ��
						      }
						      else
						      {
						      		if(port_report[3] == 4)		//-��������ʱ������,Ҳ��ҪӦ��
								      {
								      	  MOD_KG_rec_frame_type = MOD_KG_SET_screen_FRAME;
								      	  MOD_KG_SET_screen_deal();
								      	  MOD_KG_transmit_wait_time=cticks_ms;	//-�������ʱ���Ŀ���Ǹ���������ն˴���ʱ��
                  				MOD_KG_transmit_flag=NO;
								      }
								      else
								      {
								      		if(port_report[3] == 5)		//-���յ��Ųʵ�����,�������õĶ���һ����ֵ��Ķ���,������Ҫ��֤���ͳɹ�
										      {
										      	  MOD_KG_rec_frame_type = MOD_KG_receive_set_FRAME;
										      	  MOD_KG_receive_set_deal();
										      	  MOD_KG_transmit_wait_time=cticks_ms;	//-�������ʱ���Ŀ���Ǹ���������ն˴���ʱ��
 						                  MOD_KG_transmit_flag=NO;
										      }
										      else
										      {
										      		if(port_report[3] == 6)		//-���յ��Ų�ģʽ����,�������õĶ���һ����ֵ��Ķ���,������Ҫ��֤���ͳɹ�
												      {
												      	  MOD_KG_rec_frame_type = MOD_KG_mode_set_FRAME;
												      	  MOD_KG_mode_set_deal();
												      	  MOD_KG_transmit_wait_time=cticks_ms;	//-�������ʱ���Ŀ���Ǹ���������ն˴���ʱ��
 								                  MOD_KG_transmit_flag=NO;
												      }
												      else
												      {
												      		if(port_report[3] == 8)		//-���յ��Ų�ģʽ����,�������õĶ���һ����ֵ��Ķ���,������Ҫ��֤���ͳɹ�
														      {
														      	  MOD_KG_rec_frame_type = MOD_KG_chage_page_FRAME;
														      	  MOD_KG_chage_page_deal();
														      	  MOD_KG_transmit_wait_time=cticks_ms;	//-�������ʱ���Ŀ���Ǹ���������ն˴���ʱ��
														      }
														      else
														      {
														      		if(port_report[3] == 9)		//-���յ��Ų�ģʽ����,�������õĶ���һ����ֵ��Ķ���,������Ҫ��֤���ͳɹ�
																      {
																      	  MOD_KG_rec_frame_type = MOD_KG_chage_pic_FRAME;
																      	  MOD_KG_chage_pic_deal();
																      	  MOD_KG_transmit_wait_time=cticks_ms;	//-�������ʱ���Ŀ���Ǹ���������ն˴���ʱ��
     													     	      MOD_KG_transmit_flag=NO;
																      }
																      else
																      {
																      		if(port_report[3] == 10)		//-���յ��Ų�ģʽ����,�������õĶ���һ����ֵ��Ķ���,������Ҫ��֤���ͳɹ�
																		      {
																		      	  MOD_KG_rec_frame_type = MOD_KG_SET_loop_FRAME;
																		      	  MOD_KG_SET_loop_deal();
																		      	  MOD_KG_transmit_wait_time=cticks_ms;	//-�������ʱ���Ŀ���Ǹ���������ն˴���ʱ��
     															     	      MOD_KG_transmit_flag=NO;
																		      }
																		      else
																		      {
																		      		if(port_report[3] == 11)		//-���յ��Ų�ģʽ����,�������õĶ���һ����ֵ��Ķ���,������Ҫ��֤���ͳɹ�
																				      {
																				      	  MOD_KG_rec_frame_type = MOD_KG_send_state_FRAME;
																				      	  MOD_KG_send_state_deal();
																				      	  MOD_KG_transmit_wait_time=cticks_ms;	//-�������ʱ���Ŀ���Ǹ���������ն˴���ʱ��
 																	         	      MOD_KG_transmit_flag=NO;
																				      }
																				      else
																				      {
																				      		if(port_report[3] == 12)		//-���յ��Ų�ģʽ����,�������õĶ���һ����ֵ��Ķ���,������Ҫ��֤���ͳɹ�
																						      {
																						      	  MOD_KG_rec_frame_type = MOD_KG_re_Weather_FRAME;
																						      	  MOD_KG_re_Weather_deal();
																						      	  MOD_KG_transmit_wait_time=cticks_ms;	//-�������ʱ���Ŀ���Ǹ���������ն˴���ʱ��
 																			         	      MOD_KG_transmit_flag=NO;
																						      }
																						      else
																						      {
																						      		if(port_report[3] == 13)		//-ָ����ʾ��һҳ����������
																								      {//?�������ݵĶ�ʧ�ǽ������յ�һ�λ��Ǹ���û�н��յ�,ʵ������ǽ��յ���,���Ǳ�������û����ȷʶ��.
																								      	  MOD_KG_rec_frame_type = MOD_KG_re_show_FRAME;
																								      	  MOD_KG_re_show_deal();
																								      	  MOD_KG_transmit_wait_time=cticks_ms;	//-�������ʱ���Ŀ���Ǹ���������ն˴���ʱ��
																				          	      MOD_KG_transmit_flag=NO;
																								      }
																								      else
																								      {
																								      		if(port_report[3] == 14)		//-ָ����ʾ��һҳ����������
																										      {
																										      	  MOD_KG_rec_frame_type = MOD_KG_polling_state_FRAME;
																										      	  MOD_KG_polling_state_deal();
																										      	  MOD_KG_transmit_wait_time=cticks_ms;	//-�������ʱ���Ŀ���Ǹ���������ն˴���ʱ��
																						          	      MOD_KG_transmit_flag=NO;
																										      }
																										      else
																										      {
																										      		if(port_report[3] == 15)		//-
																												      {
																												      	  MOD_KG_rec_frame_type = MOD_KG_polling_PM25_FRAME;
																												      	  MOD_KG_polling_PM25_deal();
																												      	  MOD_KG_transmit_wait_time=cticks_ms;	//-�������ʱ���Ŀ���Ǹ���������ն˴���ʱ��
																												      }
																												      else
																												      {
																												      		if(port_report[3] == 16)		//-
																														      {
																														      	  MOD_KG_rec_frame_type = MOD_KG_end_PM25_FRAME;
																														      	  MOD_KG_end_PM25_deal();
																														      	  MOD_KG_transmit_wait_time=cticks_ms;	//-�������ʱ���Ŀ���Ǹ���������ն˴���ʱ��
																														      }
																														      else
																														      {
																														      		if(port_report[3] == 17)		//-
																																      {
																																      	  MOD_KG_rec_frame_type = MOD_KG_polling_hl_FRAME;
																																      	  MOD_KG_polling_hl_deal();
																																      	  MOD_KG_transmit_wait_time=cticks_ms;	//-�������ʱ���Ŀ���Ǹ���������ն˴���ʱ��
 																													         	      MOD_KG_transmit_flag=NO;
																																      }
																														      	  else
																														      	  {
																														      	  	  if(port_report[3] == 19)		//-
																																		      {
																																		      	  MOD_KG_rec_frame_type = MOD_KG_SET_hl_time_FRAME;
																																		      	  MOD_KG_SET_hl_time_deal();
																																		      	  MOD_KG_transmit_wait_time=cticks_ms;	//-�������ʱ���Ŀ���Ǹ���������ն˴���ʱ��
																														          	      MOD_KG_transmit_flag=NO;
																																		      }
																														      	      else
																														      	      {
																														      	      		if(port_report[3] == 0x94)		//-���յ�7620��ȷ��Ӧ��,�������е㲻ͬ
																																				      {
																																				      	  MOD_KG_rec_frame_type = MOD_KG_send_leddis_flag_FRAME;
																																				      	  MOD_KG_send_leddis_flag_deal();
																																				      	  MOD_KG_transmit_wait_time=cticks_ms;	//-�������ʱ���Ŀ���Ǹ���������ն˴���ʱ��
																																				      }
																																				      else
																																				      {
																																				      		if(port_report[3] == 0x15)		//-
																																						      {
																																						      	  MOD_KG_rec_frame_type = MOD_KG_SET_voice_flag_FRAME;
																																						      	  MOD_KG_SET_voice_flag_deal();
																																						      	  MOD_KG_transmit_wait_time=cticks_ms;	//-�������ʱ���Ŀ���Ǹ���������ն˴���ʱ��
 																																			         	      MOD_KG_transmit_flag=NO;
																																						      }
																																						      else
																																						      {
																																						      		if(port_report[3] == 0x16)		//-
																																								      {
																																								      	  MOD_KG_rec_frame_type = MOD_KG_polling_voice_flag_FRAME;
																																								      	  MOD_KG_polling_voice_flag_deal();
																																								      	  MOD_KG_transmit_wait_time=cticks_ms;	//-�������ʱ���Ŀ���Ǹ���������ն˴���ʱ��
 																																					         	      MOD_KG_transmit_flag=NO;
																																								      }
																																								      else
																																								      {
																																								      		if(port_report[3] == 0x17)		//-
																																										      {
																																										      	  MOD_KG_rec_frame_type = MOD_KG_re_gateway_flag_FRAME;
																																										      	  MOD_KG_re_gateway_flag_deal();
																																										      	  MOD_KG_transmit_wait_time=cticks_ms;	//-�������ʱ���Ŀ���Ǹ���������ն˴���ʱ��
 																																							         	      MOD_KG_transmit_flag=NO;
																																										      }
																																										      else
																																										      {
																																										      		if(port_report[3] == 0x1B)		//-
																																												      {
																																												      	  MOD_KG_rec_frame_type = MOD_KG_send_voice_mode_FRAME;
																																												      	  MOD_KG_send_voice_mode_deal();
																																												      	  MOD_KG_transmit_wait_time=cticks_ms;
																																								          	      MOD_KG_transmit_flag=NO;
																																												      }
																																												      else
																																												      {
																																												      		if(port_report[3] == 0x1D)		//-
																																														      {
																																														      	  MOD_KG_rec_frame_type = MOD_KG_send_es705_status_FRAME;
																																														      	  MOD_KG_send_es705_status_deal();
																																														      	  MOD_KG_transmit_wait_time=cticks_ms;	//-�������ʱ���Ŀ���Ǹ���������ն˴���ʱ��
																																														      }
																																														      else
																																														      {
																																														      		if(port_report[3] == 0xC0)		//-�����Ӧ��
																																																      {
																																																      	  MOD_KG_rec_frame_type = MOD_KG_control_HRL_FRAME;
																																																      	  MOD_KG_control_HRL_deal();
																																																      	  MOD_KG_transmit_wait_time=cticks_ms;	//-�������ʱ���Ŀ���Ǹ���������ն˴���ʱ��
																																																      }
																																																      else
																																																      {
																																																      		if(port_report[3] == 0xC1)		//-���յ�7620��ȷ��Ӧ��,�������е㲻ͬ
																																																		      {
																																																		      	  MOD_KG_rec_frame_type = MOD_KG_leddis_end_FRAME;
																																																		      	  MOD_KG_leddis_end_deal();
																																																		      	  MOD_KG_transmit_wait_time=cticks_ms;	//-�������ʱ���Ŀ���Ǹ���������ն˴���ʱ��
																																																		      }
																																																		      else
																																																		      {
																																																		      		if(port_report[3] == 0x9A)		//-���յ�7620��ȷ��Ӧ��,�������е㲻ͬ
																																																				      {
																																																				      	  MOD_KG_rec_frame_type = MOD_KG_send_HRL_mode_FRAME;
																																																				      	  MOD_KG_send_HRL_mode_deal();
																																																				      	  MOD_KG_transmit_wait_time=cticks_ms;	//-�������ʱ���Ŀ���Ǹ���������ն˴���ʱ��
																																																				      }
																																																				      else
																																																				      {
                                                                                                                  if(port_report[3] == 0x21)
                                                                                                                  {
                                                                                                                      MOD_KG_rec_frame_type = MOD_KG_set_hl_set_FRAME;
                                                                                                                      MOD_KG_set_hl_set_deal();
                                                                                                                      MOD_KG_transmit_wait_time=cticks_ms;	//-�������ʱ���Ŀ���Ǹ���������ն˴���ʱ��
                                                                                                                  }
                                                                                                                  else
                                                                                                                  {
                                                                                                                  	 if(port_report[3] == 0x1C)		//-��λ����
                                                                                                                  	 {
                                                                                                                  	 		MOD_KG_set_reset_deal();
                                                                                                                  	 }
                                                                                                                  	 else
                                                                                                                     {
                                                                                                                       if(port_report[3] == 0x24)		//-
                                                                                                                       {
                                                                                                                          MOD_KG_rec_frame_type = MOD_KG_7620_inquire_version_FRAME;
                                                                                                                          MOD_KG_inquire_version_deal();
                                                                                                                          MOD_KG_transmit_wait_time=cticks_ms;
                                                                                                                       }
                                                                                                                       else
                                                                                                                       {
                                                                                                                         if(port_report[3] == 0x23)		//-
                                                                                                                         {
                                                                                                                            MOD_KG_rec_frame_type = MOD_KG_7620_inquire_update_FRAME;
                                                                                                                            MOD_KG_inquire_update_deal();
                                                                                                                            MOD_KG_transmit_wait_time=cticks_ms;
                                                                                                                         }
                                                                                                                         else
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
	      	//-MOD_KG_transmit_flag=YES;	//-��ʱû�н��յ�Ҳ������֯��������
	      	//-MOD_KG_wait_replay=NO;	//-��ʾ���ڻ�û�еȴ��ظ�
	      	//-MOD_KG_transmit_wait_time=Time_1ms_Counter;

	       	//-MOD_KG_comm_err_counter[port_send[0][0]]++;	//-��ʱ�������
	    }
	    if(Judge_Time_In_MainLoop(MOD_KG_transmit_wait_time,75)==YES)
	    	MOD_KG_transmit_flag=YES;	//-һ�����յ���Ч����,��ô80msʱ���ſ��Դ����´ο��ܵı���
inrda:
		   //-�������� ,,�������һ������,�೤ʱ�䷢��һ��,����˵������ʹ���������ʲô
		   if((UART1_transmit_flag==YES) && (MOD_KG_transmit_flag == YES))		//-Ŀǰ����ν����˫���,���Ǻ�����Ҫ����һ������,��֤���ͻ������е����ݷ��ͳ�ȥ�˲ſ�����֯����,���ﲻ��Ҫ�ȴ���Ӧ��
		   {
		   	  if(Judge_Time_In_MainLoop(MOD_KG_transmit_wait_time,100)==YES)	//-ֻ�й��˵ȴ�ʱ��Żᷢ��,����ʱ����Ҫ��Ϊ�˸����ն��㹻�Ĵ���ʱ��
			   	  switch(UART1_transmit_control)
	          {
	                case 1:
	                     //MOD_KG_rec_frame_type=MOD_KG_YX_FRAME;	//-��ʾ��Ҫ���յ�֡�����������ֵ,���յ�������ֵ������ȷ��
	                     //MOD_KG_wait_replay=YES;
	                     MOD_KG_polling_PM25_cmd();	//-����Լ�����������,�ʴ��͵�,����˿��ϵ�����װ�ö��ǽ��ղ�ѯ
	                     MOD_KG_transmit_wait_time=cticks_ms;

	                     UART1_transmit_flag = NO;		//-���︴���˾Ͳ��ȴ�Ӧ����ֱ�ӷ��Ա����
    									 UART1_transmit_control = 0;
	                     //-MOD_KG_send_sense_data();
	                     //-port_send_sense_data[0] = 0;
	                     //MOD_KG_now_poll_addr++;	//-�ӻ���ַ

	                     //-MOD_KG_transmit_flag=NO;	//-���ڿ�ʼ���ܷ��ͱ�����
	                     //-MOD_KG_rxd_head_flag=NO;	//-Ϊ��ͬ��׼����,���ǲ��������еĶ�����������
	                     //-MOD_KG_rxd_wait_time=cticks_ms;	//-Ӧ����Ҳ����ʱЧ�Ե�

	                     //-MOD_KG_transmit_control=1;	//-���еĶ˿�װ�ò�ѯһ��֮��,�ٻ���һ�౨�Ĳ�ѯ

	                     break;
	                case 2:
	                     //MOD_KG_rec_frame_type=MOD_KG_YX_FRAME;	//-��ʾ��Ҫ���յ�֡�����������ֵ,���յ�������ֵ������ȷ��
	                     //MOD_KG_wait_replay=YES;
	                     MOD_KG_end_PM25_cmd();	//-����Լ�����������,�ʴ��͵�,����˿��ϵ�����װ�ö��ǽ��ղ�ѯ
	                     MOD_KG_transmit_wait_time=cticks_ms;

	                     UART1_transmit_flag = NO;		//-���︴���˾Ͳ��ȴ�Ӧ����ֱ�ӷ��Ա����
    									 UART1_transmit_control = 0;
	                     //-MOD_KG_send_sense_data();
	                     //-port_send_sense_data[0] = 0;
	                     //MOD_KG_now_poll_addr++;	//-�ӻ���ַ

	                     //-MOD_KG_transmit_flag=NO;	//-���ڿ�ʼ���ܷ��ͱ�����
	                     //-MOD_KG_rxd_head_flag=NO;	//-Ϊ��ͬ��׼����,���ǲ��������еĶ�����������
	                     //-MOD_KG_rxd_wait_time=cticks_ms;	//-Ӧ����Ҳ����ʱЧ�Ե�

	                     //-MOD_KG_transmit_control=1;	//-���еĶ˿�װ�ò�ѯһ��֮��,�ٻ���һ�౨�Ĳ�ѯ

	                     break;
	                case 3:		//-�ر����������

	                	   MOD_KG_end_HRL_cmd();	//-����Լ�����������,�ʴ��͵�,����˿��ϵ�����װ�ö��ǽ��ղ�ѯ
	                     MOD_KG_transmit_wait_time=cticks_ms;
	                	   break;
	                case 4:		//-�������������

	                	   MOD_KG_can_HRL_cmd();	//-����Լ�����������,�ʴ��͵�,����˿��ϵ�����װ�ö��ǽ��ղ�ѯ
	                     MOD_KG_transmit_wait_time=cticks_ms;
	                	   break;
	                case 5:		//-����Ŀǰ������������״̬����ʾ����

	                	   MOD_KG_send_leddis_flag_cmd();	//-����Լ�����������,�ʴ��͵�,����˿��ϵ�����װ�ö��ǽ��ղ�ѯ
	                     MOD_KG_transmit_wait_time=cticks_ms;
	                	   break;
	                case 6:		//-����Ŀǰ������������״̬����ʾ����

	                	   MOD_KG_send_leddis_end_cmd();	//-����Լ�����������,�ʴ��͵�,����˿��ϵ�����װ�ö��ǽ��ղ�ѯ
	                     MOD_KG_transmit_wait_time=cticks_ms;
	                	   break;
	                case 7:		//-����Ŀǰ������������״̬����ʾ����

	                	   MOD_KG_send_HRL_mode_cmd();	//-����Լ�����������,�ʴ��͵�,����˿��ϵ�����װ�ö��ǽ��ղ�ѯ
	                     MOD_KG_transmit_wait_time=cticks_ms;
	                	   break;
	                case 8:
	                     MOD_KG_control_FUN_cmd();	//-����Լ�����������,�ʴ��͵�,����˿��ϵ�����װ�ö��ǽ��ղ�ѯ
	                     MOD_KG_transmit_wait_time=cticks_ms;

	                     UART1_transmit_flag = NO;		//-���︴���˾Ͳ��ȴ�Ӧ����ֱ�ӷ��Ա����
    									 UART1_transmit_control = 0;
	                     break;
	                case 9:			//-��������״̬,�Ա�ͳһ״̬��Ϣ
	                     MOD_KG_update_status_cmd();	//-����Լ�����������,�ʴ��͵�,����˿��ϵ�����װ�ö��ǽ��ղ�ѯ
	                     MOD_KG_transmit_wait_time=cticks_ms;

	                     UART1_transmit_flag = NO;		//-���︴���˾Ͳ��ȴ�Ӧ����ֱ�ӷ��Ա����
    									 UART1_transmit_control = 0;
	                     break;
	                default:
	                     break;
	          }

		   }
 	 }

}


//-�µ�Э�鲻��Ҫ�ȴ�Ӧ��,���Խ����ȴ���������,��ֹ������������
//-������Ҫ�ȴ�Ӧ��,������Ҫ��ʱ����

//-2016/1/8 9:02:10
//-����������7620�Ѿ��������ظ�������3��,����STM32Ҳ�Ѿ����յ���,��ư�STM32Ҳ���յ���,��ô����ķ��ͳ�ȥ����?

