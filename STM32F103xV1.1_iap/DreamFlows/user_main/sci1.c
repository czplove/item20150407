/*
����:ʹ��һ�������߼�,����һ��512�ֽڵĻ�����,Ȼ��ѭ���洢
����:Ҳʹ��512�ֽڵĿռ䱣������,Ȼ����һ�����ͱ�־,ֱ�����ݷ��ͳ�ȥΪֹ

����3 ��Ϊ��ư��7620֮��ͨѶ͸����,���ھ����������κδ���,����˫��͸��
7620��������Ҫ���ݵ���ư�Ķ���,ֱ���ô���3����
STM32����������,ֱ���ڴ���3�д���,�������κ�����,�����ñ�־λ�ô���2�Լ���֯������7620����
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
#define   MOD_KG_SET_hl_time_FRAME		  	    		 0x13
#define   MOD_KG_send_leddis_flag_FRAME		     		 0x14
#define   MOD_KG_SET_voice_flag_FRAME		     		   0x15
#define   MOD_KG_polling_voice_flag_FRAME		   	   0x16
#define   MOD_KG_re_gateway_flag_FRAME		    	   0x17

#define   MOD_KG_send_voice_mode_FRAME	      	   0x1B

#define   MOD_KG_set_hl_set_FRAME	     	           0x21



#define   MOD_KG_control_HRL_FRAME		  	    	   0x40
#define   MOD_KG_leddis_end_FRAME	  	  	    	   0x41
#define   MOD_KG_send_HRL_mode_FRAME	  	     	   0x42
#define   MOD_KG_control_FUN_FRAME	    	     	   0x43
#define   MOD_KG_update_status_FRAME	    	     	 0x44


extern void RunLed_stata_judge(void);

//-ΪЭ�鴦����ʱ����ı���
BYTE  MOD_TC_rxd_head_flag;
BYTE  MOD_TC_rec_OK;
BYTE  MOD_TC_rec_frame_type;
WORD  MOD_TC_rxd_wait_time;
BYTE  MOD_TC_transmit_flag;
BYTE  MOD_TC_transmit_control;
WORD  MOD_TC_transmit_wait_time;



void UART3_Init(void)
{
  port_recv_pt[1] = 0;
  port_send_pt[1] = 0;
  port_recv_dl[1] = 0;

  //-Э���ʼ������
  port_deal_flag[1] = 0;
  MOD_TC_transmit_flag=YES;
  MOD_TC_transmit_control = 1;

}


void UART3_Rx_Deal(void)
{
  port_recv_pt[1] = 512 - DMA_GetCurrDataCounter(DMA1_Channel3);	//-�õ����ո���,�����Ϊ����ƫ�����Ļ�,���ڵ���ֵ����ָ�����ռ��
  if(port_recv_pt[1] >= 512)
  	port_recv_pt[1] = 0;
}

void UART3_Tx_Deal(void)
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

  if(port_send_len[1] != 0)	//-һ��Ҫ��֤���ݷ��ͳ�ȥ��,��ʹ��DMA��������
  {
		  DMA_Cmd(DMA1_Channel2, DISABLE);
		  DMA1_Channel2->CNDTR = port_send_len[1]; //-���������Ĵ���,ָʾʣ��Ĵ������ֽ���Ŀ
		  DMA_Cmd(DMA1_Channel2, ENABLE);
		  port_send_len[1] = 0;
	}
}

void RunLed_stata_judge_new(BYTE page,BYTE *data) //-Ϊ��ͬ���������ݵ�ָʾ��
{
  WORD temp_data;

  temp_data = (data[0] << 8) + data[1];

    if(page == 0)
    {
        pm_data = temp_data;
    }
    else
    {
      if(page == 1)
      {
          Noise_Value = temp_data;
      }
      else
      {
          if(page == 2)
          {
              VOC_data = temp_data;
          }
          else
          {
            if(page == 3)
            {
                co2_data = temp_data;
            }
            //-else
            //-{
//-
            //-}
        }
      }
    }

    RunLed_stata_judge();
}

////////////////////////////////////////////////////////////////////////////////
/*
����1:��ѯ�������豸��ID��
7620���ͣ�
0xaa 0x55 0x05 0x02 0x01 CRC
STM32��Ӧ��
0xaa 0x55 0x0b 0x02 0x01 0x01 0x02 0x03 0x04 0x05 0x06 CRC






*/
unsigned int MOD_TC_CRC16(unsigned char *MOD_KG_CRC16_start,unsigned char MOD_KG_CRC16_bytes)    //*xΪָ��ÿ��ǰ5�����ݵ�ָ��
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

void MOD_TC_clear_port_report_deal(void)
{
	 WORD i,len,temp_loop;

	 len = port_report[2] + 2;
	 temp_loop = port_recv_dl[1];

	 for(i=0;i<len;i++)
	 {
	 	  if(temp_loop > 0)
	 	  	temp_loop = temp_loop - 1;
	 	  else
	 	  	temp_loop = 511;
	    port_recv[1][temp_loop] = 0;
	 }


	 for(i=0;i<len;i++)
	   port_report[i] = 0;
}

void MOD_TC_LED_display_deal(void)		//-��7620�´�����ݽ��н���
{
	  //-WORD the_ram_ax;

	  //-�ر���һ���������洢����,�ñ�־λ,Ȼ��ˢ�µ�ʱ��������������Ƿ���Ч
	  //-����еĻ����л���ʾ

}

void MOD_TC_data_ack_deal(void)
{
	Device_communication_cn = 0;
}


void MOD_TC_SET_screen_deal(void)		//-��������ʱ��
{
	  //-WORD the_ram_ax;

	  if(port_report[4] == 0x01)		//-
	  {
	  	 /*led_display_long = port_report[5];	//-����ʱ��
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
			  port_deal_flag[1] = 0xaa;*/
			  //-�����ݷ���,�ж����Ƿ���ȷ�����ȷ�Ļ�,��7620����һ��ȷ�϶�֡�Ϳ���
			  //-0xaa 0x55 0x06 0x84 0xaa 0xaa XX XX
			  UART3_TO_UART2_FLAG = port_report[3] | 0x80;
	  }

}

void MOD_TC_receive_set_deal(void)		//-�Ų�ģʽ����
{
	  //-WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	  HL_flag	= 1;
		    HL_ld_R_user[0] = port_report[5];
		    HL_ld_G_user[0] = port_report[6];
		    HL_ld_B_user[0] = port_report[7];

			  UART3_TO_UART2_FLAG = port_report[3] | 0x80;
	  }

}

void MOD_TC_mode_set_deal(void)		//-�Ų�ģʽ����
{
	  //-WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	  HL_flag	= port_report[4];		//-�ѽ��յ���ģʽ���з���
			  UART3_TO_UART2_FLAG = port_report[3] | 0x80;
	  }

}

void MOD_TC_chage_pic_deal(void)		//-�Ų�ģʽ����
{
	  //-WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {

			  UART3_TO_UART2_FLAG = port_report[3] | 0x80;
	  }

}

void MOD_TC_SET_loop_deal(void)		//-�Ų�ģʽ����
{
	  //-WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {

			  UART3_TO_UART2_FLAG = port_report[3] | 0x80;
	  }

}

void MOD_TC_send_state_deal(void)		//-�Ų�ģʽ����
{
	  //-WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {

			  UART3_TO_UART2_FLAG = port_report[3] | 0x80;
	  }

}

void MOD_TC_re_Weather_deal(void)		//-�Ų�ģʽ����
{
	  //-WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {

			  UART3_TO_UART2_FLAG = port_report[3] | 0x80;
	  }

}

void MOD_TC_re_show_deal(void)		//-�Ų�ģʽ����
{
	  //-WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {

			  UART3_TO_UART2_FLAG = port_report[3] | 0x80;
	  }
	   //-test_cn[2]++;
     //-test_cn[4] = 3;
}

void MOD_TC_polling_state_deal(void)		//-��ѯ��ư��ڲ�����״̬,�����Լ�״ֵ̬
{
	  //-WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	  STM32_shedeng_selfT_flag1 = port_report[4];
			  UART3_TO_UART2_FLAG = port_report[3] | 0x80;
	  }

}

void MOD_TC_polling_PM25_deal(void)		//-���յ�����������Ȼ��,�ñ�־λ�Ϳ�����
{
	  //-WORD the_ram_ax;

	  FAN_RUN_flag = 1;		//-��ʼ���з���
	  FAN_RUN_wait_time = Time_2048ms_Counter;

    if((port_report[4] & 0x80) != 0)  //-�б��Ƿ�����,����Ҫ��������������
    {
	    UART3_TO_UART2_FLAG = 0x94;		//-���⴦��
	    UART1_led_disdata00 = (port_report[4] & 0x7f);
	    UART1_led_disdata01 = port_report[5];
	    UART1_led_disdata02 = port_report[6];

      RunLed_stata_judge_new(UART1_led_disdata00,&port_report[5]);
      UART1_sloop_flag = 1;		//-�Ƴ����ݿ����
      UART1_sloop_wait_time = cticks_ms;
		}

    HL_flag = port_report[7];

		if(HL_flag == 0)
		{
				RunLed_stata_num = (port_report[4] & 0x7f);
		  	HRL_RUN_flag = 0xaa;
	  }
    else
    {
    	  RunLed_stata_num = (port_report[4] & 0x7f);
        HRL_RUN_flag = 0;
    }
}

void MOD_TC_control_FUN_deal(void)		//-���յ�����������Ȼ��,�ñ�־λ�Ϳ�����
{
	  //-WORD the_ram_ax;

	  FAN_RUN_flag = 1;		//-��ʼ���з���
	  FAN_RUN_wait_time = Time_2048ms_Counter;

	  RunLed_stata_num = port_report[5];

		HRL_RUN_flag = 0;	//-�ոշ�ת����һҳ��ʱ��,���ܻ���ˢ��״̬,�����͸��ȹرջ���ָʾ��,
}

void MOD_TC_update_status_deal(void)		//-���յ�����������Ȼ��,�ñ�־λ�Ϳ�����
{
	  //-WORD the_ram_ax;

	  if(port_report[4] == 0x55)
	  {
	  	 if((port_report[5] == 0) || (port_report[5] == 1)  || (port_report[5] == 2) || (port_report[5] == 3))
	  	 {
	  	 	  RunLed_stata_num = port_report[5];
  	      HRL_RUN_flag = 0xaa;

  	      if(port_report[5] != 2)
  	      	FAN_RUN_flag = 1;
	  	 }
	  	 else
	  	 {
	  	 	  FAN_RUN_wait_flag = 1;		//-�رշ��ȿ�ʼ��ʱ
	  	    FAN_RUN_wait_time = Time_2048ms_Counter;
	  	 }
	  }
	  else
	  {
	  	 FAN_RUN_wait_flag = 1;		//-�رշ��ȿ�ʼ��ʱ
	  	 FAN_RUN_wait_time = Time_2048ms_Counter;
	  }

	  if(port_report[6] != 0)
	  	 HRL_RUN_flag = 0;			//-ÿ�α仯��Ҫ�����Ƿ�Ϩ�������

	  led_display_start = 0x55;

	  //-�����Ƿ񲥱�
	  if(port_report[9] == 0x55)
	  {
	  	 UART3_TO_UART2_FLAG = 0x94;
	     UART1_led_disdata00 = port_report[5];
	     UART1_led_disdata01 = port_report[7];
	     UART1_led_disdata02 = port_report[8];

       RunLed_stata_judge_new(UART1_led_disdata00,&port_report[7]);
       UART1_sloop_flag = 1;		//-�Ƴ����ݿ����
       UART1_sloop_wait_time = cticks_ms;
	  }
}

void MOD_TC_end_PM25_deal(void)		//-���յ�Ӧ��֮��Ͳ���,��������
{
	  //-WORD the_ram_ax;

	  //-UART1_transmit_flag = NO;		//-������֯���ݷ���
    //-UART1_transmit_control = 0;	//-�������������͵�����
    //-led_display_start = 0;

}

void MOD_TC_control_HRL_deal(void)		//-���յ�����������Ȼ��,�ñ�־λ�Ϳ�����
{
	  //-WORD the_ram_ax;

	  if(port_report[4] == 0x55)
	  	 HRL_RUN_flag = 0;	//-�Ųʵ�����,�ر������
	  //-else if(port_report[4] == 0xaa)		//-Ŀǰֻ��֤��ͬʱ��,�Ͳ�������.
	  //-	 HRL_RUN_flag = 0x55;		//-

	  UART3_TO_UART2_FLAG = port_report[3] | 0x80;

}

void MOD_TC_send_leddis_flag_deal(void)		//-��������ʾҳ�л��ᷢ������������
{
	  //-WORD the_ram_ax;

	  if(port_report[7] == 0x55)		//-������֯���ݷ���
	  {//-����0x55��ת�����칩��������
	    UART3_TO_UART2_FLAG = port_report[3] | 0x80;
	    UART1_led_disdata00 = port_report[4];
	    UART1_led_disdata01 = port_report[5];
	    UART1_led_disdata02 = port_report[6];

      //-�������������������ݵ�ʱ��,����ָʾ��,����Ҫ�������������任һ��
      RunLed_stata_judge_new(UART1_led_disdata00,&port_report[5]);
      UART1_sloop_flag = 1;		//-�Ƴ����ݿ����
      UART1_sloop_wait_time = cticks_ms;
  	}

    HL_flag = port_report[8];
  	RunLed_stata_num = port_report[4];
    if(((port_report[4] == 2) || (port_report[4] == 1)) && (HL_flag == 0))
      HRL_RUN_flag = 0xaa;		//-�����������Ҫ����Ҫ�ų���ʾ��
    else
	  	HRL_RUN_flag = 0;

	  if((port_report[4] != 0) || (port_report[4] != 2)	|| (port_report[4] != 3))
	  {
	  	 FAN_RUN_wait_time = Time_2048ms_Counter;
		   FAN_RUN_wait_flag = 1;
	  }

	  led_display_start = 0x55;
}

void MOD_TC_end_HRL_deal(void)		//-���յ�Ӧ��֮��Ͳ���,��������
{
	  //-WORD the_ram_ax;


  	//-RunLed_stata_num = port_report[4];
  	HRL_RUN_flag = 0;		//-������������Ϩ���ͻᷢ��������״ֵ̬,���ʱ��������������
  	UART3_TO_UART2_FLAG = port_report[3] | 0x80;

  	FAN_RUN_wait_time = Time_2048ms_Counter;
		FAN_RUN_wait_flag = 1;

		led_display_start = 0;

}

void MOD_TC_send_HRL_mode_deal(void)		//-�û��ᷢ�����ڵ��Ųʵ�ģʽ
{
	  //-WORD the_ram_ax;


  	if(port_report[5] !=0)
  		HRL_RUN_flag = 0;		//-ֻҪ�Ųʵ����žͲ����������
  	else
    {
      if(led_display_start == 0x55)
        HRL_RUN_flag = 0xaa;
    }

    HL_flag	= port_report[5];
    HL_ld_R_user[0] = port_report[6];
    HL_ld_G_user[0] = port_report[7];
    HL_ld_B_user[0] = port_report[8];

  	UART3_TO_UART2_FLAG = port_report[3] | 0x80;

}

void MOD_TC_polling_hl_deal(void)		//-���յ�Ӧ��֮��Ͳ���,��������
{
	  //-WORD the_ram_ax;

	  HL_flag = port_report[4];		//-������֯���ݷ���
    UART3_TO_UART2_FLAG = port_report[3] | 0x80;

}

void MOD_TC_SET_hl_time_deal(void)		//-���յ�Ӧ��֮��Ͳ���,��������
{
	  //-WORD the_ram_ax;

	  //-HL_flag = port_report[4];		//-������֯���ݷ���
    UART3_TO_UART2_FLAG = port_report[3] | 0x80;

}

void MOD_TC_SET_voice_flag_deal(void)		//-���յ�Ӧ��֮��Ͳ���,��������
{
	  //-WORD the_ram_ax;


    UART3_TO_UART2_FLAG = port_report[3] | 0x80;		//-���Ի�Ҳ���Բ���,ǰ���Ѿ������

}

void MOD_TC_polling_voice_flag_deal(void)		//-���յ�Ӧ��֮��Ͳ���,��������
{
	  //-WORD the_ram_ax;


    UART3_TO_UART2_FLAG = port_report[3] | 0x80;
	  voice_flag = port_report[4];
}

void MOD_TC_re_gateway_flag_deal(void)		//-���յ�Ӧ��֮��Ͳ���,��������
{
	  //-WORD the_ram_ax;


    UART3_TO_UART2_FLAG = port_report[3] | 0x80;
}

void MOD_TC_send_voice_mode_deal(void)		//-���յ�Ӧ��֮��Ͳ���,��������
{
	  //-WORD the_ram_ax;


    UART3_TO_UART2_FLAG = port_report[3] | 0x80;
}

void MOD_TC_set_hl_set_deal(void)		//-���յ�Ӧ��֮��Ͳ���,��������
{
	  //-WORD the_ram_ax;

		HL_ld_brightness = port_report[4];
    UART3_TO_UART2_FLAG = port_report[3] | 0x80;
}






//-���洦��ͨѶ���յ�������,��ʵ����һ���������ϲ�Э��Ĵ���,���������������ڵ���
void LDISP_Main()
{
	 WORD the_ram_ax,the_ram_bx,the_ram_cx,i;
	 WORD temp_data,temp_data1,delta_len,temp_loop,temp_int;


	 //-�жϷ����Ƿ������,��������˿��������߼�����,���򲻴���
	 if(DMA_GetFlagStatus(DMA1_FLAG_TC2))		//-����˵�����������
   {//-ֻ�е���������˲ſ������¸�DMA���ʹ�������
   	  for(temp_loop = 0;temp_loop < 43;temp_loop++)
   	  	UART3_port_send[temp_loop] = 0;
   	  port_deal_flag[1] = 0;		//-��һ����0ֵ�Ϳ������´μ�鷢��ʱ������������
   	  DMA_ClearFlag(DMA1_FLAG_TC2);
   }

	 if((port_deal_flag[1] == 0) && (UART3_TO_UART2_FLAG == 0))	//-����᲻������һ�η���ʧ�ܶ�������Ҳ���ܷ���,,����UART3_TO_UART2_FLAGΪ0��ʼ������յ��ı���,��ֹ,������û�з��ͳ�ȥ������,��ɵĶ�ʧ
   {
   	   //-���ȴ�����յ�������,���û�����ݽ��յ�,����֯������Ҫ���͵�����
   	   if(port_recv_pt[1]!=port_recv_dl[1])
   	   {

   	   	   if(MOD_TC_rxd_head_flag==NO)	//-���յ������ݻ�û�д����ʱ�����NO
           {
           	   MOD_TC_rxd_wait_time=cticks_ms;	//-������Ϊ�˽�������
		   	   	   temp_data = port_recv_pt[1];
		   	   	   temp_data1 = port_recv_dl[1];
		           if(temp_data1>temp_data)	//-ǰ����Ǵ���ָ���ʵ�ʵĽ���ָ����бȽ�
		               delta_len=(temp_data+512)-temp_data1;
		           else
		               delta_len=temp_data-temp_data1;	//-һ���ĳ���
		           for(temp_loop=temp_data1;temp_loop<(delta_len+temp_data1);temp_loop++)
		           {
		        	   if(port_recv[1][port_recv_dl[1]]==0xaa)	//-����ط��Ƚϵ��Ǵ�վ��ַ,�����Ҿ���û���κι��ɾ���ͨѶ
		        	   {	//-����һ�п������õ�
		        	     the_ram_ax=(port_recv_dl[1]+1)&0x1ff;
		        	     if(temp_data == the_ram_ax)	//-���Ԥȡ��ָ��ָ��յĿռ�,˵�����ݻ�û�е�,��Ҫ�ȴ�
		        	     	 break;
		        	     if(port_recv[1][the_ram_ax]==0x55)	//-�Ƚϵ��ǹ�����
		        	     {
		        	         MOD_TC_rxd_head_flag=YES;	//-��ʾ�Ѿ��ɹ�ʶ����յ����±��ĵ�ͷ��
		        	         break;
		        	     }
		        	   }
		        	   port_recv_dl[1]++;	//-����һ���ֵı���
		        	   port_recv_dl[1]&=0x1ff;
		           }
   	   	   }
   	   	   if(MOD_TC_rxd_head_flag==YES)	//-���յ������ݻ�û�д����ʱ�����NO
       		 {
       		 	   temp_data = port_recv_pt[1];
       		 	   if(port_recv_dl[1]>temp_data)
               		delta_len=(temp_data+512)-port_recv_dl[1];
               else
               		delta_len=temp_data-port_recv_dl[1];

               if(delta_len>6)	//-���ٻ���4���ֽڲ�����֯һ������
		           {
		               temp_int=(port_recv_dl[1]+2)&0x1ff;
		               if(delta_len>=(unsigned short)(port_recv[1][temp_int]+2))	//-�õ��ı��ĳ��Ⱥ������ϵı��ĳ��Ƚ��бȽ�
		               {
		                  MOD_TC_rxd_head_flag=NO;
		                  MOD_TC_rec_OK=YES;
                      goto rec_ok_deal;	//-�������ؿ���,���������Ϊ�ǳɹ����յ�һ�����ر�����
		               }

		           }

     			 }
   	   }
   	   goto rxd_out_time;		//?����������Ǽ򵥵�����ģʽ,����ֱ�Ӳ�ѯ����
rec_ok_deal:
	    if(MOD_TC_rec_OK==YES)	//-�϶��Ƿ�ֹ�ܷɵ�.
	    {	//-������Ϳ���˵��Ӧ�����Ѿ�������
	        MOD_TC_rec_OK=NO;	//-�ɹ����յ������ݿ�ʼ������֮��,�ͻָ�0
	        MOD_TC_transmit_flag=YES;
	        //-MOD_TC_transmit_wait_time=Time_1ms_Counter;	//-��Ȼ����˵���Է���������,���ǻ���Ҫ��ʱһ��ʱ��,��Ϊ���յ���������Ҫ����
	        //-�����Ƕ�ʵ�����ݵĴ���,������Ǻ��Ĳ���
	        the_ram_bx=(port_recv_dl[1]+3)&0x1ff;;
	        if(port_recv[1][the_ram_bx]!=0xFF)	//-����ǶԹ�������ж�,�����벻ͬ�жϵ�����Ҳ��һ��
	        {	//-����������һ�ִ������ڿ��Բ���
	          	the_ram_ax=(port_recv_dl[1]+2)&0x1ff;
	          	temp_int=port_recv[1][the_ram_ax]+2+port_recv_dl[1];
	          	for(temp_loop=port_recv_dl[1];temp_loop<temp_int;temp_loop++)	//-���������ɵ����ܾ��Ǳ�֤��λ����Ҫ����ı����ֽ�
	          	{	//-�򵥵Ĳ���Ҫ���������Ǹ��ӵĻ�����Ҫ��,��ô�������˵û������Ծͻ�ܺ�
	                 if(temp_loop<=511)
	           	       port_report[temp_loop-port_recv_dl[1]]=port_recv[1][temp_loop];
	                 else
	           	       port_report[temp_loop-port_recv_dl[1]]=port_recv[1][temp_loop-512];	//-�ѵ��Ǹ��ٸ��µ�Ե����Ҫ��ǰ���Ƴ���
	          	}	//-���߻���һ�ֿ����Ծ���ͳһ����
	         	//-port_recv_dl[1]+=delta_len;	//-����ط��������������Ĵ�����
            port_recv_dl[1]+=(port_report[2] + 2);
	         	port_recv_dl[1]&=0x1ff;
	         	temp_int=MOD_TC_CRC16(&port_report[0],port_report[2]);
	         	if((LOBYTE(temp_int)!=port_report[port_report[2]+0])||(HIBYTE(temp_int)!=port_report[port_report[2]+1]))	//-����CRC���
	         	{	//-������Ĳ�ȷ����,���У�鲻��ȷ�Ļ���ֱ�Ӱ�ͷ���,����Ѱ��
	          		goto inrda;	//-������˵���ɹ����յ��ı���CRCУ��û��ͨ��
	          }
	        }
	        else
	        {
	            //-port_recv_dl[1]+=delta_len;	//-Ŀǰֱ������
              port_recv_dl[1]+=7;
	            port_recv_dl[1]&=0x1ff;
	            goto inrda;
	        }


      //-��ʱ�ȸ�ֵ,��������������Դ����߼�
      //-��������ж��½��ձ���,Ȼ�������������
      //-test_cn[5]++;
    	//-��ȷ���ݵĴ���
////////////////////////////////////////////////////////////////////////////////
			if(port_report[3] == 0x81)	//-�жϹ�����
			{
					MOD_TC_data_ack_deal();
			}
			else
			{
				if(port_report[3] == 2)	//-�жϹ�����
	      {
	      	 if(port_report[4] == 1)
	      	 {
	      	 	   MOD_TC_rec_frame_type = MOD_KG_polling_ID_FRAME;
	      	 }
	      	 else
	      	 {
	      	 	   MOD_TC_rec_frame_type = MOD_KG_polling_data_FRAME;
	      	 }
	      }
	      else
	      {//-����LED������ͼƬ
	      	if(port_report[3] == 3)
	      	{
	      	  MOD_TC_rec_frame_type = MOD_KG_LED_display_FRAME;
	      	  //-����ҪӦ��
                  MOD_TC_LED_display_deal();
						    	//-MOD_TC_transmit_flag=NO;
	      	}
	      	else
	      	{
	      			if(port_report[3] == 0x84)
	      			{
	      					MOD_TC_rec_frame_type = MOD_KG_SET_screen_FRAME;
	      					//-
                  MOD_TC_SET_screen_deal();
	      			}
	      			else
	      			{
	      					if(port_report[3] == 0x85)
						      {//-���յ���������ʱ���,���ر���,�������,��Ҫ���ݸ�7620
						      	  MOD_TC_rec_frame_type = MOD_KG_receive_set_FRAME;
						      	  //-
                  		MOD_TC_receive_set_deal();
						      }
						      else
						      {
						      		if(port_report[3] == 0x86)		//-���յ��Ų�ģʽ����,�������õĶ���һ����ֵ��Ķ���,������Ҫ��֤���ͳɹ�
								      {
								      	  MOD_TC_rec_frame_type = MOD_KG_mode_set_FRAME;
								      	  //-
                  				MOD_TC_mode_set_deal();
								      }
								      else
								      {
								      	if(port_report[3] == 0x89)		//-���յ��Ų�ģʽ����,�������õĶ���һ����ֵ��Ķ���,������Ҫ��֤���ͳɹ�
									      {
									      	  MOD_TC_rec_frame_type = MOD_KG_chage_pic_FRAME;
									      	  MOD_TC_chage_pic_deal();
									      }
									      else
									      {
									      	if(port_report[3] == 0x8A)		//-���յ��Ų�ģʽ����,�������õĶ���һ����ֵ��Ķ���,������Ҫ��֤���ͳɹ�
										      {
										      	  MOD_TC_rec_frame_type = MOD_KG_SET_loop_FRAME;
										      	  MOD_TC_SET_loop_deal();
										      }
										      else
										      {
										      	if(port_report[3] == 0x8B)		//-���յ��Ų�ģʽ����,�������õĶ���һ����ֵ��Ķ���,������Ҫ��֤���ͳɹ�
											      {
											      	  MOD_TC_rec_frame_type = MOD_KG_send_state_FRAME;
											      	  MOD_TC_send_state_deal();
											      }
											      else
											      {
											      	if(port_report[3] == 0x8C)		//-���յ��Ų�ģʽ����,�������õĶ���һ����ֵ��Ķ���,������Ҫ��֤���ͳɹ�
												      {
												      	  MOD_TC_rec_frame_type = MOD_KG_re_Weather_FRAME;
												      	  MOD_TC_re_Weather_deal();
												      }
												      else
												      {
												      	if(port_report[3] == 0x8D)		//-���յ��Ų�ģʽ����,�������õĶ���һ����ֵ��Ķ���,������Ҫ��֤���ͳɹ�
													      {
													      	  MOD_TC_rec_frame_type = MOD_KG_re_show_FRAME;
													      	  MOD_TC_re_show_deal();
													      }
													      else
													      {
													      	if(port_report[3] == 0x8E)		//-���յ��Ų�ģʽ����,�������õĶ���һ����ֵ��Ķ���,������Ҫ��֤���ͳɹ�
														      {
														      	  MOD_TC_rec_frame_type = MOD_KG_polling_state_FRAME;
														      	  MOD_TC_polling_state_deal();
														      }
														      else
														      {
														      	if(port_report[3] == 0x0F)		//-���յ���ư巢�͹���������,����Ҫ��7620ת���Լ�֪������
															      {
															      	  MOD_TC_rec_frame_type = MOD_KG_polling_PM25_FRAME;
															      	  MOD_TC_polling_PM25_deal();
															      }
															      else
															      {
															      	if(port_report[3] == 0x10)		//-���յ���ư巢�͹���������,����Ҫ��7620ת���Լ�֪������
																      {
																      	  MOD_TC_rec_frame_type = MOD_KG_end_PM25_FRAME;
																      	  MOD_TC_end_PM25_deal();
																      }
																      else
																      {
																      	if(port_report[3] == 0x91)		//-�漰������Ӧ��Ķ���Ҫ�����λ��һ
																	      {
																	      	  MOD_TC_rec_frame_type = MOD_KG_polling_hl_FRAME;
																	      	  MOD_TC_polling_hl_deal();
																	      }
																	      else
																	      {
																	      	if(port_report[3] == 0x93)		//-���յ���ư巢�͹���������,����Ҫ��7620ת���Լ�֪������
																		      {
																		      	  MOD_TC_rec_frame_type = MOD_KG_SET_hl_time_FRAME;
																		      	  MOD_TC_SET_hl_time_deal();
																		      }
																		      else
																		      {
																		      	if(port_report[3] == 0x14)		//-�������ư��״η��͹���
																			      {
																			      	  MOD_TC_rec_frame_type = MOD_KG_send_leddis_flag_FRAME;
																			      	  MOD_TC_send_leddis_flag_deal();
																			      }
																			      else
																			      {
																			      	if(port_report[3] == 0x95)		//-���յ���ư巢�͹���������,����Ҫ��7620ת���Լ�֪������
																				      {
																				      	  MOD_TC_rec_frame_type = MOD_KG_SET_voice_flag_FRAME;
																				      	  MOD_TC_SET_voice_flag_deal();
																				      }
																				      else
																				      {
																				      	if(port_report[3] == 0x96)		//-���յ���ư巢�͹���������,����Ҫ��7620ת���Լ�֪������
																					      {
																					      	  MOD_TC_rec_frame_type = MOD_KG_polling_voice_flag_FRAME;
																					      	  MOD_TC_polling_voice_flag_deal();
																					      }
																					      else
																					      {
																					      	if(port_report[3] == 0x97)		//-���յ���ư巢�͹���������,����Ҫ��7620ת���Լ�֪������
																						      {
																						      	  MOD_TC_rec_frame_type = MOD_KG_re_gateway_flag_FRAME;
																						      	  MOD_TC_re_gateway_flag_deal();
																						      }
																						      else
																						      {
																						      	if(port_report[3] == 0x9B)		//-���յ���ư巢�͹���������,����Ҫ��7620ת���Լ�֪������
																							      {
																							      	  MOD_TC_rec_frame_type = MOD_KG_send_voice_mode_FRAME;
																							      	  MOD_TC_send_voice_mode_deal();
																							      }
																							      else
																							      {
																							      	if(port_report[3] == 0x40)		//-
																								      {
																								      	  MOD_TC_rec_frame_type = MOD_KG_control_HRL_FRAME;
																								      	  MOD_TC_control_HRL_deal();
																								      }
																								      else
																								      {
																								      		if(port_report[3] == 0x41)		//-
																										      {
																										      	  MOD_TC_rec_frame_type = MOD_KG_leddis_end_FRAME;
																										      	  MOD_TC_end_HRL_deal();
																										      }
																										      else
																										      {
																										      	if(port_report[3] == 0x42)		//-
																											      {
																											      	  MOD_TC_rec_frame_type = MOD_KG_send_HRL_mode_FRAME;
																											      	  MOD_TC_send_HRL_mode_deal();
																											      }
																											      else
																											      {
																											      	if(port_report[3] == 0x43)		//-
																												      {
																												      	  MOD_TC_rec_frame_type = MOD_KG_control_FUN_FRAME;
																												      	  MOD_TC_control_FUN_deal();
																												      }
																												      else
																												      {
																												      	if(port_report[3] == 0x44)		//-
																													      {
																													      	  MOD_TC_rec_frame_type = MOD_KG_update_status_FRAME;
																													      	  MOD_TC_update_status_deal();
																													      }
																													      else
																													      {
																													      	if(port_report[3] == 0xA1)		//-
																														      {
																														      	  MOD_TC_rec_frame_type = MOD_KG_set_hl_set_FRAME;
																														      	  MOD_TC_set_hl_set_deal();
																														      }
																														      else
																														      {
																														      	 if(port_report[3] == 0x9C)	//-��λ
																														      	 {
																														      	 		if((port_report[4] == 0x03) && (port_report[5] == 0x55))
																														      	 			 while(1);
																														      	 }
																														      	 else
      																															 		MOD_TC_rec_frame_type = 255;
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

    	   MOD_TC_clear_port_report_deal();
      }
rxd_out_time:	//-ִ�е�����˵�����ճ�ʱ,����ɹ�,�������ڿ��Լ���������,,�����������ִ�������,����
	    if(Judge_Time_In_MainLoop(MOD_TC_rxd_wait_time,MOD_KG_WAIT_TIME_VALUE)==YES)	//-����ͨ�ŵĻ��ǲ�Ӧ�ó�ʱ��,����ʱ����Ϊ����,�������³�ʼ��
	    {	//-������һ������֮��,�����ȴ��ش�ʱ��֮��,�Ϳ��Դ��·���һ��
	      	MOD_TC_rec_OK=NO;
	      	MOD_TC_rxd_head_flag=NO;
	      	MOD_TC_rxd_wait_time=cticks_ms;
	      	MOD_TC_transmit_flag=YES;	//-��ʾ���ڿ�����֯��������
	      	//-MOD_TC_wait_replay=NO;	//-��ʾ���ڻ�û�еȴ��ظ�
	      	//-MOD_TC_transmit_wait_time=Time_1ms_Counter;

	       	//-MOD_TC_comm_err_counter[port_send[0][0]]++;	//-��ʱ�������


	    }
inrda:
		   //-�������� ,,�������һ������,�೤ʱ�䷢��һ��,����˵������ʹ���������ʲô
		   if(MOD_TC_transmit_flag==YES)		//-Ŀǰ����ν����˫���
		   {

	        //-�������3,���͵�����ȫ�����Դ���2,�Լ�����֯�κη���
	        the_ram_ax = port_send_pt[1];
	        the_ram_bx = port_send_dl[1];
	        if(the_ram_ax != the_ram_bx)
	        {
	        	 if(the_ram_ax>=the_ram_bx)
	        	 		the_ram_cx = the_ram_ax-the_ram_bx;
	        	 else
	        	 	  the_ram_cx = 512+the_ram_ax-the_ram_bx;

	        	 for(i=0;i < the_ram_cx;i++)
	        	 {
	        	 	  the_ram_ax = (port_send_dl[1] + i) & 0x1ff;
	        	 		UART3_port_send[i] = port_send[1][the_ram_ax];		//-���շ������ݵĻ���,ǰ����ÿ�δӻ�������ȡ������,���ڷ���
	        	 		port_send[1][the_ram_ax] = 0;		//-Ϊ�˿ɿ�����,����ʧЧ����0
	        	 }

	        	 port_send_dl[1] = (port_send_dl[1] + the_ram_cx) & 0x1ff;
	        	 port_deal_flag[1] = 0xaa;    //-��֤���ڷ��ͽ���֮ǰ,�ٴ���֯����
	        	 port_send_len[1] = the_ram_cx;
             //-test_cn[1]++;    //-������

             //-if((test_cn[4] == 1) && (Judge_Time_In_MainLoop(test_cn_wait_time,2)==YES))
             //-{
             //-   test_cn[4] = 2;
             //-}
	        }

		   }
 	 }

}


//-ͬ�������������������������ʱ����Ҫ�����

//-���ڵ�����:������ȷ���յ���Ҳ������,����Ĵ������ȴ��ʧ�˱���,��Ϊû�н��յ�����
