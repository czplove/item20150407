/*
接收:使用一个常规逻辑,开辟一个512字节的缓冲区,然后循环存储
发送:也使用512字节的空间保存内容,然后置一个发送标志,直到内容发送出去为止
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
#define   MOD_KG_send_HRL_mode_FRAME		      	   0x1A			//-这个值比较特殊.主板和射灯板使用的不同
#define   MOD_KG_send_voice_mode_FRAME	      	   0x1B
#define   MOD_KG_send_es705_mode_FRAME	      	   0x1C
#define   MOD_KG_send_es705_event_type	      	   0x1E
#define   MOD_KG_send_HRL_ONOFF_FRAME 	      	   0x1F
#define   MOD_KG_send_inquire_state_FRAME 	   	   0x20
#define   MOD_KG_set_hl_set_FRAME	     	           0x21

extern void RunLed_stata_judge(void);
extern void RunLed_stata_judge_new(BYTE page,BYTE *data);

//-为协议处理临时定义的变量
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

  //-协议初始化内容
  port_deal_flag[0] = 0;
  MOD_KG_transmit_control = 0;	//-等待启动完成,然后开始发送周期报文

  UART0_transmit_flag = 0x55;
  UART0_start_tx_time = Time_2048ms_Counter;

  UART1_sloop_flag = 0;

}

void UART1_start(void)
{
  //-协议初始化内容
  MOD_KG_transmit_control = 1;	//-等待启动完成,然后开始发送周期报文

}


void UART_Rx_Deal(void)
{
  port_recv_pt[0] = 512 - DMA_GetCurrDataCounter(DMA1_Channel6);	//-得到接收个数,如果作为数组偏移量的话,现在的数值就是指向待存空间的
  if(port_recv_pt[0] >= 512)
  	port_recv_pt[0] = 0;
}

void UART_Tx_Deal(void)
{
  /* Enable USARTy DMA TX request */
  //-USART_DMACmd(USARTy, USART_DMAReq_Tx, ENABLE);			//-启动DMA发送

  if(port_send_len[0] != 0)	//-一定要保证内容发送出去了,再使用DMA发送数据
  {
		  DMA_Cmd(DMA1_Channel7, DISABLE);
		  DMA1_Channel7->CNDTR = port_send_len[0]; //-传输数量寄存器,指示剩余的待传输字节数目
		  DMA_Cmd(DMA1_Channel7, ENABLE);
      //-if(port_send_len[0] != 42)		//-测试用
      //-  port_send_len[0] = 0;   //- 04 13 8B 00 01 46 70总线上周期出现这样的数据,但是32并没有发送出去

      port_send_len[0] = 0;
	}
}


////////////////////////////////////////////////////////////////////////////////
/*
例子1:查询传感器设备（ID）
7620发送：
0xaa 0x55 0x05 0x02 0x01 CRC
STM32回应：
0xaa 0x55 0x0b 0x02 0x01 0x01 0x02 0x03 0x04 0x05 0x06 CRC






*/
unsigned int MOD_KG_CRC16(unsigned char *MOD_KG_CRC16_start,unsigned char MOD_KG_CRC16_bytes)    //*x为指向每行前5个数据的指针
{	//-计算冗余校验CRC
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

	 		//-使用的临时缓存清除
	 for(i=0;i<len;i++)
	   port_report[i] = 0;
}

void MOD_KG_polling_ID_cmd(void)
{
	  WORD the_ram_ax;

	  port_send[0][0] = 0xaa;		//-两字节包头
	  port_send[0][1] = 0x55;

	  port_send[0][2] = 0x05;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

	  port_send[0][3] = 0x02;		//-功能码:01h 传感器主动上报

	  //-有效数据
	  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
	  port_send[0][4] = 0x01;		//-数据长度

	  //-CRC16
	  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
	  port_send[0][5] =LOBYTE(the_ram_ax);
	  port_send[0][6] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

	  //-发送长度
	  port_send_len[0] = port_send[0][2] + 2;
	  port_deal_flag[0] = 0xaa;

	  cticks_500ms = 0;
}

void MOD_KG_send_sense_data(void)
{
	  WORD the_ram_ax,i;

	  port_send[0][0] = 0xaa;		//-两字节包头
	  port_send[0][1] = 0x55;

	  port_send[0][2] = 40;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

	  port_send[0][3] = 0x01;		//-功能码:01h 传感器主动上报

	  //-有效数据
	  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
	  port_send[0][4] = 0x0A;		//-数据长度		,,CO2
	  port_send[0][5] = 0xA0;		//-设备ID
	  port_send[0][6] = 0x04;		//-功能码
	  port_send[0][7] = 0x01;		//-数据描述
	  port_send[0][8] = HIBYTE(port_send_sense_data[1]);		//-数据位
	  port_send[0][9] = LOBYTE(port_send_sense_data[1]);

	  port_send[0][10] = 0x0A;		//-数据长度		,,温度
	  port_send[0][11] = 0xD1;		//-设备ID
	  port_send[0][12] = 0x04;		//-功能码
	  port_send[0][13] = 0x02;		//-数据描述
	  port_send[0][14] = HIBYTE(port_send_sense_data[2]);		//-数据位
	  port_send[0][15] = LOBYTE(port_send_sense_data[2]);

	  port_send[0][16] = 0x0A;		//-数据长度		,,VOC
	  port_send[0][17] = 0xD6;		//-设备ID
	  port_send[0][18] = 0x04;		//-功能码
	  port_send[0][19] = 0x02;		//-数据描述
	  port_send[0][20] = HIBYTE(port_send_sense_data[3]);		//-数据位
	  port_send[0][21] = LOBYTE(port_send_sense_data[3]);

	  port_send[0][22] = 0x0A;		//-数据长度		,,PM2.5
	  port_send[0][23] = 0xD5;		//-设备ID
	  port_send[0][24] = 0x04;		//-功能码
	  port_send[0][25] = 0x01;		//-数据描述
	  port_send[0][26] = HIBYTE(port_send_sense_data[4]);		//-数据位
	  port_send[0][27] = LOBYTE(port_send_sense_data[4]);

	  port_send[0][28] = 0x0A;		//-数据长度		,,噪声
	  port_send[0][29] = 0xD4;		//-设备ID
	  port_send[0][30] = 0x04;		//-功能码
	  port_send[0][31] = 0x01;		//-数据描述
	  port_send[0][32] = HIBYTE(port_send_sense_data[5]);		//-数据位
	  port_send[0][33] = LOBYTE(port_send_sense_data[5]);

	  port_send[0][34] = 0x01;		//-数据长度		,,预留
	  port_send[0][35] = 0x02;		//-设备ID
	  port_send[0][36] = HIBYTE(NTC_data);		//-功能码			//-光敏温度
	  port_send[0][37] = LOBYTE(NTC_data);		//-数据描述
	  port_send[0][38] = HIBYTE(port_send_sense_data[6]);		//-温度2
	  port_send[0][39] = LOBYTE(port_send_sense_data[6]);

	  //-CRC16
	  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
	  port_send[0][40] =LOBYTE(the_ram_ax);
	  port_send[0][41] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

	  if((Judge_LongTime_In_MainLoop(MOD_KG_Uto7620_wait_time,146) == YES) || (port_send_sense_data[0] == 0x55))		//-大约5分钟发一次
	  {
	  	MOD_KG_Uto7620_wait_time = Time_2048ms_Counter;
	  	port_send_sense_data[0] = 0;
		  //-发送长度,有启动发送的功能
		  port_send_len[0] = port_send[0][2] + 2;
		  port_deal_flag[0] = 0xaa;
		}

	  //-同时向射灯板转发,只要不覆盖没有发送出去的数据就可以
	  for(i=0;i<(port_send[0][2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_send[0][i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }

	  //-上送了数据显示屏才显示,这个时候才有必要判断切换显示状态
	  RunLed_stata_judge();
}

void MOD_KG_send_inquire_state(void)
{
	  WORD the_ram_ax;

	  port_send[0][0] = 0xaa;		//-两字节包头
	  port_send[0][1] = 0x55;

	  port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

	  port_send[0][3] = 0x20;		//-功能码:01h 传感器主动上报

	  //-有效数据
	  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
	  port_send[0][4] = 0x00;		//-数据长度		,,CO2
	  port_send[0][5] = 0x00;		//-设备ID

	  //-CRC16
	  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
	  port_send[0][6] =LOBYTE(the_ram_ax);
	  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

		  //-发送长度,有启动发送的功能
		  port_send_len[0] = port_send[0][2] + 2;
		  port_deal_flag[0] = 0xaa;

}

void MOD_KG_polling_ID_ack(void)		//-对7620查询的应答
{
	  WORD the_ram_ax;

	  port_send[0][0] = 0xaa;		//-两字节包头
	  port_send[0][1] = 0x55;

	  port_send[0][2] = 0x0b;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

	  port_send[0][3] = 0x02;		//-功能码:01h 传感器主动上报

	  //-有效数据
	  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
	  port_send[0][4] = 0x01;		//-数据长度
	  port_send[0][5] = 0xA0;
	  port_send[0][6] = 0xD4;
	  port_send[0][7] = 0xD5;
	  port_send[0][8] = 0xD6;
	  port_send[0][9] = 0x05;
	  port_send[0][10] = 0x06;

	  //-CRC16
	  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
	  port_send[0][11] =LOBYTE(the_ram_ax);
	  port_send[0][12] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

	  //-发送长度
	  port_send_len[0] = port_send[0][2] + 2;
	  port_deal_flag[0] = 0xaa;
}

//-rece:0xaa 0x55 0x05 0x02 0x02 CRC
//-send:0xaa 0x55 0x05 0x02 0x02 数据 CRC
void MOD_KG_polling_data_ack(void)		//-对7620查询的应答
{
	  WORD the_ram_ax;

	  port_send[0][0] = 0xaa;		//-两字节包头
	  port_send[0][1] = 0x55;

	  port_send[0][2] = 40;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

	  port_send[0][3] = 0x01;		//-功能码:01h 传感器主动上报

	  //-有效数据
	  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
	  port_send[0][4] = 0x0A;		//-数据长度		,,CO2
	  port_send[0][5] = 0xA0;		//-设备ID
	  port_send[0][6] = 0x04;		//-功能码
	  port_send[0][7] = 0x01;		//-数据描述
	  port_send[0][8] = HIBYTE(port_send_sense_data[1]);		//-数据位
	  port_send[0][9] = LOBYTE(port_send_sense_data[1]);

	  port_send[0][10] = 0x0A;		//-数据长度		,,温度
	  port_send[0][11] = 0xD1;		//-设备ID
	  port_send[0][12] = 0x04;		//-功能码
	  port_send[0][13] = 0x02;		//-数据描述
	  port_send[0][14] = HIBYTE(port_send_sense_data[2]);		//-数据位
	  port_send[0][15] = LOBYTE(port_send_sense_data[2]);

	  port_send[0][16] = 0x0A;		//-数据长度		,,VOC
	  port_send[0][17] = 0xD6;		//-设备ID
	  port_send[0][18] = 0x04;		//-功能码
	  port_send[0][19] = 0x02;		//-数据描述
	  port_send[0][20] = HIBYTE(port_send_sense_data[3]);		//-数据位
	  port_send[0][21] = LOBYTE(port_send_sense_data[3]);

	  port_send[0][22] = 0x0A;		//-数据长度		,,PM2.5
	  port_send[0][23] = 0xD5;		//-设备ID
	  port_send[0][24] = 0x04;		//-功能码
	  port_send[0][25] = 0x01;		//-数据描述
	  port_send[0][26] = HIBYTE(port_send_sense_data[4]);		//-数据位
	  port_send[0][27] = LOBYTE(port_send_sense_data[4]);

	  port_send[0][28] = 0x0A;		//-数据长度		,,噪声
	  port_send[0][29] = 0xD4;		//-设备ID
	  port_send[0][30] = 0x04;		//-功能码
	  port_send[0][31] = 0x01;		//-数据描述
	  port_send[0][32] = HIBYTE(port_send_sense_data[5]);		//-数据位
	  port_send[0][33] = LOBYTE(port_send_sense_data[5]);

	  port_send[0][34] = 0x01;		//-数据长度		,,预留 暂用温度2
	  port_send[0][35] = 0x02;		//-设备ID
	  port_send[0][36] = 0x03;		//-功能码
	  port_send[0][37] = 0x04;		//-数据描述
	  port_send[0][38] = HIBYTE(port_send_sense_data[6]);		//-数据位
	  port_send[0][39] = LOBYTE(port_send_sense_data[6]);

	  //-CRC16
	  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
	  port_send[0][40] =LOBYTE(the_ram_ax);
	  port_send[0][41] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

	  //-发送长度
	  port_send_len[0] = port_send[0][2] + 2;
	  port_deal_flag[0] = 0xaa;
}

void MOD_KG_LED_display_deal(void)		//-对7620下达的数据进行接收
{
	  //-WORD the_ram_ax;

	  //-特别定义一个缓冲区存储数据,置标志位,然后刷新的时候检查这里的数据是否有效
	  //-如果有的话就切换显示

}

void MOD_KG_SET_screen_deal_ack(void)		//-设置屏保时间
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-屏保时间
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = 0xaa;		//-数据长度
			  port_send[0][5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }



}

void MOD_KG_receive_set_deal_ack(void)		//-设置炫彩灯颜色返回
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-屏保时间
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 8;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = 1;		//-数据长度
			  port_send[0][5] = HL_ld_R_user[0];
			  port_send[0][6] = HL_ld_G_user[0];
			  port_send[0][7] = HL_ld_B_user[0];


			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][8] =LOBYTE(the_ram_ax);
			  port_send[0][9] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }



}

void MOD_KG_mode_set_deal_ack(void)		//-设置屏保时间
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-屏保时间
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = HL_flag;		//-数据长度
			  port_send[0][5] = 0xaa;

        if(HL_flag == 3)  //-测试用
			 		HL_flag = 3;
			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }



}

void MOD_KG_chage_pic_deal_ack(void)		//-设置屏保时间
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-屏保时间
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = 0xaa;		//-数据长度
			  port_send[0][5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }



}

void MOD_KG_SET_loop_deal_ack(void)		//-设置屏保时间
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-屏保时间
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = 0xaa;		//-数据长度
			  port_send[0][5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }



}

void MOD_KG_send_state_deal_ack(void)		//-设置屏保时间
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-屏保时间
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = 0xaa;		//-数据长度
			  port_send[0][5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }



}

void MOD_KG_re_Weather_deal_ack(void)		//-设置屏保时间
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-屏保时间
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = 0xaa;		//-数据长度
			  port_send[0][5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }



}

void MOD_KG_re_show_deal_ack(void)		//-设置屏保时间
{
	  WORD the_ram_ax;
	  static BYTE i=0;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-屏保时间
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = 0xaa;		//-数据长度
			  port_send[0][5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
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

void MOD_KG_polling_state_deal_ack(void)		//-设置屏保时间
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-屏保时间
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = STM32_shedeng_selfT_flag1;		//-数据长度
			  port_send[0][5] = 0;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }



}

void MOD_KG_polling_hl_deal_ack(void)		//-设置屏保时间
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-屏保时间
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = HL_flag;		//-数据长度
			  port_send[0][5] = 0x00;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }



}

void MOD_KG_SET_hl_time_deal_ack(void)		//-设置屏保时间
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-屏保时间
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = 0xaa;		//-数据长度
			  port_send[0][5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }



}

void MOD_KG_control_HRL_deal_ack(void)		//-这里组织应答内容仅仅是为了控制串口发送,不对7620有数据
{
	  WORD the_ram_ax,i;
	  BYTE	temp_report[32];

	  //-if(port_report[4] == 0x01)		//-
	  {


       //-如果数据成功接收到之后就应答一次
			  temp_report[0] = 0xaa;		//-两字节包头
			  temp_report[1] = 0x55;

			  temp_report[2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  temp_report[3] = 0xC0;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  temp_report[4] = 0xaa;		//-数据长度
			  temp_report[5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&temp_report[0],temp_report[2]);
			  temp_report[6] =LOBYTE(the_ram_ax);
			  temp_report[7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  for(i=0;i<(temp_report[2] + 2);i++)
			  {
			  	port_send[1][port_send_pt[1]] = temp_report[i];
			  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
			  }

	  }



}

void MOD_KG_leddis_end_ack(void)		//-这里组织应答内容仅仅是为了控制串口发送,不对7620有数据
{
	  WORD the_ram_ax,i;
	  BYTE	temp_report[32];

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-屏保时间
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-如果数据成功接收到之后就应答一次
			  temp_report[0] = 0xaa;		//-两字节包头
			  temp_report[1] = 0x55;

			  temp_report[2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  temp_report[3] = 0xC1;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  temp_report[4] = 0xaa;		//-数据长度
			  temp_report[5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&temp_report[0],temp_report[2]);
			  temp_report[6] =LOBYTE(the_ram_ax);
			  temp_report[7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  for(i=0;i<(temp_report[2] + 2);i++)
			  {
			  	port_send[1][port_send_pt[1]] = temp_report[i];
			  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
			  }

	  }



}

void MOD_KG_send_HRL_mode_ack(void)		//-这里组织应答内容仅仅是为了控制串口发送,不对7620有数据
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-屏保时间
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 8;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = 0x1A;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = HL_flag;		//-数据长度
			  port_send[0][5] = HL_ld_R_user[0];
			  port_send[0][6] = HL_ld_G_user[0];		//-数据长度
			  port_send[0][7] = HL_ld_B_user[0];

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][8] =LOBYTE(the_ram_ax);
			  port_send[0][9] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;

	  }



}

void MOD_KG_send_leddis_flag_ack(void)		//-
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-屏保时间
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 7;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = 0x14;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = UART1_led_disdata00;		//-数据长度
			  port_send[0][5] = UART1_led_disdata01;
			  port_send[0][6] = UART1_led_disdata02;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][7] =LOBYTE(the_ram_ax);
			  port_send[0][8] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;

	  }

//-				test_cn[6]++;		//-测试用

}

void MOD_KG_SET_voice_flag_ack(void)		//-设置屏保时间
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-屏保时间
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = 0xaa;		//-数据长度
			  port_send[0][5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }



}

void MOD_KG_polling_voice_flag_ack(void)		//-设置屏保时间
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-屏保时间
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = voice_flag;		//-数据长度
			  port_send[0][5] = 0;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }



}

void MOD_KG_re_gateway_flag_ack(void)		//-设置屏保时间
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-屏保时间
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = 0xaa;		//-数据长度
			  port_send[0][5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }



}

void MOD_KG_send_voice_mode_ack(void)		//-设置屏保时间
{
	  WORD the_ram_ax;
	  //-static BYTE i;		//?同一个函数里面不能使用几个一样的局部变量命名

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-屏保时间
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = 0xaa;		//-数据长度
			  port_send[0][5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
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

void MOD_KG_set_hl_set_ack(void)		//-设置屏保时间
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-屏保时间
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = UART3_TO_UART2_FLAG;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = HL_ld_brightness;		//-数据长度
			  port_send[0][5] = 0x00;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
	  }



}

void MOD_KG_polling_PM25_ack(void)		//-这里组织应答内容仅仅是为了控制串口发送,不对7620有数据
{
	  WORD the_ram_ax,i;
	  BYTE	temp_report[32];

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-屏保时间
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-如果数据成功接收到之后就应答一次
			  temp_report[0] = 0xaa;		//-两字节包头
			  temp_report[1] = 0x55;

			  temp_report[2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  temp_report[3] = 0x8F;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  temp_report[4] = 0xaa;		//-数据长度
			  temp_report[5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&temp_report[0],temp_report[2]);
			  temp_report[6] =LOBYTE(the_ram_ax);
			  temp_report[7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  for(i=0;i<(temp_report[2] + 2);i++)
			  {
			  	port_send[1][port_send_pt[1]] = temp_report[i];
			  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
			  }

	  }



}



void MOD_KG_receive_set_deal(void)		//-用户设置灯的颜色,目前支持设置四种颜色
{
	  WORD i;


	  for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }

    HRL_RUN_flag = 0;

}

void MOD_KG_mode_set_deal(void)		//-炫彩模式设置
{
	  WORD i;


	  for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;

      //-if(port_report[4] == 2)   //-测试用
      //-  port_report[4] = 2;
      //-if(port_report[4] == 3)   //-测试用
      //-  port_report[4] = 3;
      //-if(port_report[4] == 4)   //-测试用
      //-  port_report[4] = 4;
      //-if(port_report[4] == 5)   //-测试用
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

void MOD_KG_chage_page_deal(void)		//-翻页
{
	  WORD i;


	  for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }

}

void MOD_KG_SET_screen_deal(void)		//-设置屏保时间
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }

}

void MOD_KG_chage_pic_deal(void)		//-设置屏保时间
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }

}

void MOD_KG_SET_loop_deal(void)		//-设置屏保时间
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }

}

void MOD_KG_send_state_deal(void)		//-设置屏保时间
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }

}

void MOD_KG_re_Weather_deal(void)		//-设置屏保时间
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }

}

void MOD_KG_re_show_deal(void)		//-设置屏保时间
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }
    //-test_cn[0]++;    //-测试计数用
    //-test_cn_wait_time = cticks_ms;
	  //-test_cn[4] = 1;

    led_display_start = 0x55;
    if(HL_flag == 0)
      HRL_RUN_flag = 0xaa;

    RunLed_stata_num = port_report[4];

    RunLed_stata_judge_new(RunLed_stata_num,&port_report[5]);
}

void MOD_KG_polling_state_deal(void)		//-设置屏保时间
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }

}

void MOD_KG_polling_PM25_deal(void)		//-设置屏保时间
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }

}

void MOD_KG_end_PM25_deal(void)		//-设置屏保时间
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
	  	 //-led_display_long = port_report[5];	//-屏保时间
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = 0x92;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = STM32_UP_selfT_flag1;		//-数据长度
			  port_send[0][5] = 0x00;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
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

void MOD_KG_SET_voice_flag_deal(void)		//-把接收到的数据进行透明传输
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }
}

void MOD_KG_polling_voice_flag_deal(void)		//-把接收到的数据进行透明传输
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }
}

void MOD_KG_re_gateway_flag_deal(void)		//-把接收到的数据进行透明传输
{
	  WORD i;


    HRL_RUN_flag = 0;		//-网关启动完成熄灭跑马灯
    UART0_transmit_flag = 0;
    HRL_RUN_ONOFF = port_report[4];

    MOD_KG_transmit_control = 1;	//-网关启动完成,开始心跳报文的传输
    MOD_KG_transmit_wait_time=cticks_ms;  //-一个不是发送心跳而是延时发送启动应答
    //-如果数据成功接收到之后就应答一次
			  /*port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = 0x97;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = 0xaa;		//-数据长度
			  port_send[0][5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;*/

			  for(i=0;i<(port_report[2] + 2);i++)		//-把这个进程还需要转发给射灯板
			  {
			  	port_send[1][port_send_pt[1]] = port_report[i];
			  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
			  }
}

void MOD_KG_re_ZigBee_flag_deal(void)		//-把接收到的数据进行透明传输
{
	  WORD the_ram_ax;


    if(port_report[4] == 1)
    {//-zigbee允许加网
    	 zigbee_flag = 1;
    	 zigbee_wait_time = Time_2048ms_Counter;		//-允许计时开始
    }
    else if(port_report[4] == 0)
    {//-zigbee禁止加网
    	 zigbee_flag = 0;
    }

    //-HRL_RUN_flag = 0;		//-

    //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = port_report[3] | 0x80;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = port_report[4];		//-数据长度
			  port_send[0][5] = 0;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
}


void MOD_KG_send_HRL_mode_deal(void)		//-把接收到的数据进行透明传输
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }
}

void MOD_KG_send_voice_mode_deal(void)		//-把接收到的数据进行透明传输
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }
}

void MOD_KG_send_es705_mode_deal(void)		//-把接收到的数据进行透明传输
{
	  WORD i;

		/*
	  if(es705_mode == 0)
    {
      if(port_report[4] == 1)
      {//-zigbee允许加网
         es705_mode_file = port_report[5];
         f_rec = 1;
         es705_mode = 1;
         es705_training_count = 0;
      }
      else if(port_report[4] == 2)
      {//-zigbee禁止加网
         f_rec = 1;
         es705_mode = 2;
         es705_training_count = 0;
      }
      else
      {
         return;
      }


    //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = port_report[3] | 0x80;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = port_report[4];		//-数据长度
			  port_send[0][5] = port_report[5];

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
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

            port_send[0][0] = 0xaa;		//-两字节包头
            port_send[0][1] = 0x55;

            port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

            port_send[0][3] = port_report[3] | 0x80;		//-功能码:01h 传感器主动上报

            //-有效数据
            //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
            port_send[0][4] = port_report[4];		//-数据长度
            port_send[0][5] = port_report[5];

            //-CRC16
            the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
            port_send[0][6] =LOBYTE(the_ram_ax);
            port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

            //-发送长度
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

//-STM32主动上送语音信息
void MOD_KG_send_es705_deal(void)
{
	   WORD the_ram_ax,i;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 //-led_display_long = port_report[5];	//-屏保时间
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 7;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = 0x1d;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = es705_training_status;		//-数据长度
			  port_send[0][5] = es705_training_count;
        port_send[0][6] = es705_mode_file;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][7] =LOBYTE(the_ram_ax);
			  port_send[0][8] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
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
	  	 //-led_display_long = port_report[5];	//-屏保时间
	  	 //-cticks_s_page = 0;
       //-led_display_start = 0x55;

       //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = 0x1e;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = es705_event_type >> 8;		//-数据长度
			  port_send[0][5] = es705_event_type & 0xFF;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
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

void MOD_KG_send_HRL_ONOFF_ack(void)		//-把接收到的数据进行透明传输
{
	  WORD the_ram_ax;


    HRL_RUN_ONOFF = port_report[4];


    //-HRL_RUN_flag = 0;		//-

    //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = port_report[3] | 0x80;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = HRL_RUN_ONOFF;		//-数据长度
			  port_send[0][5] = 0;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag[0] = 0xaa;
}

void MOD_KG_send_inquire_state_ack(void)		//-把接收到的数据进行透明传输
{
	  //-WORD the_ram_ax;

	  HRL_RUN_flag = 0;		//-网关启动完成熄灭跑马灯
    UART0_transmit_flag = 0;
    MOD_KG_transmit_control = 1;
}

void MOD_KG_set_hl_set_deal(void)		//-把接收到的数据进行透明传输
{
	  WORD i;


    for(i=0;i<(port_report[2] + 2);i++)
	  {
	  	port_send[1][port_send_pt[1]] = port_report[i];
	  	port_send_pt[1] = (port_send_pt[1] + 1) & 0x1ff;
	  }
}







//-下面处理通讯接收到的内容,其实就是一个单独的上层协议的处理,在主函数里面周期调用
//-这里有一个重要的作用是传递信息
void CDT9702_Main()
{
	 WORD the_ram_ax,the_ram_bx;
	 WORD temp_data,temp_data1,delta_len,temp_loop,temp_int;


	 //-判断发送是否结束了,如果结束了可以正常逻辑处理,否则不处理
	 if(DMA_GetFlagStatus(DMA1_FLAG_TC7))		//-进入说明发送完成了
   {//-只有当传输完成了才可以重新给DMA发送传输命令
   	  for(temp_loop = 0;temp_loop < 43;temp_loop++)
   	  	port_send[0][temp_loop] = 0;
   	  port_deal_flag[0] = 0;		//-给一个非0值就可以再下次检查发送时发发送命令了
   	  //-MOD_KG_transmit_flag=YES;
   	  DMA_ClearFlag(DMA1_FLAG_TC7);
   }

	 if(port_deal_flag[0] == 0)	//-这里会不会由于一次发送失败而导致再也不能发送
   {
   	   //-首先处理接收到的数据,如果没有内容接收到,才组织可能需要发送的内容
   	   if((port_recv_pt[0]!=port_recv_dl[0]) && (MOD_KG_transmit_flag == YES))
   	   {

   	   	   if(MOD_KG_rxd_head_flag==NO)	//-接收到的数据还没有处理的时候就是NO
           {
           	   MOD_KG_rxd_wait_time=cticks_ms;		//-后加的有待测试
		   	   	   temp_data = port_recv_pt[0];
		   	   	   temp_data1 = port_recv_dl[0];
		           if(temp_data1>temp_data)	//-前面的是处理指针比实际的接收指针进行比较
		               delta_len=(temp_data+512)-temp_data1;
		           else
		               delta_len=temp_data-temp_data1;	//-一共的长度
		           for(temp_loop=temp_data1;temp_loop<(delta_len+temp_data1);temp_loop++)
		           {
		        	   if(port_recv[0][port_recv_dl[0]]==0xaa)	//-这个地方比较的是从站地址,但是我觉得没有任何规律就是通讯
		        	   {	//-利用一切可以利用的
		        	     the_ram_ax=(port_recv_dl[0]+1)&0x1ff;
		        	     if(temp_data == the_ram_ax)
		        	     	 break;
		        	     if(port_recv[0][the_ram_ax]==0x55)	//-比较的是功能码
		        	     {
		        	         MOD_KG_rxd_head_flag=YES;	//-表示已经成功识别接收到的新报文的头了
		        	         break;
		        	     }
		        	   }
		        	   port_recv_dl[0]++;	//-舍弃一个字的报文
		        	   port_recv_dl[0]&=0x1ff;
		           }
   	   	   }
   	   	   if(MOD_KG_rxd_head_flag==YES)	//-接收到的数据还没有处理的时候就是NO
       		 {
       		 	   temp_data = port_recv_pt[0];
       		 	   if(port_recv_dl[0]>temp_data)
               		delta_len=(temp_data+512)-port_recv_dl[0];
               else
               		delta_len=temp_data-port_recv_dl[0];

               if(delta_len>6)	//-至少还有4个字节才能组织一包内容
		           {
		               temp_int=(port_recv_dl[0]+2)&0x1ff;
		               if(delta_len>=(unsigned short)(port_recv[0][temp_int]+2))	//-得到的报文长度和理论上的报文长度进行比较
		               {
		                  MOD_KG_rxd_head_flag=NO;
		                  MOD_KG_rec_OK=YES;
                      goto rec_ok_deal;	//-经过重重考核,到这里就认为是成功接收到一个返回报文了
		               }

		           }

     			 }
   	   }
   	   goto rxd_out_time;		//?由于这个不是简单的主从模式,所以直接查询发送
rec_ok_deal:
	    if(MOD_KG_rec_OK==YES)	//-肯定是防止跑飞的.
	    {	//-到这里就可以说明应答报文已经可以了
	        MOD_KG_rec_OK=NO;	//-成功接收到的数据开始处理了之后,就恢复0
	        MOD_KG_transmit_flag=YES;
	        test_cn_wait_time = cticks_ms;	//-测试用
	        //-MOD_KG_transmit_wait_time=Time_1ms_Counter;	//-虽然上面说可以发送数据了,但是还是要延时一段时间,因为接收到的数据需要处理
	        //-下面是对实际数据的处理,处理的是核心部分
	        the_ram_bx=(port_recv_dl[0]+3)&0x1ff;;
	        if(port_recv[0][the_ram_bx]!=0xFF)	//-这个是对功能码的判断,功能码不同判断的依据也不一样
	        {	//-这里是宁外一种处理现在可以不管
	          	the_ram_ax=(port_recv_dl[0]+2)&0x1ff;
	          	temp_int=port_recv[0][the_ram_ax]+2+port_recv_dl[0];
	          	for(temp_loop=port_recv_dl[0];temp_loop<temp_int;temp_loop++)	//-上面这样干的秘密就是保证定位到需要处理的报文字节
	          	{	//-简单的不需要这样处理但是复杂的还是需要的,那么这样用了得话兼容性就会很好
	                 if(temp_loop<=511)
	           	       port_report[temp_loop-port_recv_dl[0]]=port_recv[0][temp_loop];
	                 else
	           	       port_report[temp_loop-port_recv_dl[0]]=port_recv[0][temp_loop-512];	//-难道是高速更新的缘故需要提前复制出来
	          	}	//-或者还有一种可能性就是统一处理
	         	//-port_recv_dl[0]+=delta_len;	//-这个地方就舍弃了这样的处理报文
            port_recv_dl[0]+=(port_report[2] + 2);
	         	port_recv_dl[0]&=0x1ff;
	         	temp_int=MOD_KG_CRC16(&port_report[0],port_report[2]);
	         	if((LOBYTE(temp_int)!=port_report[port_report[2]+0])||(HIBYTE(temp_int)!=port_report[port_report[2]+1]))	//-进行CRC检查
	         	{	//-由于这的不确定性,如果校验不正确的话就直接把头舍掉,继续寻找
	          		goto inrda;	//-到这里说明成功接收到的报文CRC校验没有通过
	          }
	        }
	        else
	        {
	            //-port_recv_dl[0]+=delta_len;	//-到这里就可以把接收缓冲区中的内容舍弃了,已经拷贝出来了
              port_recv_dl[0]+=7;
	            port_recv_dl[0]&=0x1ff;
	            goto inrda;
	        }




      //-临时先赋值,后面这个变量可以处理逻辑
      //-可以组合判断下接收报文,然后决定下面走向

////////////////////////////////////////////////////////////////////////////////
				if(port_report[3] == 2)	//-判断功能码
	      {
	      	 if(port_report[4] == 1)
	      	 {
	      	 	   MOD_KG_rec_frame_type = MOD_KG_polling_ID_FRAME;
	      	 	   //-简单的东西不搞那么复杂,直接发送
                  MOD_KG_polling_ID_ack();
                  MOD_KG_transmit_wait_time=cticks_ms;  //-一但有内容通讯了,那么周期上送延迟2S
						    	MOD_KG_transmit_flag=NO;
	      	 }
	      	 else
	      	 {
	      	 	   MOD_KG_rec_frame_type = MOD_KG_polling_data_FRAME;
	      	 	   //-简单的东西不搞那么复杂,直接发送
                  MOD_KG_polling_data_ack();
                  MOD_KG_transmit_wait_time=cticks_ms;
						    	MOD_KG_transmit_flag=NO;
	      	 }
	      }
	      else
	      {//-接收LED点阵屏图片
	      	 if(port_report[3] == 3)
	      	 {
	      	  	MOD_KG_rec_frame_type = MOD_KG_LED_display_FRAME;
	      	  	//-不需要应答
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
		      	 		if(port_report[3] == 5)		//-接收的炫彩灯设置,这里设置的都是一个定值类的东西,所以需要保证发送成功
					      {
					      	  MOD_KG_rec_frame_type = MOD_KG_receive_set_FRAME;
					      	  MOD_KG_receive_set_deal();
	                  MOD_KG_transmit_wait_time=cticks_ms;
					      }
					      else
					      {
					      	if(port_report[3] == 6)		//-接收的炫彩模式设置,这里设置的都是一个定值类的东西,所以需要保证发送成功
						      {
						      	  MOD_KG_rec_frame_type = MOD_KG_mode_set_FRAME;
						      	  //-7620设置炫彩灯模式,此处接收到之后,不需要处理,而是直接通过另一个串口透传出去
	                  MOD_KG_mode_set_deal();
	                  MOD_KG_transmit_wait_time=cticks_ms;
						      }
						      else
						      {
						      	if(port_report[3] == 8)		//-接收的炫彩模式设置,这里设置的都是一个定值类的东西,所以需要保证发送成功
							      {
							      	  MOD_KG_rec_frame_type = MOD_KG_chage_page_FRAME;
							      	  MOD_KG_chage_page_deal();
			          	      //-MOD_KG_transmit_flag=NO;
			          	      MOD_KG_transmit_wait_time=cticks_ms;
							      }
							      else
							      {
							      	if(port_report[3] == 9)		//-接收的炫彩模式设置,这里设置的都是一个定值类的东西,所以需要保证发送成功
								      {
								      	  MOD_KG_rec_frame_type = MOD_KG_chage_pic_FRAME;
								      	  MOD_KG_chage_pic_deal();
          	      				MOD_KG_transmit_wait_time=cticks_ms;
								      }
								      else
								      {
								      	if(port_report[3] == 0x0A)		//-接收的炫彩模式设置,这里设置的都是一个定值类的东西,所以需要保证发送成功
									      {
									      	  MOD_KG_rec_frame_type = MOD_KG_SET_loop_FRAME;
									      	  MOD_KG_SET_loop_deal();
          	     						MOD_KG_transmit_wait_time=cticks_ms;
									      }
									      else
									      {
									      	if(port_report[3] == 0x0B)		//-接收的炫彩模式设置,这里设置的都是一个定值类的东西,所以需要保证发送成功
										      {
										      	  MOD_KG_rec_frame_type = MOD_KG_send_state_FRAME;
										      	  MOD_KG_send_state_deal();
          	      						MOD_KG_transmit_wait_time=cticks_ms;
										      }
										      else
										      {
										      	if(port_report[3] == 0x0C)		//-接收的炫彩模式设置,这里设置的都是一个定值类的东西,所以需要保证发送成功
											      {
											      	  MOD_KG_rec_frame_type = MOD_KG_re_Weather_FRAME;
											      	  MOD_KG_re_Weather_deal();
          	      							MOD_KG_transmit_wait_time=cticks_ms;
											      }
											      else
											      {
											      	if(port_report[3] == 0x0D)		//-指定显示哪一页传感器数据
												      {
												      	  MOD_KG_rec_frame_type = MOD_KG_re_show_FRAME;
												      	  MOD_KG_re_show_deal();
								          	      MOD_KG_transmit_wait_time=cticks_ms;  //-针对播报的情况,周期数据多延迟15S
								                  UART1_sloop_flag = 1;
								                  UART1_sloop_wait_time = cticks_ms;
												      }
												      else
												      {
												      	if(port_report[3] == 0x0E)		//-指定显示哪一页传感器数据
													      {
													      	  MOD_KG_rec_frame_type = MOD_KG_polling_state_FRAME;
													      	  MOD_KG_polling_state_deal();
          	      									MOD_KG_transmit_wait_time=cticks_ms;
													      }
													      else
													      {
													      	if(port_report[3] == 0x0F)		//?指定显示哪一页传感器数据
														      {
														      	  MOD_KG_rec_frame_type = MOD_KG_polling_PM25_FRAME;
														      	  MOD_KG_polling_PM25_deal();
          	      										MOD_KG_transmit_wait_time=cticks_ms;
														      }
														      else
														      {
														      	if(port_report[3] == 0x10)		//-指定显示哪一页传感器数据
															      {
															      	  MOD_KG_rec_frame_type = MOD_KG_end_PM25_FRAME;
															      	  MOD_KG_end_PM25_deal();
          	      											MOD_KG_transmit_wait_time=cticks_ms;
															      }
															      else
															      {
															      	if(port_report[3] == 0x11)		//-指定显示哪一页传感器数据
																      {
																      	  MOD_KG_rec_frame_type = MOD_KG_polling_hl_FRAME;
																      	  MOD_KG_polling_hl_deal();
          	      												MOD_KG_transmit_wait_time=cticks_ms;
																      }
																      else
																      {
																      	if(port_report[3] == 0x12)		//-指定显示哪一页传感器数据
																	      {
																	      	  MOD_KG_rec_frame_type = MOD_KG_sys_state_FRAME;
																	      	  MOD_KG_sys_state_deal();
													                  MOD_KG_transmit_wait_time=cticks_ms;
													          	      MOD_KG_transmit_flag=NO;
																	      }
																	      else
																	      {
																	      	if(port_report[3] == 0x13)		//-指定显示哪一页传感器数据
																		      {
																		      	  MOD_KG_rec_frame_type = MOD_KG_SET_hl_time_FRAME;
																		      	  MOD_KG_SET_hl_time_deal();
          	      														MOD_KG_transmit_wait_time=cticks_ms;
																		      }
																		      else
																		      {
																		      	if(port_report[3] == 0x94)		//-7620给的应答
																			      {
																			      	  MOD_KG_rec_frame_type = MOD_KG_send_leddis_flag_FRAME;
																			      	  MOD_KG_send_leddis_flag_deal();
          	      															MOD_KG_transmit_wait_time=cticks_ms;
																			      }
																			      else
																			      {
																			      	if(port_report[3] == 0x15)		//?指定显示哪一页传感器数据
																				      {
																				      	  MOD_KG_rec_frame_type = MOD_KG_SET_voice_flag_FRAME;
																				      	  MOD_KG_SET_voice_flag_deal();
          	      																MOD_KG_transmit_wait_time=cticks_ms;
																				      }
																				      else
																				      {
																				      	if(port_report[3] == 0x16)		//-指定显示哪一页传感器数据
																					      {
																					      	  MOD_KG_rec_frame_type = MOD_KG_polling_voice_flag_FRAME;
																					      	  MOD_KG_polling_voice_flag_deal();
          	      																	MOD_KG_transmit_wait_time=cticks_ms;
																					      }
																					      else
																					      {
																					      	if(port_report[3] == 0x17)		//-接收网关的启动进程
																						      {
																						      	  MOD_KG_rec_frame_type = MOD_KG_re_gateway_flag_FRAME;
																						      	  MOD_KG_re_gateway_flag_deal();
          	      																		MOD_KG_transmit_wait_time=cticks_ms;
																						      }
																						      else
																						      {
																						      	if(port_report[3] == 0x19)		//-接收网关的启动进程
																							      {
																							      	  MOD_KG_rec_frame_type = MOD_KG_re_ZigBee_flag_FRAME;
																							      	  MOD_KG_re_ZigBee_flag_deal();
																			                  MOD_KG_transmit_wait_time=cticks_ms;
																			          	      MOD_KG_transmit_flag=NO;
																							      }
																							      else
																							      {
																							      	if(port_report[3] == 0x9A)		//-接收网关的启动进程
																								      {
																								      	  MOD_KG_rec_frame_type = MOD_KG_send_HRL_mode_FRAME;
																								      	  MOD_KG_send_HRL_mode_deal();
          	      																				MOD_KG_transmit_wait_time=cticks_ms;
																								      }
																								      else
																								      {
																								      	if(port_report[3] == 0x1B)		//-接收网关的启动进程
																									      {
																									      	  MOD_KG_rec_frame_type = MOD_KG_send_voice_mode_FRAME;
																									      	  MOD_KG_send_voice_mode_deal();
          	     																						MOD_KG_transmit_wait_time=cticks_ms;
																									      }
																									      else
																									      {
																									      	if(port_report[3] == 0x1C)		//-接收网关的启动进程
																										      {
																										      	  MOD_KG_rec_frame_type = MOD_KG_send_es705_mode_FRAME;
																										      	  MOD_KG_send_es705_mode_deal();
																						                  MOD_KG_transmit_wait_time=cticks_ms;
																						          	      MOD_KG_transmit_flag=NO;
																										      }
																										      else
																										      {
																										      	if(port_report[3] == 0x9e)		//-接收网关的启动进程
																											      {
																											      	  MOD_KG_rec_frame_type = MOD_KG_send_es705_event_type;
																											      	  MOD_KG_send_es705_event_type_ack();
          	      																							MOD_KG_transmit_wait_time=cticks_ms;
																											      }
																											      else
																											      {
																											      	if(port_report[3] == 0x1F)		//-接收网关的启动进程
																												      {
																												      	  MOD_KG_rec_frame_type = MOD_KG_send_HRL_ONOFF_FRAME;
																												      	  MOD_KG_send_HRL_ONOFF_ack();
																								                  MOD_KG_transmit_wait_time=cticks_ms;
																								          	      MOD_KG_transmit_flag=NO;
																												      }
																												      else
																												      {
																												      	if(port_report[3] == 0xA0)		//-接收网关的启动进程
																													      {
																													      	  MOD_KG_rec_frame_type = MOD_KG_send_inquire_state_FRAME;
																													      	  MOD_KG_send_inquire_state_ack();
          	      																									MOD_KG_transmit_wait_time=cticks_ms;
																													      }
																													      else
																													      {
																													      		if(port_report[3] == 0x21)		//-接收网关的启动进程
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

    			MOD_KG_clear_port_report_deal();		//-每次处理完毕之后清除缓存区防止错误使用

    	}
rxd_out_time:	//-执行到这里说明接收超时,或处理成功,反正现在可以继续发送了,,正常情况都会执行这里的,首先
	    if(Judge_Time_In_MainLoop(MOD_KG_rxd_wait_time,MOD_KG_WAIT_TIME_VALUE)==YES)	//-正常通信的话是不应该超时的,若过时就认为出错,下面重新初始化
	    {	//-当发送一个报文之后,超过等待回答时间之后,就可以从新发送一次
	      	MOD_KG_rec_OK=NO;
	      	MOD_KG_rxd_head_flag=NO;
	      	MOD_KG_rxd_wait_time=cticks_ms;
	      	//-MOD_KG_transmit_flag=YES;	//-表示现在可以组织发送内容
	      	//-MOD_KG_wait_replay=NO;	//-表示现在还没有等待回复
	      	//-MOD_KG_transmit_wait_time=Time_1ms_Counter;

	       	//-MOD_KG_comm_err_counter[port_send[0][0]]++;	//-超时出错计数


	    }
	    if(Judge_Time_In_MainLoop(MOD_KG_transmit_wait_time,80)==YES)
	    	MOD_KG_transmit_flag=YES;	//-一但接收到有效报文,那么80ms时间后才可以处理下次可能的报文
inrda:
		   //-主动发送 ,,这里就有一个问题,多长时间发送一次,或者说这个发送触发条件是什么
		   if(MOD_KG_transmit_flag==YES)		//-目前无所谓的是双向的,所有的启动发送之后都需要禁止发送,直到内存内的东西被发送出去
		   {
		   	  //-if((Judge_Time_In_MainLoop(MOD_KG_transmit_wait_time,10000)==YES) || (port_send_sense_data[0] == 0x55))	//-只有过了等待时间才会发送
		   	  if(Judge_Time_In_MainLoop(MOD_KG_transmit_wait_time,10)==YES)	//-一但有内容发送出去,需要给接收端足够的处理时间然后再发送
			   	  switch(MOD_KG_transmit_control)
	          {
	          	    case 0:
                       if(Judge_Time_In_MainLoop(MOD_KG_transmit_wait_time,1000)==YES)
                       {
                          MOD_KG_send_inquire_state();		//-启动之后主动查询网关是否启动完成
                          MOD_KG_transmit_flag=NO;
                          MOD_KG_transmit_wait_time=cticks_ms;	//-应答报文也是有时效性的
                       }
	          	    	   break;
	                case 1:
	                     //-MOD_KG_rec_frame_type=MOD_KG_YX_FRAME;	//-表示需要接收的帧的类型是这个值,接收到这样的值才是正确的
	                     //-MOD_KG_wait_replay=YES;
	                     //-MOD_KG_polling_ID_cmd();	//-本规约定义的内容是,问答型的,这个端口上的所有装置都是接收查询
	                     if((Judge_Time_In_MainLoop(MOD_KG_transmit_wait_time,1900)==YES) && (UART1_sloop_flag == 0))	//-周期上送的优先级最低,只有空闲时周期上送,一但有任何数据都退出总线
	                     {
	                     		MOD_KG_send_sense_data();  //-测试隐掉
	                     		Device_communication_cn++;
	                     		if(Device_communication_cn > 60)
	                     		{//-超过30次都没有应答认为和上面通讯终止了,需要处理些特殊情况
	                     			HRL_RUN_flag = 0;
	                     		}
	                     		//-port_send_sense_data[0] = 0;
	                     		//-MOD_KG_now_poll_addr++;	//-从机地址

	                     		MOD_KG_transmit_flag=NO;	//-现在开始不能发送报文了
	                     		MOD_KG_rxd_head_flag=NO;	//-为类同步准备的,但是并不是所有的东西都是那样
	                     		MOD_KG_transmit_wait_time=cticks_ms;	//-应答报文也是有时效性的
	                   	 }
	                     MOD_KG_transmit_control=2;	//-所有的端口装置查询一遍之后,再换下一类报文查询

	                     break;
	                case 2:
	                     //-MOD_KG_rec_frame_type=MOD_KG_YX_FRAME;	//-表示需要接收的帧的类型是这个值,接收到这样的值才是正确的
	                     //-MOD_KG_wait_replay=YES;
	                     //-MOD_KG_polling_ID_cmd();	//-本规约定义的内容是,问答型的,这个端口上的所有装置都是接收查询

                       if(UART3_TO_UART2_FLAG == 0)
                       {

                       }
                       else
                       {
                         if(UART3_TO_UART2_FLAG == 0x84)	//-这些标志位都是在串口3中置位的,内容本串口自己组织向7620发送
                         {
                           MOD_KG_SET_screen_deal_ack();
                           MOD_KG_transmit_flag=NO;
                           MOD_KG_transmit_wait_time=cticks_ms;
                         }
                         else
                         {
                         		if(UART3_TO_UART2_FLAG == 0x85)	//-这些标志位都是在串口3中置位的,内容本串口自己组织向7620发送
                         		{
	                            MOD_KG_receive_set_deal_ack();
	                            MOD_KG_transmit_flag=NO;
	                            MOD_KG_transmit_wait_time=cticks_ms;
                          	}
                          	else
                          	{
                          		if(UART3_TO_UART2_FLAG == 0x86)	//-这些标志位都是在串口3中置位的,内容本串口自己组织向7620发送
		                          {
		                            MOD_KG_mode_set_deal_ack();
		                            MOD_KG_transmit_flag=NO;
		                            MOD_KG_transmit_wait_time=cticks_ms;
		                          }
		                          else
		                          {
		                          	 if(UART3_TO_UART2_FLAG == 0x89)	//-这些标志位都是在串口3中置位的,内容本串口自己组织向7620发送
				                         {
				                            MOD_KG_chage_pic_deal_ack();
				                            MOD_KG_transmit_flag=NO;
				                            MOD_KG_transmit_wait_time=cticks_ms;
				                         }
				                         else
				                         {
				                         	 if(UART3_TO_UART2_FLAG == 0x8A)	//-这些标志位都是在串口3中置位的,内容本串口自己组织向7620发送
					                         {
					                            MOD_KG_SET_loop_deal_ack();
					                            MOD_KG_transmit_flag=NO;
					                            MOD_KG_transmit_wait_time=cticks_ms;
					                         }
					                         else
					                         {
					                         	 if(UART3_TO_UART2_FLAG == 0x8B)	//-这些标志位都是在串口3中置位的,内容本串口自己组织向7620发送
						                         {
						                            MOD_KG_send_state_deal_ack();
						                            MOD_KG_transmit_flag=NO;
						                            MOD_KG_transmit_wait_time=cticks_ms;
						                         }
						                         else
						                         {
						                         	 if(UART3_TO_UART2_FLAG == 0x8C)	//-这些标志位都是在串口3中置位的,内容本串口自己组织向7620发送
							                         {
							                            MOD_KG_re_Weather_deal_ack();
							                            MOD_KG_transmit_flag=NO;
							                            MOD_KG_transmit_wait_time=cticks_ms;
							                         }
							                         else
							                         {
							                         	 if(UART3_TO_UART2_FLAG == 0x8D)	//-这些标志位都是在串口3中置位的,内容本串口自己组织向7620发送
								                         {
								                            MOD_KG_re_show_deal_ack();
								                            MOD_KG_transmit_flag=NO;
								                            MOD_KG_transmit_wait_time=cticks_ms;
								                         }
								                         else
								                         {
								                         	 if(UART3_TO_UART2_FLAG == 0x8E)	//-这些标志位都是在串口3中置位的,内容本串口自己组织向7620发送
									                         {
									                            MOD_KG_polling_state_deal_ack();
									                            MOD_KG_transmit_flag=NO;
									                            MOD_KG_transmit_wait_time=cticks_ms;
									                         }
									                         else
									                         {
									                         	 if(UART3_TO_UART2_FLAG == 0x91)	//-这些标志位都是在串口3中置位的,内容本串口自己组织向7620发送
										                         {
										                            MOD_KG_polling_hl_deal_ack();
										                            MOD_KG_transmit_flag=NO;
										                            MOD_KG_transmit_wait_time=cticks_ms;
										                         }
										                         else
										                         {
										                         	 if(UART3_TO_UART2_FLAG == 0x93)	//-这些标志位都是在串口3中置位的,内容本串口自己组织向7620发送
											                         {
											                            MOD_KG_SET_hl_time_deal_ack();
											                            MOD_KG_transmit_flag=NO;
											                            MOD_KG_transmit_wait_time=cticks_ms;
											                         }
											                         else
											                         {
											                         	 if(UART3_TO_UART2_FLAG == 0xC0)	//-这些标志位都是在串口3中置位的,内容本串口自己组织向7620发送
												                         {
												                            MOD_KG_control_HRL_deal_ack();
												                            MOD_KG_transmit_flag=NO;
												                            MOD_KG_transmit_wait_time=cticks_ms;
												                         }
												                         else
												                         {
												                         	 if(UART3_TO_UART2_FLAG == 0x94)	//-这些标志位都是在串口3中置位的,内容本串口自己组织向7620发送
													                         {
													                            MOD_KG_send_leddis_flag_ack();
													                            MOD_KG_transmit_flag=NO;
													                            MOD_KG_transmit_wait_time=cticks_ms;
													                         }
													                         else
													                         {
													                         	 if(UART3_TO_UART2_FLAG == 0x95)	//-这些标志位都是在串口3中置位的,内容本串口自己组织向7620发送
														                         {
														                            MOD_KG_SET_voice_flag_ack();
														                            MOD_KG_transmit_flag=NO;
														                            MOD_KG_transmit_wait_time=cticks_ms;
														                         }
														                         else
														                         {
														                         	 if(UART3_TO_UART2_FLAG == 0x96)	//-这些标志位都是在串口3中置位的,内容本串口自己组织向7620发送
															                         {
															                            MOD_KG_polling_voice_flag_ack();
															                            MOD_KG_transmit_flag=NO;
															                            MOD_KG_transmit_wait_time=cticks_ms;
															                         }
															                         else
															                         {
															                         	 if(UART3_TO_UART2_FLAG == 0xC1)	//-这些标志位都是在串口3中置位的,内容本串口自己组织向7620发送
																                         {
																                            MOD_KG_leddis_end_ack();
																                            MOD_KG_transmit_flag=NO;
																                            MOD_KG_transmit_wait_time=cticks_ms;
																                         }
																                         else
																                         {
																                         	 if(UART3_TO_UART2_FLAG == 0xC2)	//-这些标志位都是在串口3中置位的,内容本串口自己组织向7620发送
																	                         {
																	                            MOD_KG_send_HRL_mode_ack();
																	                            MOD_KG_transmit_flag=NO;
																	                            MOD_KG_transmit_wait_time=cticks_ms;
																	                         }
																	                         else
																	                         {
																	                         	 if(UART3_TO_UART2_FLAG == 0x97)	//-这些标志位都是在串口3中置位的,内容本串口自己组织向7620发送
																		                         {
																		                            MOD_KG_re_gateway_flag_ack();
																		                            MOD_KG_transmit_flag=NO;
																		                            MOD_KG_transmit_wait_time=cticks_ms;
																		                         }
																		                         else
																		                         {
																		                         	 if(UART3_TO_UART2_FLAG == 0x9B)	//-这些标志位都是在串口3中置位的,内容本串口自己组织向7620发送
																			                         {
																			                            MOD_KG_send_voice_mode_ack();
																			                            MOD_KG_transmit_flag=NO;
																			                            MOD_KG_transmit_wait_time=cticks_ms;
																			                         }
																			                         else
																			                         {
																			                         	 if(UART3_TO_UART2_FLAG == 0xA1)	//-这些标志位都是在串口3中置位的,内容本串口自己组织向7620发送
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
	                     //-MOD_KG_transmit_flag=NO;	//-现在开始不能发送报文了
	                     MOD_KG_rxd_head_flag=NO;	//-为类同步准备的,但是并不是所有的东西都是那样
	                     //-MOD_KG_transmit_wait_time=cticks_ms;	//-应答报文也是有时效性的

	                     MOD_KG_transmit_control=1;	//-所有的端口装置查询一遍之后,再换下一类报文查询

	                     break;
                  case 3:

	                     if(es705_TO_UART2_FLAG == 1)	//-这些标志位都是在串口3中置位的,内容本串口自己组织向7620发送
                       {
	                     		MOD_KG_send_es705_deal();
                          if(es705_training_count == 4)
                             es705_training_count = 0;   //-发送结束了才可以清零
                          es705_TO_UART2_FLAG = 0;
                       }
                       else if((es705_TO_UART2_FLAG & 0x7f) == 2)
                       {
                          if(es705_TO_UART2_FLAG == 0x02)
                          {
                             MOD_KG_send_es705_event_type_deal();  //-这个地方等待应答需要设置一个时间段
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
                       //-MOD_KG_rxd_head_flag=NO;    //-感觉这里不需要清除,因为是双工的,否则可能会有丢失
                       MOD_KG_transmit_control=1;	//-所有的端口装置查询一遍之后,再换下一类报文查询
                       break;
	                default:
	                     break;
	          }

		   }
 	 }

}

//-第一步STM32向7620和射灯板发送数据		不需要应答
//-第二步7620向STM32发送数据		我作为从机是可以立即应答的
//-第三步7620向射灯板发送数据,内容直接透传到射灯板

//-射灯板向7620发送数据

//-总之只要没有我需要等待应答的,我都可以立即发送
//-这里不需要向任何设备下达任何命令,所以我也是从机,不需要等待发送

//-2016.1.7 19:46
//-通过抓包发现数据7620发送出来了(挂接串口PC端读到了数据),通过观察
//-接收指针STM32也的确接收到了数据,但是就是没有应答.

//?目前怀疑可能串口1的发送方式,和本身所有串口的删除方式有冲突,可能导致正常报文的丢失.


