/*
接收:使用一个常规逻辑,开辟一个512字节的缓冲区,然后循环存储
发送:也使用512字节的空间保存内容,然后置一个发送标志,直到内容发送出去为止

功能:和主板上的STM32通讯.
是否采用查询的方式接收数据?我觉得作为一个终端显示设备,没有必要去特地查询
很多数据,如果是周期刷新的话,就尽量刷新而不采用查询方式.必要的信息才查询.
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


#define   MOD_KG_send_max	  	     	   5		//-决定一个内容重复发送的次数

//-为协议处理临时定义的变量
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

  //-协议初始化内容
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
  port_recv_pt[0] = 512 - DMA_GetCurrDataCounter(DMA1_Channel5);	//-得到接收个数,如果作为数组偏移量的话,现在的数值就是指向待存空间的
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
  //-USART_DMACmd(USARTy, USART_DMAReq_Tx, ENABLE);			//-启动DMA发送

  if(port_send_len[0] != 0)	//-一定要保证内容发送出去了,再使用DMA发送数据
  {
		  DMA_Cmd(DMA1_Channel4, DISABLE);
		  DMA1_Channel4->CNDTR = port_send_len[0]; //-传输数量寄存器,指示剩余的待传输字节数目
		  DMA_Cmd(DMA1_Channel4, ENABLE);
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
	  port_deal_flag = 0xaa;

	  cticks_500ms = 0;
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
	  port_send[0][5] = 0x01;
	  port_send[0][6] = 0x02;
	  port_send[0][7] = 0x03;
	  port_send[0][8] = 0x04;
	  port_send[0][9] = 0x05;
	  port_send[0][10] = 0x06;

	  //-CRC16
	  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
	  port_send[0][11] =LOBYTE(the_ram_ax);
	  port_send[0][12] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

	  //-发送长度
	  port_send_len[0] = port_send[0][2] + 2;
	  port_deal_flag = 0xaa;
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
	  port_send[0][8] = LOBYTE(port_send_sense_data[1]);		//-数据位
	  port_send[0][9] = HIBYTE(port_send_sense_data[1]);

	  port_send[0][10] = 0x0A;		//-数据长度		,,温度
	  port_send[0][11] = 0xD1;		//-设备ID
	  port_send[0][12] = 0x04;		//-功能码
	  port_send[0][13] = 0x02;		//-数据描述
	  port_send[0][14] = LOBYTE(port_send_sense_data[2]);		//-数据位
	  port_send[0][15] = HIBYTE(port_send_sense_data[2]);

	  port_send[0][16] = 0x0A;		//-数据长度		,,湿度
	  port_send[0][17] = 0xD2;		//-设备ID
	  port_send[0][18] = 0x04;		//-功能码
	  port_send[0][19] = 0x02;		//-数据描述
	  port_send[0][20] = LOBYTE(port_send_sense_data[3]);		//-数据位
	  port_send[0][21] = HIBYTE(port_send_sense_data[3]);

	  port_send[0][22] = 0x0A;		//-数据长度		,,PM2.5
	  port_send[0][23] = 0x44;		//-设备ID
	  port_send[0][24] = 0x04;		//-功能码
	  port_send[0][25] = 0x01;		//-数据描述
	  port_send[0][26] = LOBYTE(port_send_sense_data[4]);		//-数据位
	  port_send[0][27] = HIBYTE(port_send_sense_data[4]);

	  port_send[0][28] = 0x0A;		//-数据长度		,,噪声
	  port_send[0][29] = 0xD4;		//-设备ID
	  port_send[0][30] = 0x04;		//-功能码
	  port_send[0][31] = 0x01;		//-数据描述
	  port_send[0][32] = LOBYTE(port_send_sense_data[5]);		//-数据位
	  port_send[0][33] = HIBYTE(port_send_sense_data[5]);

	  port_send[0][34] = 0x01;		//-数据长度		,,预留
	  port_send[0][35] = 0x02;		//-设备ID
	  port_send[0][36] = 0x03;		//-功能码
	  port_send[0][37] = 0x04;		//-数据描述
	  port_send[0][38] = LOBYTE(port_send_sense_data[6]);		//-数据位
	  port_send[0][39] = HIBYTE(port_send_sense_data[6]);

	  //-CRC16
	  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
	  port_send[0][40] =LOBYTE(the_ram_ax);
	  port_send[0][41] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

	  //-发送长度
	  port_send_len[0] = port_send[0][2] + 2;
	  port_deal_flag = 0xaa;
}
/*
采用通讯图片优先接收显示的结构,然后本系统有触发条件了,就切换显示,比如手势滑动
0xaa 0x55 0x03 数据 CRC
*/
void MOD_KG_LED_display_deal(void)		//-对主板STM32下达的数据进行接收,在本系统和通讯图片之间如何切换?
{
    u8 i;
	  //-WORD the_ram_ax;
    UINT32	*led_display_data_pt;

	  if(led_display_txpic_flag != 0x55)
	  {
			  if(led_display_num == 5)		//-两个缓冲区互为备用,不正在显示的可以组织待显示数据
				{//-led_display_data2[16]
					 led_display_data_pt = &led_display_data6[0];

			  }
			  else
			  {//-led_display_data1[16]
					 led_display_data_pt = &led_display_data5[0];

			  }

			  for(i = 0;i < 16;i++)	//-填充满了一张图片
				{
					 led_display_data_pt[i] = (port_report[i*3+3] << 16) + (port_report[i*3+4] << 8) + port_report[i*3+5];
				}

				led_display_txpic_flag = 0x55;
		}
	  //-特别定义一个缓冲区存储数据,置标志位,然后刷新的时候检查这里的数据是否有效
	  //-如果有的话就切换显示

}

void MOD_KG_receive_data_deal(void)		//-接收来自主板STM32的传感器数据
{
	  WORD temp_data,the_ram_ax;

	  //-2016/2/3 9:25:22	增加报文应答内容,向主板更新射灯板状态
				port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = 0x81;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = 0;		//-数据长度
			  port_send[0][5] = 0;		//-数据长度

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

	  if(UART1_renew_flag != 0x55)
		{//-不等说明可以更新数据库内容
		  if(port_report[5] == 0xA0)		//-设备ID号:CO2值
		  {
		  	 temp_data = port_report[9] + (port_report[8] << 8);
		  	 if(temp_data > 0)
		  	 	co2_data = temp_data;
		  }

	  if(port_report[11] == 0xD1)		//-设备ID号:温度
	  {
	  	  temp_data = port_report[15] + (port_report[14] << 8);
	  	  //-if(temp_data > 0)
	  	  	temperature_data = temp_data;
	  }

	  //-if(port_report[17] == 0xD2)		//-设备ID号:湿度
	  //-{
	  //-	 humidity_data = port_report[20] + (port_report[21] << 8);
	  //-}

	  if(port_report[23] == 0xD5)		//-设备ID号:PM2.5
	  {
	  	 temp_data = port_report[27] + (port_report[26] << 8);
	  	 if(temp_data > 0)
	  	 		pm_data =	temp_data;
	  }

	  if(port_report[29] == 0xD4)		//-设备ID号:噪声
	  {
	  	 temp_data = port_report[33] + (port_report[32] << 8);
	  	 if(temp_data > 0)
	  	 		Noise_Value = temp_data;
	  }

	  if(port_report[17] == 0xD6)		//-设备ID号:VOC
	  {
	  	 temp_data = port_report[21] + (port_report[20] << 8);
	  	 if(temp_data > 0)
	  	 		VOC_data = temp_data;
	  }

	  if(led_display_new != 0xa5)
	  	 led_display_new = 0x55;		//-刷新现有的显示数据,不是光光改变定值就行,需要显示出来

	  //-特别定义一个缓冲区存储数据,置标志位,然后刷新的时候检查这里的数据是否有效
	  //-如果有的话就切换显示

	    //-一但接收到心跳报文就认为系统已经启动起来了,那么结束开机动画,恢复正常显示
	    led_display_flag = 3;
		}
}

void MOD_KG_SET_screen_deal(void)		//-设置屏保时间
{
	  WORD the_ram_ax;

	  if(port_report[4] == 0x01)		//-
	  {
	  	 led_display_long = port_report[5];	//-屏保时间
	  	 EEP_Data_flag = 0x55;		//-数值被修改了
	  	 cticks_s_page = 0;
       led_display_start = 0x55;

       //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

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
			  port_deal_flag = 0xaa;
	  }



}

void MOD_KG_mode_set_deal(void)		//-炫彩模式设置
{
	  WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	 HL_flag = port_report[4];	//-炫彩模式设置,00h ：关闭 ； 01h 自动；  02 用户模式
	  	 if(HL_flag == 0)
	  	 {
	  	 	  HL_RED_pwmval = 0;
		 	  	HL_GREEN_pwmval = 0;
		 	  	HL_BLUE_pwmval = 0;
	  	 }
			 HL_new_value_flag = 1;
			 HL_run_time = Time_2048ms_Counter;

       //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = port_report[3] | 0x80;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = port_report[4];		//-数据长度

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][5] =LOBYTE(the_ram_ax);
			  port_send[0][6] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;
	  }



}

void MOD_KG_receive_set_deal(void)		//-炫彩灯设置,目前支持设置四种颜色
{
	  WORD the_ram_ax;


	  //-aa5515051047DD22000080ff002500ff0000ff8000
    if(port_report[4] == 0x10)		//-设置固定颜色的RGB值
	  {
	  	  HL_ld_R_user[0] = port_report[5];
		    HL_ld_G_user[0] = port_report[6];
		    HL_ld_B_user[0] = port_report[7];

		    //-if(((HL_GREEN_pwmval + HL_BLUE_pwmval) >250) && (HL_RED_pwmval < 127))
		    //-	HL_RED_pwmval = 0;
		    //-else if(((HL_GREEN_pwmval + HL_BLUE_pwmval) >250) && (HL_RED_pwmval >= 127) && (HL_RED_pwmval <= 180))
		    //-	HL_RED_pwmval = 9;

		    //-HL_RED_pwmval = (PWM_Period_Value * HL_RED_pwmval) / 255;   //-红
        //-HL_GREEN_pwmval = (PWM_Period_Value * HL_GREEN_pwmval) / 255;		//-绿
        //-HL_BLUE_pwmval = (PWM_Period_Value * HL_BLUE_pwmval) / 255;	//-蓝

        HL_run_time = Time_2048ms_Counter;		//-设置了单色后灯也就亮了,所以需要重新计时关闭时间

        HL_new_value_flag = 1;
        HL_flag = 1;


		    //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = port_report[3] | 0x80;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = port_report[4];		//-数据位0
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
			  port_send[0][10] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;
	  }

}

void MOD_KG_chage_page_deal(void)		//-接收翻页命令,由于仅仅测试用暂时不用应答
{
	  //-WORD the_ram_ax;

	  if(led_display_start == 0x55)		//-设备ID号:CO2值
	  {
	  	 ps_flag = port_report[4];
	  	 ps_flag_led = port_report[4];
	  }
	  ps_flag_led_end = port_report[4];
	  ps_flag_led_disp = port_report[4];	  //-这里需要特别注意,不换显示内容,但是刷屏的过程是一样要的
	  led_disp_L_init();


	  //-特别定义一个缓冲区存储数据,置标志位,然后刷新的时候检查这里的数据是否有效
	  //-如果有的话就切换显示

}

void MOD_KG_chage_pic_deal(void)		//-接收翻页命令,由于仅仅测试用暂时不用应答
{
	  WORD the_ram_ax;

	  if(led_display_txpic_flag != 0x55)
	  {
	  	 led_display_txpic_num = port_report[4];

       led_display_txpic_flag = 0x55;

       port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = port_report[3] | 0x80;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = port_report[4];		//-数据长度
			  port_send[0][5] = port_report[5];		//-数据长度

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;
	  }

	  cticks_s_page = 0;
    led_display_start = 0x55;

	  //-特别定义一个缓冲区存储数据,置标志位,然后刷新的时候检查这里的数据是否有效
	  //-如果有的话就切换显示

}

void MOD_KG_SET_loop_deal(void)		//-改变呼吸周期
{
	  WORD the_ram_ax;

	  cticks_ms_pwm_loop = port_report[4] + (port_report[5] << 8);

	  //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = port_report[3] | 0x80;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = port_report[4];		//-数据长度
			  port_send[0][5] = port_report[5];		//-数据长度

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;
}

void MOD_KG_send_state_deal(void)		//-接收主机现在的状态,进行对应的状态显示
{
	  WORD the_ram_ax;

	  if(port_report[4] == 1)
	  {
	  	 led_display_txpic_flag = 0x55;		//-让处理程序准备图片
	  	 led_display_txpic_num = 0;
	  	 cartoon_end_wait_time = cticks_ms;
	  	 //-led_display_new = 0xa5;	//-启动动画
	  	 //-led_display_flag = 7;
	  }
	  else
	  {
	  	 led_display_new = 0;
	  	 led_display_flag = 3;
	  	 led_display_txpic_flag = 0x55;		//-让处理程序准备图片
	  	 led_display_txpic_num = 9;		//-指定固定的图片
	  }


	  //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = port_report[3] | 0x80;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = port_report[4];		//-数据长度
			  port_send[0][5] = port_report[5];		//-数据长度

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;
}

void MOD_KG_re_Weather_deal(void)		//-
{
	  WORD the_ram_ax;


	  	 Weather_flag = port_report[4];		//-具体天气数据
	  	 led_display_page = 4;
	  	 led_display_page_end = 4;		//-有了天气后增加一页显示
	  	 led_display_new = 0x55;
	  	 led_display_txpic_flag = 0;

	  	 cticks_s_page = 0;			//-亮屏
       led_display_start = 0x55;




	  //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = port_report[3] | 0x80;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = port_report[4];		//-数据长度
			  port_send[0][5] = port_report[5];		//-数据长度

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;
}

void MOD_KG_re_show_deal(void)		//-接收显示特定页的命令
{
	  WORD the_ram_ax;
	  //-test_cn++;    //-测试计数用
	  led_display_page = port_report[4];
	  voice_keep_data_flag = 0;	//-既然显示页更新了,那么就需要解冻可能的冻结数据
	  UART1_renew_flag = 0x55;		//-一但接收到指定的内容显示,就一段时间内不运行传口更新数据库
	  UART1_renew_wait_time = Time_2048ms_Counter;

	  cticks_s_page = 0;
    led_display_start = 0x55;
	  led_display_new = 0x55;		//-刷新现有的显示数据,不是光光改变定值就行,需要显示出来
	  led_display_deal_flag = 0xaa;	//-给这个数值的目的是为了保证先准备内容,再显示
	  led_display_ye_ok = 0;

	  the_ram_ax = port_report[6] + (port_report[5] << 8);
	  if(led_display_page == 0)
    {//-显示PM2.5
    	 pm_data = the_ram_ax;		//-可以考虑给个有效值范围,这样可以出错了,也给个假数据
    }
    else if(led_display_page == 1)
    {//-噪音
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
    {//-天气
    	 Weather_flag = the_ram_ax;		//-
    }
    else if(led_display_page == 20)
    {//-光强
    	 als_data = the_ram_ax;
    }
    else if(led_display_page == 21)
    {//-按键值
    	 CSKEY_DATA = the_ram_ax;
    }

	  //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = port_report[3] | 0x80;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = port_report[4];		//-数据长度
			  port_send[0][5] = port_report[5];		//-数据长度
			  port_send[0][6] = port_report[6];		//-数据长度

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][7] =LOBYTE(the_ram_ax);
			  port_send[0][8] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

}

void MOD_KG_polling_state_deal(void)		//-接收到查询器件状态命令之后,直接返回状态值
{
	  WORD the_ram_ax;



	  //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = port_report[3] | 0x80;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = STM32_UP_selfT_flag1;		//-数据长度
			  port_send[0][5] = 0;		//-数据长度

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

}

void MOD_KG_polling_PM25_deal(void)		//-接收到应答之后就不再,发送命令
{
	  //-WORD the_ram_ax;

	  UART1_transmit_flag = NO;		//-可以组织内容发送
    UART1_transmit_control = 0;	//-决定了主动发送的内容

}

void MOD_KG_end_PM25_deal(void)		//-接收到应答之后就不再,发送命令
{
	  //-WORD the_ram_ax;

	  UART1_transmit_flag = NO;		//-可以组织内容发送
    UART1_transmit_control = 0;	//-决定了主动发送的内容

}

void MOD_KG_control_HRL_deal(void)		//-不管内容,只要接收到应答就认为命令有效,无需再次发送
{
	  //-WORD the_ram_ax;

	  UART1_transmit_flag = NO;		//-可以组织内容发送
    UART1_transmit_control = 0;	//-决定了主动发送的内容


}

void MOD_KG_leddis_end_deal(void)		//-不管内容,只要接收到应答就认为命令有效,无需再次发送
{
	  //-WORD the_ram_ax;

	  UART1_transmit_flag = NO;		//-可以组织内容发送
    UART1_transmit_control = 0;	//-决定了主动发送的内容


}

void MOD_KG_send_HRL_mode_deal(void)		//-不管内容,只要接收到应答就认为命令有效,无需再次发送
{
	  //-WORD the_ram_ax;

	  UART1_transmit_flag = NO;		//-可以组织内容发送
    UART1_transmit_control = 0;	//-决定了主动发送的内容


}

void MOD_KG_polling_hl_deal(void)		//-接收到应答之后就不再,发送命令
{
	  WORD the_ram_ax;



	  //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = port_report[3] | 0x80;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = HL_flag;		//-数据长度
			  port_send[0][5] = 0;		//-数据长度

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

}

//-下面是主动发送的报文
void MOD_KG_polling_PM25_cmd(void)		//-接收到查询器件状态命令之后,直接返回状态值
{
	  WORD the_ram_ax;



	  //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 8;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = 0x0F;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  if(voice_flag == 0x55)	//-增加一个播报开关
			  	port_send[0][4] = led_display_page | 0x80;		//-数据长度
			  else
		      port_send[0][4] = led_display_page;		//-数据长度

			  port_send[0][5] = HIBYTE(led_display_data);		//-数据长度
			  port_send[0][6] = LOBYTE(led_display_data);		//-数据长度
			  if(led_display_page == 4)	//-防错的,必须保证页码变化和数值同步对应
			  {
			  	 port_send[0][5] = 0;		//-数据长度
           if(Weather_flag > 5)
             Weather_flag = 1;  //-实践证明有报错误数据的时候,所以当判断为错误数据时就固定上送1,这个仅仅暂时使用,需要找到本质原因
			     port_send[0][6] = Weather_flag;		//-数据长度
			  }

			  port_send[0][7] = HL_flag;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][8] =LOBYTE(the_ram_ax);
			  port_send[0][9] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

}

void MOD_KG_end_PM25_cmd(void)		//-接收到查询器件状态命令之后,直接返回状态值
{
	  WORD the_ram_ax;



	  //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = 0x10;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = 0;		//-数据长度
			  port_send[0][5] = 0;		//-数据长度

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

}

void MOD_KG_control_FUN_cmd(void)		//-接收到查询器件状态命令之后,直接返回状态值
{
	  WORD the_ram_ax;



	  //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = 0x43;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = 0x55;		//-数据长度
			  port_send[0][5] = led_display_page;		//-数据长度

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

}

void MOD_KG_update_status_cmd(void)		//-接收到查询器件状态命令之后,直接返回状态值
{
	  WORD the_ram_ax;



	  //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 10;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = 0x44;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = led_display_start;		//-点阵屏亮灭	0x55说明亮屏
			  port_send[0][5] = led_display_page;		  //-点阵屏显示页
			  port_send[0][6] = HL_flag;      //-炫彩灯编号
			  port_send[0][7] = HIBYTE(led_display_data);      //-当前显示数据也上送下
			  port_send[0][8] = LOBYTE(led_display_data);
			  if(led_display_page == 4)	//-天气是后加的需要特殊处理下
			  {
			  	 port_send[0][7] = 0;		//-数据长度
           if(Weather_flag > 5)
             Weather_flag = 1;
			     port_send[0][8] = Weather_flag;		//-数据长度
			  }
			  port_send[0][9] = voice_flag;		//-表示是否进行语音播报,不播报的话就不向7620转发

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][10] =LOBYTE(the_ram_ax);
			  port_send[0][11] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

}

void MOD_KG_end_HRL_cmd(void)		//-接收到查询器件状态命令之后,直接返回状态值
{
	  WORD the_ram_ax;
	  static BYTE end_HRL_cn=0;


	  //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = 0x40;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = 0x55;		//-数据长度
			  port_send[0][5] = 0;		//-数据长度

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

	   end_HRL_cn++;
	   if(end_HRL_cn >= MOD_KG_send_max)
	   {
	   	  end_HRL_cn = 0;
	   	  UART1_transmit_flag = NO;		//-可以组织内容发送
    		UART1_transmit_control = 0;	//-决定了主动发送的内容
	   }
}

void MOD_KG_can_HRL_cmd(void)		//-接收到查询器件状态命令之后,直接返回状态值
{
	  WORD the_ram_ax;
	  static BYTE can_HRL_cn=0;


	  //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = 0x40;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = 0xaa;		//-数据长度
			  port_send[0][5] = 0;		//-数据长度

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

		 can_HRL_cn++;
	   if(can_HRL_cn >= MOD_KG_send_max)
	   {
	   	  can_HRL_cn = 0;
	   	  UART1_transmit_flag = NO;		//-可以组织内容发送
    		UART1_transmit_control = 0;	//-决定了主动发送的内容
	   }

}

void MOD_KG_send_HRL_mode_cmd(void)		//-接收到查询器件状态命令之后,直接返回状态值
{
	  WORD the_ram_ax;
	  static BYTE send_HRL_mode_cn=0;


	  //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 9;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = 0x42;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  if(HL_flag == 0)
			  	port_send[0][4] = 0xaa;		//-对主板跑马灯的控制位,开启跑马灯
			  else
			  	port_send[0][4] = 0x55;		//-对主板跑马灯的控制位	,关闭跑马灯
			  port_send[0][5] = HL_flag;		//-炫彩灯模式
			  port_send[0][6] = HL_ld_R_user[0];		//-当前用户RGB值
			  port_send[0][7] = HL_ld_G_user[0];
			  port_send[0][8] = HL_ld_B_user[0];

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][9] =LOBYTE(the_ram_ax);
			  port_send[0][10] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

	   send_HRL_mode_cn++;
	   if(send_HRL_mode_cn >= MOD_KG_send_max)
	   {
	   	  send_HRL_mode_cn = 0;
	   	  UART1_transmit_flag = NO;		//-可以组织内容发送
    		UART1_transmit_control = 0;	//-决定了主动发送的内容
	   }
}

void MOD_KG_send_leddis_flag_cmd(void)		//-接收到查询器件状态命令之后,直接返回状态值
{
	  WORD the_ram_ax;
	  static BYTE send_leddis_flag_cn=0;
	  //-0xaa 0x55

	  //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 9;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = 0x14;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = led_display_page;		//-数据长度
			  port_send[0][5] = HIBYTE(led_display_data);		//-数据长度
			  port_send[0][6] = LOBYTE(led_display_data);		//-数据长度
			 	if(led_display_page == 4)	//-天气是后加的需要特殊处理下
			  {
			  	 port_send[0][5] = 0;		//-数据长度
           if(Weather_flag > 5)
             Weather_flag = 1;
			     port_send[0][6] = Weather_flag;		//-数据长度
			  }

			 	port_send[0][7] = voice_flag;		//-表示是否进行语音播报,不播报的话就不向7620转发
			 	port_send[0][8] = HL_flag;
			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][9] =LOBYTE(the_ram_ax);
			  port_send[0][10] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

        //-test_cn++;    //-测试计数用
			  send_leddis_flag_cn++;
	   if(send_leddis_flag_cn >= MOD_KG_send_max)
	   {
	   	  send_leddis_flag_cn = 0;
	   	  UART1_transmit_flag = NO;		//-可以组织内容发送
    		UART1_transmit_control = 0;	//-决定了主动发送的内容
	   }

}

void MOD_KG_send_leddis_end_cmd(void)		//-主动向主板发送点阵屏熄灭状态
{
	  WORD the_ram_ax;
	  static BYTE send_leddis_end_cn=0;
	  //-0xaa 0x55

	  //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = 0x41;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = 0;		//-数据长度
			  port_send[0][5] = 0;		//-数据长度

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

			  send_leddis_end_cn++;
	   if(send_leddis_end_cn >= MOD_KG_send_max)
	   {
	   	  send_leddis_end_cn = 0;
	   	  UART1_transmit_flag = NO;		//-可以组织内容发送
    		UART1_transmit_control = 0;	//-决定了主动发送的内容
	   }

}

void MOD_KG_SET_hl_time_deal(void)		//-改变呼吸周期
{
	  WORD the_ram_ax;

	  HL_run_time_delay = port_report[4] + (port_report[5] << 8);

	  //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = port_report[3] | 0x80;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = port_report[4];		//-数据长度
			  port_send[0][5] = port_report[5];		//-数据长度

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;
}

void MOD_KG_send_leddis_flag_deal(void)		//-说明已经收到了,不需要再发送了
{
	  //-WORD the_ram_ax;

		UART1_transmit_flag = NO;		//-可以组织内容发送
    UART1_transmit_control = 0;	//-决定了主动发送的内容

}

void MOD_KG_SET_voice_flag_deal(void)		//-改变呼吸周期
{
	  WORD the_ram_ax;

	  voice_flag = port_report[4];
	  EEP_Data_flag = 0x55;		//-数值被修改了

	  //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = port_report[3] | 0x80;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = port_report[4];		//-数据长度
			  port_send[0][5] = port_report[5];		//-数据长度

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;
}

void MOD_KG_polling_voice_flag_deal(void)		//-改变呼吸周期
{
	  WORD the_ram_ax;



	  //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = port_report[3] | 0x80;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = voice_flag;		//-数据长度
			  port_send[0][5] = 0;		//-数据长度

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;
}

void MOD_KG_re_gateway_flag_deal(void)		//-启动完成之后结束开始动画
{
	  WORD the_ram_ax;

	  //-进程处理
	  led_display_new = 0x55;
	  led_display_flag = 3;

    //-更新炫彩灯亮度定值
    HL_ld_brightness[0] = port_report[5];

	  //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = port_report[3] | 0x80;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = 0xaa;		//-数据长度
			  port_send[0][5] = 0xaa;		//-数据长度

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;
}

void MOD_KG_send_voice_mode_deal(void)		//-启动完成之后结束开始动画
{
	  BYTE temp_data8;
	  WORD the_ram_ax;
	  int res;

	  //-进程处理
	  if(port_report[4] == 0)
	  {	//-关闭敲击
	  	//-ADXL_TAP_off_flag = 0x55;		//?这里应该需要考虑丢失报文不能恢复常态的情况
	  	//-ADXL_TAP_off_time = cticks_ms;
	  	//-不再关闭而是调节灵敏度
      //-上层计算公式是V/3+57,那么V的范围是0~100,所以我接收到的范围是57~90,区间值33
	  	temp_data8 = port_report[5]/3;//- + 57;
      temp_data8 = temp_data8 * 7;
	  	res = ADXL345_init_re(temp_data8);
	  	if(res != 1)
	  		ADXL345_init_re(temp_data8);
	  }
	  else
	  {//-恢复正常阀值
	  	ADXL_TAP_off_flag = 0;
	  	res = ADXL345_init_re(0);
	  	if(res != 1)
	  		ADXL345_init();
	  }

	  //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = port_report[3] | 0x80;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = 0xaa;		//-数据长度
			  port_send[0][5] = 0xaa;		//-数据长度

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;
        //-test_cn++;    //-测试计数用
}

void MOD_KG_send_es705_status_deal(void)	//-需要关闭手势等
{
	 //-WORD the_ram_ax;

	 if(port_report[5] == (es705_training_count + 1))
	    es705_training_count = port_report[5];		//-学习成功的次数
	 es705_training_flag = 0x55;
	 es705_training_wait_time = Time_2048ms_Counter;


	 led_display_txpic_num = 20 +es705_training_count;		//-指示哪个图片现在被显示
	 led_display_txpic_flag = 0x55;
	 cticks_s_page = 0;
   led_display_start = 0x55;

   if(es705_training_count == 4)
   {
   	  led_display_txpic_flag = 0;
   	  es705_training_count = 0;
   }
}

void MOD_KG_set_hl_set_deal(void)		//-改变呼吸周期
{
	  WORD the_ram_ax;

	  if(port_report[4] > 100)
	  	 HL_ld_brightness[1] = 100;
	  else
	  	 HL_ld_brightness[1] = port_report[4];

    HL_ld_brightness_flag = 0x55;
    HL_new_value_flag = 1;

	  //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = port_report[3] | 0x80;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = port_report[4];		//-数据长度
			  port_send[0][5] = port_report[5];		//-数据长度

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;
}

void MOD_KG_inquire_version_deal(void)		//-
{
	  WORD the_ram_ax;

	  //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = port_report[3] | 0x80;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = HIBYTE(STM32_SYS_VERSION_NUM);		//-数据长度
			  port_send[0][5] = LOBYTE(STM32_SYS_VERSION_NUM);		//-数据长度

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;
}


void MOD_KG_inquire_update_deal(void)		//-
{
	  WORD the_ram_ax;

    if((port_report[4] == 2) && (port_report[5] == 0x55))
    {
        FLASH_Unlock();						//解锁
				FLASH_ErasePage(APP_CONFIG_ADDR);//擦除这个扇区
				FLASH_ProgramHalfWord(APP_CONFIG_ADDR,0x0000);	//-不能运行可以运行
				FLASH_ErasePage(APP_CONFIG1_ADDR);//擦除这个扇区
				FLASH_ProgramHalfWord(APP_CONFIG2_ADDR,0x0000);	//-启动复制
        FLASH_ProgramHalfWord(APP_CONFIG1_ADDR,0x0000);	//-启动复制
				FLASH_Lock();//上锁

	  //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = port_report[3] | 0x80;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = 0x02;		//-数据长度
			  port_send[0][5] = 0xaa;		//-数据长度

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

        while(1);   //-等待复位重启,进入IAP程序
    }
}

void MOD_KG_set_reset_deal(void)		//-改变呼吸周期
{
	  WORD the_ram_ax;

	  if((port_report[4] == 0x03) && (port_report[5] == 0x55))
	  {


	  //-如果数据成功接收到之后就应答一次
			  port_send[0][0] = 0xaa;		//-两字节包头
			  port_send[0][1] = 0x55;

			  port_send[0][2] = port_report[2];		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[0][3] = port_report[3] | 0x80;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[0][4] = port_report[4];		//-数据长度
			  port_send[0][5] = port_report[5];		//-数据长度

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0][0],port_send[0][2]);
			  port_send[0][6] =LOBYTE(the_ram_ax);
			  port_send[0][7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[0][2] + 2;
			  port_deal_flag = 0xaa;

			  while(1);
		}
}








//-下面处理通讯接收到的内容,其实就是一个单独的上层协议的处理,在主函数里面周期调用
void CDT9702_Main()
{
	 WORD the_ram_ax,the_ram_bx;
	 WORD temp_data,temp_data1,delta_len,temp_loop,temp_int;


	 //-判断发送是否结束了,如果结束了可以正常逻辑处理,否则不处理
	 if(DMA_GetFlagStatus(DMA1_FLAG_TC4))		//-进入说明发送完成了
   {//-只有当传输完成了才可以重新给DMA发送传输命令
   	  //-内容发送完成之后全部清理,由于现在不知道每个的空间大小了,那么选取一块大的清零
   	  for(temp_loop = 0;temp_loop < 43;temp_loop++)
   	  	port_send[0][temp_loop] = 0;
   	  port_deal_flag = 0;		//-给一个非0值就可以再下次检查发送时发发送命令了
   	  //-MOD_KG_transmit_flag=YES;		//-当我的内容发送出去了也可以立即发送内容了??这里应该不能立即组织内容的发送,这样下面的内容会被覆盖,接收的标志位
   	  DMA_ClearFlag(DMA1_FLAG_TC4);
   }

	 if(port_deal_flag == 0)	//-这里会不会由于一次发送失败而导致再也不能发送
   {
   	   //-首先处理接收到的数据,如果没有内容接收到,才组织可能需要发送的内容
   	   if((port_recv_pt[0]!=port_recv_dl[0]) && (MOD_KG_transmit_flag == YES))	//-增加一个判断条件,处理了一个之后一段时间之后才可以处理其它内容
   	   {

   	   	   if(MOD_KG_rxd_head_flag!=YES)	//-接收到的数据还没有处理的时候就是NO
           {
           		 MOD_KG_rxd_wait_time=cticks_ms;	//-这个的目的是为了一但接收不到新数据就从头开始计时,也就是一个有效帧不能断开太长时间
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
	        MOD_KG_transmit_flag=YES;	//-接收到应答就可以发送内容了
	        //-MOD_KG_transmit_wait_time=Time_1ms_Counter;	//-虽然上面说可以发送数据了,但是还是要延时一段时间,因为接收到的数据需要处理
	        //-下面是对实际数据的处理,处理的是核心部分
	        the_ram_bx=(port_recv_dl[0]+3)&0x1ff;;
	        if(port_recv[0][the_ram_bx]!=0xff)	//-这个是对功能码的判断,功能码不同判断的依据也不一样
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
	         	//-port_recv_dl[0]+=delta_len;	//-舍弃了整个缓冲区的报文
	         	port_recv_dl[0]+=(port_report[2] + 2); //-舍弃检测到的有效报文
            port_recv_dl[0]&=0x1ff;
	         	temp_int=MOD_KG_CRC16(&port_report[0],port_report[2]);
	         	if((LOBYTE(temp_int)!=port_report[port_report[2]+0])||(HIBYTE(temp_int)!=port_report[port_report[2]+1]))	//-进行CRC检查
	         	{	//-由于这的不确定性,如果校验不正确的话就直接把头舍掉,继续寻找
	          		goto inrda;	//-到这里说明成功接收到的报文CRC校验没有通过
	          }
	        }
	        else
	        {
	            //-port_recv_dl[0]+=delta_len;	//-这个地方就舍弃了这样的处理报文
	         	  port_recv_dl[0]+=7; //-舍弃检测到的有效报文
              port_recv_dl[0]&=0x1ff;
	            goto inrda;
	        }


////////////////////////////////////////////////////////////////////////////////
					if(port_report[3] == 1)
					{
							MOD_KG_rec_frame_type = MOD_KG_receive_data_FRAME;
							MOD_KG_receive_data_deal();
							MOD_KG_transmit_wait_time=cticks_ms;	//-增加这个时间的目的是给足下面接收端处理时间
					}
					else
					{
							if(port_report[3] == 2)
							{
								 if(port_report[4] == 1)
				      	 {
				      	 	   MOD_KG_rec_frame_type = MOD_KG_polling_ID_FRAME;
				      	 	   MOD_KG_polling_ID_ack();
				      	 	   MOD_KG_transmit_wait_time=cticks_ms;	//-增加这个时间的目的是给足下面接收端处理时间
						    		 MOD_KG_transmit_flag=NO;
				      	 }
				      	 else
				      	 {
				      	 	   MOD_KG_rec_frame_type = MOD_KG_polling_data_FRAME;
				      	 	   MOD_KG_polling_data_ack();
				      	 	   MOD_KG_transmit_wait_time=cticks_ms;	//-增加这个时间的目的是给足下面接收端处理时间
		  					     MOD_KG_transmit_flag=NO;
				      	 }
							}
							else
							{
									if(port_report[3] == 3)
						      {//-接收LED点阵屏图片
						      	  MOD_KG_rec_frame_type = MOD_KG_LED_display_FRAME;
						      	  MOD_KG_LED_display_deal();
						      	  MOD_KG_transmit_wait_time=cticks_ms;	//-增加这个时间的目的是给足下面接收端处理时间
						      }
						      else
						      {
						      		if(port_report[3] == 4)		//-三、屏保时间设置,也需要应答
								      {
								      	  MOD_KG_rec_frame_type = MOD_KG_SET_screen_FRAME;
								      	  MOD_KG_SET_screen_deal();
								      	  MOD_KG_transmit_wait_time=cticks_ms;	//-增加这个时间的目的是给足下面接收端处理时间
                  				MOD_KG_transmit_flag=NO;
								      }
								      else
								      {
								      		if(port_report[3] == 5)		//-接收的炫彩灯设置,这里设置的都是一个定值类的东西,所以需要保证发送成功
										      {
										      	  MOD_KG_rec_frame_type = MOD_KG_receive_set_FRAME;
										      	  MOD_KG_receive_set_deal();
										      	  MOD_KG_transmit_wait_time=cticks_ms;	//-增加这个时间的目的是给足下面接收端处理时间
 						                  MOD_KG_transmit_flag=NO;
										      }
										      else
										      {
										      		if(port_report[3] == 6)		//-接收的炫彩模式设置,这里设置的都是一个定值类的东西,所以需要保证发送成功
												      {
												      	  MOD_KG_rec_frame_type = MOD_KG_mode_set_FRAME;
												      	  MOD_KG_mode_set_deal();
												      	  MOD_KG_transmit_wait_time=cticks_ms;	//-增加这个时间的目的是给足下面接收端处理时间
 								                  MOD_KG_transmit_flag=NO;
												      }
												      else
												      {
												      		if(port_report[3] == 8)		//-接收的炫彩模式设置,这里设置的都是一个定值类的东西,所以需要保证发送成功
														      {
														      	  MOD_KG_rec_frame_type = MOD_KG_chage_page_FRAME;
														      	  MOD_KG_chage_page_deal();
														      	  MOD_KG_transmit_wait_time=cticks_ms;	//-增加这个时间的目的是给足下面接收端处理时间
														      }
														      else
														      {
														      		if(port_report[3] == 9)		//-接收的炫彩模式设置,这里设置的都是一个定值类的东西,所以需要保证发送成功
																      {
																      	  MOD_KG_rec_frame_type = MOD_KG_chage_pic_FRAME;
																      	  MOD_KG_chage_pic_deal();
																      	  MOD_KG_transmit_wait_time=cticks_ms;	//-增加这个时间的目的是给足下面接收端处理时间
     													     	      MOD_KG_transmit_flag=NO;
																      }
																      else
																      {
																      		if(port_report[3] == 10)		//-接收的炫彩模式设置,这里设置的都是一个定值类的东西,所以需要保证发送成功
																		      {
																		      	  MOD_KG_rec_frame_type = MOD_KG_SET_loop_FRAME;
																		      	  MOD_KG_SET_loop_deal();
																		      	  MOD_KG_transmit_wait_time=cticks_ms;	//-增加这个时间的目的是给足下面接收端处理时间
     															     	      MOD_KG_transmit_flag=NO;
																		      }
																		      else
																		      {
																		      		if(port_report[3] == 11)		//-接收的炫彩模式设置,这里设置的都是一个定值类的东西,所以需要保证发送成功
																				      {
																				      	  MOD_KG_rec_frame_type = MOD_KG_send_state_FRAME;
																				      	  MOD_KG_send_state_deal();
																				      	  MOD_KG_transmit_wait_time=cticks_ms;	//-增加这个时间的目的是给足下面接收端处理时间
 																	         	      MOD_KG_transmit_flag=NO;
																				      }
																				      else
																				      {
																				      		if(port_report[3] == 12)		//-接收的炫彩模式设置,这里设置的都是一个定值类的东西,所以需要保证发送成功
																						      {
																						      	  MOD_KG_rec_frame_type = MOD_KG_re_Weather_FRAME;
																						      	  MOD_KG_re_Weather_deal();
																						      	  MOD_KG_transmit_wait_time=cticks_ms;	//-增加这个时间的目的是给足下面接收端处理时间
 																			         	      MOD_KG_transmit_flag=NO;
																						      }
																						      else
																						      {
																						      		if(port_report[3] == 13)		//-指定显示哪一页传感器数据
																								      {//?报文内容的丢失是仅仅接收到一次还是根本没有接收到,实际情况是接收到了,但是本处理函数没有正确识别到.
																								      	  MOD_KG_rec_frame_type = MOD_KG_re_show_FRAME;
																								      	  MOD_KG_re_show_deal();
																								      	  MOD_KG_transmit_wait_time=cticks_ms;	//-增加这个时间的目的是给足下面接收端处理时间
																				          	      MOD_KG_transmit_flag=NO;
																								      }
																								      else
																								      {
																								      		if(port_report[3] == 14)		//-指定显示哪一页传感器数据
																										      {
																										      	  MOD_KG_rec_frame_type = MOD_KG_polling_state_FRAME;
																										      	  MOD_KG_polling_state_deal();
																										      	  MOD_KG_transmit_wait_time=cticks_ms;	//-增加这个时间的目的是给足下面接收端处理时间
																						          	      MOD_KG_transmit_flag=NO;
																										      }
																										      else
																										      {
																										      		if(port_report[3] == 15)		//-
																												      {
																												      	  MOD_KG_rec_frame_type = MOD_KG_polling_PM25_FRAME;
																												      	  MOD_KG_polling_PM25_deal();
																												      	  MOD_KG_transmit_wait_time=cticks_ms;	//-增加这个时间的目的是给足下面接收端处理时间
																												      }
																												      else
																												      {
																												      		if(port_report[3] == 16)		//-
																														      {
																														      	  MOD_KG_rec_frame_type = MOD_KG_end_PM25_FRAME;
																														      	  MOD_KG_end_PM25_deal();
																														      	  MOD_KG_transmit_wait_time=cticks_ms;	//-增加这个时间的目的是给足下面接收端处理时间
																														      }
																														      else
																														      {
																														      		if(port_report[3] == 17)		//-
																																      {
																																      	  MOD_KG_rec_frame_type = MOD_KG_polling_hl_FRAME;
																																      	  MOD_KG_polling_hl_deal();
																																      	  MOD_KG_transmit_wait_time=cticks_ms;	//-增加这个时间的目的是给足下面接收端处理时间
 																													         	      MOD_KG_transmit_flag=NO;
																																      }
																														      	  else
																														      	  {
																														      	  	  if(port_report[3] == 19)		//-
																																		      {
																																		      	  MOD_KG_rec_frame_type = MOD_KG_SET_hl_time_FRAME;
																																		      	  MOD_KG_SET_hl_time_deal();
																																		      	  MOD_KG_transmit_wait_time=cticks_ms;	//-增加这个时间的目的是给足下面接收端处理时间
																														          	      MOD_KG_transmit_flag=NO;
																																		      }
																														      	      else
																														      	      {
																														      	      		if(port_report[3] == 0x94)		//-接收的7620的确认应答,和其他有点不同
																																				      {
																																				      	  MOD_KG_rec_frame_type = MOD_KG_send_leddis_flag_FRAME;
																																				      	  MOD_KG_send_leddis_flag_deal();
																																				      	  MOD_KG_transmit_wait_time=cticks_ms;	//-增加这个时间的目的是给足下面接收端处理时间
																																				      }
																																				      else
																																				      {
																																				      		if(port_report[3] == 0x15)		//-
																																						      {
																																						      	  MOD_KG_rec_frame_type = MOD_KG_SET_voice_flag_FRAME;
																																						      	  MOD_KG_SET_voice_flag_deal();
																																						      	  MOD_KG_transmit_wait_time=cticks_ms;	//-增加这个时间的目的是给足下面接收端处理时间
 																																			         	      MOD_KG_transmit_flag=NO;
																																						      }
																																						      else
																																						      {
																																						      		if(port_report[3] == 0x16)		//-
																																								      {
																																								      	  MOD_KG_rec_frame_type = MOD_KG_polling_voice_flag_FRAME;
																																								      	  MOD_KG_polling_voice_flag_deal();
																																								      	  MOD_KG_transmit_wait_time=cticks_ms;	//-增加这个时间的目的是给足下面接收端处理时间
 																																					         	      MOD_KG_transmit_flag=NO;
																																								      }
																																								      else
																																								      {
																																								      		if(port_report[3] == 0x17)		//-
																																										      {
																																										      	  MOD_KG_rec_frame_type = MOD_KG_re_gateway_flag_FRAME;
																																										      	  MOD_KG_re_gateway_flag_deal();
																																										      	  MOD_KG_transmit_wait_time=cticks_ms;	//-增加这个时间的目的是给足下面接收端处理时间
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
																																														      	  MOD_KG_transmit_wait_time=cticks_ms;	//-增加这个时间的目的是给足下面接收端处理时间
																																														      }
																																														      else
																																														      {
																																														      		if(port_report[3] == 0xC0)		//-主板的应答
																																																      {
																																																      	  MOD_KG_rec_frame_type = MOD_KG_control_HRL_FRAME;
																																																      	  MOD_KG_control_HRL_deal();
																																																      	  MOD_KG_transmit_wait_time=cticks_ms;	//-增加这个时间的目的是给足下面接收端处理时间
																																																      }
																																																      else
																																																      {
																																																      		if(port_report[3] == 0xC1)		//-接收的7620的确认应答,和其他有点不同
																																																		      {
																																																		      	  MOD_KG_rec_frame_type = MOD_KG_leddis_end_FRAME;
																																																		      	  MOD_KG_leddis_end_deal();
																																																		      	  MOD_KG_transmit_wait_time=cticks_ms;	//-增加这个时间的目的是给足下面接收端处理时间
																																																		      }
																																																		      else
																																																		      {
																																																		      		if(port_report[3] == 0x9A)		//-接收的7620的确认应答,和其他有点不同
																																																				      {
																																																				      	  MOD_KG_rec_frame_type = MOD_KG_send_HRL_mode_FRAME;
																																																				      	  MOD_KG_send_HRL_mode_deal();
																																																				      	  MOD_KG_transmit_wait_time=cticks_ms;	//-增加这个时间的目的是给足下面接收端处理时间
																																																				      }
																																																				      else
																																																				      {
                                                                                                                  if(port_report[3] == 0x21)
                                                                                                                  {
                                                                                                                      MOD_KG_rec_frame_type = MOD_KG_set_hl_set_FRAME;
                                                                                                                      MOD_KG_set_hl_set_deal();
                                                                                                                      MOD_KG_transmit_wait_time=cticks_ms;	//-增加这个时间的目的是给足下面接收端处理时间
                                                                                                                  }
                                                                                                                  else
                                                                                                                  {
                                                                                                                  	 if(port_report[3] == 0x1C)		//-软复位命令
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

    			MOD_KG_clear_port_report_deal();		//-每次处理完毕之后清除缓存区防止错误使用

    	}
rxd_out_time:	//-执行到这里说明接收超时,或处理成功,反正现在可以继续发送了,,正常情况都会执行这里的,首先
	    if(Judge_Time_In_MainLoop(MOD_KG_rxd_wait_time,MOD_KG_WAIT_TIME_VALUE)==YES)	//-正常通信的话是不应该超时的,若过时就认为出错,下面重新初始化
	    {	//-当发送一个报文之后,超过等待回答时间之后,就可以从新发送一次
	      	MOD_KG_rec_OK=NO;
	      	MOD_KG_rxd_head_flag=NO;
	      	MOD_KG_rxd_wait_time=cticks_ms;
	      	//-MOD_KG_transmit_flag=YES;	//-超时没有接收到也可以组织发送内容
	      	//-MOD_KG_wait_replay=NO;	//-表示现在还没有等待回复
	      	//-MOD_KG_transmit_wait_time=Time_1ms_Counter;

	       	//-MOD_KG_comm_err_counter[port_send[0][0]]++;	//-超时出错计数
	    }
	    if(Judge_Time_In_MainLoop(MOD_KG_transmit_wait_time,75)==YES)
	    	MOD_KG_transmit_flag=YES;	//-一但接收到有效报文,那么80ms时间后才可以处理下次可能的报文
inrda:
		   //-主动发送 ,,这里就有一个问题,多长时间发送一次,或者说这个发送触发条件是什么
		   if((UART1_transmit_flag==YES) && (MOD_KG_transmit_flag == YES))		//-目前无所谓的是双向的,但是后面需要增加一个条件,保证发送缓冲区中的内容发送出去了才可以组织内容,这里不需要等待到应答
		   {
		   	  if(Judge_Time_In_MainLoop(MOD_KG_transmit_wait_time,100)==YES)	//-只有过了等待时间才会发送,这里时间主要是为了给接收端足够的处理时间
			   	  switch(UART1_transmit_control)
	          {
	                case 1:
	                     //MOD_KG_rec_frame_type=MOD_KG_YX_FRAME;	//-表示需要接收的帧的类型是这个值,接收到这样的值才是正确的
	                     //MOD_KG_wait_replay=YES;
	                     MOD_KG_polling_PM25_cmd();	//-本规约定义的内容是,问答型的,这个端口上的所有装置都是接收查询
	                     MOD_KG_transmit_wait_time=cticks_ms;

	                     UART1_transmit_flag = NO;		//-这里复杂了就不等待应答了直接发以便结束
    									 UART1_transmit_control = 0;
	                     //-MOD_KG_send_sense_data();
	                     //-port_send_sense_data[0] = 0;
	                     //MOD_KG_now_poll_addr++;	//-从机地址

	                     //-MOD_KG_transmit_flag=NO;	//-现在开始不能发送报文了
	                     //-MOD_KG_rxd_head_flag=NO;	//-为类同步准备的,但是并不是所有的东西都是那样
	                     //-MOD_KG_rxd_wait_time=cticks_ms;	//-应答报文也是有时效性的

	                     //-MOD_KG_transmit_control=1;	//-所有的端口装置查询一遍之后,再换下一类报文查询

	                     break;
	                case 2:
	                     //MOD_KG_rec_frame_type=MOD_KG_YX_FRAME;	//-表示需要接收的帧的类型是这个值,接收到这样的值才是正确的
	                     //MOD_KG_wait_replay=YES;
	                     MOD_KG_end_PM25_cmd();	//-本规约定义的内容是,问答型的,这个端口上的所有装置都是接收查询
	                     MOD_KG_transmit_wait_time=cticks_ms;

	                     UART1_transmit_flag = NO;		//-这里复杂了就不等待应答了直接发以便结束
    									 UART1_transmit_control = 0;
	                     //-MOD_KG_send_sense_data();
	                     //-port_send_sense_data[0] = 0;
	                     //MOD_KG_now_poll_addr++;	//-从机地址

	                     //-MOD_KG_transmit_flag=NO;	//-现在开始不能发送报文了
	                     //-MOD_KG_rxd_head_flag=NO;	//-为类同步准备的,但是并不是所有的东西都是那样
	                     //-MOD_KG_rxd_wait_time=cticks_ms;	//-应答报文也是有时效性的

	                     //-MOD_KG_transmit_control=1;	//-所有的端口装置查询一遍之后,再换下一类报文查询

	                     break;
	                case 3:		//-关闭主板跑马灯

	                	   MOD_KG_end_HRL_cmd();	//-本规约定义的内容是,问答型的,这个端口上的所有装置都是接收查询
	                     MOD_KG_transmit_wait_time=cticks_ms;
	                	   break;
	                case 4:		//-允许主板跑马灯

	                	   MOD_KG_can_HRL_cmd();	//-本规约定义的内容是,问答型的,这个端口上的所有装置都是接收查询
	                     MOD_KG_transmit_wait_time=cticks_ms;
	                	   break;
	                case 5:		//-发送目前点阵屏所处的状态和显示内容

	                	   MOD_KG_send_leddis_flag_cmd();	//-本规约定义的内容是,问答型的,这个端口上的所有装置都是接收查询
	                     MOD_KG_transmit_wait_time=cticks_ms;
	                	   break;
	                case 6:		//-发送目前点阵屏所处的状态和显示内容

	                	   MOD_KG_send_leddis_end_cmd();	//-本规约定义的内容是,问答型的,这个端口上的所有装置都是接收查询
	                     MOD_KG_transmit_wait_time=cticks_ms;
	                	   break;
	                case 7:		//-发送目前点阵屏所处的状态和显示内容

	                	   MOD_KG_send_HRL_mode_cmd();	//-本规约定义的内容是,问答型的,这个端口上的所有装置都是接收查询
	                     MOD_KG_transmit_wait_time=cticks_ms;
	                	   break;
	                case 8:
	                     MOD_KG_control_FUN_cmd();	//-本规约定义的内容是,问答型的,这个端口上的所有装置都是接收查询
	                     MOD_KG_transmit_wait_time=cticks_ms;

	                     UART1_transmit_flag = NO;		//-这里复杂了就不等待应答了直接发以便结束
    									 UART1_transmit_control = 0;
	                     break;
	                case 9:			//-主动发送状态,以便统一状态信息
	                     MOD_KG_update_status_cmd();	//-本规约定义的内容是,问答型的,这个端口上的所有装置都是接收查询
	                     MOD_KG_transmit_wait_time=cticks_ms;

	                     UART1_transmit_flag = NO;		//-这里复杂了就不等待应答了直接发以便结束
    									 UART1_transmit_control = 0;
	                     break;
	                default:
	                     break;
	          }

		   }
 	 }

}


//-新的协议不需要等待应答,所以仅仅等待完整报文,防止错误锁死就行
//-这里需要等待应答,所以需要延时发送

//-2016/1/8 9:02:10
//-分析到现在7620已经把内容重复传送了3次,主板STM32也已经接收到了,射灯板STM32也接收到了,那么这里的发送出去了吗?

