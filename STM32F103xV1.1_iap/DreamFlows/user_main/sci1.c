/*
接收:使用一个常规逻辑,开辟一个512字节的缓冲区,然后循环存储
发送:也使用512字节的空间保存内容,然后置一个发送标志,直到内容发送出去为止

串口3 作为射灯板和7620之间通讯透传口,现在决定不进行任何处理,仅仅双向透传
7620发来的需要传递到射灯板的东西,直接让串口3传送
STM32传出的数据,直接在串口3中处理,不传递任何数据,仅仅置标志位让串口2自己组织数据向7620发送
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

//-为协议处理临时定义的变量
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

  //-协议初始化内容
  port_deal_flag[1] = 0;
  MOD_TC_transmit_flag=YES;
  MOD_TC_transmit_control = 1;

}


void UART3_Rx_Deal(void)
{
  port_recv_pt[1] = 512 - DMA_GetCurrDataCounter(DMA1_Channel3);	//-得到接收个数,如果作为数组偏移量的话,现在的数值就是指向待存空间的
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
  //-USART_DMACmd(USARTy, USART_DMAReq_Tx, ENABLE);			//-启动DMA发送

  if(port_send_len[1] != 0)	//-一定要保证内容发送出去了,再使用DMA发送数据
  {
		  DMA_Cmd(DMA1_Channel2, DISABLE);
		  DMA1_Channel2->CNDTR = port_send_len[1]; //-传输数量寄存器,指示剩余的待传输字节数目
		  DMA_Cmd(DMA1_Channel2, ENABLE);
		  port_send_len[1] = 0;
	}
}

void RunLed_stata_judge_new(BYTE page,BYTE *data) //-为了同步上面数据的指示灯
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
例子1:查询传感器设备（ID）
7620发送：
0xaa 0x55 0x05 0x02 0x01 CRC
STM32回应：
0xaa 0x55 0x0b 0x02 0x01 0x01 0x02 0x03 0x04 0x05 0x06 CRC






*/
unsigned int MOD_TC_CRC16(unsigned char *MOD_KG_CRC16_start,unsigned char MOD_KG_CRC16_bytes)    //*x为指向每行前5个数据的指针
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

void MOD_TC_LED_display_deal(void)		//-对7620下达的数据进行接收
{
	  //-WORD the_ram_ax;

	  //-特别定义一个缓冲区存储数据,置标志位,然后刷新的时候检查这里的数据是否有效
	  //-如果有的话就切换显示

}

void MOD_TC_data_ack_deal(void)
{
	Device_communication_cn = 0;
}


void MOD_TC_SET_screen_deal(void)		//-设置屏保时间
{
	  //-WORD the_ram_ax;

	  if(port_report[4] == 0x01)		//-
	  {
	  	 /*led_display_long = port_report[5];	//-屏保时间
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
			  port_deal_flag[1] = 0xaa;*/
			  //-有内容返回,判断下是否正确如果正确的话,向7620发送一个确认短帧就可以
			  //-0xaa 0x55 0x06 0x84 0xaa 0xaa XX XX
			  UART3_TO_UART2_FLAG = port_report[3] | 0x80;
	  }

}

void MOD_TC_receive_set_deal(void)		//-炫彩模式设置
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

void MOD_TC_mode_set_deal(void)		//-炫彩模式设置
{
	  //-WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	  HL_flag	= port_report[4];		//-把接收到的模式进行返回
			  UART3_TO_UART2_FLAG = port_report[3] | 0x80;
	  }

}

void MOD_TC_chage_pic_deal(void)		//-炫彩模式设置
{
	  //-WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {

			  UART3_TO_UART2_FLAG = port_report[3] | 0x80;
	  }

}

void MOD_TC_SET_loop_deal(void)		//-炫彩模式设置
{
	  //-WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {

			  UART3_TO_UART2_FLAG = port_report[3] | 0x80;
	  }

}

void MOD_TC_send_state_deal(void)		//-炫彩模式设置
{
	  //-WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {

			  UART3_TO_UART2_FLAG = port_report[3] | 0x80;
	  }

}

void MOD_TC_re_Weather_deal(void)		//-炫彩模式设置
{
	  //-WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {

			  UART3_TO_UART2_FLAG = port_report[3] | 0x80;
	  }

}

void MOD_TC_re_show_deal(void)		//-炫彩模式设置
{
	  //-WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {

			  UART3_TO_UART2_FLAG = port_report[3] | 0x80;
	  }
	   //-test_cn[2]++;
     //-test_cn[4] = 3;
}

void MOD_TC_polling_state_deal(void)		//-查询射灯板内部器件状态,返回自检状态值
{
	  //-WORD the_ram_ax;

	  //-if(port_report[4] == 0x01)		//-
	  {
	  	  STM32_shedeng_selfT_flag1 = port_report[4];
			  UART3_TO_UART2_FLAG = port_report[3] | 0x80;
	  }

}

void MOD_TC_polling_PM25_deal(void)		//-接收到这样的内容然后,置标志位就可以了
{
	  //-WORD the_ram_ax;

	  FAN_RUN_flag = 1;		//-开始运行风扇
	  FAN_RUN_wait_time = Time_2048ms_Counter;

    if((port_report[4] & 0x80) != 0)  //-判别是否上送,若需要语音播报就上送
    {
	    UART3_TO_UART2_FLAG = 0x94;		//-特殊处理
	    UART1_led_disdata00 = (port_report[4] & 0x7f);
	    UART1_led_disdata01 = port_report[5];
	    UART1_led_disdata02 = port_report[6];

      RunLed_stata_judge_new(UART1_led_disdata00,&port_report[5]);
      UART1_sloop_flag = 1;		//-推迟数据库更新
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

void MOD_TC_control_FUN_deal(void)		//-接收到这样的内容然后,置标志位就可以了
{
	  //-WORD the_ram_ax;

	  FAN_RUN_flag = 1;		//-开始运行风扇
	  FAN_RUN_wait_time = Time_2048ms_Counter;

	  RunLed_stata_num = port_report[5];

		HRL_RUN_flag = 0;	//-刚刚翻转到这一页的时候,可能还在刷屏状态,这样就该先关闭环境指示灯,
}

void MOD_TC_update_status_deal(void)		//-接收到这样的内容然后,置标志位就可以了
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
	  	 	  FAN_RUN_wait_flag = 1;		//-关闭风扇开始计时
	  	    FAN_RUN_wait_time = Time_2048ms_Counter;
	  	 }
	  }
	  else
	  {
	  	 FAN_RUN_wait_flag = 1;		//-关闭风扇开始计时
	  	 FAN_RUN_wait_time = Time_2048ms_Counter;
	  }

	  if(port_report[6] != 0)
	  	 HRL_RUN_flag = 0;			//-每次变化都要决定是否熄灭跑马灯

	  led_display_start = 0x55;

	  //-决定是否播报
	  if(port_report[9] == 0x55)
	  {
	  	 UART3_TO_UART2_FLAG = 0x94;
	     UART1_led_disdata00 = port_report[5];
	     UART1_led_disdata01 = port_report[7];
	     UART1_led_disdata02 = port_report[8];

       RunLed_stata_judge_new(UART1_led_disdata00,&port_report[7]);
       UART1_sloop_flag = 1;		//-推迟数据库更新
       UART1_sloop_wait_time = cticks_ms;
	  }
}

void MOD_TC_end_PM25_deal(void)		//-接收到应答之后就不再,发送命令
{
	  //-WORD the_ram_ax;

	  //-UART1_transmit_flag = NO;		//-可以组织内容发送
    //-UART1_transmit_control = 0;	//-决定了主动发送的内容
    //-led_display_start = 0;

}

void MOD_TC_control_HRL_deal(void)		//-接收到这样的内容然后,置标志位就可以了
{
	  //-WORD the_ram_ax;

	  if(port_report[4] == 0x55)
	  	 HRL_RUN_flag = 0;	//-炫彩灯亮了,关闭跑马灯
	  //-else if(port_report[4] == 0xaa)		//-目前只保证不同时亮,就不启动了.
	  //-	 HRL_RUN_flag = 0x55;		//-

	  UART3_TO_UART2_FLAG = port_report[3] | 0x80;

}

void MOD_TC_send_leddis_flag_deal(void)		//-点阵屏显示页切换会发送这样的命令
{
	  //-WORD the_ram_ax;

	  if(port_report[7] == 0x55)		//-可以组织内容发送
	  {//-等于0x55才转发后天供语音播报
	    UART3_TO_UART2_FLAG = port_report[3] | 0x80;
	    UART1_led_disdata00 = port_report[4];
	    UART1_led_disdata01 = port_report[5];
	    UART1_led_disdata02 = port_report[6];

      //-上面主动推送下来数据的时候,环境指示灯,就需要根据上面的情况变换一下
      RunLed_stata_judge_new(UART1_led_disdata00,&port_report[5]);
      UART1_sloop_flag = 1;		//-推迟数据库更新
      UART1_sloop_wait_time = cticks_ms;
  	}

    HL_flag = port_report[8];
  	RunLed_stata_num = port_report[4];
    if(((port_report[4] == 2) || (port_report[4] == 1)) && (HL_flag == 0))
      HRL_RUN_flag = 0xaa;		//-如果这里有需要的需要排除显示的
    else
	  	HRL_RUN_flag = 0;

	  if((port_report[4] != 0) || (port_report[4] != 2)	|| (port_report[4] != 3))
	  {
	  	 FAN_RUN_wait_time = Time_2048ms_Counter;
		   FAN_RUN_wait_flag = 1;
	  }

	  led_display_start = 0x55;
}

void MOD_TC_end_HRL_deal(void)		//-接收到应答之后就不再,发送命令
{
	  //-WORD the_ram_ax;


  	//-RunLed_stata_num = port_report[4];
  	HRL_RUN_flag = 0;		//-当点阵屏发送熄灭后就会发送这样的状态值,这个时候允许亮加网灯
  	UART3_TO_UART2_FLAG = port_report[3] | 0x80;

  	FAN_RUN_wait_time = Time_2048ms_Counter;
		FAN_RUN_wait_flag = 1;

		led_display_start = 0;

}

void MOD_TC_send_HRL_mode_deal(void)		//-敲击会发送现在的炫彩灯模式
{
	  //-WORD the_ram_ax;


  	if(port_report[5] !=0)
  		HRL_RUN_flag = 0;		//-只要炫彩灯亮着就不会亮跑马灯
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

void MOD_TC_polling_hl_deal(void)		//-接收到应答之后就不再,发送命令
{
	  //-WORD the_ram_ax;

	  HL_flag = port_report[4];		//-可以组织内容发送
    UART3_TO_UART2_FLAG = port_report[3] | 0x80;

}

void MOD_TC_SET_hl_time_deal(void)		//-接收到应答之后就不再,发送命令
{
	  //-WORD the_ram_ax;

	  //-HL_flag = port_report[4];		//-可以组织内容发送
    UART3_TO_UART2_FLAG = port_report[3] | 0x80;

}

void MOD_TC_SET_voice_flag_deal(void)		//-接收到应答之后就不再,发送命令
{
	  //-WORD the_ram_ax;


    UART3_TO_UART2_FLAG = port_report[3] | 0x80;		//-可以或也可以不或,前面已经或过了

}

void MOD_TC_polling_voice_flag_deal(void)		//-接收到应答之后就不再,发送命令
{
	  //-WORD the_ram_ax;


    UART3_TO_UART2_FLAG = port_report[3] | 0x80;
	  voice_flag = port_report[4];
}

void MOD_TC_re_gateway_flag_deal(void)		//-接收到应答之后就不再,发送命令
{
	  //-WORD the_ram_ax;


    UART3_TO_UART2_FLAG = port_report[3] | 0x80;
}

void MOD_TC_send_voice_mode_deal(void)		//-接收到应答之后就不再,发送命令
{
	  //-WORD the_ram_ax;


    UART3_TO_UART2_FLAG = port_report[3] | 0x80;
}

void MOD_TC_set_hl_set_deal(void)		//-接收到应答之后就不再,发送命令
{
	  //-WORD the_ram_ax;

		HL_ld_brightness = port_report[4];
    UART3_TO_UART2_FLAG = port_report[3] | 0x80;
}






//-下面处理通讯接收到的内容,其实就是一个单独的上层协议的处理,在主函数里面周期调用
void LDISP_Main()
{
	 WORD the_ram_ax,the_ram_bx,the_ram_cx,i;
	 WORD temp_data,temp_data1,delta_len,temp_loop,temp_int;


	 //-判断发送是否结束了,如果结束了可以正常逻辑处理,否则不处理
	 if(DMA_GetFlagStatus(DMA1_FLAG_TC2))		//-进入说明发送完成了
   {//-只有当传输完成了才可以重新给DMA发送传输命令
   	  for(temp_loop = 0;temp_loop < 43;temp_loop++)
   	  	UART3_port_send[temp_loop] = 0;
   	  port_deal_flag[1] = 0;		//-给一个非0值就可以再下次检查发送时发发送命令了
   	  DMA_ClearFlag(DMA1_FLAG_TC2);
   }

	 if((port_deal_flag[1] == 0) && (UART3_TO_UART2_FLAG == 0))	//-这里会不会由于一次发送失败而导致再也不能发送,,增加UART3_TO_UART2_FLAG为0开始处理接收到的报文,防止,有内容没有发送出去覆盖了,造成的丢失
   {
   	   //-首先处理接收到的数据,如果没有内容接收到,才组织可能需要发送的内容
   	   if(port_recv_pt[1]!=port_recv_dl[1])
   	   {

   	   	   if(MOD_TC_rxd_head_flag==NO)	//-接收到的数据还没有处理的时候就是NO
           {
           	   MOD_TC_rxd_wait_time=cticks_ms;	//-仅仅是为了接收完整
		   	   	   temp_data = port_recv_pt[1];
		   	   	   temp_data1 = port_recv_dl[1];
		           if(temp_data1>temp_data)	//-前面的是处理指针比实际的接收指针进行比较
		               delta_len=(temp_data+512)-temp_data1;
		           else
		               delta_len=temp_data-temp_data1;	//-一共的长度
		           for(temp_loop=temp_data1;temp_loop<(delta_len+temp_data1);temp_loop++)
		           {
		        	   if(port_recv[1][port_recv_dl[1]]==0xaa)	//-这个地方比较的是从站地址,但是我觉得没有任何规律就是通讯
		        	   {	//-利用一切可以利用的
		        	     the_ram_ax=(port_recv_dl[1]+1)&0x1ff;
		        	     if(temp_data == the_ram_ax)	//-如果预取的指针指向空的空间,说明数据还没有到,需要等待
		        	     	 break;
		        	     if(port_recv[1][the_ram_ax]==0x55)	//-比较的是功能码
		        	     {
		        	         MOD_TC_rxd_head_flag=YES;	//-表示已经成功识别接收到的新报文的头了
		        	         break;
		        	     }
		        	   }
		        	   port_recv_dl[1]++;	//-舍弃一个字的报文
		        	   port_recv_dl[1]&=0x1ff;
		           }
   	   	   }
   	   	   if(MOD_TC_rxd_head_flag==YES)	//-接收到的数据还没有处理的时候就是NO
       		 {
       		 	   temp_data = port_recv_pt[1];
       		 	   if(port_recv_dl[1]>temp_data)
               		delta_len=(temp_data+512)-port_recv_dl[1];
               else
               		delta_len=temp_data-port_recv_dl[1];

               if(delta_len>6)	//-至少还有4个字节才能组织一包内容
		           {
		               temp_int=(port_recv_dl[1]+2)&0x1ff;
		               if(delta_len>=(unsigned short)(port_recv[1][temp_int]+2))	//-得到的报文长度和理论上的报文长度进行比较
		               {
		                  MOD_TC_rxd_head_flag=NO;
		                  MOD_TC_rec_OK=YES;
                      goto rec_ok_deal;	//-经过重重考核,到这里就认为是成功接收到一个返回报文了
		               }

		           }

     			 }
   	   }
   	   goto rxd_out_time;		//?由于这个不是简单的主从模式,所以直接查询发送
rec_ok_deal:
	    if(MOD_TC_rec_OK==YES)	//-肯定是防止跑飞的.
	    {	//-到这里就可以说明应答报文已经可以了
	        MOD_TC_rec_OK=NO;	//-成功接收到的数据开始处理了之后,就恢复0
	        MOD_TC_transmit_flag=YES;
	        //-MOD_TC_transmit_wait_time=Time_1ms_Counter;	//-虽然上面说可以发送数据了,但是还是要延时一段时间,因为接收到的数据需要处理
	        //-下面是对实际数据的处理,处理的是核心部分
	        the_ram_bx=(port_recv_dl[1]+3)&0x1ff;;
	        if(port_recv[1][the_ram_bx]!=0xFF)	//-这个是对功能码的判断,功能码不同判断的依据也不一样
	        {	//-这里是宁外一种处理现在可以不管
	          	the_ram_ax=(port_recv_dl[1]+2)&0x1ff;
	          	temp_int=port_recv[1][the_ram_ax]+2+port_recv_dl[1];
	          	for(temp_loop=port_recv_dl[1];temp_loop<temp_int;temp_loop++)	//-上面这样干的秘密就是保证定位到需要处理的报文字节
	          	{	//-简单的不需要这样处理但是复杂的还是需要的,那么这样用了得话兼容性就会很好
	                 if(temp_loop<=511)
	           	       port_report[temp_loop-port_recv_dl[1]]=port_recv[1][temp_loop];
	                 else
	           	       port_report[temp_loop-port_recv_dl[1]]=port_recv[1][temp_loop-512];	//-难道是高速更新的缘故需要提前复制出来
	          	}	//-或者还有一种可能性就是统一处理
	         	//-port_recv_dl[1]+=delta_len;	//-这个地方就舍弃了这样的处理报文
            port_recv_dl[1]+=(port_report[2] + 2);
	         	port_recv_dl[1]&=0x1ff;
	         	temp_int=MOD_TC_CRC16(&port_report[0],port_report[2]);
	         	if((LOBYTE(temp_int)!=port_report[port_report[2]+0])||(HIBYTE(temp_int)!=port_report[port_report[2]+1]))	//-进行CRC检查
	         	{	//-由于这的不确定性,如果校验不正确的话就直接把头舍掉,继续寻找
	          		goto inrda;	//-到这里说明成功接收到的报文CRC校验没有通过
	          }
	        }
	        else
	        {
	            //-port_recv_dl[1]+=delta_len;	//-目前直接舍弃
              port_recv_dl[1]+=7;
	            port_recv_dl[1]&=0x1ff;
	            goto inrda;
	        }


      //-临时先赋值,后面这个变量可以处理逻辑
      //-可以组合判断下接收报文,然后决定下面走向
      //-test_cn[5]++;
    	//-正确内容的处理
////////////////////////////////////////////////////////////////////////////////
			if(port_report[3] == 0x81)	//-判断功能码
			{
					MOD_TC_data_ack_deal();
			}
			else
			{
				if(port_report[3] == 2)	//-判断功能码
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
	      {//-接收LED点阵屏图片
	      	if(port_report[3] == 3)
	      	{
	      	  MOD_TC_rec_frame_type = MOD_KG_LED_display_FRAME;
	      	  //-不需要应答
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
						      {//-接收到屏保设置时间的,返回报文,这个报文,需要传递给7620
						      	  MOD_TC_rec_frame_type = MOD_KG_receive_set_FRAME;
						      	  //-
                  		MOD_TC_receive_set_deal();
						      }
						      else
						      {
						      		if(port_report[3] == 0x86)		//-接收的炫彩模式设置,这里设置的都是一个定值类的东西,所以需要保证发送成功
								      {
								      	  MOD_TC_rec_frame_type = MOD_KG_mode_set_FRAME;
								      	  //-
                  				MOD_TC_mode_set_deal();
								      }
								      else
								      {
								      	if(port_report[3] == 0x89)		//-接收的炫彩模式设置,这里设置的都是一个定值类的东西,所以需要保证发送成功
									      {
									      	  MOD_TC_rec_frame_type = MOD_KG_chage_pic_FRAME;
									      	  MOD_TC_chage_pic_deal();
									      }
									      else
									      {
									      	if(port_report[3] == 0x8A)		//-接收的炫彩模式设置,这里设置的都是一个定值类的东西,所以需要保证发送成功
										      {
										      	  MOD_TC_rec_frame_type = MOD_KG_SET_loop_FRAME;
										      	  MOD_TC_SET_loop_deal();
										      }
										      else
										      {
										      	if(port_report[3] == 0x8B)		//-接收的炫彩模式设置,这里设置的都是一个定值类的东西,所以需要保证发送成功
											      {
											      	  MOD_TC_rec_frame_type = MOD_KG_send_state_FRAME;
											      	  MOD_TC_send_state_deal();
											      }
											      else
											      {
											      	if(port_report[3] == 0x8C)		//-接收的炫彩模式设置,这里设置的都是一个定值类的东西,所以需要保证发送成功
												      {
												      	  MOD_TC_rec_frame_type = MOD_KG_re_Weather_FRAME;
												      	  MOD_TC_re_Weather_deal();
												      }
												      else
												      {
												      	if(port_report[3] == 0x8D)		//-接收的炫彩模式设置,这里设置的都是一个定值类的东西,所以需要保证发送成功
													      {
													      	  MOD_TC_rec_frame_type = MOD_KG_re_show_FRAME;
													      	  MOD_TC_re_show_deal();
													      }
													      else
													      {
													      	if(port_report[3] == 0x8E)		//-接收的炫彩模式设置,这里设置的都是一个定值类的东西,所以需要保证发送成功
														      {
														      	  MOD_TC_rec_frame_type = MOD_KG_polling_state_FRAME;
														      	  MOD_TC_polling_state_deal();
														      }
														      else
														      {
														      	if(port_report[3] == 0x0F)		//-接收到射灯板发送过来的命令,不需要向7620转发自己知道即可
															      {
															      	  MOD_TC_rec_frame_type = MOD_KG_polling_PM25_FRAME;
															      	  MOD_TC_polling_PM25_deal();
															      }
															      else
															      {
															      	if(port_report[3] == 0x10)		//-接收到射灯板发送过来的命令,不需要向7620转发自己知道即可
																      {
																      	  MOD_TC_rec_frame_type = MOD_KG_end_PM25_FRAME;
																      	  MOD_TC_end_PM25_deal();
																      }
																      else
																      {
																      	if(port_report[3] == 0x91)		//-涉及到对外应答的都需要把最高位置一
																	      {
																	      	  MOD_TC_rec_frame_type = MOD_KG_polling_hl_FRAME;
																	      	  MOD_TC_polling_hl_deal();
																	      }
																	      else
																	      {
																	      	if(port_report[3] == 0x93)		//-接收到射灯板发送过来的命令,不需要向7620转发自己知道即可
																		      {
																		      	  MOD_TC_rec_frame_type = MOD_KG_SET_hl_time_FRAME;
																		      	  MOD_TC_SET_hl_time_deal();
																		      }
																		      else
																		      {
																		      	if(port_report[3] == 0x14)		//-这个是射灯板首次发送过来
																			      {
																			      	  MOD_TC_rec_frame_type = MOD_KG_send_leddis_flag_FRAME;
																			      	  MOD_TC_send_leddis_flag_deal();
																			      }
																			      else
																			      {
																			      	if(port_report[3] == 0x95)		//-接收到射灯板发送过来的命令,不需要向7620转发自己知道即可
																				      {
																				      	  MOD_TC_rec_frame_type = MOD_KG_SET_voice_flag_FRAME;
																				      	  MOD_TC_SET_voice_flag_deal();
																				      }
																				      else
																				      {
																				      	if(port_report[3] == 0x96)		//-接收到射灯板发送过来的命令,不需要向7620转发自己知道即可
																					      {
																					      	  MOD_TC_rec_frame_type = MOD_KG_polling_voice_flag_FRAME;
																					      	  MOD_TC_polling_voice_flag_deal();
																					      }
																					      else
																					      {
																					      	if(port_report[3] == 0x97)		//-接收到射灯板发送过来的命令,不需要向7620转发自己知道即可
																						      {
																						      	  MOD_TC_rec_frame_type = MOD_KG_re_gateway_flag_FRAME;
																						      	  MOD_TC_re_gateway_flag_deal();
																						      }
																						      else
																						      {
																						      	if(port_report[3] == 0x9B)		//-接收到射灯板发送过来的命令,不需要向7620转发自己知道即可
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
																														      	 if(port_report[3] == 0x9C)	//-软复位
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
rxd_out_time:	//-执行到这里说明接收超时,或处理成功,反正现在可以继续发送了,,正常情况都会执行这里的,首先
	    if(Judge_Time_In_MainLoop(MOD_TC_rxd_wait_time,MOD_KG_WAIT_TIME_VALUE)==YES)	//-正常通信的话是不应该超时的,若过时就认为出错,下面重新初始化
	    {	//-当发送一个报文之后,超过等待回答时间之后,就可以从新发送一次
	      	MOD_TC_rec_OK=NO;
	      	MOD_TC_rxd_head_flag=NO;
	      	MOD_TC_rxd_wait_time=cticks_ms;
	      	MOD_TC_transmit_flag=YES;	//-表示现在可以组织发送内容
	      	//-MOD_TC_wait_replay=NO;	//-表示现在还没有等待回复
	      	//-MOD_TC_transmit_wait_time=Time_1ms_Counter;

	       	//-MOD_TC_comm_err_counter[port_send[0][0]]++;	//-超时出错计数


	    }
inrda:
		   //-主动发送 ,,这里就有一个问题,多长时间发送一次,或者说这个发送触发条件是什么
		   if(MOD_TC_transmit_flag==YES)		//-目前无所谓的是双向的
		   {

	        //-这个串口3,发送的内容全部来自串口2,自己不组织任何发送
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
	        	 		UART3_port_send[i] = port_send[1][the_ram_ax];		//-接收发送内容的缓存,前面是每次从缓存中提取的数据,用于发送
	        	 		port_send[1][the_ram_ax] = 0;		//-为了可靠增加,数据失效后即清0
	        	 }

	        	 port_send_dl[1] = (port_send_dl[1] + the_ram_cx) & 0x1ff;
	        	 port_deal_flag[1] = 0xaa;    //-保证了在发送结束之前,再次组织数据
	        	 port_send_len[1] = the_ram_cx;
             //-test_cn[1]++;    //-测试用

             //-if((test_cn[4] == 1) && (Judge_Time_In_MainLoop(test_cn_wait_time,2)==YES))
             //-{
             //-   test_cn[4] = 2;
             //-}
	        }

		   }
 	 }

}


//-同样这个不是主机所以整个总线时序不需要你控制

//-现在的问题:内容正确接收到了也看到了,上面的处理程序却丢失了报文,认为没有接收到内容
