/*
接收:使用一个常规逻辑,开辟一个512字节的缓冲区,然后循环存储
发送:也使用512字节的空间保存内容,然后置一个发送标志,直到内容发送出去为止
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
#define   MOD_KG_send_HRL_mode_FRAME		      	   0x1A			//-这个值比较特殊.主板和射灯板使用的不同
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
  MOD_KG_rxd_head_flag=NO;
  //-UART0_transmit_flag = 0x55;
  //-UART0_start_tx_time = Time_2048ms_Counter;

  //-UART1_sloop_flag = 0;

}

void UART1_start(void)
{
  //-协议初始化内容
  MOD_KG_transmit_control = 1;	//-等待启动完成,然后开始发送周期报文

}


void UART_Rx_Deal(void)
{
#ifndef STM3210E_UP
  port_recv_pt[0] = 512 - DMA_GetCurrDataCounter(DMA1_Channel6);	//-得到接收个数,如果作为数组偏移量的话,现在的数值就是指向待存空间的
  if(port_recv_pt[0] >= 512)
  	port_recv_pt[0] = 0;
#else
  port_recv_pt[0] = 512 - DMA_GetCurrDataCounter(DMA1_Channel5);	//-得到接收个数,如果作为数组偏移量的话,现在的数值就是指向待存空间的
  if(port_recv_pt[0] >= 512)
  	port_recv_pt[0] = 0;
#endif
}

void UART_Tx_Deal(void)
{
  /* Enable USARTy DMA TX request */
  //-USART_DMACmd(USARTy, USART_DMAReq_Tx, ENABLE);			//-启动DMA发送
#ifndef STM3210E_UP
  if(port_send_len[0] != 0)	//-一定要保证内容发送出去了,再使用DMA发送数据
  {
		  DMA_Cmd(DMA1_Channel7, DISABLE);
		  DMA1_Channel7->CNDTR = port_send_len[0]; //-传输数量寄存器,指示剩余的待传输字节数目
		  DMA_Cmd(DMA1_Channel7, ENABLE);
      //-if(port_send_len[0] != 42)		//-测试用
      //-  port_send_len[0] = 0;   //- 04 13 8B 00 01 46 70总线上周期出现这样的数据,但是32并没有发送出去

      port_send_len[0] = 0;
	}
#else
  if(port_send_len[0] != 0)	//-一定要保证内容发送出去了,再使用DMA发送数据
  {
		  DMA_Cmd(DMA1_Channel4, DISABLE);
		  DMA1_Channel4->CNDTR = port_send_len[0]; //-传输数量寄存器,指示剩余的待传输字节数目
		  DMA_Cmd(DMA1_Channel4, ENABLE);
		  port_send_len[0] = 0;
	}
#endif
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
	    port_recv[temp_loop] = 0;
	 }

	 		//-使用的临时缓存清除
	 for(i=0;i<len;i++)
	   port_report[i] = 0;
}

//系统软复位

void Sys_Soft_Reset(void)
{
    SCB->AIRCR =0X05FA0000|(u32)0x04;
}

void Sys_run_old(void)
{
  WORD the_ram_ax;

  if(*(__IO uint16_t*)(APP_CONFIG1_ADDR) != 0x5555)
  {//-强制使用老程序的前提是老程序没有被修改
    FLASH_Unlock();						//解锁
		FLASH_ErasePage(APP_CONFIG_ADDR);//擦除这个扇区
		FLASH_ProgramHalfWord(APP_CONFIG_ADDR,0x5555);	//-更新失败可以运行老程序
    if(*(__IO uint16_t*)(APP_CONFIG_ADDR) == 0x5555)
    {
        port_send[0] = 0xaa;		//-两字节包头
			  port_send[1] = 0x55;

			  port_send[2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[3] = 0x9c;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[4] = 0x04;		//-数据长度
			  port_send[5] = 0x55;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0],port_send[2]);
			  port_send[6] =LOBYTE(the_ram_ax);
			  port_send[7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[2] + 2;
			  port_deal_flag[0] = 0xaa;
        UART_Tx_Deal();
#ifndef STM3210E_UP
        while(DMA_GetFlagStatus(DMA1_FLAG_TC7) != 1);  //-等待发送结束
#else
        while(DMA_GetFlagStatus(DMA1_FLAG_TC4) != 1);
#endif
    }
	  FLASH_Lock();//上锁
  }
}


void MOD_KG_LED_display_deal(void)		//-对7620下达的数据进行接收
{
	  WORD the_ram_ax;


       //-如果数据成功接收到之后就应答一次
			  port_send[0] = 0xaa;		//-两字节包头
			  port_send[1] = 0x55;

			  port_send[2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[3] = UART3_TO_UART2_FLAG;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[4] = 0xaa;		//-数据长度
			  port_send[5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0],port_send[2]);
			  port_send[6] =LOBYTE(the_ram_ax);
			  port_send[7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[2] + 2;
			  port_deal_flag[0] = 0xaa;

}


void MOD_KG_HEX_package_deal(void)		//-对7620下达的数据进行接收
{
	  WORD the_ram_ax;
	  unsigned char temp_flag;

    //-对接收到的报文进行前期剥离
    recv_hex_lines = (port_report[4] << 8) + port_report[5];
    expect_hex_lines = recv_hex_lines;

    the_ram_ax = port_report[2] - 6;	//-需要处理报文的长度
    //-跳转到内部处理HEX报文,如果正确的话就应答允许下次报文发送,中间的丢失重发错位等等,让上位机控制
    temp_flag = UpdatePackageDataHandle(&port_report[6], the_ram_ax);
    if((temp_flag == 0x55) || (expect_hex_lines > recv_hex_lines))	//-可以的话就进行应答,否则让上位机进行重发
    {
    	//-如果数据成功接收到之后就应答一次
			  port_send[0] = 0xaa;		//-两字节包头
			  port_send[1] = 0x55;

			  port_send[2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[3] = 0xA2;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  expect_hex_lines -= 1;
			  port_send[4] = HIBYTE(expect_hex_lines);		//-数据长度
			  port_send[5] = LOBYTE(expect_hex_lines);

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0],port_send[2]);
			  port_send[6] =LOBYTE(the_ram_ax);
			  port_send[7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[2] + 2;
			  port_deal_flag[0] = 0xaa;
    }

}


void MOD_KG_inquire_version_deal(void)		//-对7620下达的数据进行接收
{
	  WORD the_ram_ax;
	  //-unsigned char temp_flag;


    //-if(temp_flag == 0x55)	//-可以的话就进行应答,否则让上位机进行重发
    {
    	//-如果数据成功接收到之后就应答一次
			  port_send[0] = 0xaa;		//-两字节包头
			  port_send[1] = 0x55;

			  port_send[2] = 8;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[3] = 0xA4;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[4] = 0;		//-数据长度
			  port_send[5] = 0;

        port_send[6] = 0;		//-数据长度
			  port_send[7] = 0;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0],port_send[2]);
			  port_send[8] =LOBYTE(the_ram_ax);
			  port_send[9] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[2] + 2;
			  port_deal_flag[0] = 0xaa;
    }

}

void MOD_KG_inquire_update_deal(void)		//-把接收到的数据进行透明传输
{
	  WORD the_ram_ax;

    if((port_report[4] == 1) && (port_report[5] == 0x55))
    {//-主板接收到升级命令
        //-run_in_iap();   //-退出正常程序,进入IAP程序

        //-如果数据成功接收到之后就应答一次
			  port_send[0] = 0xaa;		//-两字节包头
			  port_send[1] = 0x55;

			  port_send[2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[3] = 0xa3;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[4] = 0x01;		//-数据长度
			  port_send[5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0],port_send[2]);
			  port_send[6] =LOBYTE(the_ram_ax);
			  port_send[7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[2] + 2;
			  port_deal_flag[0] = 0xaa;

        //-while(1);   //-等待复位重启,进入IAP程序
    }

    if((port_report[4] == 2) && (port_report[5] == 0x55))
    {//-主板接收到升级命令
        //-run_in_iap();   //-退出正常程序,进入IAP程序

        //-如果数据成功接收到之后就应答一次
			  port_send[0] = 0xaa;		//-两字节包头
			  port_send[1] = 0x55;

			  port_send[2] = 6;		//-整包长： 1个字节，从包头开始，到CRC16之前的数据。

			  port_send[3] = 0xa3;		//-功能码:01h 传感器主动上报

			  //-有效数据
			  //-这里的有效数据又是以块为单位的"数据长度+设备ID+功能码+数据描述+数据位"
			  port_send[4] = 0x02;		//-数据长度
			  port_send[5] = 0xaa;

			  //-CRC16
			  the_ram_ax=MOD_KG_CRC16(&port_send[0],port_send[2]);
			  port_send[6] =LOBYTE(the_ram_ax);
			  port_send[7] =HIBYTE(the_ram_ax);	//-直到这里所有的报文内容都已经准备好了

			  //-发送长度
			  port_send_len[0] = port_send[2] + 2;
			  port_deal_flag[0] = 0xaa;

        //-while(1);   //-等待复位重启,进入IAP程序
    }

    //-复位为初始状态等待升级
    addrCur = FLASH_APP1_ADDR;
    IAP_DataCur = 0;
    IAP_HEX_expectant_pt = 0x4000;
}



//-下面处理通讯接收到的内容,其实就是一个单独的上层协议的处理,在主函数里面周期调用
//-这里有一个重要的作用是传递信息
void CommandScan()
{
	 WORD the_ram_ax,the_ram_bx;
	 WORD temp_data,temp_data1,delta_len,temp_loop,temp_int;


   UART_Rx_Deal();
   UART_Tx_Deal();

#ifndef STM3210E_UP
	 //-判断发送是否结束了,如果结束了可以正常逻辑处理,否则不处理
	 if(DMA_GetFlagStatus(DMA1_FLAG_TC7))		//-进入说明发送完成了
   {//-只有当传输完成了才可以重新给DMA发送传输命令
   	  //-for(temp_loop = 0;temp_loop < 43;temp_loop++)
   	  //-	port_send[temp_loop] = 0;
   	  port_deal_flag[0] = 0;		//-给一个非0值就可以再下次检查发送时发发送命令了
   	  //-MOD_KG_transmit_flag=YES;
   	  DMA_ClearFlag(DMA1_FLAG_TC7);
   }
#else
    //-判断发送是否结束了,如果结束了可以正常逻辑处理,否则不处理
	 if(DMA_GetFlagStatus(DMA1_FLAG_TC4))		//-进入说明发送完成了
   {//-只有当传输完成了才可以重新给DMA发送传输命令
   	  //-内容发送完成之后全部清理,由于现在不知道每个的空间大小了,那么选取一块大的清零
   	  //-for(temp_loop = 0;temp_loop < 43;temp_loop++)
   	  //-	port_send[0][temp_loop] = 0;
   	  port_deal_flag[0] = 0;		//-给一个非0值就可以再下次检查发送时发发送命令了
   	  //-MOD_KG_transmit_flag=YES;		//-当我的内容发送出去了也可以立即发送内容了??这里应该不能立即组织内容的发送,这样下面的内容会被覆盖,接收的标志位
   	  DMA_ClearFlag(DMA1_FLAG_TC4);
   }
#endif

   if(port_deal_flag[0] == 0)
   {
      if((port_recv_pt[0]!=port_recv_dl[0]))
      {
           if(MOD_KG_rxd_head_flag==NO)	//-接收到的数据还没有处理的时候就是NO
           {
           	   //-MOD_KG_rxd_wait_time=cticks_ms;		//-后加的有待测试
		   	   	   temp_data = port_recv_pt[0];
		   	   	   temp_data1 = port_recv_dl[0];
		           if(temp_data1>temp_data)	//-前面的是处理指针比实际的接收指针进行比较
		               delta_len=(temp_data+512)-temp_data1;
		           else
		               delta_len=temp_data-temp_data1;	//-一共的长度
		           for(temp_loop=temp_data1;temp_loop<(delta_len+temp_data1);temp_loop++)
		           {
		        	   if(port_recv[port_recv_dl[0]]==0xaa)	//-这个地方比较的是从站地址,但是我觉得没有任何规律就是通讯
		        	   {	//-利用一切可以利用的
		        	     the_ram_ax=(port_recv_dl[0]+1)&0x1ff;
		        	     if(temp_data == the_ram_ax)
		        	     	 break;
		        	     if(port_recv[the_ram_ax]==0x55)	//-比较的是功能码
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
		               if(delta_len>=(unsigned short)(port_recv[temp_int]+2))	//-得到的报文长度和理论上的报文长度进行比较
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
              //-到这里就可以说明应答报文已经可以了
              MOD_KG_rxd_head_flag=NO;
              MOD_KG_rec_OK=NO;	//-成功接收到的数据开始处理了之后,就恢复0
              MOD_KG_transmit_flag=YES;
              //-test_cn_wait_time = cticks_ms;	//-测试用
              //-MOD_KG_transmit_wait_time=Time_1ms_Counter;	//-虽然上面说可以发送数据了,但是还是要延时一段时间,因为接收到的数据需要处理
              //-下面是对实际数据的处理,处理的是核心部分
              the_ram_bx=(port_recv_dl[0]+3)&0x1ff;;
              if(port_recv[the_ram_bx]!=0xFF)	//-这个是对功能码的判断,功能码不同判断的依据也不一样
              {	//-这里是宁外一种处理现在可以不管
                  the_ram_ax=(port_recv_dl[0]+2)&0x1ff;
                  temp_int=port_recv[the_ram_ax]+2+port_recv_dl[0];
                  for(temp_loop=port_recv_dl[0];temp_loop<temp_int;temp_loop++)	//-上面这样干的秘密就是保证定位到需要处理的报文字节
                  {	//-简单的不需要这样处理但是复杂的还是需要的,那么这样用了得话兼容性就会很好
                       if(temp_loop<=511)
                         port_report[temp_loop-port_recv_dl[0]]=port_recv[temp_loop];
                       else
                         port_report[temp_loop-port_recv_dl[0]]=port_recv[temp_loop-512];	//-难道是高速更新的缘故需要提前复制出来
                  }	//-或者还有一种可能性就是统一处理
                //-port_recv_dl[0]+=delta_len;	//-这个地方就舍弃了这样的处理报文
                port_recv_dl[0]+=(port_report[2] + 2);
                port_recv_dl[0]&=0x1ff;
                temp_int=MOD_KG_CRC16(&port_report[0],port_report[2]);
                if((LOBYTE(temp_int)!=port_report[port_report[2]+0])||(HIBYTE(temp_int)!=port_report[port_report[2]+1]))	//-进行CRC检查
                {	//-由于这的不确定性,如果校验不正确的话就直接把头舍掉,继续寻找
                    return;	//-到这里说明成功接收到的报文CRC校验没有通过
                }
              }
              else
              {
                  //-port_recv_dl[0]+=delta_len;	//-到这里就可以把接收缓冲区中的内容舍弃了,已经拷贝出来了
                  port_recv_dl[0]+=7;
                  port_recv_dl[0]&=0x1ff;
                  return;
              }

              if(port_report[3] == 0x1C)	//-判断功能码
              {
                 if((port_report[4] == 0x03) && (port_report[5] == 0x55))
                 {//-启动软复位
                     MOD_KG_rec_frame_type = MOD_KG_polling_ID_FRAME;
                     Sys_Soft_Reset();
                 }
                 else
                 {
                   if((port_report[4] == 0x04) && (port_report[5] == 0x55))
                   {//-更新失败强制使用老程序
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
                        {//-查询版本号
                           MOD_KG_inquire_version_deal();

                        }
                        else
                        {
                          if(port_report[3] == 0x23)
                          {//-主机通知STM32进行程序升级
                             MOD_KG_inquire_update_deal();

                          }
                          else
                              MOD_KG_rec_frame_type = MOD_KG_polling_data_FRAME;
                        }

		                  }
                  }
              }

              //-MOD_KG_clear_port_report_deal();		//-每次处理完毕之后清除缓存区防止错误使用
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


