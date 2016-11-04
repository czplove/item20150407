#include "user_conf.h"

/*
2015/8/4 14:06:52
跑马灯,有一种简单形式就是三种颜色分别同时点亮.
还有一种情况就是跑马(可能不只两个灯一起跑)
*/



/*
PB15	=	LD_SDI		移位寄存器串行数据输入端口
PB13	=	LD_CLK		移位寄存器时钟输入端口
PC07	=	LD_LAT		移位寄存器的数据锁存控制端口
PC06	=	LD_OE	        16路恒流源输出使能端（低电平有效）
*/

#define LD_OE(BitVal)  			GPIO_WriteBit(GPIOC,GPIO_Pin_6,BitVal)
#define LD_SDI(BitVal)  		GPIO_WriteBit(GPIOB,GPIO_Pin_15,BitVal)
#define LD_CLK(BitVal)  		GPIO_WriteBit(GPIOB,GPIO_Pin_13,BitVal)
#define LD_LAT(BitVal)  		GPIO_WriteBit(GPIOC,GPIO_Pin_7,BitVal)

void send_byte(u8 data)
{
  
  LD_SDI((BitAction)(data&0x01));
  //-翻转时钟信号上升沿时采用 
  LD_CLK((BitAction)1);
  asm("NOP");
  LD_CLK((BitAction)0);  
  asm("NOP");
}

void send_39bytes(u8 data8,u32 data32)      //-一个数据是39位,这里是最终输出
{
  u8 count;

  for(count=0;count<32;count++)
  {
    send_byte(data32>>count);    
  }
  
  for(count=0;count<7;count++)
  {
    send_byte(data8>>count);    
  }
  
  LD_LAT((BitAction)1);		//-新的数据输出
  asm("NOP");
  LD_LAT((BitAction)0);
  LD_OE((BitAction)0);
}

void HRL_init(void)
{
   LD_CLK((BitAction)0); 
   LD_SDI((BitAction)0);
   LD_LAT((BitAction)0);
	 LD_OE((BitAction)1); 
   //-LD_OE((BitAction)0);	//-在状态确认之前不能使能,否则会出现不确定状态,比如闪下 
}



/*
数据分析
数据位从高到低对应V0到V38
三色全灭			0x0000000000
三色全亮			0xFFFFFFFFFF
蓝灯全亮			0x4924924924
红灯全亮			0x9249249249
绿灯全亮			0x2492492492    GREEN

红色一对			0xFFFFFFFFFB
*/
UINT8 HRL_data[][5] = {
	0x00,0x00,0x00,0x00,0x00,			//-三色全灭
	0xFF,0xFF,0xFF,0xFF,0xFF,			//-三色全亮,白色
	0x49,0x24,0x92,0x49,0x24,			//-红灯全亮
	0x92,0x49,0x24,0x92,0x49,			//-蓝灯全亮
	0x24,0x92,0x49,0x24,0x92,			//-绿灯全亮
	0x00,0x00,0x80,0x00,0x04,			//-红色一对
	0x00,0x04,0x00,0x00,0x20,			//-NEXT
	0x00,0x20,0x00,0x01,0x00,			//-NEXT
	0x01,0x00,0x00,0x08,0x00,			//-NEXT
	0x08,0x00,0x00,0x40,0x00,			//-NEXT
	0x40,0x00,0x02,0x00,0x00,			//-NEXT
	//-0x00,0x00,0x10,0x00,0x00,			//-NEXT
	//-0x00,0x00,0x00,0x00,0x00,			//-NEXT
	//-0x00,0x00,0x00,0x00,0x00,			//-NEXT
	//-0x00,0x00,0x00,0x00,0x00,			//-NEXT
	//-0x00,0x00,0x00,0x00,0x00,			//-NEXT
	//-0x00,0x00,0x00,0x00,0x00,			//-NEXT
	//-0x00,0x00,0x00,0x00,0x00,			//-NEXT
	0x00,0x00,0x20,0x00,0x01,			//-蓝色一对
	0x00,0x01,0x00,0x00,0x08,			//-NEXT
	0x00,0x08,0x00,0x00,0x40,			//-NEXT
	0x00,0x40,0x00,0x02,0x00,			//-NEXT
	0x02,0x00,0x00,0x10,0x00,			//-NEXT
	0x10,0x00,0x00,0x80,0x00,			//-NEXT
	//-0x00,0x00,0x04,0x00,0x00,			//-NEXT
	//-0x00,0x00,0x00,0x00,0x00,			//-NEXT
	//-0x00,0x00,0x00,0x00,0x00,			//-NEXT
	//-0x00,0x00,0x00,0x00,0x00,			//-NEXT
	//-0x00,0x00,0x00,0x00,0x00,			//-NEXT
	//-0x00,0x00,0x00,0x00,0x00,			//-NEXT
	//-0x00,0x00,0x00,0x00,0x00,			//-NEXT
	0x00,0x00,0x40,0x00,0x02,			//-绿色一对
	0x00,0x02,0x00,0x00,0x10,			//-NEXT
	0x00,0x10,0x00,0x00,0x80,			//-NEXT
	0x00,0x80,0x00,0x04,0x00,			//-NEXT
	0x04,0x00,0x00,0x20,0x00,			//-NEXT
	0x20,0x00,0x01,0x00,0x00,			//-NEXT
	//-0x00,0x00,0x08,0x00,0x00,			//-NEXT
	//-0x00,0x00,0x00,0x00,0x00,			//-NEXT
	//-0x00,0x00,0x00,0x00,0x00,			//-NEXT
	//-0x00,0x00,0x00,0x00,0x00,			//-NEXT
	//-0x00,0x00,0x00,0x00,0x00,			//-NEXT
	//-0x00,0x00,0x00,0x00,0x00,			//-NEXT
	//-0x00,0x00,0x00,0x00,0x00,			//-NEXT
  0xdb,0x6d,0xb6,0xdb,0x6d,			//-红色加蓝色=粉色
  0x6d,0xb6,0xdb,0x6d,0xb6,			//-红色加绿色=翠绿
  0xb6,0xdb,0x6d,0xb6,0xdb,			//-蓝色加绿色=青色
};

void HRL_sub(void)	//-跑马灯处理程序
{
	 static UINT8  temp_data8=0,i = 2,temp_mode=0;
	 static UINT32 temp_data32=0x00000002;
	 //-static UINT8  temp_data8_mask;
	 //-static UINT32 temp_data32_mask;
	 UINT8  temp_data8_v654,temp_data32_v313029;
	 
   //-HRL_RUN_flag = 0;
	 //-temp_data8 = HRL_data[HRL_pt][0];
	 //-temp_data32 = (HRL_data[HRL_pt][1] << 24) + (HRL_data[HRL_pt][2] << 16) + (HRL_data[HRL_pt][3] << 8) + HRL_data[HRL_pt][4];
if(HRL_RUN_flag == 0x55)
{
	 //-if(Judge_LongTime_In_MainLoop(HL_run_time,10)==YES)
	 //-{
	 //-	  HL_run_time = Time_2048ms_Counter;
	 //-	
	 //-	  temp_mode++;
	 //-	  if(temp_mode > 3)
	 //-	  	temp_mode = 0;
	 //-}	
	 temp_mode = 2;
			 
		if(temp_mode == 0)  
		{
		   if(Judge_Time_In_MainLoop(HRL_pt_time00,20)==YES) 
		   {//-一个蓝色底色,红色转圈
		   	HRL_pt_time00 = cticks_ms;
		   temp_data8 = 0x92;
		   temp_data32 = 0x49249249;
		   if(i <= 31)
		   {  
		     
		     temp_data32 = temp_data32 | (1 << i);
		     i += 3;
		   } 
		   else if(i <= 38)
		   {
		     
		     temp_data8 = temp_data8 | (1 << (i - 32));
		     i += 3;
		   } 
		   else
		   {
		      i = 2;
		      temp_data32 = temp_data32 | (1 << i);
		      i += 3;
		   } 
		   
		   send_39bytes(temp_data8,temp_data32);
		   }
		}   
		else if(temp_mode == 1) 
		{
			 if(Judge_Time_In_MainLoop(HRL_pt_time01,100)==YES) 
		   {//-三种颜色依次增长,长满后换下一种
		   	HRL_pt_time01 = cticks_ms;
		   	
		   //-temp_data8 = 0x24;
		   //-temp_data32 = 0x92492492;
		   
		   HRL_pt++;
		     if(HRL_pt > 12)
		     {
		      	HRL_pt = 0;
		      	HRL_color_pt++;
		      	if(HRL_color_pt > 2)
		      	{
		      		HRL_color_pt = 0;
		      	}
		      		
		      	if(HRL_color_pt == 0)
		      	{	
		      		 //-HRL_color_pt = 1
		      	temp_data8 = 0;
		      	temp_data32 = 0;
		      	i = 0;
		      	
		      	}
		      	else if(HRL_color_pt == 1)
		      	{	
		      		 //-HRL_color_pt = 1
		      	temp_data8 = 0;
		      	temp_data32 = 0;
		      	i = 1;
		      	}
		      	else
		      	{	
		      		 //-HRL_color_pt = 1
		      	temp_data8 = 0;
		      	temp_data32 = 0;
		      	  i = 2;
		      	}
		     } 	
		     
		   //-temp_data8 = HRL_data[HRL_pt][0];
			 //-temp_data32 = (HRL_data[HRL_pt][1] << 24) + (HRL_data[HRL_pt][2] << 16) + (HRL_data[HRL_pt][3] << 8) + HRL_data[HRL_pt][4];
			 
		   if(i <= 31)
		   {  
		     
		     temp_data32 = temp_data32 | (1 << i);
		     i += 3;
		   } 
		   else if(i <= 38)
		   {
		     
		     temp_data8 = temp_data8 | (1 << (i - 32));
		     i += 3;
		   } 
		   else
		   {
		      i = 2;
		      temp_data32 = temp_data32 | (1 << i);
		      i += 3;
		   } 
		   
		   send_39bytes(temp_data8,temp_data32);
		   }
		}   
		else if(temp_mode == 2)   
		{
		   if(Judge_Time_In_MainLoop(HRL_pt_time01,120)==YES) 
		   {//-一种颜色在一定灯的情况下,旋转一定时间,然后增加一个灯再旋转一定时间
		   	HRL_pt_time01 = cticks_ms;
		   	
		   //-temp_data8 = 0;
		   //-temp_data32 = 0x00000002;
		   //-使用一种窗口效应应该可以达到简单灯光的效果
		   //-大概90S启动完成,那么在这个过程中13个灯每个灯平均转90/13=6.923秒,每转一次120毫秒
		   HRL_pt++;
		     if(HRL_pt > 64)	//-每个固定灯旋转一段时间后再换另一种固定数量灯旋转
		     {//-窗口越来越大
		     	  HRL_pt = 0;
		     	
		        //-HRL_color_pt = HRL_color_pt + 3;
		        //-HRL_color_pt++;
		      	if(HRL_color_pt == 0)
		      	{
		      		temp_data8 = 0;
		      		temp_data32 = 0x00000002;
		      		HRL_color_pt = 1;
		      	}
		      	else if(HRL_color_pt == 1)
		      	{
		      		temp_data8 = 0;
		      		temp_data32 = 0x00000012;
		      		HRL_color_pt = 2;
		      	}
		      	else if(HRL_color_pt == 2)
		      	{
		      		temp_data8 = 0;
		      		temp_data32 = 0x00000092;
		      		HRL_color_pt = 3;
		      	}
		      	else if(HRL_color_pt == 3)
		      	{
		      		temp_data8 = 0;
		      		temp_data32 = 0x00000492;
		      		HRL_color_pt = 4;
		      	}	
		      	else if(HRL_color_pt == 4)
		      	{
		      		temp_data8 = 0;
		      		temp_data32 = 0x00002492;
		      		HRL_color_pt = 5;
		      	}		
		      	else if(HRL_color_pt == 5)
		      	{
		      		temp_data8 = 0;
		      		temp_data32 = 0x00012492;
		      		HRL_color_pt = 6;
		      	}	
		      	else if(HRL_color_pt == 6)
		      	{
		      		temp_data8 = 0;
		      		temp_data32 = 0x00092492;
		      		HRL_color_pt = 7;
		      	}	
		      	else if(HRL_color_pt == 7)
		      	{
		      		temp_data8 = 0;
		      		temp_data32 = 0x00492492;
		      		HRL_color_pt = 8;
		      	}	
		      	else if(HRL_color_pt == 8)
		      	{
		      		temp_data8 = 0;
		      		temp_data32 = 0x02492492;
		      		HRL_color_pt = 9;
		      	}	
		      	else if(HRL_color_pt == 9)
		      	{
		      		temp_data8 = 0;
		      		temp_data32 = 0x12492492;
		      		HRL_color_pt = 10;
		      	}	
		      	else if(HRL_color_pt == 10)
		      	{
		      		temp_data8 = 0;
		      		temp_data32 = 0x92492492;
		      		HRL_color_pt = 11;
		      	}	
		      	else//- if(HRL_color_pt == 11)
		      	{
		      		temp_data8 = 0x04;
		      		temp_data32 = 0x92492492;
		      		HRL_color_pt = 12;
		      	}	
		      	//-else//- if(HRL_color_pt == 12)
		      	//-{
		      	//-	temp_data8 = 0x24;		
		      	//-	temp_data32 = 0x92492492;
		      	//-	//-HRL_color_pt = 0;		//-这样结束后就继续旋转而不从头开始
		      	//-}
		
		     }
		     
		   //-temp_data8 = HRL_data[HRL_pt][0];
			 //-temp_data32 = (HRL_data[HRL_pt][1] << 24) + (HRL_data[HRL_pt][2] << 16) + (HRL_data[HRL_pt][3] << 8) + HRL_data[HRL_pt][4];
			 //-实现整体循环左移3位
			 temp_data8_v654 = temp_data8 >> 4;
			 temp_data32_v313029 = temp_data32 >> 29;
		   temp_data8 = temp_data8 << 3;
		   temp_data32 = temp_data32 << 3;
		   temp_data8 = temp_data8 | temp_data32_v313029;
		   temp_data32 = temp_data32 | temp_data8_v654;
		   
		     //-temp_data8 = temp_data8 & temp_data8_mask;
		     //-temp_data32 = temp_data32 & temp_data32_mask;
		
		
		   
		   send_39bytes(temp_data8,temp_data32);
		   }
		}   
		else//- if(temp_mode == 1)   
		{
		   if(Judge_Time_In_MainLoop(HRL_pt_time01,200)==YES) 
		   {//-一个起点向另一个终点会和
		   	HRL_pt_time01 = cticks_ms;
		   	
		
		        //-4924924924
		        if(HRL_color_pt == 0)
		      	{
		      		temp_data8 = 0x00;
		      		temp_data32 = 0x00000004;
		      		HRL_color_pt = 1;
		      	}
		        else if(HRL_color_pt == 1)
		      	{
		      		temp_data8 = 0x40;
		      		temp_data32 = 0x00000020;
		      		HRL_color_pt = 2;
		      	}
		      	else if(HRL_color_pt == 2)
		      	{
		      		temp_data8 = 0x48;
		      		temp_data32 = 0x00000120;
		      		HRL_color_pt = 3;
		      	}
		      	else if(HRL_color_pt == 3)
		      	{
		      		temp_data8 = 0x49;
		      		temp_data32 = 0x00000920;
		      		HRL_color_pt = 4;
		      	}
		      	else if(HRL_color_pt == 4)
		      	{
		      		temp_data8 = 0x49;
		      		temp_data32 = 0x20004920;
		      		HRL_color_pt = 5;
		      	}	
		      	else if(HRL_color_pt == 5)
		      	{
		      		temp_data8 = 0x49;
		      		temp_data32 = 0x24024920;
		      		HRL_color_pt = 6;
		      	}		
		      	else//- if(HRL_color_pt == 5)
		      	{
		      		temp_data8 = 0x49;
		      		temp_data32 = 0x24924920;
		      		HRL_color_pt = 0;
		      	}	
		      	
		     
		   //-temp_data8 = HRL_data[HRL_pt][0];
			 //-temp_data32 = (HRL_data[HRL_pt][1] << 24) + (HRL_data[HRL_pt][2] << 16) + (HRL_data[HRL_pt][3] << 8) + HRL_data[HRL_pt][4];
			 
		
		   
		   send_39bytes(temp_data8,temp_data32);
		   }
		}	 
}
else if((HRL_RUN_flag == 0xaa) && (HRL_RUN_ONOFF == 0x55))		//-显示的时候才需要亮灯
{//-显示数据状态指示
  //?环境指示显示存在一个问题就是我显示的时候是实时数据,而播报的时候可能是APP推送的旧数据
	 if(RunLed_stata_num == 0)	
	 {	
	 	 temp_mode = pm_data_flag;
	 }
	 else if(RunLed_stata_num == 1)
	 {
	 	 temp_mode = Noise_Value_flag;
	 }
   else if(RunLed_stata_num == 2)
	 {
	 	 temp_mode = VOC_data_flag;
	 }
	 else if(RunLed_stata_num == 3)
	 {
	 	 temp_mode = CO2_data_flag;
	 }	
	 else
	 	 temp_mode = 3;
	 
	 if(temp_mode == 0)
   {//-优 绿色
   	  temp_data8 = 0x24;
      temp_data32 = 0x92492492;
   }
   else if(temp_mode == 1)
   {//-良 蓝色
   	  temp_data8 = 0x49;
      temp_data32 = 0x24924924;
   }
   else if(temp_mode == 2)
   {//-差 红色
   	  temp_data8 = 0x92;
      temp_data32 = 0x49249249;
   }
   else
   {//-不确定就熄灭状态
   	  temp_data8 = 0;
      temp_data32 = 0;
   }
   
   
	 send_39bytes(temp_data8,temp_data32);
		
}
else
{//-保证灯是灭的
	if((zigbee_flag == 1) && (HL_flag == 0))		//-这个的优先级是最低的,仅仅比灭灯高点
	{
		 temp_data8 = 0x24;
     temp_data32 = 0x92492492;
	}
	else
	{
		 temp_data8 = 0x00;
     temp_data32 = 0x00000000;
	}		
	
	if((zigbee_flag == 1) && (Judge_LongTime_In_MainLoop(zigbee_wait_time,88)==YES))
	{
		 zigbee_flag = 0;
	}	
	
  //-temp_data8 = 0x24;
  //-temp_data32 = 0x92492492;
  
	send_39bytes(temp_data8,temp_data32);
}

}

//-4 绿色
//-1 蓝色
//-4 红色0x00000924
/*
红0x24 0x92492492
蓝0x12 0x49249249
绿0x49 0x24924924

*/

