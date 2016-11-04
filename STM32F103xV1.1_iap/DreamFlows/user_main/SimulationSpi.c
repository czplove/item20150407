#include "user_conf.h"

/*
2015/8/4 14:06:52
�����,��һ�ּ���ʽ����������ɫ�ֱ�ͬʱ����.
����һ�������������(���ܲ�ֻ������һ����)
*/



/*
PB15	=	LD_SDI		��λ�Ĵ���������������˿�
PB13	=	LD_CLK		��λ�Ĵ���ʱ������˿�
PC07	=	LD_LAT		��λ�Ĵ���������������ƶ˿�
PC06	=	LD_OE	        16·����Դ���ʹ�ܶˣ��͵�ƽ��Ч��
*/

#define LD_OE(BitVal)  			GPIO_WriteBit(GPIOC,GPIO_Pin_6,BitVal)
#define LD_SDI(BitVal)  		GPIO_WriteBit(GPIOB,GPIO_Pin_15,BitVal)
#define LD_CLK(BitVal)  		GPIO_WriteBit(GPIOB,GPIO_Pin_13,BitVal)
#define LD_LAT(BitVal)  		GPIO_WriteBit(GPIOC,GPIO_Pin_7,BitVal)

void send_byte(u8 data)
{
  
  LD_SDI((BitAction)(data&0x01));
  //-��תʱ���ź�������ʱ���� 
  LD_CLK((BitAction)1);
  asm("NOP");
  LD_CLK((BitAction)0);  
  asm("NOP");
}

void send_39bytes(u8 data8,u32 data32)      //-һ��������39λ,�������������
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
  
  LD_LAT((BitAction)1);		//-�µ��������
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
   //-LD_OE((BitAction)0);	//-��״̬ȷ��֮ǰ����ʹ��,�������ֲ�ȷ��״̬,�������� 
}



/*
���ݷ���
����λ�Ӹߵ��Ͷ�ӦV0��V38
��ɫȫ��			0x0000000000
��ɫȫ��			0xFFFFFFFFFF
����ȫ��			0x4924924924
���ȫ��			0x9249249249
�̵�ȫ��			0x2492492492    GREEN

��ɫһ��			0xFFFFFFFFFB
*/
UINT8 HRL_data[][5] = {
	0x00,0x00,0x00,0x00,0x00,			//-��ɫȫ��
	0xFF,0xFF,0xFF,0xFF,0xFF,			//-��ɫȫ��,��ɫ
	0x49,0x24,0x92,0x49,0x24,			//-���ȫ��
	0x92,0x49,0x24,0x92,0x49,			//-����ȫ��
	0x24,0x92,0x49,0x24,0x92,			//-�̵�ȫ��
	0x00,0x00,0x80,0x00,0x04,			//-��ɫһ��
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
	0x00,0x00,0x20,0x00,0x01,			//-��ɫһ��
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
	0x00,0x00,0x40,0x00,0x02,			//-��ɫһ��
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
  0xdb,0x6d,0xb6,0xdb,0x6d,			//-��ɫ����ɫ=��ɫ
  0x6d,0xb6,0xdb,0x6d,0xb6,			//-��ɫ����ɫ=����
  0xb6,0xdb,0x6d,0xb6,0xdb,			//-��ɫ����ɫ=��ɫ
};

void HRL_sub(void)	//-����ƴ������
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
		   {//-һ����ɫ��ɫ,��ɫתȦ
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
		   {//-������ɫ��������,��������һ��
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
		   {//-һ����ɫ��һ���Ƶ������,��תһ��ʱ��,Ȼ������һ��������תһ��ʱ��
		   	HRL_pt_time01 = cticks_ms;
		   	
		   //-temp_data8 = 0;
		   //-temp_data32 = 0x00000002;
		   //-ʹ��һ�ִ���ЧӦӦ�ÿ��Դﵽ�򵥵ƹ��Ч��
		   //-���90S�������,��ô�����������13����ÿ����ƽ��ת90/13=6.923��,ÿתһ��120����
		   HRL_pt++;
		     if(HRL_pt > 64)	//-ÿ���̶�����תһ��ʱ����ٻ���һ�̶ֹ���������ת
		     {//-����Խ��Խ��
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
		      	//-	//-HRL_color_pt = 0;		//-����������ͼ�����ת������ͷ��ʼ
		      	//-}
		
		     }
		     
		   //-temp_data8 = HRL_data[HRL_pt][0];
			 //-temp_data32 = (HRL_data[HRL_pt][1] << 24) + (HRL_data[HRL_pt][2] << 16) + (HRL_data[HRL_pt][3] << 8) + HRL_data[HRL_pt][4];
			 //-ʵ������ѭ������3λ
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
		   {//-һ���������һ���յ���
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
else if((HRL_RUN_flag == 0xaa) && (HRL_RUN_ONOFF == 0x55))		//-��ʾ��ʱ�����Ҫ����
{//-��ʾ����״ָ̬ʾ
  //?����ָʾ��ʾ����һ�������������ʾ��ʱ����ʵʱ����,��������ʱ�������APP���͵ľ�����
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
   {//-�� ��ɫ
   	  temp_data8 = 0x24;
      temp_data32 = 0x92492492;
   }
   else if(temp_mode == 1)
   {//-�� ��ɫ
   	  temp_data8 = 0x49;
      temp_data32 = 0x24924924;
   }
   else if(temp_mode == 2)
   {//-�� ��ɫ
   	  temp_data8 = 0x92;
      temp_data32 = 0x49249249;
   }
   else
   {//-��ȷ����Ϩ��״̬
   	  temp_data8 = 0;
      temp_data32 = 0;
   }
   
   
	 send_39bytes(temp_data8,temp_data32);
		
}
else
{//-��֤�������
	if((zigbee_flag == 1) && (HL_flag == 0))		//-��������ȼ�����͵�,��������Ƹߵ�
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

//-4 ��ɫ
//-1 ��ɫ
//-4 ��ɫ0x00000924
/*
��0x24 0x92492492
��0x12 0x49249249
��0x49 0x24924924

*/

