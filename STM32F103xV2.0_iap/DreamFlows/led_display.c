/*
2015/4/9 19:03:41	by zj
led_display.c  ������������������ʾ,��ʵ�ֹ��ܲ�����������

2015/7/27 15:17:12
���ڲ���ÿ���ж�ˢ��һ�еĿ����жϷ�ʽ:
��һ�� ��������ȡ����
�ڶ��� ��֯һҳ��Ҫ��ʾ������
������ ��ʱ(80ms)����Ҫ��ʾ��ͼƬ���д���,�������Ҫ��ʾ��ͼƬ
���Ĳ� �Դ������Ҫ��ʾ��ͼƬɵ��ʽˢ�о�����

*/
#include "user_conf.h"

//-BYTE  Temp_Flag;


//-�궨��,�������洦��,���տɵ���λ��
#define LD_OE(BitVal)  			GPIO_WriteBit(GPIOA,GPIO_Pin_6,BitVal)

#define LD_ADDR_y(PortVal)  			    GPIO_Write(GPIOC,PortVal)

#define LD_LE(BitVal)  			GPIO_WriteBit(GPIOB,GPIO_Pin_0,BitVal)

#define LD_SDI(BitVal)  		GPIO_WriteBit(GPIOA,GPIO_Pin_7,BitVal)
#define LD_CLK(BitVal)  		GPIO_WriteBit(GPIOA,GPIO_Pin_5,BitVal)
#define LD_LAT(BitVal)  		GPIO_WriteBit(GPIOA,GPIO_Pin_4,BitVal)

extern void led_disp_L_init(void);
#define ZIKU_cartoon_num 6
/*UINT8 *ZIKU_cartoonxx[] = {&ZIKU_cartoon01[0][0],&ZIKU_cartoon02[0][0],&ZIKU_cartoon03[0][0],
&ZIKU_cartoon04[0][0],&ZIKU_cartoon05[0][0],&ZIKU_cartoon06[0][0],&ZIKU_cartoon07[0][0],
&ZIKU_cartoon08[0][0],&ZIKU_cartoon09[0][0],&ZIKU_cartoon10[0][0],&ZIKU_cartoon11[0][0],
&ZIKU_cartoon12[0][0],&ZIKU_cartoon13[0][0],&ZIKU_cartoon14[0][0],&ZIKU_cartoon15[0][0],
&ZIKU_cartoon16[0][0],&ZIKU_cartoon17[0][0],&ZIKU_cartoon18[0][0],&ZIKU_cartoon19[0][0],
&ZIKU_cartoon20[0][0],&ZIKU_cartoon21[0][0]};*/

UINT8 *ZIKU_cartoonxx[] = {&ZIKU_cartoon01[0][0],&ZIKU_cartoon02[0][0],&ZIKU_cartoon03[0][0],
&ZIKU_cartoon04[0][0],&ZIKU_cartoon05[0][0],&ZIKU_cartoon06[0][0]};

/*
74HC4514---4��16������оƬ

PA6	=	LD_OE			enable input (active LOW)		,,�����Ϊ�͵Ļ����16�����Ŷ�Ϊ0
PB0		LD_LE     latch enable input (active HIGH)	,,�����ֵַ,��Чʱ(Ϊ0)�ı��ֵַ,��Ӱ��������
PC0	=	LD_A0
PC1	=	LD_A1
PC2	=	LD_A2
PC3	=	LD_A3

Q0 to Q15       multiplexer outputs (active HIGH)


SC6616---16λ����LED������

PA7	=	LD_SDI		��λ�Ĵ���������������˿�
PA5	=	LD_CLK		��λ�Ĵ���ʱ������˿�							,,�����ز���
PA4	=	LD_LAT		��λ�Ĵ���������������ƶ˿�				,,������������ֵ�Ƿ�������λ�Ĵ���
PA6	=	LD_OE			16·����Դ���ʹ�ܶˣ��͵�ƽ��Ч��	,,��Чʱ16·���Ϊ����̬

���������ƶ˾��������Ƿ�д��������,���͵�ƽʱ�������ڵ����ݲ��仯,�ߵ�ƽʱ��λ�Ĵ�����ֵд��������.

ע:
�����κ�ʱ��ֻ��ѡ������һ���ж�������ʾ,���������Ҫ����һ��������ͼ���Ļ�,ֻ�ܿ���ˢ����������
���������󿴵�"����"��ͼ��.

�Ӿ�������:�����е����ȵĹ�̼����Ӿ�����ʱ��ԼΪ0��05��0��2�롣
���������������20ms֮��ȫ��ˢһ��,������,Ƶ��Խ��Խ��
*/

void send_bit(u8 data)
{

  LD_SDI((BitAction)(data&0x01));
  //-��תʱ���ź�������ʱ����
  LD_CLK((BitAction)0);
  LD_CLK((BitAction)1);
}

void send_byte(u32 data,u8 length)      //-һ�е������͵�������
{
  u8 count;
  //U2_LATCH=1;
  for(count=0;count<length;count++)
  {
    //U2_LATCH=1;
    send_bit(data>>count);
  }
  //-������Ч��,���������ݽ��и���
  //-LD_LAT((BitAction)1);
  //-NOP_Delay(2);
  //-LD_LAT((BitAction)0);
}

#if 0
//-x ��ʾ��ʾ�к�(0~15) ,,�ɵ�����
void display(u8 x,u32 y)
{
	//-u8 i;
	u16  hang_num;

	//-LD_OE((BitAction)1);		//-�޸�һ���ϲ���ʱ,����ǰ���к��ǲ��̶���,���Ա����ȹرյ����߶�ȷ�����
  //-������׼���ó�ʱ���������,Ȼ����.��������л��еĻ�,�����Ӿ���������к�
  send_byte(y,24);	//-�ܹ���24��,һ�����ݵ���ʾ��Ҫ�ƶ�24��,����ת��

  hang_num = GPIO_ReadOutputData(GPIOC);
	hang_num = (hang_num&0xfff0) | (x&0x0f);
	LD_ADDR_y(hang_num);
	LD_LE((BitAction)1);
	LD_LAT((BitAction)1);
	LD_LE((BitAction)0);
  LD_LAT((BitAction)0);
	//-LD_OE((BitAction)0);
  //-����ˢ������ֵ,������Ҫ���,,�����ת�����п�����ʾ���Ӱ�첻��
  //-LD_LE(1);			//-LD_LE Ϊ0ʱ���ݲ��ܽ���������
  //-LD_LE(0);
}
#else
//-x ��ʾ��ʾ�к�(0~15)
void display(u8 x,u32 y)
{
	//-u8 i;
	u16  hang_num = 0;

	hang_num = hang_num | (1 << (15 - x));
  //-hang_num = ~hang_num;
  //-hang_num = 0xffff;
	send_byte(hang_num,16);

	//-LD_OE((BitAction)1);		//-�޸�һ���ϲ���ʱ,����ǰ���к��ǲ��̶���,���Ա����ȹرյ����߶�ȷ�����
  //-������׼���ó�ʱ���������,Ȼ����.��������л��еĻ�,�����Ӿ���������к�
  y = y << 8;
  //-y = 0xffffff00;   //-1ʱ��Ӧ������͵�ƽ ����
  //-y = 0x00000000;   //-0ȫ��
  send_byte(y,32);	//-�ܹ���24��,һ�����ݵ���ʾ��Ҫ�ƶ�24��,����ת��


  //-hang_num = GPIO_ReadOutputData(GPIOC);
	//-hang_num = (hang_num&0xfff0) | (x&0x0f);
	//-LD_ADDR_y(hang_num);
	LD_LE((BitAction)1);
	LD_LAT((BitAction)1);
	LD_LE((BitAction)0);
  LD_LAT((BitAction)0);

}
#endif
////////////////////////////////////////////////////////////////////////////////
//-��һ����������
//-Ȼ�����ж���,��Ϊһ����ʾ�ն���ʾȫ�����������ָ��
void led_display_init(void)
{
    u8 i;

    LD_LE((BitAction)1);	//-LD_LE Ϊ0ʱ���ݲ��ܽ���������,,���������������ֱͨ,��Ϊ4����ַ��ͬʱ�仯,����ֻ�ߵ�ַ,û�����ݲ�����ռ����
    LD_LAT((BitAction)0);	//-LD_LATΪ0ʱ���ݲ��ܽ���������
    display(0,0);		//-ʹ����ʾ֮ǰˢ����ʼֵ,���������ϵ���һ����Ч��
    LD_OE((BitAction)0);	//-LD_OE Ϊ0ʱ���������������״̬

		//-�������ֵ
    led_display_flag = 0;		//-0��ʾ��ʼ����
    cartoon_start_time = Time_2048ms_Counter;
    led_display_num = 3;		//-��ʼ������
    led_display_y = 0;
    led_display_new = 0xa5;	//-��������0xa5

    led_display_page = 0;			//-��ʼ��ʾ��Χ0��4ҳ
    led_display_page_end = 3;

    Weather_flag = 0;
    led_display_data_flag = 0;
    //-i2c1_newdata_flag = 0;
    //-����һ���㹻������������ϽǱ߽�,�����ط���������������Խ��
    led_data_x = 12;		//-8;		,,�����12�Ļ�,��ô�����ж�23�ν����������
    led_data_y = 8;		//-�����8�Ļ������17�ζ���Խ��
    led_display_cn = 0;
    test_pt = 0;

    for(i = 0;i < 16;i++)	//-�������һ��ͼƬ
		{
			 	test_SIN_data[i] = (ZIKU_SIN[i][0] << 16) + (ZIKU_SIN[i][1] << 8) + ZIKU_SIN[i][2];
			 	led_display_Vdata0[0] = 0;
			 	//-led_display_data_pt[i] = 0;
		}

		led_display_data3[0] = 0xaaaaaa;
		led_display_data3[1] = 0xaaaaaa;
		led_display_data3[2] = 0x555555;
		led_display_data3[3] = 0x555555;
		led_display_data3[4] = 0xaaaaaa;
		led_display_data3[5] = 0xaaaaaa;
		led_display_data3[6] = 0x555555;
		led_display_data3[7] = 0x555555;
		led_display_data3[8] = 0xaaaaaa;
		led_display_data3[9] = 0xaaaaaa;
		led_display_data3[10] = 0x555555;
		led_display_data3[11] = 0x555555;
		led_display_data3[12] = 0xaaaaaa;
		led_display_data3[13] = 0xaaaaaa;
		led_display_data3[14] = 0x555555;
		led_display_data3[15] = 0x555555;
		led_display_data4[0] = 0x555555;
		led_display_data4[1] = 0x555555;
		led_display_data4[2] = 0xaaaaaa;
		led_display_data4[3] = 0xaaaaaa;
		led_display_data4[4] = 0x555555;
		led_display_data4[5] = 0x555555;
		led_display_data4[6] = 0xaaaaaa;
		led_display_data4[7] = 0xaaaaaa;
		led_display_data4[8] = 0x555555;
		led_display_data4[9] = 0x555555;
		led_display_data4[10] = 0xaaaaaa;
		led_display_data4[11] = 0xaaaaaa;
		led_display_data4[12] = 0x555555;
		led_display_data4[13] = 0x555555;
		led_display_data4[14] = 0xaaaaaa;
		led_display_data4[15] = 0xaaaaaa;


		led_display_start = 0x55;		//-���ж�����׼������,����ˢ����ʾ.
		led_display_ye_ok = 0;
		led_disp_L_init();
}

void led_disp_L_init(void)		//-�������������ĳ�ʼ׼��
{
	  cticks_s_page = 0;				//-������ʼ��ʱ
	  led_display_start = 0x55;	//-��ʼ��ʾ
	  led_display_new = 0x55;	//-��������ֵ
	  led_display_y = 0;	//-���ϵ���ˢ���Ĺ���
	  led_display_deal_flag = 0xaa;	//-��Щ��־λ����Ϊ�˱�֤ʱ���,������ʾ�������
	  led_display_ye_ok = 0;
	  voice_keep_data_flag = 0;		//-ֻҪ��Ϊ���������ݾ���Ҫ�ⶳ
}

//-#define test_led_display

//-�������е��õ����������Ӻ���,���ڿ��Կ��Ǵ��˲�����˵�
//-���ڰѺ������붨ʱ�ж��н���ˢ��
void led_display(void)	//-���еĶ�̬Ч�����������������ȥ��,ÿ��ˢ��ʱ��仯��,�����������Ƕ���Ч��.
{
   u8 i,temp_x,temp_y,temp_i;
   UINT32	*led_display_data_pt;
   UINT32 temp_data,temp_dataA,temp_dataB;
   static u8 temp_page=0;
   UINT8 *ZIKU_pt;


   //-ͨ������������ʾ������򵥹۲���������
   //-ÿ�ν����ж���ʾ����һ��
   if(led_display_flag == 0)	//-��������
   {
   	  //-i = i2c1_newdata_flag & 0x0f;     //-һ���жϻ�һ����ʾ
   	  //-i = 5;
   	  //-display(i,0xffffff);
   	  if(led_display_num == 3)	//-�ж�����ĸ�������������׼������,������ʾ,
		  {
		   	 led_display_data_pt = &led_display_data3[0];
		  }
		  else//- if(led_display_num == 4)
		  {
		   	 led_display_data_pt = &led_display_data4[0];
		  }

		  if(led_display_y >= 1)	//-����������������ʾ
		  {
		  	 //-led_display_y = 0;
		  	 if(led_display_num == 3)
		  	 	 led_display_num = 4;
		  	 else
		  	 	 led_display_num = 3;
		  }

		  if(Judge_Time_In_MainLoop(cartoon_page_change_time,220)==YES)
		  {

		  	 cartoon_page_change_time = cticks_ms;
         if(temp_page & 0x80)
         {
            temp_page--;
            if(temp_page <= 0x80)
              temp_page = 0;
         }
         else
         {
           temp_page++;
           if(temp_page >= (ZIKU_cartoon_num-1))
           {
             temp_page |= 0x80;
           }
         }
		  }

#ifdef test_led_display
      ZIKU_pt = &ZIKU_FF[0][0];
#else
      i = (temp_page & 0x7f);
      ZIKU_pt = ZIKU_cartoonxx[i];
		  /*if(i == 0)
		     ZIKU_pt = &ZIKU_cartoon01[0][0];
		  else if(i == 1)
		     ZIKU_pt = &ZIKU_cartoon02[0][0];
		  else if(i == 2)
		     ZIKU_pt = &ZIKU_cartoon03[0][0];
		  else if(i == 3)
		     ZIKU_pt = &ZIKU_cartoon04[0][0];
		  else if(i == 4)
		     ZIKU_pt = &ZIKU_cartoon05[0][0];
		  else if(i == 5)
		     ZIKU_pt = &ZIKU_cartoon06[0][0];
		  else if(i == 6)
		     ZIKU_pt = &ZIKU_cartoon07[0][0];
		  else if(i == 7)
		     ZIKU_pt = &ZIKU_cartoon08[0][0];
		  else if(i == 8)
		     ZIKU_pt = &ZIKU_cartoon09[0][0];
		  else if(i == 9)
		     ZIKU_pt = &ZIKU_cartoon10[0][0];
      else if(i == 10)
		     ZIKU_pt = &ZIKU_cartoon11[0][0];
      else if(i == 11)
		     ZIKU_pt = &ZIKU_cartoon12[0][0];
      else if(i == 12)
		     ZIKU_pt = &ZIKU_cartoon13[0][0];
      else if(i == 13)
		     ZIKU_pt = &ZIKU_cartoon14[0][0];
      else if(i == 14)
		     ZIKU_pt = &ZIKU_cartoon15[0][0];
      else if(i == 15)
		     ZIKU_pt = &ZIKU_cartoon16[0][0];
      else if(i == 16)
		     ZIKU_pt = &ZIKU_cartoon17[0][0];
      else if(i == 17)
		     ZIKU_pt = &ZIKU_cartoon18[0][0];
      else if(i == 18)
		     ZIKU_pt = &ZIKU_cartoon19[0][0];
      else if(i == 19)
		     ZIKU_pt = &ZIKU_cartoon20[0][0];
      else if(i == 20)
		     ZIKU_pt = &ZIKU_cartoon21[0][0];
      else if(i == 21)
		     ZIKU_pt = &ZIKU_cartoon22[0][0];
      else if(i == 22)
		     ZIKU_pt = &ZIKU_cartoon23[0][0];
      else if(i == 23)
		     ZIKU_pt = &ZIKU_cartoon24[0][0];
      else if(i == 24)
		     ZIKU_pt = &ZIKU_cartoon25[0][0];
      else if(i == 25)
		     ZIKU_pt = &ZIKU_cartoon26[0][0];
      else if(i == 26)
		     ZIKU_pt = &ZIKU_cartoon27[0][0];
      else if(i == 27)
		     ZIKU_pt = &ZIKU_cartoon28[0][0];
      else //-if(temp_page == 28)
		     ZIKU_pt = &ZIKU_cartoon29[0][0];*/
#endif


		  for(i = 0;i < 16;i++)	//-�������һ��ͼƬ
			 {
			 	   led_display_data_pt[i] = (ZIKU_pt[i*3+0] << 16) + (ZIKU_pt[i*3+1] << 8) + ZIKU_pt[i*3+2];
			 }

		  for(i = 0;i < 16;i++)
   	  {
   	  	 temp_data = led_display_data_pt[i];
   	  	 if(led_display_Vbuffx == 1)	//-�ж�����ĸ�������������׼������,������ʾ,
			   {
			   	  led_display_Vdata2[i] = temp_data;
			   }
			   else//- if(led_display_Vbuffx == 2)
			   {
			   	  led_display_Vdata1[i] = temp_data;
			   }

		     //-display(i,temp_data);		//-������ʵԽ����,�����Ƕ�ȡ������������ν
		     //-NOP_Delay(150);	//-���ʱ�䲻��̫��,������ѭ��ִ�в���,������Ҫʵ��
   	  }
   	  if(led_display_Vbuffx == 1)	//-�ж�����ĸ�������������׼������,������ʾ,
			{
			   led_display_Vbuffx = 2;
			}
			else//- if(led_display_Vbuffx == 2)
			{
			   led_display_Vbuffx = 1;
			}

   }
   else if(led_display_flag == 1)  //-������,һ�ε������еĵ�
   {
      for(i=0;i<16;i++)
      {
      	 if(led_display_Vbuffx == 1)	//-�ж�����ĸ�������������׼������,������ʾ,
			   {
			   	  led_display_Vdata2[i] = 0xffffff;
			   }
			   else//- if(led_display_Vbuffx == 2)
			   {
			   	  led_display_Vdata1[i] = 0xffffff;
			   }

        //-display(i,0xffffff);
        //-NOP_Delay(150);
      }
      if(led_display_Vbuffx == 1)	//-�ж�����ĸ�������������׼������,������ʾ,
			{
			   led_display_Vbuffx = 2;
			}
			else//- if(led_display_Vbuffx == 2)
			{
			   led_display_Vbuffx = 1;
			}

   }
   else if(led_display_flag == 2)		//-��ʾ�ض���
   {
   	  //-display(led_display_y,0xffffff);		//-��ʾָ����
   }
   else if(led_display_flag == 3)		//-��������ʾ(���ϵ���)
   {
   	  temp_y = led_display_y;

   	  /*if(led_display_num == 1)	//-�ж�����ĸ�������������׼������,������ʾ,
		  {
		   	 led_display_data_pt = &led_display_data1[0];
		  }
		  else if(led_display_num == 2)
		  {
		   	 led_display_data_pt = &led_display_data2[0];
		  }
		  else if(led_display_num == 5)	//-5 6��������������������ͨѶ�ӿ���׼���õ�
		  {
		   	 led_display_data_pt = &led_display_data5[0];
		  }
		  else if(led_display_num == 6)
		  {
		   	 led_display_data_pt = &led_display_data6[0];
		  }
		  else
		  {
		  	 led_display_data_pt = &led_display_data4[0];
		  	 //-���ֻ��һ��������������Ҫ�ֳ�׼������
		  	 for(i = 0;i < 16;i++)	//-������ת��ΪͼƬ����,����������ջ�����
			   {
		  	     led_display_data4[i] = 0;
		  	 }

			   for(i = 0;i < 16;i++)	//-������ת��ΪͼƬ����
			   {
				   if(temp_y <= 16)	//-������ת��ΪͼƬ����
				   {

			   	  	 	  //-��������ʼ��ʾ����
						   	  //-temp_data = 0xffffffff;
						   	  //-temp_data = temp_data >> (32 - temp_y*2);
						      //-led_display_data4[i] =  led_display_data3[i] & temp_data;		//-ÿ������֮���һ���ո�
                  //-�������ҿ�ʼ��ʾ����
                  temp_data = 0xffffffff;
						   	  temp_data = temp_data << (32 - temp_y*2);
						      led_display_data4[i] =  led_display_data3[i] & temp_data;		//-ÿ������֮���һ���ո�

				   }
			   }

			   temp_y = 16;
		  }*/
		  led_display_data_pt = &led_display_data0[0];

		  if(ps_flag_led_end != 2)
		  {//-���ϵ���
	   	  for(i = 0;i < temp_y;i++)	//-��������������ָ��������,�����ʾ,������֯����
	   	  {

	   	  	 if(temp_y < 16)
	   	  	 {
	   	  	 	   //-temp_data = 0xffffff;
	   	  	 	   //-temp_data = ~led_display_data_pt[i];
	   	  	 	   //-temp_data = led_display_data_pt[i] | 0x003C00;
	   	  	 	   //-�����Ѿ��л����Ĺ�����,������Ҫ��β�Ͳü���
	   	  	 	   if(temp_y > 2)
	   	  	 	   {
	   	  	 	      if(i >= (temp_y - 3))
	   	  	 	      	 temp_data = 0xffffff;	//-led_display_data_pt[i] | 0xffffff;
	   	  	 	      else
	   	  	 	      	 temp_data = led_display_data_pt[i];

	   	  	 	   }
	   	  	 	   else
	   	  	 	   	  temp_data = led_display_data_pt[i];

			     	   //-temp_i = i;


					   	 //-display(temp_i,temp_data);
					   	 //-NOP_Delay(150);
			     }
			     else
			     {//-ˢһ����������
			     	   //-if(i > 15)
			     	   //-	  temp_data = 15;
			     	   //-else
			     	   //-	  temp_data	= i;
			     	   temp_data = led_display_data_pt[i];
			     	   //-���ڲ����м��书��,��������һ���ж��������Ҫˢ������,�Ͳ�ִ��ˢ������,�����������
			     	   //-if(led_display_data_pt[temp_data] != 0)
			     	   //-{

			     	   	 	//-display(temp_data,led_display_data_pt[temp_data]);		//-������ʵԽ����,�����Ƕ�ȡ������������ν
			     	   	 	//-NOP_Delay(250);	//-���ʱ�䲻��̫��,������ѭ��ִ�в���,������Ҫʵ��
			     	   //-}
			     }

	   	  	 if(led_display_Vbuffx == 1)	//-�ж�����ĸ�������������׼������,������ʾ,
				   {
				   	  led_display_Vdata2[i] = temp_data;
				   }
				   else//- if(led_display_Vbuffx == 2)
				   {
				   	  led_display_Vdata1[i] = temp_data;
				   }

	   	  }

	   	  if(led_display_Vbuffx == 1)	//-�ж�����ĸ�������������׼������,������ʾ,
				{
				   led_display_Vbuffx = 2;
				}
				else//- if(led_display_Vbuffx == 2)
				{
				   led_display_Vbuffx = 1;
				}

   		}
   		else
   		{//-�ض�Ϊ����������ʾʱ��������Ĵ���,��������¶������ϵ���
   			for(i = 0;i < temp_y;i++)	//-��������������ָ��������,�����ʾ,������֯����
	   	  {
	   	  	 if(temp_y < 16)
	   	  	 {
	   	  	 	   //-temp_data = 0xffffff;
	   	  	 	   //-temp_data = ~led_display_data_pt[i];
	   	  	 	   //-temp_data = led_display_data_pt[i] | 0x003C00;
	   	  	 	   //-�����Ѿ��л����Ĺ�����,������Ҫ��β�Ͳü���
	   	  	 	   temp_i = 15 - i;
	   	  	 	   temp_i = temp_i & 0x0f;
	   	  	 	   if(temp_y > 2)
	   	  	 	   {
	   	  	 	      if(i >= (temp_y - 3))
	   	  	 	      	 temp_data = 0xffffff;		//-led_display_data_pt[temp_i] | 0xffffff;
	   	  	 	      else
	   	  	 	      	 temp_data = led_display_data_pt[temp_i];

	   	  	 	   }
	   	  	 	   else
	   	  	 	   	  temp_data = led_display_data_pt[temp_i];


					   	 //-display(temp_i,temp_data);
					   	 //-NOP_Delay(150);
					   	 //-display(temp_i,0);
			     }
			     else
			     {//-ˢһ����������
			     	   //-if(i > 15)
			     	   //-	  temp_data = 15;
			     	   //-else
			     	   	  temp_i	= i;
			     	   temp_data = led_display_data_pt[i];
			     	   //-���ڲ����м��书��,��������һ���ж��������Ҫˢ������,�Ͳ�ִ��ˢ������,�����������
			     	   //-if(led_display_data_pt[temp_data] != 0)
			     	   //-{
			     	   	 	//-display(temp_data,led_display_data_pt[temp_data]);		//-������ʵԽ����,�����Ƕ�ȡ������������ν
			     	   	 	//-NOP_Delay(150);	//-���ʱ�䲻��̫��,������ѭ��ִ�в���,������Ҫʵ��
			     	   //-}
			     }
			     if(led_display_Vbuffx == 1)	//-�ж�����ĸ�������������׼������,������ʾ,
				   {
				   	  led_display_Vdata2[temp_i] = temp_data;
				   }
				   else//- if(led_display_Vbuffx == 2)
				   {
				   	  led_display_Vdata1[temp_i] = temp_data;
				   }

	   	  }
	   	  if(led_display_Vbuffx == 1)	//-�ж�����ĸ�������������׼������,������ʾ,
				{
				   led_display_Vbuffx = 2;
				}
				else//- if(led_display_Vbuffx == 2)
				{
				   led_display_Vbuffx = 1;
				}

	   	}


   	  //-if(led_display_txpic_flag == 0x55)
   	  //-	led_display_txpic_flag = 0xaa;		//-ͨѶ���·���ͼƬ��ʾ����,����ͨ��ͨѶ�ڸ�����
   }
   else if(led_display_flag == 4)		//-����תȦ
   {
   	   /*
   	   if(led_display_cn_f == 0x55)
   	   {
		   	   if(led_display_cn & 0x01)
		   	   {//-��������
				   	   for(temp_data = 1;temp_data <= led_display_cn;temp_data++)
					   	   if((led_data_y >=0) && ((led_data_x - temp_data) >= 0))
					   	   {
					   	      led_display_data_round[led_data_y] = led_display_data_round[led_data_y] | (1 << (led_data_x - temp_data));
					   	      test_x[test_pt] = led_data_x - temp_data;
					   	      test_y[test_pt++] = led_data_y;
					   	   }

				   	   led_data_x = led_data_x - led_display_cn;

				   	   for(temp_data = 1;temp_data <= led_display_cn;temp_data++)
				   	      if(((led_data_y + temp_data) < 16) && (led_data_x >= 0))
				   	      {
				   	      	led_display_data_round[led_data_y + temp_data] = led_display_data_round[led_data_y + temp_data] | (1 << led_data_x );
				   	      	test_x[test_pt] = led_data_x;
					   	      test_y[test_pt++] = led_data_y + temp_data;
				   	      }


				   	   led_data_y = led_data_y + led_display_cn;
		   	   }
		   	   else
		   	 	 {//-ż������

				   	   for(temp_data = 1;temp_data <= led_display_cn;temp_data++)
					   	   if((led_data_y >=0) && ((led_data_x + temp_data) >= 0))
					   	   {
					   	      led_display_data_round[led_data_y] = led_display_data_round[led_data_y] | (1 << (led_data_x + temp_data));
					   	      test_x[test_pt] = led_data_x + temp_data;
						   	    test_y[test_pt++] = led_data_y;
					   	   }

				   	   led_data_x = led_data_x + led_display_cn;

				   	   for(temp_data = 1;temp_data <= led_display_cn;temp_data++)
				   	      if((led_data_y - temp_data) >=0)
				   	      {
				   	      	led_display_data_round[led_data_y - temp_data] = led_display_data_round[led_data_y - temp_data] | (1 << led_data_x );
				   	      	test_x[test_pt] = led_data_x;
					   	      test_y[test_pt++] = led_data_y - temp_data;
				   	      }

				   	    //-if(((led_data_y + temp_data) >=0) &&((led_data_y + temp_data) < 16))
				   	   led_data_y = led_data_y - led_display_cn;
		   	 	 }
		   	 	 led_display_cn_f = 0;
   	   }*/



   	   if(led_display_cn_f == 0x55)
   	   {
   	   	  for(temp_y=0;temp_y<16;temp_y++)
		   	  {

				   	  	led_display_data_round[temp_y] = 0;

		   	  }

   	   	  //-led_display_data_round[test_y[test_pt]] = led_display_data_round[test_y[test_pt]] | (1 << test_x[test_pt] );
   	   	  //-led_display_data_round[test_y[test_pt+1]] = led_display_data_round[test_y[test_pt+1]] | (1 << test_x[test_pt+1] );
   	   	  //-led_display_data_round[test_y[test_pt+2]] = led_display_data_round[test_y[test_pt+2]] | (1 << test_x[test_pt+2] );
   	   	  //-led_display_data_round[test_y[test_pt+3]] = led_display_data_round[test_y[test_pt+3]] | (1 << test_x[test_pt+3] );
   	   	  //-led_display_data_round[test_y[test_pt+4]] = led_display_data_round[test_y[test_pt+4]] | (1 << test_x[test_pt+4] );
   	   	  //-�洢�м����
   	   	  //-led_display_data_round1[test_y[test_pt]] = led_display_data_round1[test_y[test_pt]] | (1 << test_x[test_pt] );
   	   }

   	 	if(led_display_num == 1)
		  {
		   	 led_display_data_pt = &led_display_data1[0];
		  }
		  else if(led_display_num == 2)
		  {
		   	 led_display_data_pt = &led_display_data2[0];
		  }

		  for(i = 0;i < 16;i++)
   	  {
   	  	  temp_data = led_display_data_pt[i] & led_display_data_round1[i];
   	  	 	temp_data = temp_data | led_display_data_round[i];


		     	   if(i == 1)
		     	   	temp_i = 2;
		     	   else if(i == 2)
		     	   	temp_i = 1;
		     	   else
		     	   	temp_i = i;
		     	   //-���ڲ����м��书��,��������һ���ж��������Ҫˢ������,�Ͳ�ִ��ˢ������,�����������
		     	   //-if(led_display_data_pt[temp_data] != 0)
		     	   {
		     	   	 	//-display(temp_i,temp_data);		//-������ʵԽ����,�����Ƕ�ȡ������������ν
		     	   	 	//-NOP_Delay(150);	//-���ʱ�䲻��̫��,������ѭ��ִ�в���,������Ҫʵ��
		     	   }


   	  	 //-}
   	  }

   }
   else if(led_display_flag == 5)		//-���Ҳ��ƶ�
   {

   	  if(led_display_cn_f == 0x55)
   	  {
   	  	 for(i = 0;i < 16;i++)
   	     {
   	     	  temp_dataA = test_SIN_data[i] >> 23;
   	     	  temp_dataB = test_SIN_data[i] << 1;
   	     	  test_SIN_data[i] = temp_dataA | temp_dataB;
   	   	    //-led_display_data_round[test_y[test_pt]] = led_display_data_round[test_y[test_pt]] | (1 << test_x[test_pt] );

   	   	 }
   	   	 led_display_cn_f = 0;
   	  }



   	  if(led_display_num == 1)
		  {
		   	 led_display_data_pt = &led_display_data1[0];
		  }
		  else if(led_display_num == 2)
		  {
		   	 led_display_data_pt = &led_display_data2[0];
		  }

   	  for(i = 0;i < 16;i++)
   	  {
   	  	 	temp_data = test_SIN_data[i];

		     	   if(i == 1)
		     	   	temp_i = 2;
		     	   else if(i == 2)
		     	   	temp_i = 1;
		     	   else
		     	   	temp_i = i;
		     	   //-���ڲ����м��书��,��������һ���ж��������Ҫˢ������,�Ͳ�ִ��ˢ������,�����������
		     	   //-if(led_display_data_pt[temp_data] != 0)
		     	   {
		     	   	 	//-display(temp_i,temp_data);		//-������ʵԽ����,�����Ƕ�ȡ������������ν
		     	   	 	//-NOP_Delay(150);	//-���ʱ�䲻��̫��,������ѭ��ִ�в���,������Ҫʵ��
		     	   }


   	  	 //-}
   	  }
   }
   else if(led_display_flag == 6)		//-���м���������ɢ
   {

   	  if(led_display_cn_f == 0x55)
   	  {
   	  	  for(temp_y=0;temp_y<16;temp_y++)
		   	  {

				   	  	test_SIN_data[temp_y] = 0;

		   	  }

		   	  for(temp_y=test_pt;temp_y<16-test_pt;temp_y++)
		   	  {
		   	  	 for(temp_x=test_pt;temp_x<24-test_pt;temp_x++)
				   	 {
				   	  	test_SIN_data[temp_y] = test_SIN_data[temp_y] | (1 << temp_x);
				   	 }
		   	  }
		   	  led_display_cn_f = 0;
   	  }

      if(led_display_num == 1)
		  {
		   	 led_display_data_pt = &led_display_data1[0];
		  }
		  else if(led_display_num == 2)
		  {
		   	 led_display_data_pt = &led_display_data2[0];
		  }

   	  for(i = 0;i < 16;i++)
   	  {
   	  	 	temp_data = test_SIN_data[i] & led_display_data_pt[i];

		     	   if(i == 1)
		     	   	temp_i = 2;
		     	   else if(i == 2)
		     	   	temp_i = 1;
		     	   else
		     	   	temp_i = i;
		     	   //-���ڲ����м��书��,��������һ���ж��������Ҫˢ������,�Ͳ�ִ��ˢ������,�����������
		     	   //-if(led_display_data_pt[temp_data] != 0)
		     	   {
		     	   	 	//-display(temp_i,temp_data);		//-������ʵԽ����,�����Ƕ�ȡ������������ν
		     	   	 	//-NOP_Delay(150);	//-���ʱ�䲻��̫��,������ѭ��ִ�в���,������Ҫʵ��
		     	   }
		   	     //-display(temp_i,0);

   	  	 //-}
   	  }
   }
   else if(led_display_flag == 7)		//-�������Ͻ�����ʾ,����WIFI����,һֱ����ˢ
   {
   	  temp_y = led_display_y;		//-�����ʾ����

      /*if(led_display_num == 5)
		  {
		   	 led_display_data_pt = &led_display_data5[0];
		  }
		  else if(led_display_num == 6)
		  {
		   	 led_display_data_pt = &led_display_data6[0];
		  }*/

		  led_display_data_pt = &led_display_data0[0];

   	  for(i = 0;i < temp_y;i++)	//-��������������ָ��������,�����ʾ,������֯����
   	  {
   	  	 if(temp_y < 16)
   	  	 {
						 temp_i = 15 - i;
   	  	 	   temp_data = led_display_data_pt[temp_i];


				   	 //-display(temp_i,temp_data);
				   	 //-NOP_Delay(150);
		     }
		     else
		     {//-ˢһ����������
		     	   //-if(i > 15)
		     	   //-	  temp_i = 15;
		     	   //-else
		     	   	  temp_i	= i;
		     	   temp_data = led_display_data_pt[i];
		     	   //-���ڲ����м��书��,��������һ���ж��������Ҫˢ������,�Ͳ�ִ��ˢ������,�����������
		     	   if(led_display_data_pt[temp_i] != 0)
		     	   {
		     	   	 	//-display(temp_i,led_display_data_pt[temp_i]);		//-������ʵԽ����,�����Ƕ�ȡ������������ν
		     	   	 	//-NOP_Delay(150);	//-���ʱ�䲻��̫��,������ѭ��ִ�в���,������Ҫʵ��
		     	   }
		     }

		     if(led_display_Vbuffx == 1)	//-�ж�����ĸ�������������׼������,������ʾ,
			   {
			  	  led_display_Vdata2[temp_i] = temp_data;
			   }
			   else//- if(led_display_Vbuffx == 2)
			   {
			  	  led_display_Vdata1[temp_i] = temp_data;
			   }
   	  }
   	  if(led_display_Vbuffx == 1)	//-�ж�����ĸ�������������׼������,������ʾ,
		  {
		     led_display_Vbuffx = 2;
		  }
		  else//- if(led_display_Vbuffx == 2)
		  {
		     led_display_Vbuffx = 1;
		  }

   	  //-if(led_display_txpic_flag == 0x55)
   	  //-	led_display_txpic_flag = 0xaa;		//-ͨѶ���·���ͼƬ��ʾ����,����ͨ��ͨѶ�ڸ�����
   }
   else if(led_display_flag == 8)		//-���Զ���������,��ī�������ƶ�
   {
   	  /*if(led_display_num == 1)
		  {
		   	 led_display_data_pt = &led_display_data1[0];
		  }
		  else if(led_display_num == 2)
		  {
		   	 led_display_data_pt = &led_display_data2[0];
		  }*/

		  led_display_data_pt = &led_display_data0[0];

		  if(led_display_cn_f == 0x55)
   	  {
   	  	 for(i = 0;i < 16;i++)
   	     {
   	     	  if((i == 5) || (i == 6) || (i == 7))
   	     	  {
	   	     	  led_display_data_temp[i] =  led_display_data_pt[i] | (7 << led_display_move_pt);
   	   	    }
            else
              led_display_data_temp[i] =  led_display_data_pt[i];

   	   	 }
   	   	 led_display_cn_f = 0;


	   	  for(i = 0;i < 16;i++)
	   	  {
	         temp_data =  led_display_data_temp[i];

	   	     if(led_display_Vbuffx == 1)	//-�ж�����ĸ�������������׼������,������ʾ,
					 {
					 	  led_display_Vdata2[i] = temp_data;
					 }
					 else//- if(led_display_Vbuffx == 2)
					 {
					 	  led_display_Vdata1[i] = temp_data;
					 }
	   	     //-display(i,temp_data);		//-������ʵԽ����,�����Ƕ�ȡ������������ν
			 	   //-NOP_Delay(150);	//-���ʱ�䲻��̫��,������ѭ��ִ�в���,������Ҫʵ��
	   	  }

	   	  if(led_display_Vbuffx == 1)	//-�ж�����ĸ�������������׼������,������ʾ,
				{
				   led_display_Vbuffx = 2;
				}
				else//- if(led_display_Vbuffx == 2)
				{
				   led_display_Vbuffx = 1;
				}
		  }
   }
   else		//-��ʾ����
   {
   	  /*
      //for(i=0;i<16;i++)
      //{//��ʾ����
        display(15,0x30060);
        //-display(15,0);		//-������Ŀ����Ϊ������������ݿ�����ʾ����Ӧ������,��ʵ��ȫ�����Ȱ�����׼��������ʾ,������ֱ�Ӷ�����
        //-NOP_Delay(70*300);	//-�������ʱ�����ǵƵ���������,������ҪȨ������
        display(14,0x48090);
        //-display(14,0);
        //-NOP_Delay(20);
        display(13,0x84108);
        //-display(13,0);
        //-NOP_Delay(20);
        display(12,0x102204);
        //-display(12,0);
        display(11,0x201c02);
        //-display(11,0);
        display(10,0x200002);
        //-display(10,0);
        display(9,0x100004);
        //-display(9,0);
        display(8,0x80008);
        //-display(8,0);
        display(7,0x40010);
        //-display(7,0);
        display(6,0x20020);
        //-display(6,0);
        display(5,0x10040);
        //-display(5,0);
        display(4,0x8080);
        //-display(4,0);
        display(3,0x4100);
        //-display(3,0);
        display(2,0x2200);
        //-display(2,0);
        display(1,0x1400);
        //-display(1,0);
        display(0,0x800);
        display(0,0);
        //-delay(10);
      //}
      */
   }
}

//-�Գ�����д�����ȡ������Ҫ������
void led_display_deal(void)
{
     //-ʵ���˻�ҳ��ʾ
     if(ps_flag_led == 1)
	   {//-1 ���ϵ���
	 	    led_display_page++;
     	  if(led_display_page > led_display_page_end)
     	 	   led_display_page = 0;

	 	    ps_flag_led = 0;
	 	    ps_flag_led_end = 1;
	 	    ps_flag_led_disp = 1;
	 	    led_display_new = 0x55;
	 	    led_display_txpic_flag = 0;		//-������ʾ�̻�ͼƬ
	 	    voice_keep_data_flag = 0;		//-ˢ��֮�������ⶳ����

	 	    if((led_display_page == 0) || (led_display_page == 3))
	 	    {//-���л���PM2.5 CO2��ʱ����Ҫ�ж��Ƿ���������,û�о���Ҫ�ȴ�
	 	    	 if(led_display_data_flag != 2)
	 	    	 {
	 	    	 	 Sensor_data_wait_time = cticks_ms;
	 	    	 	 led_display_data_flag = 1;
	 	    	 }

	 	    	 if(led_display_data_flag == 2)
	 	    	 {
	 	    	 	  //-��������,�Ӻ���,����Ϊ�˲���ͬ������ô��
			  	 		UART1_transmit_flag = YES;		//-������֯���ݷ���
		          UART1_transmit_control = 1;	//-�������������͵�����
	 	    	 }
	 	    	 else
	 	    	 {
	 	    	    //-��������,�Ӻ���,����Ϊ�˲���ͬ������ô��
			  	 		UART1_transmit_flag = YES;		//-������֯���ݷ���
		          UART1_transmit_control = 8;	//-�������������͵�����
        	 }
	 	    }
	 	    else
	 	    {
	 	    	 led_display_flag = 3;	//-�л�֮��ֻҪ������Ҫ�ȴ����ݸ��µĲ˵�,����������ʾ
	 	    	 //-if(led_display_data_flag == 2)
	 	    	 //-{
	 	    	 //-	 Sensor_data_wait_time = cticks_ms;		//-�л�������ҳ����Ч�Կ�ʼ��ʱ
	 	    	 //-	 led_display_data_flag = 3;		//-��Ч����ʧЧ��ʼ��ʱ
	 	    	 //-}
	 	    	 //-2015/8/17 20:51:07 by zj
	 	       //-��Ŀǰ��������״ֵ̬��·����������������һ��
	 	       //-if(voice_flag == 0x55) //-����PM2.5��CO2��Ҫ�ر�Ȼ�ǲ���,��Ϊ��һ��ʵʱ������
	 	       {//-����ÿ�ζ�����,�Ƿ�����7620������Ĳ���������STM32����
		 	       UART1_transmit_flag = YES;		//-������֯���ݷ���
	           UART1_transmit_control = 5;	//-�������������͵�����
	           voice_keep_data_flag = 0;		//-ʹ��������,��ֵ0��Ϊ��ˢ��ֵ,Ȼ�󶳽����µ���ʾ����
         	 }

	 	    }


	   }
	   else if (ps_flag_led == 2)
	   {//-2 ���µ���
     	  if(led_display_page > 0)
     	 	   led_display_page--;
     	 	else
     	 		 led_display_page = led_display_page_end;

	 	     ps_flag_led = 0;
	 	     ps_flag_led_end = 2;
	 	     ps_flag_led_disp = 2;
	 	     led_display_new = 0x55;		//-ÿ��һҳ���б�Ҫ����׼������
	 	     led_display_txpic_flag = 0;
	 	     voice_keep_data_flag = 0;		//-ˢ��֮�������ⶳ����

	 	    if((led_display_page == 0) || (led_display_page == 3))
	 	    {//-���л���PM2.5 CO2��ʱ����Ҫ�ж��Ƿ���������,û�о���Ҫ�ȴ�
	 	    	 if(led_display_data_flag != 2)
	 	    	 {
	 	    	 	 Sensor_data_wait_time = cticks_ms;
	 	    	 	 led_display_data_flag = 1;

	 	    	 }

	 	    	 if(led_display_data_flag == 2)
	 	    	 {
	 	    	 	  //-��������,�Ӻ���,����Ϊ�˲���ͬ������ô��
			  	 		UART1_transmit_flag = YES;		//-������֯���ݷ���
		          UART1_transmit_control = 1;	//-�������������͵�����
	 	    	 }
	 	    	 else
	 	    	 {
	 	    	    //-��������,�Ӻ���,����Ϊ�˲���ͬ������ô��
			  	 		UART1_transmit_flag = YES;		//-������֯���ݷ���
		          UART1_transmit_control = 8;	//-�������������͵�����
        	 }
	 	    }
	 	    else
	 	    {
	 	    	 led_display_flag = 3;	//-�л�֮��ֻҪ������Ҫ�ȴ����ݸ��µĲ˵�,����������ʾ
	 	    	 //-if(led_display_data_flag == 2)	//-ʧЧ��ʼ��ʱ��Ϩ����Ļ��ʼ
	 	    	 //-{
	 	    	 //-	 Sensor_data_wait_time = cticks_ms;		//-�л�������ҳ����Ч�Կ�ʼ��ʱ
	 	    	 //-	 led_display_data_flag = 3;		//-��Ч����ʧЧ��ʼ��ʱ
	 	    	 //-}
	 	    	 //-2015/8/17 20:51:07 by zj
	 	       //-��Ŀǰ��������״ֵ̬��·����������������һ��
	 	       //-if(voice_flag == 0x55)
	 	       {
		 	       UART1_transmit_flag = YES;		//-������֯���ݷ���
	           UART1_transmit_control = 5;	//-�������������͵�����
	           voice_keep_data_flag = 0;		//-ʹ��������,��ֵ0��Ϊ��ˢ��ֵ,Ȼ�󶳽����µ���ʾ����
         	 }

	 	    }

	   }

    if(ps_flag_led_dis == 0x55)
    {
    	 ps_flag_led_dis = 0;

       voice_keep_data_flag = 0;
       if((led_display_page == 0) || (led_display_page == 3)) //-Ϊ�˱�֤����������ʱ��Ҳ���Թ�����ʾ������
	 	    {//-���л���PM2.5 CO2��ʱ����Ҫ�ж��Ƿ���������,û�о���Ҫ�ȴ�
	 	    	 if(led_display_data_flag != 2)
	 	    	 {
	 	    	 	 Sensor_data_wait_time = cticks_ms;
	 	    	 	 led_display_data_flag = 1;
	 	    	 }

	 	    	 /*if(led_display_data_flag == 2)
	 	    	 {
	 	    	 	  //-��������,�Ӻ���,����Ϊ�˲���ͬ������ô��
			  	 		UART1_transmit_flag = YES;		//-������֯���ݷ���
		          UART1_transmit_control = 1;	//-�������������͵�����
	 	    	 }
	 	    	 else
	 	    	 {
	 	    	    //-��������,�Ӻ���,����Ϊ�˲���ͬ������ô��
			  	 		UART1_transmit_flag = YES;		//-������֯���ݷ���
		          UART1_transmit_control = 8;	//-�������������͵�����
        	 }*/
	 	    }
       else
       {
          UART1_transmit_flag = YES;		//-������֯���ݷ���
          UART1_transmit_control = 9;
       }

    }

    /*
    if(CSKEY_DATA == 1)
    {//-����BIN1��ʾ��ֵ
    	  led_display_page = 8;
    }
    else if(CSKEY_DATA == 5)
    {//-����BIN5�ָ�ѭ��ҳ��ʾ
    	  led_display_page = 0;
    }
    */
    if(voice_keep_data_flag == 0)
    {
		    //-led_display_page = 3;
		    //-PM2.5 CO2 �¶� ʪ�� ���� ʱ��	��ǿ ����ֵ
		    if(led_display_page == 0)
		    {//-��ʾPM2.5
		    	 led_display_data = pm_data;		//-���Կ��Ǹ�����Чֵ��Χ,�������Գ�����,Ҳ����������
		    	 //-led_display_data = 12;
		    }
		    else if(led_display_page == 1)
		    {//-�¶ȱ��滻������
		    	 //-led_display_data = temperature_data;
	         led_display_data = Noise_Value;
		    }
		    else if(led_display_page == 2)
		    {//-ʪ�ȱ��滻��VOC
		    	 //-led_display_data = humidity_data;
		    	 led_display_data = VOC_data;
		    }
		    else if(led_display_page == 3)
		    {//-CO2
		    	 led_display_data = co2_data;
		    }
		    else if(led_display_page == 4)
		    {//-����
		    	 //-led_display_data = Noise_Value;		//-������ʱ���
		    }
		    else if(led_display_page == 20)
		    {//-��ǿ
		    	 led_display_data = als_data / 100;
		    }
		    else if(led_display_page == 21)
		    {//-����ֵ
		    	 led_display_data = CSKEY_DATA;
		    }
    }
    //-ͨ���������Ѵ�����Ϣ��ʾ����,��ʽ Exx
    //-if(sys_err_flag != 0)
    //-  led_display_data = sys_err_flag;
    if((UART1_transmit_control == 5) || (UART1_transmit_control == 1) || (UART1_transmit_control == 9))	//-�������͵�����Ϊ�˲���һ�¶���һ��ʱ��
    {
    	voice_keep_data_flag = 0x55;
    	voice_keep_data_time = cticks_ms;
    }

    if(Judge_Time_In_MainLoop(voice_keep_data_time,8000)==YES)
    {
    	 voice_keep_data_flag = 0;		//-��ʱ����������ʾ����
    }


}

//-׼����Ҫ��ʾ������,׼����֮��,�ñ�־,�ó���ˢ��,����׼������һ����̬"ͼƬ"
void led_display_var(void)
{
	 UINT8  wei4,wei3,wei2,wei1,sign,xwei;
	 u8 i;
	 UINT32	*led_display_data_pt;		//-ָ���д����ĵ�ַ
	 UINT8 *ZIKU_pt;
	 UINT32	led_display_data_var[7];
	 UINT8  pic_x;		//-0��23		һ��������5*7�Ĵ�С
	 UINT8  pic_y;    //-0��15

	 //-led_display_data =99;
	 if((led_display_new == 0x55) && (led_display_txpic_flag == 0))	//-����ʾͨѶ��ͼƬ,Ȼ���ˢ�Լ�ϵͳ��ͼƬ,���ǿ�������ʱҲ��ˢ�Լ���ͼƬ
	 {
	 	   //-��һ�������ݷ���Ϊ��λ��
	 	   //-���ݵ����λΪ����Ϊ,����λΪ����ֵ
	 	   sign = 0;
	 	   if((led_display_data & 0x8000) != 0)
	 	   {
	 	   	  led_display_data &= 0x7fff;
	 	   	  sign = 1;
	 	   }
			 wei1 = led_display_data % 10;
			 wei2 = led_display_data / 10;
			 wei2 = wei2 % 10;
			 wei3 = led_display_data / 100;
			 wei3 = wei3 % 10;
			 wei4 = led_display_data / 1000;
			 wei4 = wei4 % 10;		//-��������Ŀ���Ƿ�ֹԽ��,���Զ�λ,������ʾ�����Ĳ���

			 if(wei4 == 0)
			 {
				 if(wei3 == 0)
				 {
				 	  if(wei2 != 0)
				    {
				 	     wei3 = 10;
				 	     xwei = 2;		//-ָʾ��ʾ��λ����
				    }
				    else
				    {
				    	 wei2 = 10;
				    	 wei3 = 10;
				    	 xwei = 1;
				    }
				 }
				 else
				 {
				 	 wei4 = 10;
				 	 xwei = 3;
				 }
			 }
			 else
			 	 xwei = 4;

			 if(sign == 1)
			 	 wei3 = 11;		//-��Ը���ֻ���¶�,��������ͷ���λ3��

			 pic_x = 0;
			 pic_y = 0;
			 //-�ر��޸��ض�λ��ֵ,������λ3
			 //-if(sys_err_flag != 0)
			 //-	  wei3 = 10;

      //-led_display_page = 6;
			//-PM2.5 CO2 �¶� ʪ��  ���� ���� ʱ�� ��ǿ ����ֵ
			if(led_display_page == 0)
	    {//-��ʾPM2.5
	    	 ZIKU_pt = &ZIKU_PM25[0][0];
	    	 pic_x = 0;
			   pic_y = 1;
	    }
	    else if(led_display_page == 1)
	    {//-�¶ȱ��滻������
	    	 //-ZIKU_pt = &ZIKU_TEMP[0][0];
	    	 //-pic_x = 9;
			   //-pic_y = 5;
			   ZIKU_pt = &ZIKU_yawp[0][0];
	    	 //-pic_x = 11;
			   //-pic_y = 5;
			   pic_x = 0;
			   pic_y = 1;
	    }
	    else if(led_display_page == 2)
	    {//-ʪ�ȱ��滻��VOC
	    	 //-ZIKU_pt = &ZIKU_humidity[0][0];
	    	 //-pic_x = 9;
			   //-pic_y = 4;
			   ZIKU_pt = &ZIKU_VOC[0][0];
	    	 pic_x = 0;
			   pic_y = 1;
	    }
      else if(led_display_page == 3)
	    {//-CO2
	    	 ZIKU_pt = &ZIKU_CO2[0][0];
	    	 pic_x = 0;
			   pic_y = 1;
	    }
	    //-else if(led_display_page == 4)
	    //-{//-����
	    //-	 ZIKU_pt = &ZIKU_yawp[0][0];
	    //-	 pic_x = 11;
			//-   pic_y = 5;
	    //-}
	    else if(led_display_page == 4)
	    {//-����
         if(Weather_flag == 1)
	    	 	  ZIKU_pt = &ZIKU_sunshine[0][0];		//-��
	    	 else if(Weather_flag == 2)
	    	 	  ZIKU_pt = &ZIKU_shade[0][0];				//-��
	    	 else if(Weather_flag == 3)
	    	 	  ZIKU_pt = &ZIKU_cloudy[0][0];				//-����
	    	 else if(Weather_flag == 4)
	    	 	  ZIKU_pt = &ZIKU_rain[0][0];	  		//-��
	    	 else if(Weather_flag == 5)
	    	 	  ZIKU_pt = &ZIKU_snow[0][0];	  		//-ѩ
	    	 else
	    	 	  ZIKU_pt = &ZIKU_cry[0][0];		//-0����������ʾ����ʧ��,�����,���������ܵ�����
	    	 pic_x = 0;
			   pic_y = 0;
	    }
	    else if(led_display_page == 6)
	    {//-ʱ��
	    	 ZIKU_pt = &ZIKU_TIME[0][0];		//-���ﶨʱ�л�����������ҫ�Ĺ���
	    	 pic_x = 0;
			   pic_y = 4;
			   //-wei1 = m_min % 10;		//-����������⴦��
			   //-wei2 = m_min / 10;
			   //-wei3 = m_hour % 10;
			   //-wei4 = m_hour / 10;
	    }
	    else if(led_display_page == 20)
	    {//-��ǿ
	    	 //-ZIKU_pt = als_data / 100;
	    	 pic_x = 0;
			   pic_y = 0;
	    }
	    else if(led_display_page == 21)
	    {//-����ֵ
	    	 //-ZIKU_pt = CSKEY_DATA;
	    	 pic_x = 9;
			   pic_y = 4;
	    }

	    if(pic_x == 0)
	    	//-x�ᴦ��,�Ա�ʱʱ����
	    	pic_x = (24 - xwei*6)/2 + pic_x;

			 /*if(led_display_num == 1)		//-������������Ϊ����,��������ʾ�Ŀ�����֯����ʾ����
			 {//-led_display_data2[16]
			 	  led_display_data_pt = &led_display_data2[0];

			 }
			 else
			 {//-led_display_data1[16]
			 	  led_display_data_pt = &led_display_data1[0];

			 }*/
			 led_display_data_pt = &led_display_data0[0];	//-��Ŀǰ��Ҫ��ʾ�����ݷ������������,��׼����ԭʼ��һ��ͼƬ,���ڿ��Զ�����ͼƬ�����Ա���Ч��


			 for(i = 0;i < 16;i++)	//-�������һ��ͼƬ
			 {
			 	   led_display_data_pt[i] = (ZIKU_pt[i*3+0] << 16) + (ZIKU_pt[i*3+1] << 8) + ZIKU_pt[i*3+2];
			 }

			 if(((led_display_data_flag != 1) || ((led_display_page != 0) && (led_display_page != 3))) && (led_display_page != 4))
			 {//-������0 ����Ҫ��֯����
				 //-��Ҫ��֯һ�����ݵ�ͼƬ
				 for(i = 0;i < 7;i++)	//-������ת��ΪͼƬ����
				 {
				     led_display_data_var[i] =  (ZIKU_XIAO[wei4][i] << 18) + (ZIKU_XIAO[wei3][i] << 12) + (ZIKU_XIAO[wei2][i] << 6) + ZIKU_XIAO[wei1][i];		//-ÿ������֮���һ���ո�
				 }
			 }
			 else
			 {//-û������ֱ�Ӹ�0
			 	  for(i = 0;i < 7;i++)	//-������ת��ΪͼƬ����
				  {
				     led_display_data_var[i] =  0;		//-ÿ������֮���һ���ո�
				  }
			 }

			 for(i = 0;i < 7;i++)	//-������ͼ�ںϵ�һ��
			 {
			 	   led_display_data_pt[i + pic_y] |= led_display_data_var[i] << pic_x;
			 }

			 if((led_display_data_flag == 1) && ((led_display_page == 0) || (led_display_page == 3)))
			 {
			 	  led_display_y = 16;
	 	 			led_display_flag = 8;	//-��ʾ�ȴ����ݸ���״̬
	 	 	 }

			 /*if(led_display_num == 1)		//-������������Ϊ����,��������ʾ�Ŀ�����֯����ʾ����
			 {//-led_display_data2[16]
			 	  led_display_data_pt = &led_display_data2[0];
			 	  led_display_num = 2;		//-ֻ�����ݸ��ĺ���֮������л���ʾ��
			 }
			 else
			 {//-led_display_data1[16]
			 	  led_display_data_pt = &led_display_data1[0];
			 	  led_display_num = 1;
			 }*/

			 //-led_display_new = 0xaa;	//-����׼������,�����л���ʾ��
			 //-led_display_new = 0xa5;	//-��ʱ��������
			 //-led_display_y = 0;
			 //-for(i = 0;i < 16;i++)	//-������ת��ΪͼƬ����
			 //-{
			 //-    led_display_data3[i] =  led_display_data_pt[i];		//-ÿ������֮���һ���ո�
			 //-    //-led_display_data3[i] =  0xffffff;		//-ÿ������֮���һ���ո�
			 //-    //-led_display_data_pt[i] =  0xffffff;		//-ÿ������֮���һ���ո�
			 //-}
			 //-led_display_num = 3;
			 led_display_new = 0xaa;	//-����׼������,�����л���ʾ��
	 }

	 if(led_display_txpic_flag == 0x55)
	 {//-����ˢͼƬ׼��
	 	  /*if(led_display_num == 5)		//-������������Ϊ����,��������ʾ�Ŀ�����֯����ʾ����
			{//-led_display_data2[16]
				 led_display_data_pt = &led_display_data6[0];

			}
			else
			{//-led_display_data1[16]
				 led_display_data_pt = &led_display_data5[0];

			}*/

			led_display_data_pt = &led_display_data0[0];		//-׼����ԭʼ��һ��ͼƬ,���ڿ��Զ�����ͼƬ�����Ա���Ч��

			if(led_display_txpic_num == 0)
	    {//-
	    	 ZIKU_pt = &ZIKU_WIFI[0][0];
	    }
	    else if(led_display_txpic_num == 1)
	    {//-
	    	 ZIKU_pt = &ZIKU_cloudy[0][0];
	    }
	    else if(led_display_txpic_num == 2)
	    {//-
	    	 ZIKU_pt = &ZIKU_sunshine[0][0];
	    }
	    else if(led_display_txpic_num == 3)
	    {//-
	    	 ZIKU_pt = &ZIKU_lightning[0][0];
	    }
	    else if(led_display_txpic_num == 4)
	    {//-
	    	 ZIKU_pt = &ZIKU_rain[0][0];
	    }
	    else if(led_display_txpic_num == 5)
	    {//-
	    	 ZIKU_pt = &ZIKU_WIFIOFF[0][0];
	    }
	    else if(led_display_txpic_num == 6)
	    {//-
	    	 ZIKU_pt = &ZIKU_DROOM[0][0];
	    }
	    else if(led_display_txpic_num == 7)
	    {//-
	    	 ZIKU_pt = &ZIKU_HOOMIN[0][0];
	    }
	    else if(led_display_txpic_num == 8)
	    {//-
	    	 ZIKU_pt = &ZIKU_HOMEOUT[0][0];
	    }
	    else if(led_display_txpic_num == 9)
	    {//-
	    	 ZIKU_pt = &ZIKU_laugh[0][0];
	    }
	    else if(led_display_txpic_num == 10)
	    {//-
	    	 ZIKU_pt = &ZIKU_cry[0][0];
	    }
	    else if(led_display_txpic_num == 20)		//-�������ѧϰ����ʾ
	    {//-
	    	 ZIKU_pt = &ZIKU_00[0][0];
	    }
	    else if(led_display_txpic_num == 21)		//-�������ѧϰ����ʾ
	    {//-
	    	 ZIKU_pt = &ZIKU_01[0][0];
	    }
	    else if(led_display_txpic_num == 22)		//-�������ѧϰ����ʾ
	    {//-
	    	 ZIKU_pt = &ZIKU_02[0][0];
	    }
	    else if(led_display_txpic_num == 23)		//-�������ѧϰ����ʾ
	    {//-
	    	 ZIKU_pt = &ZIKU_03[0][0];
	    }
	    else if(led_display_txpic_num == 24)		//-�������ѧϰ����ʾ
	    {//-
	    	 ZIKU_pt = &ZIKU_04[0][0];
	    }
	    else if(led_display_txpic_num == 255)
	    {//-
	    	 ZIKU_pt = &ZIKU_FF[0][0];
	    }
	    else
	    {//-
	    	 ZIKU_pt = &ZIKU_user[0][0];
	    }

			for(i = 0;i < 16;i++)	//-�������һ��ͼƬ
			{
				 led_display_data_pt[i] = (ZIKU_pt[i*3+0] << 16) + (ZIKU_pt[i*3+1] << 8) + ZIKU_pt[i*3+2];
			}

			/*if(led_display_num == 5)		//-������������Ϊ����,��������ʾ�Ŀ�����֯����ʾ����
			{//-led_display_data2[16]
				 led_display_num = 6;
			}
			else
			{//-led_display_data1[16]
				 led_display_num = 5;
			}*/

			//-�ж��Ƿ����ض�����
			if(led_display_txpic_num == 0)
			{
				 led_display_new = 0xa5;	//-��������
	  	   led_display_flag = 7;
			}
			else
				 led_display_flag = 3;		//-������ʾ

			led_display_txpic_flag = 0xaa;

	 }

	 if((led_display_flag == 7) && (Judge_Time_In_MainLoop(cartoon_end_wait_time,20000)==YES))		//-���65536
	 {
	 	  led_display_new = 0;
	  	led_display_flag = 3;
	  	//-led_display_new = 0xa5;	//-����������ʾ
	  	led_display_txpic_num = 10;	//-Ŀǰָ��Ϊ�Զ��������Ҫ�ǿ���.
      led_display_txpic_flag = 0x55;	//-��ʱ��ʾָ����ͼƬ�Ա�������

	 }

	 //-if((led_display_data_flag == 1) && ((led_display_page == 0) || (led_display_page == 3)))
	 //-	 led_display_flag = 8;	//-��ʾ�ȴ����ݸ���״̬
}

//-����һ����ʾЧ����Ҫһ��ͼƬһ��ͼƬ��ˢ,������һ������һ�����ݵ��޸�
//-��ô�Ҿ���Ҫ�޸ĵ�ǰȫ������������
//-����������ˢһ��̶�������,���������ǰ���ܸ�������,�����������̿����ǵ�һ��������ʱ��
//?ͻȻ������ʱ����Ҫ��֤��ʼ״̬,�����ſ�����ʾ��ȷ,�������һ��ʱ��Ĳ�ȷ��״̬
void led_display_ye_it(void)		//-ÿ���ж�ˢ�����յ�������
{
	  if((led_display_start == 0x55) || (led_display_flag == 0))		//- && (led_display_deal_flag == 0x55))		//-ֻҪ�����˾���Ҫʱʱ׼��ҳ����,��Ϊ������������,û��׼���ò����ܵ�
    {//-����ļ�ʱ��׼�������ݶ�����Ҫˢ����ʱ��׼��������,���Ϩ���˾Ͳ���Ҫ��
	    if(led_display_new != 0xa5)
	    {
	    	  if(led_display_y > 15)
			    	 led_display_y = 16;		//-Ϊ��ˢ���һ��,��һ��������16

			    if(((led_display_start == 0x55) || (led_display_page == 0)) && (led_display_y < 16))
			       led_display_y++;


	    }
	    else
	    {//-���ڶ�����,���ڲ��ý���ʾ��ʽ
	    	  //-if((led_display_start == 0x55) && (led_display_y < 16))
	    	  if(led_display_y < 16)
			       led_display_y++;

	    	  if(led_display_y > 15)
			    {
			    	led_display_y = 0;		//-Ϊ��ˢ���һ��,��һ��������16
			    	//-led_display_new = 0x55;	//-��ʱ����������������
			    }

	    }
	    //-if(led_display_new == 0xaa)
	    //-	led_display_cn++;			//-ˢ��һ����ʾ����1

	    //-���������ƶ�����
	    if(led_display_move_pt < 21)
	    {
	    	 led_display_move_pt++;
	    	 led_display_cn_f = 0x55;
	    }
	    else
	    {
	    	 led_display_move_pt = 0;
	    	 led_display_cn_f = 0x55;
	    }

	    led_display();		//-���ն�ʱ�л���Ҫ��ʾ������,���ڶ�������
	    led_display_ye_ok = 0x55;		//-ҳ����׼������
    }
}

void led_display_hang_it(void)		//-ÿ���ж�ˢ�����յ�������
{
	  UINT32 temp_data;
	  UINT8  temp_i;

	  if(((led_display_start == 0x55) || (led_display_flag == 0)) && (led_display_ye_ok == 0x55))		//-�����ڿ���״̬��ʱ����һֱ��ʾ��,ֱ�����������ſ�������
    {
       if(led_display_pageing == 0)
       {
       	 //-׼��һҳ����,Ȼ��ÿҳ����ˢ��5�κ�׼����һҳ����,���û�и��µĻ�,��ʹ�õ�ǰ����
         if(led_display_Vbuffx == 1)	//-�ж�����ĸ�������������׼������,������ʾ,
         {
            led_display_Vdata_pt = &led_display_Vdata1[0];
         }
         else if(led_display_Vbuffx == 2)
         {
            led_display_Vdata_pt = &led_display_Vdata2[0];
         }
         else//- if(led_display_Vbuffx == 0)	//-��ͻȻ������ʱ����Ҫ׼������������ʾ,��������֮ǰ�����ݶ�����Ч��
         {
            led_display_Vdata_pt = &led_display_Vdata0[0];
         }
         led_display_pageing = 0x55;
		   }
		   if((ps_flag_led_disp == 2) || (led_display_flag == 7))
		   {
	        temp_i = led_display_Hx;
	   	  	temp_i = (~temp_i) & 0x0f;
	   	  	temp_data = led_display_Vdata_pt[temp_i];
	        display(temp_i,temp_data);
	        if(led_display_Hx == 15)
	        	ps_flag_led_disp = 0;
    	 }
    	 else
    	 {
	        temp_data = led_display_Vdata_pt[led_display_Hx];
	        display(led_display_Hx,temp_data);
    	 }

	    led_display_Hx++;
	    if((led_display_flag == 0) || (led_display_flag == 1))
	    {
	    	if(led_display_Hx > 15)
	    	{
		    	led_display_Hx = 0;
		    	led_display_pageing = 0;		//-�ж���һ������֮��,ע��ı��ٶ�
		    	//-led_display_ye_ok = 0;	//-ˢ��������һҳ�ſ��Ի�����
		    }
	    }
	    else
		    if(led_display_Hx >= led_display_y)
		    {
		    	led_display_Hx = 0;
		    	led_display_pageing = 0;
		    	//-led_display_ye_ok = 0;
		    }
	  }
    else
    {
       display(0,0);
       led_display_Hx = 0;		//-��Ҫȷ����ʼ״̬
       led_display_pageing = 0;
       led_display_Vbuffx = 0;
       //-led_display_ye_ok = 0;
    }
}

