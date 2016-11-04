/*
2015/4/9 19:03:41	by zj
led_display.c  用于驱动点阵屏的显示,先实现功能层次最后再整合

2015/7/27 15:17:12
现在采用每次中断刷新一行的快速中断方式:
第一步 主函数提取数据
第二步 组织一页需要显示的内容
第三步 定时(80ms)对需要显示的图片进行处理,处理成需要显示的图片
第四步 对处理后需要显示的图片傻瓜式刷行就行了

*/
#include "user_conf.h"

//-BYTE  Temp_Flag;


//-宏定义,这样方面处理,最终可调整位置
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
74HC4514---4到16的译码芯片

PA6	=	LD_OE			enable input (active LOW)		,,如果不为低的话输出16个引脚都为0
PB0		LD_LE     latch enable input (active HIGH)	,,锁存地址值,无效时(为0)改变地址值,不影响输出结果
PC0	=	LD_A0
PC1	=	LD_A1
PC2	=	LD_A2
PC3	=	LD_A3

Q0 to Q15       multiplexer outputs (active HIGH)


SC6616---16位恒流LED驱动器

PA7	=	LD_SDI		移位寄存器串行数据输入端口
PA5	=	LD_CLK		移位寄存器时钟输入端口							,,上升沿采样
PA4	=	LD_LAT		移位寄存器的数据锁存控制端口				,,控制锁存器的值是否来自移位寄存器
PA6	=	LD_OE			16路恒流源输出使能端（低电平有效）	,,无效时16路输出为高阻态

锁存器控制端决定数据是否写入锁存器,当低电平时锁存器内的数据不变化,高电平时移位寄存器的值写入锁存器.

注:
由于任何时候只能选择其中一行中多点进行显示,所以如果需要看到一个完整的图案的话,只能快速刷屏利用人眼
的滞留现象看到"完整"的图案.

视觉暂留性:对于中等亮度的光刺激，视觉暂留时间约为0．05至0．2秒。
这里测试下来必须20ms之内全部刷一遍,否则闪,频率越快越亮
*/

void send_bit(u8 data)
{

  LD_SDI((BitAction)(data&0x01));
  //-翻转时钟信号上升沿时采用
  LD_CLK((BitAction)0);
  LD_CLK((BitAction)1);
}

void send_byte(u32 data,u8 length)      //-一列的数据送到总线上
{
  u8 count;
  //U2_LATCH=1;
  for(count=0;count<length;count++)
  {
    //U2_LATCH=1;
    send_bit(data>>count);
  }
  //-锁存有效下,把锁存数据进行更新
  //-LD_LAT((BitAction)1);
  //-NOP_Delay(2);
  //-LD_LAT((BitAction)0);
}

#if 0
//-x 表示显示行号(0~15) ,,旧点阵屏
void display(u8 x,u32 y)
{
	//-u8 i;
	u16  hang_num;

	//-LD_OE((BitAction)1);		//-修改一行上参数时,由于前面行号是不固定的,所以必须先关闭等两者都确定后打开
  //-必须先准备好长时间的数据列,然后换行.否则快速切换列的话,由于视觉滞留会错行号
  send_byte(y,24);	//-总共是24列,一个数据的显示需要移动24次,来并转串

  hang_num = GPIO_ReadOutputData(GPIOC);
	hang_num = (hang_num&0xfff0) | (x&0x0f);
	LD_ADDR_y(hang_num);
	LD_LE((BitAction)1);
	LD_LAT((BitAction)1);
	LD_LE((BitAction)0);
  LD_LAT((BitAction)0);
	//-LD_OE((BitAction)0);
  //-上面刷新了行值,接着需要输出,,下面的转化可有可无显示结果影响不大
  //-LD_LE(1);			//-LD_LE 为0时数据不能进入锁存器
  //-LD_LE(0);
}
#else
//-x 表示显示行号(0~15)
void display(u8 x,u32 y)
{
	//-u8 i;
	u16  hang_num = 0;

	hang_num = hang_num | (1 << (15 - x));
  //-hang_num = ~hang_num;
  //-hang_num = 0xffff;
	send_byte(hang_num,16);

	//-LD_OE((BitAction)1);		//-修改一行上参数时,由于前面行号是不固定的,所以必须先关闭等两者都确定后打开
  //-必须先准备好长时间的数据列,然后换行.否则快速切换列的话,由于视觉滞留会错行号
  y = y << 8;
  //-y = 0xffffff00;   //-1时对应脚输出低电平 灯亮
  //-y = 0x00000000;   //-0全灭
  send_byte(y,32);	//-总共是24列,一个数据的显示需要移动24次,来并转串


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
//-第一步开机动画
//-然后所有动作,作为一个显示终端显示全部来自命令的指挥
void led_display_init(void)
{
    u8 i;

    LD_LE((BitAction)1);	//-LD_LE 为0时数据不能进入锁存器,,这里可以让锁存器直通,因为4个地址线同时变化,而且只走地址,没有数据不会抢占总线
    LD_LAT((BitAction)0);	//-LD_LAT为0时数据不能进入锁存器
    display(0,0);		//-使能显示之前刷个初始值,这样不会上电亮一个无效行
    LD_OE((BitAction)0);	//-LD_OE 为0时锁存数据属于输出状态

		//-软件给初值
    led_display_flag = 0;		//-0显示开始动画
    cartoon_start_time = Time_2048ms_Counter;
    led_display_num = 3;		//-初始缓冲区
    led_display_y = 0;
    led_display_new = 0xa5;	//-启动动画0xa5

    led_display_page = 0;			//-初始显示范围0到4页
    led_display_page_end = 3;

    Weather_flag = 0;
    led_display_data_flag = 0;
    //-i2c1_newdata_flag = 0;
    //-定义一个足够大的数组有右上角边界,其他地方可以随意增长不越界
    led_data_x = 12;		//-8;		,,如果是12的话,那么可以中断23次进行运算操作
    led_data_y = 8;		//-如果是8的话会进行17次而不越界
    led_display_cn = 0;
    test_pt = 0;

    for(i = 0;i < 16;i++)	//-填充满了一张图片
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


		led_display_start = 0x55;		//-所有东西都准备好了,启动刷屏显示.
		led_display_ye_ok = 0;
		led_disp_L_init();
}

void led_disp_L_init(void)		//-点阵屏由灭到亮的初始准备
{
	  cticks_s_page = 0;				//-屏保开始计时
	  led_display_start = 0x55;	//-开始显示
	  led_display_new = 0x55;	//-更新下数值
	  led_display_y = 0;	//-从上到下刷屏的过程
	  led_display_deal_flag = 0xaa;	//-这些标志位都是为了保证时序的,这样显示不会出错
	  led_display_ye_ok = 0;
	  voice_keep_data_flag = 0;		//-只要人为操作了数据就需要解冻
}

//-#define test_led_display

//-主函数中调用点阵屏处理子函数,后期可以考虑带人参数或菜单
//-现在把函数放入定时中断中进行刷屏
void led_display(void)	//-所有的动态效果都是在这里叠加上去的,每次刷的时候变化下,联动起来就是动画效果.
{
   u8 i,temp_x,temp_y,temp_i;
   UINT32	*led_display_data_pt;
   UINT32 temp_data,temp_dataA,temp_dataB;
   static u8 temp_page=0;
   UINT8 *ZIKU_pt;


   //-通过点阵屏的显示结果来简单观察程序的运行
   //-每次进入中断显示增加一行
   if(led_display_flag == 0)	//-开机动画
   {
   	  //-i = i2c1_newdata_flag & 0x0f;     //-一次中断换一行显示
   	  //-i = 5;
   	  //-display(i,0xffffff);
   	  if(led_display_num == 3)	//-判断这次哪个缓冲区的数据准备好了,可以显示,
		  {
		   	 led_display_data_pt = &led_display_data3[0];
		  }
		  else//- if(led_display_num == 4)
		  {
		   	 led_display_data_pt = &led_display_data4[0];
		  }

		  if(led_display_y >= 1)	//-两个缓冲区轮流显示
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


		  for(i = 0;i < 16;i++)	//-填充满了一张图片
			 {
			 	   led_display_data_pt[i] = (ZIKU_pt[i*3+0] << 16) + (ZIKU_pt[i*3+1] << 8) + ZIKU_pt[i*3+2];
			 }

		  for(i = 0;i < 16;i++)
   	  {
   	  	 temp_data = led_display_data_pt[i];
   	  	 if(led_display_Vbuffx == 1)	//-判断这次哪个缓冲区的数据准备好了,可以显示,
			   {
			   	  led_display_Vdata2[i] = temp_data;
			   }
			   else//- if(led_display_Vbuffx == 2)
			   {
			   	  led_display_Vdata1[i] = temp_data;
			   }

		     //-display(i,temp_data);		//-这里其实越界了,但是是读取数据所以无所谓
		     //-NOP_Delay(150);	//-这个时间不能太长,否则主循环执行不动,所以需要实测
   	  }
   	  if(led_display_Vbuffx == 1)	//-判断这次哪个缓冲区的数据准备好了,可以显示,
			{
			   led_display_Vbuffx = 2;
			}
			else//- if(led_display_Vbuffx == 2)
			{
			   led_display_Vbuffx = 1;
			}

   }
   else if(led_display_flag == 1)  //-测试用,一次点亮所有的灯
   {
      for(i=0;i<16;i++)
      {
      	 if(led_display_Vbuffx == 1)	//-判断这次哪个缓冲区的数据准备好了,可以显示,
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
      if(led_display_Vbuffx == 1)	//-判断这次哪个缓冲区的数据准备好了,可以显示,
			{
			   led_display_Vbuffx = 2;
			}
			else//- if(led_display_Vbuffx == 2)
			{
			   led_display_Vbuffx = 1;
			}

   }
   else if(led_display_flag == 2)		//-显示特定行
   {
   	  //-display(led_display_y,0xffffff);		//-显示指定行
   }
   else if(led_display_flag == 3)		//-滚动条显示(从上到下)
   {
   	  temp_y = led_display_y;

   	  /*if(led_display_num == 1)	//-判断这次哪个缓冲区的数据准备好了,可以显示,
		  {
		   	 led_display_data_pt = &led_display_data1[0];
		  }
		  else if(led_display_num == 2)
		  {
		   	 led_display_data_pt = &led_display_data2[0];
		  }
		  else if(led_display_num == 5)	//-5 6两个缓冲区的内容是在通讯接口中准备好的
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
		  	 //-这个只有一个缓冲区现在需要现场准备数据
		  	 for(i = 0;i < 16;i++)	//-把数据转化为图片编码,这里是先清空缓冲区
			   {
		  	     led_display_data4[i] = 0;
		  	 }

			   for(i = 0;i < 16;i++)	//-把数据转化为图片编码
			   {
				   if(temp_y <= 16)	//-把数据转化为图片编码
				   {

			   	  	 	  //-从右往左开始显示内容
						   	  //-temp_data = 0xffffffff;
						   	  //-temp_data = temp_data >> (32 - temp_y*2);
						      //-led_display_data4[i] =  led_display_data3[i] & temp_data;		//-每个数据之间放一个空格
                  //-从左往右开始显示内容
                  temp_data = 0xffffffff;
						   	  temp_data = temp_data << (32 - temp_y*2);
						      led_display_data4[i] =  led_display_data3[i] & temp_data;		//-每个数据之间放一个空格

				   }
			   }

			   temp_y = 16;
		  }*/
		  led_display_data_pt = &led_display_data0[0];

		  if(ps_flag_led_end != 2)
		  {//-由上到下
	   	  for(i = 0;i < temp_y;i++)	//-到这里仅仅负责把指定的内容,输出显示,不再组织内容
	   	  {

	   	  	 if(temp_y < 16)
	   	  	 {
	   	  	 	   //-temp_data = 0xffffff;
	   	  	 	   //-temp_data = ~led_display_data_pt[i];
	   	  	 	   //-temp_data = led_display_data_pt[i] | 0x003C00;
	   	  	 	   //-现在已经有滑动的过程了,但是需要把尾巴裁剪掉
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
			     {//-刷一个完整的屏
			     	   //-if(i > 15)
			     	   //-	  temp_data = 15;
			     	   //-else
			     	   //-	  temp_data	= i;
			     	   temp_data = led_display_data_pt[i];
			     	   //-由于不具有记忆功能,所以增加一个判断如果不需要刷屏的行,就不执行刷屏过程,本来就是灭的
			     	   //-if(led_display_data_pt[temp_data] != 0)
			     	   //-{

			     	   	 	//-display(temp_data,led_display_data_pt[temp_data]);		//-这里其实越界了,但是是读取数据所以无所谓
			     	   	 	//-NOP_Delay(250);	//-这个时间不能太长,否则主循环执行不动,所以需要实测
			     	   //-}
			     }

	   	  	 if(led_display_Vbuffx == 1)	//-判断这次哪个缓冲区的数据准备好了,可以显示,
				   {
				   	  led_display_Vdata2[i] = temp_data;
				   }
				   else//- if(led_display_Vbuffx == 2)
				   {
				   	  led_display_Vdata1[i] = temp_data;
				   }

	   	  }

	   	  if(led_display_Vbuffx == 1)	//-判断这次哪个缓冲区的数据准备好了,可以显示,
				{
				   led_display_Vbuffx = 2;
				}
				else//- if(led_display_Vbuffx == 2)
				{
				   led_display_Vbuffx = 1;
				}

   		}
   		else
   		{//-特定为由下向上显示时采用下面的处理,其它情况下都是由上到下
   			for(i = 0;i < temp_y;i++)	//-到这里仅仅负责把指定的内容,输出显示,不再组织内容
	   	  {
	   	  	 if(temp_y < 16)
	   	  	 {
	   	  	 	   //-temp_data = 0xffffff;
	   	  	 	   //-temp_data = ~led_display_data_pt[i];
	   	  	 	   //-temp_data = led_display_data_pt[i] | 0x003C00;
	   	  	 	   //-现在已经有滑动的过程了,但是需要把尾巴裁剪掉
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
			     {//-刷一个完整的屏
			     	   //-if(i > 15)
			     	   //-	  temp_data = 15;
			     	   //-else
			     	   	  temp_i	= i;
			     	   temp_data = led_display_data_pt[i];
			     	   //-由于不具有记忆功能,所以增加一个判断如果不需要刷屏的行,就不执行刷屏过程,本来就是灭的
			     	   //-if(led_display_data_pt[temp_data] != 0)
			     	   //-{
			     	   	 	//-display(temp_data,led_display_data_pt[temp_data]);		//-这里其实越界了,但是是读取数据所以无所谓
			     	   	 	//-NOP_Delay(150);	//-这个时间不能太长,否则主循环执行不动,所以需要实测
			     	   //-}
			     }
			     if(led_display_Vbuffx == 1)	//-判断这次哪个缓冲区的数据准备好了,可以显示,
				   {
				   	  led_display_Vdata2[temp_i] = temp_data;
				   }
				   else//- if(led_display_Vbuffx == 2)
				   {
				   	  led_display_Vdata1[temp_i] = temp_data;
				   }

	   	  }
	   	  if(led_display_Vbuffx == 1)	//-判断这次哪个缓冲区的数据准备好了,可以显示,
				{
				   led_display_Vbuffx = 2;
				}
				else//- if(led_display_Vbuffx == 2)
				{
				   led_display_Vbuffx = 1;
				}

	   	}


   	  //-if(led_display_txpic_flag == 0x55)
   	  //-	led_display_txpic_flag = 0xaa;		//-通讯口下发的图片显示过了,可以通过通讯口更新了
   }
   else if(led_display_flag == 4)		//-螺旋转圈
   {
   	   /*
   	   if(led_display_cn_f == 0x55)
   	   {
		   	   if(led_display_cn & 0x01)
		   	   {//-奇数处理
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
		   	 	 {//-偶数处理

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
   	   	  //-存储中间变量
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
		     	   //-由于不具有记忆功能,所以增加一个判断如果不需要刷屏的行,就不执行刷屏过程,本来就是灭的
		     	   //-if(led_display_data_pt[temp_data] != 0)
		     	   {
		     	   	 	//-display(temp_i,temp_data);		//-这里其实越界了,但是是读取数据所以无所谓
		     	   	 	//-NOP_Delay(150);	//-这个时间不能太长,否则主循环执行不动,所以需要实测
		     	   }


   	  	 //-}
   	  }

   }
   else if(led_display_flag == 5)		//-正弦波移动
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
		     	   //-由于不具有记忆功能,所以增加一个判断如果不需要刷屏的行,就不执行刷屏过程,本来就是灭的
		     	   //-if(led_display_data_pt[temp_data] != 0)
		     	   {
		     	   	 	//-display(temp_i,temp_data);		//-这里其实越界了,但是是读取数据所以无所谓
		     	   	 	//-NOP_Delay(150);	//-这个时间不能太长,否则主循环执行不动,所以需要实测
		     	   }


   	  	 //-}
   	  }
   }
   else if(led_display_flag == 6)		//-从中间向四周扩散
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
		     	   //-由于不具有记忆功能,所以增加一个判断如果不需要刷屏的行,就不执行刷屏过程,本来就是灭的
		     	   //-if(led_display_data_pt[temp_data] != 0)
		     	   {
		     	   	 	//-display(temp_i,temp_data);		//-这里其实越界了,但是是读取数据所以无所谓
		     	   	 	//-NOP_Delay(150);	//-这个时间不能太长,否则主循环执行不动,所以需要实测
		     	   }
		   	     //-display(temp_i,0);

   	  	 //-}
   	  }
   }
   else if(led_display_flag == 7)		//-从下向上渐进显示,比如WIFI连接,一直返回刷
   {
   	  temp_y = led_display_y;		//-最多显示几行

      /*if(led_display_num == 5)
		  {
		   	 led_display_data_pt = &led_display_data5[0];
		  }
		  else if(led_display_num == 6)
		  {
		   	 led_display_data_pt = &led_display_data6[0];
		  }*/

		  led_display_data_pt = &led_display_data0[0];

   	  for(i = 0;i < temp_y;i++)	//-到这里仅仅负责把指定的内容,输出显示,不再组织内容
   	  {
   	  	 if(temp_y < 16)
   	  	 {
						 temp_i = 15 - i;
   	  	 	   temp_data = led_display_data_pt[temp_i];


				   	 //-display(temp_i,temp_data);
				   	 //-NOP_Delay(150);
		     }
		     else
		     {//-刷一个完整的屏
		     	   //-if(i > 15)
		     	   //-	  temp_i = 15;
		     	   //-else
		     	   	  temp_i	= i;
		     	   temp_data = led_display_data_pt[i];
		     	   //-由于不具有记忆功能,所以增加一个判断如果不需要刷屏的行,就不执行刷屏过程,本来就是灭的
		     	   if(led_display_data_pt[temp_i] != 0)
		     	   {
		     	   	 	//-display(temp_i,led_display_data_pt[temp_i]);		//-这里其实越界了,但是是读取数据所以无所谓
		     	   	 	//-NOP_Delay(150);	//-这个时间不能太长,否则主循环执行不动,所以需要实测
		     	   }
		     }

		     if(led_display_Vbuffx == 1)	//-判断这次哪个缓冲区的数据准备好了,可以显示,
			   {
			  	  led_display_Vdata2[temp_i] = temp_data;
			   }
			   else//- if(led_display_Vbuffx == 2)
			   {
			  	  led_display_Vdata1[temp_i] = temp_data;
			   }
   	  }
   	  if(led_display_Vbuffx == 1)	//-判断这次哪个缓冲区的数据准备好了,可以显示,
		  {
		     led_display_Vbuffx = 2;
		  }
		  else//- if(led_display_Vbuffx == 2)
		  {
		     led_display_Vbuffx = 1;
		  }

   	  //-if(led_display_txpic_flag == 0x55)
   	  //-	led_display_txpic_flag = 0xaa;		//-通讯口下发的图片显示过了,可以通过通讯口更新了
   }
   else if(led_display_flag == 8)		//-测试动画过程中,仿墨迹左右移动
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

	   	     if(led_display_Vbuffx == 1)	//-判断这次哪个缓冲区的数据准备好了,可以显示,
					 {
					 	  led_display_Vdata2[i] = temp_data;
					 }
					 else//- if(led_display_Vbuffx == 2)
					 {
					 	  led_display_Vdata1[i] = temp_data;
					 }
	   	     //-display(i,temp_data);		//-这里其实越界了,但是是读取数据所以无所谓
			 	   //-NOP_Delay(150);	//-这个时间不能太长,否则主循环执行不动,所以需要实测
	   	  }

	   	  if(led_display_Vbuffx == 1)	//-判断这次哪个缓冲区的数据准备好了,可以显示,
				{
				   led_display_Vbuffx = 2;
				}
				else//- if(led_display_Vbuffx == 2)
				{
				   led_display_Vbuffx = 1;
				}
		  }
   }
   else		//-显示爱心
   {
   	  /*
      //for(i=0;i<16;i++)
      //{//显示爱心
        display(15,0x30060);
        //-display(15,0);		//-这样的目的是为了让上面的数据可以显示到对应的行上,其实完全可以先把数据准备好再显示,这样就直接对上了
        //-NOP_Delay(70*300);	//-这里的延时可以是灯的亮度增加,但是需要权衡利弊
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

//-对程序进行处理提取可能需要的数据
void led_display_deal(void)
{
     //-实现了换页显示
     if(ps_flag_led == 1)
	   {//-1 由上到下
	 	    led_display_page++;
     	  if(led_display_page > led_display_page_end)
     	 	   led_display_page = 0;

	 	    ps_flag_led = 0;
	 	    ps_flag_led_end = 1;
	 	    ps_flag_led_disp = 1;
	 	    led_display_new = 0x55;
	 	    led_display_txpic_flag = 0;		//-不再显示固化图片
	 	    voice_keep_data_flag = 0;		//-刷屏之后立即解冻数据

	 	    if((led_display_page == 0) || (led_display_page == 3))
	 	    {//-当切换到PM2.5 CO2上时就需要判断是否有新数据,没有就需要等待
	 	    	 if(led_display_data_flag != 2)
	 	    	 {
	 	    	 	 Sensor_data_wait_time = cticks_ms;
	 	    	 	 led_display_data_flag = 1;
	 	    	 }

	 	    	 if(led_display_data_flag == 2)
	 	    	 {
	 	    	 	  //-启动风扇,延后了,但是为了播报同步先这么做
			  	 		UART1_transmit_flag = YES;		//-可以组织内容发送
		          UART1_transmit_control = 1;	//-决定了主动发送的内容
	 	    	 }
	 	    	 else
	 	    	 {
	 	    	    //-启动风扇,延后了,但是为了播报同步先这么做
			  	 		UART1_transmit_flag = YES;		//-可以组织内容发送
		          UART1_transmit_control = 8;	//-决定了主动发送的内容
        	 }
	 	    }
	 	    else
	 	    {
	 	    	 led_display_flag = 3;	//-切换之后只要不是需要等待数据更新的菜单,都是正常显示
	 	    	 //-if(led_display_data_flag == 2)
	 	    	 //-{
	 	    	 //-	 Sensor_data_wait_time = cticks_ms;		//-切换到其它页了有效性开始计时
	 	    	 //-	 led_display_data_flag = 3;		//-有效数据失效开始计时
	 	    	 //-}
	 	    	 //-2015/8/17 20:51:07 by zj
	 	       //-把目前点阵屏的状态值给路由让他们语音播报一下
	 	       //-if(voice_flag == 0x55) //-对于PM2.5和CO2需要特别然是播报,因为有一个实时检测过程
	 	       {//-现在每次都发送,是否上送7620由里面的参数让主板STM32控制
		 	       UART1_transmit_flag = YES;		//-可以组织内容发送
	           UART1_transmit_control = 5;	//-决定了主动发送的内容
	           voice_keep_data_flag = 0;		//-使用新数据,赋值0是为了刷新值,然后冻结最新的显示数据
         	 }

	 	    }


	   }
	   else if (ps_flag_led == 2)
	   {//-2 由下到上
     	  if(led_display_page > 0)
     	 	   led_display_page--;
     	 	else
     	 		 led_display_page = led_display_page_end;

	 	     ps_flag_led = 0;
	 	     ps_flag_led_end = 2;
	 	     ps_flag_led_disp = 2;
	 	     led_display_new = 0x55;		//-每翻一页就有必要重新准备数据
	 	     led_display_txpic_flag = 0;
	 	     voice_keep_data_flag = 0;		//-刷屏之后立即解冻数据

	 	    if((led_display_page == 0) || (led_display_page == 3))
	 	    {//-当切换到PM2.5 CO2上时就需要判断是否有新数据,没有就需要等待
	 	    	 if(led_display_data_flag != 2)
	 	    	 {
	 	    	 	 Sensor_data_wait_time = cticks_ms;
	 	    	 	 led_display_data_flag = 1;

	 	    	 }

	 	    	 if(led_display_data_flag == 2)
	 	    	 {
	 	    	 	  //-启动风扇,延后了,但是为了播报同步先这么做
			  	 		UART1_transmit_flag = YES;		//-可以组织内容发送
		          UART1_transmit_control = 1;	//-决定了主动发送的内容
	 	    	 }
	 	    	 else
	 	    	 {
	 	    	    //-启动风扇,延后了,但是为了播报同步先这么做
			  	 		UART1_transmit_flag = YES;		//-可以组织内容发送
		          UART1_transmit_control = 8;	//-决定了主动发送的内容
        	 }
	 	    }
	 	    else
	 	    {
	 	    	 led_display_flag = 3;	//-切换之后只要不是需要等待数据更新的菜单,都是正常显示
	 	    	 //-if(led_display_data_flag == 2)	//-失效开始计时从熄灭屏幕开始
	 	    	 //-{
	 	    	 //-	 Sensor_data_wait_time = cticks_ms;		//-切换到其它页了有效性开始计时
	 	    	 //-	 led_display_data_flag = 3;		//-有效数据失效开始计时
	 	    	 //-}
	 	    	 //-2015/8/17 20:51:07 by zj
	 	       //-把目前点阵屏的状态值给路由让他们语音播报一下
	 	       //-if(voice_flag == 0x55)
	 	       {
		 	       UART1_transmit_flag = YES;		//-可以组织内容发送
	           UART1_transmit_control = 5;	//-决定了主动发送的内容
	           voice_keep_data_flag = 0;		//-使用新数据,赋值0是为了刷新值,然后冻结最新的显示数据
         	 }

	 	    }

	   }

    if(ps_flag_led_dis == 0x55)
    {
    	 ps_flag_led_dis = 0;

       voice_keep_data_flag = 0;
       if((led_display_page == 0) || (led_display_page == 3)) //-为了保证初次亮屏的时候也可以滚动显示检测过程
	 	    {//-当切换到PM2.5 CO2上时就需要判断是否有新数据,没有就需要等待
	 	    	 if(led_display_data_flag != 2)
	 	    	 {
	 	    	 	 Sensor_data_wait_time = cticks_ms;
	 	    	 	 led_display_data_flag = 1;
	 	    	 }

	 	    	 /*if(led_display_data_flag == 2)
	 	    	 {
	 	    	 	  //-启动风扇,延后了,但是为了播报同步先这么做
			  	 		UART1_transmit_flag = YES;		//-可以组织内容发送
		          UART1_transmit_control = 1;	//-决定了主动发送的内容
	 	    	 }
	 	    	 else
	 	    	 {
	 	    	    //-启动风扇,延后了,但是为了播报同步先这么做
			  	 		UART1_transmit_flag = YES;		//-可以组织内容发送
		          UART1_transmit_control = 8;	//-决定了主动发送的内容
        	 }*/
	 	    }
       else
       {
          UART1_transmit_flag = YES;		//-可以组织内容发送
          UART1_transmit_control = 9;
       }

    }

    /*
    if(CSKEY_DATA == 1)
    {//-按下BIN1显示键值
    	  led_display_page = 8;
    }
    else if(CSKEY_DATA == 5)
    {//-按下BIN5恢复循环页显示
    	  led_display_page = 0;
    }
    */
    if(voice_keep_data_flag == 0)
    {
		    //-led_display_page = 3;
		    //-PM2.5 CO2 温度 湿度 噪声 时间	光强 按键值
		    if(led_display_page == 0)
		    {//-显示PM2.5
		    	 led_display_data = pm_data;		//-可以考虑给个有效值范围,这样可以出错了,也给个假数据
		    	 //-led_display_data = 12;
		    }
		    else if(led_display_page == 1)
		    {//-温度被替换成噪音
		    	 //-led_display_data = temperature_data;
	         led_display_data = Noise_Value;
		    }
		    else if(led_display_page == 2)
		    {//-湿度被替换成VOC
		    	 //-led_display_data = humidity_data;
		    	 led_display_data = VOC_data;
		    }
		    else if(led_display_page == 3)
		    {//-CO2
		    	 led_display_data = co2_data;
		    }
		    else if(led_display_page == 4)
		    {//-噪声
		    	 //-led_display_data = Noise_Value;		//-噪声临时输出
		    }
		    else if(led_display_page == 20)
		    {//-光强
		    	 led_display_data = als_data / 100;
		    }
		    else if(led_display_page == 21)
		    {//-按键值
		    	 led_display_data = CSKEY_DATA;
		    }
    }
    //-通过点阵屏把错误信息显示出来,格式 Exx
    //-if(sys_err_flag != 0)
    //-  led_display_data = sys_err_flag;
    if((UART1_transmit_control == 5) || (UART1_transmit_control == 1) || (UART1_transmit_control == 9))	//-主动上送的内容为了播报一致冻结一段时间
    {
    	voice_keep_data_flag = 0x55;
    	voice_keep_data_time = cticks_ms;
    }

    if(Judge_Time_In_MainLoop(voice_keep_data_time,8000)==YES)
    {
    	 voice_keep_data_flag = 0;		//-过时结束冻结显示数据
    }


}

//-准备需要显示的数据,准备好之后,置标志,让程序刷新,这里准备的是一个静态"图片"
void led_display_var(void)
{
	 UINT8  wei4,wei3,wei2,wei1,sign,xwei;
	 u8 i;
	 UINT32	*led_display_data_pt;		//-指向待写数组的地址
	 UINT8 *ZIKU_pt;
	 UINT32	led_display_data_var[7];
	 UINT8  pic_x;		//-0到23		一个字体是5*7的大小
	 UINT8  pic_y;    //-0到15

	 //-led_display_data =99;
	 if((led_display_new == 0x55) && (led_display_txpic_flag == 0))	//-不显示通讯的图片,然后才刷自己系统的图片,不是开机动画时也才刷自己的图片
	 {
	 	   //-第一步把数据分析为三位数
	 	   //-数据的最高位为符号为,其它位为绝对值
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
			 wei4 = wei4 % 10;		//-这样做的目的是防止越界,可以丢位,但是显示出来的不错

			 if(wei4 == 0)
			 {
				 if(wei3 == 0)
				 {
				 	  if(wei2 != 0)
				    {
				 	     wei3 = 10;
				 	     xwei = 2;		//-指示显示几位数据
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
			 	 wei3 = 11;		//-针对负数只有温度,所以这个就放在位3上

			 pic_x = 0;
			 pic_y = 0;
			 //-特别修改特定位的值,这里是位3
			 //-if(sys_err_flag != 0)
			 //-	  wei3 = 10;

      //-led_display_page = 6;
			//-PM2.5 CO2 温度 湿度  噪声 天气 时间 光强 按键值
			if(led_display_page == 0)
	    {//-显示PM2.5
	    	 ZIKU_pt = &ZIKU_PM25[0][0];
	    	 pic_x = 0;
			   pic_y = 1;
	    }
	    else if(led_display_page == 1)
	    {//-温度被替换成噪音
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
	    {//-湿度被替换成VOC
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
	    //-{//-噪声
	    //-	 ZIKU_pt = &ZIKU_yawp[0][0];
	    //-	 pic_x = 11;
			//-   pic_y = 5;
	    //-}
	    else if(led_display_page == 4)
	    {//-天气
         if(Weather_flag == 1)
	    	 	  ZIKU_pt = &ZIKU_sunshine[0][0];		//-晴
	    	 else if(Weather_flag == 2)
	    	 	  ZIKU_pt = &ZIKU_shade[0][0];				//-阴
	    	 else if(Weather_flag == 3)
	    	 	  ZIKU_pt = &ZIKU_cloudy[0][0];				//-多云
	    	 else if(Weather_flag == 4)
	    	 	  ZIKU_pt = &ZIKU_rain[0][0];	  		//-雨
	    	 else if(Weather_flag == 5)
	    	 	  ZIKU_pt = &ZIKU_snow[0][0];	  		//-雪
	    	 else
	    	 	  ZIKU_pt = &ZIKU_cry[0][0];		//-0给个哭脸表示天气失败,防错的,正常不该跑到这里
	    	 pic_x = 0;
			   pic_y = 0;
	    }
	    else if(led_display_page == 6)
	    {//-时间
	    	 ZIKU_pt = &ZIKU_TIME[0][0];		//-这里定时切换可以有秒闪耀的过程
	    	 pic_x = 0;
			   pic_y = 4;
			   //-wei1 = m_min % 10;		//-特殊情况特殊处理
			   //-wei2 = m_min / 10;
			   //-wei3 = m_hour % 10;
			   //-wei4 = m_hour / 10;
	    }
	    else if(led_display_page == 20)
	    {//-光强
	    	 //-ZIKU_pt = als_data / 100;
	    	 pic_x = 0;
			   pic_y = 0;
	    }
	    else if(led_display_page == 21)
	    {//-按键值
	    	 //-ZIKU_pt = CSKEY_DATA;
	    	 pic_x = 9;
			   pic_y = 4;
	    }

	    if(pic_x == 0)
	    	//-x轴处理,以便时时居中
	    	pic_x = (24 - xwei*6)/2 + pic_x;

			 /*if(led_display_num == 1)		//-两个缓冲区互为备用,不正在显示的可以组织待显示数据
			 {//-led_display_data2[16]
			 	  led_display_data_pt = &led_display_data2[0];

			 }
			 else
			 {//-led_display_data1[16]
			 	  led_display_data_pt = &led_display_data1[0];

			 }*/
			 led_display_data_pt = &led_display_data0[0];	//-把目前需要显示的内容放在这个缓冲区,即准备最原始的一张图片,后期可以对这张图片处理以便搞出效果


			 for(i = 0;i < 16;i++)	//-填充满了一张图片
			 {
			 	   led_display_data_pt[i] = (ZIKU_pt[i*3+0] << 16) + (ZIKU_pt[i*3+1] << 8) + ZIKU_pt[i*3+2];
			 }

			 if(((led_display_data_flag != 1) || ((led_display_page != 0) && (led_display_page != 3))) && (led_display_page != 4))
			 {//-不等于0 才需要组织数据
				 //-需要组织一个数据的图片
				 for(i = 0;i < 7;i++)	//-把数据转化为图片编码
				 {
				     led_display_data_var[i] =  (ZIKU_XIAO[wei4][i] << 18) + (ZIKU_XIAO[wei3][i] << 12) + (ZIKU_XIAO[wei2][i] << 6) + ZIKU_XIAO[wei1][i];		//-每个数据之间放一个空格
				 }
			 }
			 else
			 {//-没有数据直接给0
			 	  for(i = 0;i < 7;i++)	//-把数据转化为图片编码
				  {
				     led_display_data_var[i] =  0;		//-每个数据之间放一个空格
				  }
			 }

			 for(i = 0;i < 7;i++)	//-把两张图融合到一起
			 {
			 	   led_display_data_pt[i + pic_y] |= led_display_data_var[i] << pic_x;
			 }

			 if((led_display_data_flag == 1) && ((led_display_page == 0) || (led_display_page == 3)))
			 {
			 	  led_display_y = 16;
	 	 			led_display_flag = 8;	//-显示等待数据更新状态
	 	 	 }

			 /*if(led_display_num == 1)		//-两个缓冲区互为备用,不正在显示的可以组织待显示数据
			 {//-led_display_data2[16]
			 	  led_display_data_pt = &led_display_data2[0];
			 	  led_display_num = 2;		//-只有内容更改好了之后才能切换显示号
			 }
			 else
			 {//-led_display_data1[16]
			 	  led_display_data_pt = &led_display_data1[0];
			 	  led_display_num = 1;
			 }*/

			 //-led_display_new = 0xaa;	//-数据准备好了,可以切换显示了
			 //-led_display_new = 0xa5;	//-暂时启动动画
			 //-led_display_y = 0;
			 //-for(i = 0;i < 16;i++)	//-把数据转化为图片编码
			 //-{
			 //-    led_display_data3[i] =  led_display_data_pt[i];		//-每个数据之间放一个空格
			 //-    //-led_display_data3[i] =  0xffffff;		//-每个数据之间放一个空格
			 //-    //-led_display_data_pt[i] =  0xffffff;		//-每个数据之间放一个空格
			 //-}
			 //-led_display_num = 3;
			 led_display_new = 0xaa;	//-数据准备好了,可以切换显示了
	 }

	 if(led_display_txpic_flag == 0x55)
	 {//-进行刷图片准备
	 	  /*if(led_display_num == 5)		//-两个缓冲区互为备用,不正在显示的可以组织待显示数据
			{//-led_display_data2[16]
				 led_display_data_pt = &led_display_data6[0];

			}
			else
			{//-led_display_data1[16]
				 led_display_data_pt = &led_display_data5[0];

			}*/

			led_display_data_pt = &led_display_data0[0];		//-准备最原始的一张图片,后期可以对这张图片处理以便搞出效果

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
	    else if(led_display_txpic_num == 20)		//-配合语音学习的显示
	    {//-
	    	 ZIKU_pt = &ZIKU_00[0][0];
	    }
	    else if(led_display_txpic_num == 21)		//-配合语音学习的显示
	    {//-
	    	 ZIKU_pt = &ZIKU_01[0][0];
	    }
	    else if(led_display_txpic_num == 22)		//-配合语音学习的显示
	    {//-
	    	 ZIKU_pt = &ZIKU_02[0][0];
	    }
	    else if(led_display_txpic_num == 23)		//-配合语音学习的显示
	    {//-
	    	 ZIKU_pt = &ZIKU_03[0][0];
	    }
	    else if(led_display_txpic_num == 24)		//-配合语音学习的显示
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

			for(i = 0;i < 16;i++)	//-填充满了一张图片
			{
				 led_display_data_pt[i] = (ZIKU_pt[i*3+0] << 16) + (ZIKU_pt[i*3+1] << 8) + ZIKU_pt[i*3+2];
			}

			/*if(led_display_num == 5)		//-两个缓冲区互为备用,不正在显示的可以组织待显示数据
			{//-led_display_data2[16]
				 led_display_num = 6;
			}
			else
			{//-led_display_data1[16]
				 led_display_num = 5;
			}*/

			//-判断是否是特定动画
			if(led_display_txpic_num == 0)
			{
				 led_display_new = 0xa5;	//-启动动画
	  	   led_display_flag = 7;
			}
			else
				 led_display_flag = 3;		//-正常显示

			led_display_txpic_flag = 0xaa;

	 }

	 if((led_display_flag == 7) && (Judge_Time_In_MainLoop(cartoon_end_wait_time,20000)==YES))		//-最大65536
	 {
	 	  led_display_new = 0;
	  	led_display_flag = 3;
	  	//-led_display_new = 0xa5;	//-启动动画显示
	  	led_display_txpic_num = 10;	//-目前指定为自定义后期需要是哭脸.
      led_display_txpic_flag = 0x55;	//-超时显示指定的图片以便输出结果

	 }

	 //-if((led_display_data_flag == 1) && ((led_display_page == 0) || (led_display_page == 3)))
	 //-	 led_display_flag = 8;	//-显示等待数据更新状态
}

//-还有一种显示效果需要一副图片一副图片的刷,而不是一个数据一个数据的修改
//-那么我就需要修改当前全部缓冲区内容
//-动画过程是刷一组固定的数据,在这个结束前不能更新数据,启动动画过程可以是第一次亮屏的时候
//?突然亮屏的时候需要保证初始状态,这样才可以显示正确,否则就有一段时间的不确定状态
void led_display_ye_it(void)		//-每次中断刷新最终的行内容
{
	  if((led_display_start == 0x55) || (led_display_flag == 0))		//- && (led_display_deal_flag == 0x55))		//-只要亮屏了就需要时时准备页内容,因为是两个缓冲区,没有准备好不会打架的
    {//-这里的计时和准备的内容都是需要刷屏的时候准备的所有,如果熄灭了就不需要了
	    if(led_display_new != 0xa5)
	    {
	    	  if(led_display_y > 15)
			    	 led_display_y = 16;		//-为了刷最后一行,多一个隐形行16

			    if(((led_display_start == 0x55) || (led_display_page == 0)) && (led_display_y < 16))
			       led_display_y++;


	    }
	    else
	    {//-处于动画中,现在采用渐显示方式
	    	  //-if((led_display_start == 0x55) && (led_display_y < 16))
	    	  if(led_display_y < 16)
			       led_display_y++;

	    	  if(led_display_y > 15)
			    {
			    	led_display_y = 0;		//-为了刷最后一行,多一个隐形行16
			    	//-led_display_new = 0x55;	//-临时反复启动动画过程
			    }

	    }
	    //-if(led_display_new == 0xaa)
	    //-	led_display_cn++;			//-刷新一次显示增加1

	    //-从右往左移动方块
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

	    led_display();		//-最终定时切换需要显示的内容,属于二级管理
	    led_display_ye_ok = 0x55;		//-页数据准备好了
    }
}

void led_display_hang_it(void)		//-每次中断刷新最终的行内容
{
	  UINT32 temp_data;
	  UINT8  temp_i;

	  if(((led_display_start == 0x55) || (led_display_flag == 0)) && (led_display_ye_ok == 0x55))		//-当处于开机状态的时候是一直显示的,直到开机结束才可能灭屏
    {
       if(led_display_pageing == 0)
       {
       	 //-准备一页数据,然后每页数据刷新5次后准备下一页数据,如果没有更新的话,还使用当前数据
         if(led_display_Vbuffx == 1)	//-判断这次哪个缓冲区的数据准备好了,可以显示,
         {
            led_display_Vdata_pt = &led_display_Vdata1[0];
         }
         else if(led_display_Vbuffx == 2)
         {
            led_display_Vdata_pt = &led_display_Vdata2[0];
         }
         else//- if(led_display_Vbuffx == 0)	//-当突然亮屏的时候需要准备好数据再显示,否则在这之前的数据都是无效的
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
		    	led_display_pageing = 0;		//-中断有一个快慢之分,注意改变速度
		    	//-led_display_ye_ok = 0;	//-刷完完整的一页才可以换内容
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
       led_display_Hx = 0;		//-需要确保初始状态
       led_display_pageing = 0;
       led_display_Vbuffx = 0;
       //-led_display_ye_ok = 0;
    }
}

