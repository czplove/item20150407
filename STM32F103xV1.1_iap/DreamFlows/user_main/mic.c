/*
现在一直在进行ADC转换,每50uS处理一次
主频56MHz时,每次转化时间是1uS,那么在整个系统中我处理的最小周期是1mS,那么我可以采样1000
次数据(为了数据整那么采样1024次),然后进行处理.
也就是说只要在1mS之内把原来的数据
一但采集中断之后就立即处理数据再覆盖之前处理完就行
量程是3.3V 转化为数字是4096.那么采样1代表0.806mV
*/
#include "user_conf.h"


#define   Windows_counter		100
#define   Windows_long  		10*2


int a=0;
int total=0;
long TOTAL=0;
WORD mic_Counter=0;
int Total_Average[maxcounter]={0};
WORD Windows_Counter=0;
int Windows_Average[Windows_long]={0};

/*
主循环周期查询,看看是否有有效数据产生了,这里的确定就是查询时间长短无法确定,很有可能丢失数据
这里的ADC采样本身是没有问题的,需要注意的是后期观察结果时对采样值的处理
*/   
/*int Noise_Process(void)
{
    WORD  i;
	  int temp=0;
	  static char Counter=0;
	  WORD    temp_data;

    if(ADC_Conv_flag)
    {
       ADC_Conv_flag = 0;
		
       memcpy(ADC_ConvertedValue_TEMP,ADC_ConvertedValue,sizeof(ADC_ConvertedValue));
        	
       for(i=0;i<maxbuffer;i++)		//-测量到50个数据
       {
			     if(ADC_ConvertedValue_TEMP[i]>base)
			     {
               temp =   ADC_ConvertedValue_TEMP[i] - base;
				   }
				   else
				   {
               temp =   base - ADC_ConvertedValue_TEMP[i];
				   }
			     total += temp;					   
       }
       
       total = total/maxbuffer;
			 Total_Average[Counter++]=total;	//-这里需要增加判断条件,否则越界了都不知道
	
			 if(Counter>=maxcounter)//数值里填满了100个数后开始计算并发送
			 {
          Counter = 0; 
			    for(i=0;i<maxcounter;i++)
				  {
              TOTAL  +=Total_Average[i];
			    }
				  TOTAL =TOTAL/maxcounter;
				  Noise_Value = TOTAL/5;
				  Start_thansfer=1;
				  memset(Total_Average,0x00,sizeof(Total_Average));			
				  
				  //-增加通道值,为通讯用
          if(Noise_Value >= port_send_sense_data[5])  
          {
          	 temp_data = Noise_Value - port_send_sense_data[5];
          }
          else
          	 temp_data = port_send_sense_data[5] - Noise_Value;
          	 
          port_send_sense_data[5] = Noise_Value;	 
          if(temp_data > 30)	 
          {          	 
          	 port_send_sense_data[0] = 0x55;
          }      	  	    
			 }
		//return Noise_Value;		 
		}
}
*/

//-噪声值换算为DB
//-y=kx+b 采用简单相似三角形的办法
void Noise_Process_value(WORD Data)
{
	 WORD    temp_data,temp_value,temp_b;
   float   temp_k;


	 if(Data < 20)	//-<20~30
	 {
      temp_value = Data;
      temp_k = 20/(float)10;		//-4最终输出噪声/10
      temp_b = 20;
	 }
   else if(Data < 61)	//-<20~30
	 {
      temp_value = Data - 20;
      temp_k = (61 - 20)/(float)10;		//-4最终输出噪声/10
      temp_b = 35;
	 }
   else if(Data < 71)	//-<30~40
	 {
	 	  temp_value = Data - 61;
      temp_k = (71 - 61)/(float)10;		//-4最终输出噪声/10
      temp_b = 30;
	 }
   else if(Data < 85)	//-<40~50
	 {
	 	  temp_value = Data - 71;
      temp_k = (85 - 71)/(float)10;		//-4最终输出噪声/10
      temp_b = 40;
	 }
   else if(Data < 91)	//-<50~60
	 {
	 	  temp_value = Data - 85;
      temp_k = (91 - 85)/(float)10;		//-4最终输出噪声/10
      temp_b = 50;
	 }
	 else if(Data < 103)	//-<60~70
	 {
	 	  temp_value = Data - 91;
      temp_k = (103 - 91)/(float)10;		//-4最终输出噪声/10
      temp_b = 60;
	 }
	 else if(Data < 229)	//-<70~80
	 {
	 	  temp_value = Data - 103;
      temp_k = (229 - 103)/(float)10;		//-4最终输出噪声/10
      temp_b = 70;
	 }
	 else if(Data < 375)	//-<80~90
	 {
	 	  temp_value = Data - 229;
      temp_k = (375 - 229)/(float)10;		//-4最终输出噪声/10
      temp_b = 80;
	 }
	 else if(Data < 596)	//-<90~100
	 {
	 	  temp_value = Data - 375;
      temp_k = (596 - 375)/(float)10;		//-4最终输出噪声/10
      temp_b = 90;
	 }
	 else if(Data < 980)	//-<100~110
	 {
	 	  temp_value = Data - 596;
      temp_k = (980 - 596)/(float)10;		//-4最终输出噪声/10
      temp_b = 100;
	 }
	 else//- if(temp_data < 2400)	//-<110~120
	 {
	 	  temp_value = Data - 980;
      temp_k = (2000 - 980)/(float)10;		//-4最终输出噪声/10
      temp_b = 110;
	 }
	 
	 temp_data = (WORD)(temp_value / temp_k);
	 
	 //-if(temp_b >= 100)
	 //-{	
	 //-	 temp_b = 100 + (cticks_ms % 10);   //-假值				  	 
	 //-}
	 temp_data = temp_b + temp_data;
	 //-if(temp_data >= 99)
	 //-   temp_data = 99;
	    	
	 Noise_Value = temp_data;
	 //-watch_data[watch_data_pt] = Noise_Value;
	 //-watch_data_pt = (watch_data_pt + 1) & 0x1ff;
	 //-Start_thansfer=1;
	 //-memset(Total_Average,0x00,sizeof(Total_Average));			
	 
	 //-增加通道值,为通讯用
   //-if(Noise_Value >= port_send_sense_data[5])  
   //-{
   //-	 temp_data = Noise_Value - port_send_sense_data[5];
   //-}
   //-else
   //-	 temp_data = port_send_sense_data[5] - Noise_Value;
   	 
   port_send_sense_data[5] = Noise_Value;	 
   //-if(temp_data > 30)	 
   //-{
   //-	 port_send_sense_data[0] = 0x55;
   //-}
   
   //-声音讲究的就是实时反映,所以快速变换
   /*if(Noise_Value < 35)
   {
   	 Noise_Value_flag = 0;
   }
   else if(Noise_Value < 65)
   {
   	 Noise_Value_flag = 0;
   }
   else
   {
   	 Noise_Value_flag = 2;
   }*/
}

/*
噪声定义:
噪声级为30～40分贝是比较安静的正常环境；超过50分贝就会影响睡眠和休息。由于休息不足，
疲劳不能消除，正常生理功能会受到一定的影响；70分贝以上干扰谈话，造成心烦意乱，精神不
集中，影响工作效率，甚至发生事故；长期工作或生活在90分贝以上的噪声环境，会严重影响听
力和导致其他疾病的发生。
*/
void Noise_Process_it(void)		//-基本是1mS分配处理一次
{
    WORD  i;	  
	  WORD  temp_data,temp_big,temp_small;

    //-if(ADC_Conv_flag)
    {
       ADC_Conv_flag = 0;
		
       //-memcpy(ADC_ConvertedValue_TEMP,ADC_ConvertedValue,sizeof(ADC_ConvertedValue));
       total = 0;

       temp_big = ADC_ConvertedValue[0 + ADC_Channel_num * 0];
       temp_small = ADC_ConvertedValue[0 + ADC_Channel_num * 0];

       for(i=0;i<(maxbuffer/ADC_Channel_num);i++)		//-测量到512个数据
       {
           if(ADC_ConvertedValue[ADC_Channel_num * i] > temp_big)
           {
           		temp_big = ADC_ConvertedValue[ADC_Channel_num * i];
           }
           else if(ADC_ConvertedValue[ADC_Channel_num * i] < temp_small)
           {
           		temp_small = ADC_ConvertedValue[ADC_Channel_num * i];
           }
       }       
       temp_data = temp_big - temp_small;	//-在几乎1mS的时间窗口中找到最大峰峰值
       Total_Average[mic_Counter++]=temp_data;	//-这里需要增加判断条件,否则越界了都不知道,,1.024mS区间取一个峰峰值
			 mic_Counter = mic_Counter & 0x3ff;
			 
       //-大约每100mS作为一个有效分段计算出平均值
       if(mic_Counter>=Windows_counter)
       {
       	  mic_Counter = 0;
       		for(i=0;i<Windows_counter;i++)//-累加和
				  {
           	 TOTAL += Total_Average[i];           
			    }
			    TOTAL	= TOTAL/Windows_counter;	//-得窗口均值
			    
			    Windows_Average[Windows_Counter++] = TOTAL;
			    if(Windows_Counter >= Windows_long)
          {
			    	Windows_Counter = 0;

             //-一直保留1024*1024uS的数据(共1024个512个中的最大峰峰值)
             temp_big = Windows_Average[0];
             for(i=0;i<Windows_long;i++)		//-在大约1S的窗口中寻找最大值,中间跳跃的去掉,只看这1S的窗口出现的最大值,就是结果
             {
                 if(Windows_Average[i] > temp_big)
                 {
                    temp_big = Windows_Average[i];
                 }
             }

             //-watch_data[watch_data_pt] = temp_big;    //-记录的是1S中窗口的最大值,但100mS刷一次值
             //-watch_data_pt = (watch_data_pt + 1) & 0x1ff;

             Noise_Process_value(temp_big);
          }
       }
		}
}
