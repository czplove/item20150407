/*
����һֱ�ڽ���ADCת��,ÿ50uS����һ��
��Ƶ56MHzʱ,ÿ��ת��ʱ����1uS,��ô������ϵͳ���Ҵ������С������1mS,��ô�ҿ��Բ���1000
������(Ϊ����������ô����1024��),Ȼ����д���.
Ҳ����˵ֻҪ��1mS֮�ڰ�ԭ��������
һ���ɼ��ж�֮����������������ٸ���֮ǰ���������
������3.3V ת��Ϊ������4096.��ô����1����0.806mV
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
��ѭ�����ڲ�ѯ,�����Ƿ�����Ч���ݲ�����,�����ȷ�����ǲ�ѯʱ�䳤���޷�ȷ��,���п��ܶ�ʧ����
�����ADC����������û�������,��Ҫע����Ǻ��ڹ۲���ʱ�Բ���ֵ�Ĵ���
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
        	
       for(i=0;i<maxbuffer;i++)		//-������50������
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
			 Total_Average[Counter++]=total;	//-������Ҫ�����ж�����,����Խ���˶���֪��
	
			 if(Counter>=maxcounter)//��ֵ��������100������ʼ���㲢����
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
				  
				  //-����ͨ��ֵ,ΪͨѶ��
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

//-����ֵ����ΪDB
//-y=kx+b ���ü����������εİ취
void Noise_Process_value(WORD Data)
{
	 WORD    temp_data,temp_value,temp_b;
   float   temp_k;


	 if(Data < 20)	//-<20~30
	 {
      temp_value = Data;
      temp_k = 20/(float)10;		//-4�����������/10
      temp_b = 20;
	 }
   else if(Data < 61)	//-<20~30
	 {
      temp_value = Data - 20;
      temp_k = (61 - 20)/(float)10;		//-4�����������/10
      temp_b = 35;
	 }
   else if(Data < 71)	//-<30~40
	 {
	 	  temp_value = Data - 61;
      temp_k = (71 - 61)/(float)10;		//-4�����������/10
      temp_b = 30;
	 }
   else if(Data < 85)	//-<40~50
	 {
	 	  temp_value = Data - 71;
      temp_k = (85 - 71)/(float)10;		//-4�����������/10
      temp_b = 40;
	 }
   else if(Data < 91)	//-<50~60
	 {
	 	  temp_value = Data - 85;
      temp_k = (91 - 85)/(float)10;		//-4�����������/10
      temp_b = 50;
	 }
	 else if(Data < 103)	//-<60~70
	 {
	 	  temp_value = Data - 91;
      temp_k = (103 - 91)/(float)10;		//-4�����������/10
      temp_b = 60;
	 }
	 else if(Data < 229)	//-<70~80
	 {
	 	  temp_value = Data - 103;
      temp_k = (229 - 103)/(float)10;		//-4�����������/10
      temp_b = 70;
	 }
	 else if(Data < 375)	//-<80~90
	 {
	 	  temp_value = Data - 229;
      temp_k = (375 - 229)/(float)10;		//-4�����������/10
      temp_b = 80;
	 }
	 else if(Data < 596)	//-<90~100
	 {
	 	  temp_value = Data - 375;
      temp_k = (596 - 375)/(float)10;		//-4�����������/10
      temp_b = 90;
	 }
	 else if(Data < 980)	//-<100~110
	 {
	 	  temp_value = Data - 596;
      temp_k = (980 - 596)/(float)10;		//-4�����������/10
      temp_b = 100;
	 }
	 else//- if(temp_data < 2400)	//-<110~120
	 {
	 	  temp_value = Data - 980;
      temp_k = (2000 - 980)/(float)10;		//-4�����������/10
      temp_b = 110;
	 }
	 
	 temp_data = (WORD)(temp_value / temp_k);
	 
	 //-if(temp_b >= 100)
	 //-{	
	 //-	 temp_b = 100 + (cticks_ms % 10);   //-��ֵ				  	 
	 //-}
	 temp_data = temp_b + temp_data;
	 //-if(temp_data >= 99)
	 //-   temp_data = 99;
	    	
	 Noise_Value = temp_data;
	 //-watch_data[watch_data_pt] = Noise_Value;
	 //-watch_data_pt = (watch_data_pt + 1) & 0x1ff;
	 //-Start_thansfer=1;
	 //-memset(Total_Average,0x00,sizeof(Total_Average));			
	 
	 //-����ͨ��ֵ,ΪͨѶ��
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
   
   //-���������ľ���ʵʱ��ӳ,���Կ��ٱ任
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
��������:
������Ϊ30��40�ֱ��ǱȽϰ�������������������50�ֱ��ͻ�Ӱ��˯�ߺ���Ϣ��������Ϣ���㣬
ƣ�Ͳ������������������ܻ��ܵ�һ����Ӱ�죻70�ֱ����ϸ���̸��������ķ����ң�����
���У�Ӱ�칤��Ч�ʣ����������¹ʣ����ڹ�����������90�ֱ����ϵ�����������������Ӱ����
���͵������������ķ�����
*/
void Noise_Process_it(void)		//-������1mS���䴦��һ��
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

       for(i=0;i<(maxbuffer/ADC_Channel_num);i++)		//-������512������
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
       temp_data = temp_big - temp_small;	//-�ڼ���1mS��ʱ�䴰�����ҵ������ֵ
       Total_Average[mic_Counter++]=temp_data;	//-������Ҫ�����ж�����,����Խ���˶���֪��,,1.024mS����ȡһ�����ֵ
			 mic_Counter = mic_Counter & 0x3ff;
			 
       //-��Լÿ100mS��Ϊһ����Ч�ֶμ����ƽ��ֵ
       if(mic_Counter>=Windows_counter)
       {
       	  mic_Counter = 0;
       		for(i=0;i<Windows_counter;i++)//-�ۼӺ�
				  {
           	 TOTAL += Total_Average[i];           
			    }
			    TOTAL	= TOTAL/Windows_counter;	//-�ô��ھ�ֵ
			    
			    Windows_Average[Windows_Counter++] = TOTAL;
			    if(Windows_Counter >= Windows_long)
          {
			    	Windows_Counter = 0;

             //-һֱ����1024*1024uS������(��1024��512���е������ֵ)
             temp_big = Windows_Average[0];
             for(i=0;i<Windows_long;i++)		//-�ڴ�Լ1S�Ĵ�����Ѱ�����ֵ,�м���Ծ��ȥ��,ֻ����1S�Ĵ��ڳ��ֵ����ֵ,���ǽ��
             {
                 if(Windows_Average[i] > temp_big)
                 {
                    temp_big = Windows_Average[i];
                 }
             }

             //-watch_data[watch_data_pt] = temp_big;    //-��¼����1S�д��ڵ����ֵ,��100mSˢһ��ֵ
             //-watch_data_pt = (watch_data_pt + 1) & 0x1ff;

             Noise_Process_value(temp_big);
          }
       }
		}
}
