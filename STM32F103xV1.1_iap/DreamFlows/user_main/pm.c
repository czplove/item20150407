/*

*/
#include "user_conf.h"


/*
GP2Y1050AU0F
�̻һ����ڻҳ��ȿ����еķ۳����ڼ�ⷶΧ��ʱ��������Щ�۳���ɢ��Ĺ���������Ԫ����Ϊ
��ѹ�����
�����������
1) �� �� �ʣ�2400 bit/s��
2) ÿ10ms����һ���ֽڣ���7���ֽڣ�����У��λ=Vout(H)+ Vout(L)+Vref(H)+ Vref(L)��
3) ���ݷ��͸�ʽ��
��ʼλ    Vout(H)   Vout(L)   Vref ��H��  Vref ��L��  У��λ    ����λ
0xaa �磺 0x01 �磺 0xe0 �磺 0x00 �磺   0x7a �磺   0x5b      0xff

4)���ݴ���
���յ������ݰ���ʽ�����õ�Vo��ֵ��Vo=(Vout(H)*256+Vout(L))/1024*5
���磺Vout(H)=0x01��ת��Ϊ10����Ϊ1��
Vout(L)=0xe0,ת��Ϊ10����Ϊ224��
��Vo=(1*256+224)/1024*5=2.344 V
�õ�Vo����ֵ�󣬳���ϵ��K���ɵõ��ҳ�Ũ��ֵ���ҳ�Ũ��=K*Vo

�������ﴫ����10ms����һ���ֽ�,��ô��ʵ���Կ��ǲ�ѯ��ʽ
*/

UINT16 pm_value[512];
UINT16 pm_value_pt=0;

void PM_Init(void)
{
	UINT16 i;
	
	for(i=0;i < 512;i++)
		pm_value[i] = 35;
}

UINT16  PM_UnPackUartData( char *FrameBuff)
{
    UINT16  Value_PM;
    //-char   CS = 0;
    char   i;
    char   DataIndex;                   //ָ��ƫ��
    char   FrameLenth = 0x0f;
    int    MSB_Temp=0;
    char   MSB,LSB;
    UINT8  sum;

    //����ƫ�ƣ���λ֡ͷ
    for (i = 0; i < FrameLenth; i++)
    {
        if (FrameBuff[i] == 0xAA)
        {
            DataIndex = i;
            break;
        }
    }

    //δ��λ��֡ͷ����Ϊʧ��
    if (FrameLenth == i)
    {
        return 0;
    }

    //-����һ��У��λ��֤
    sum = FrameBuff[DataIndex + 1] + FrameBuff[DataIndex + 2] + FrameBuff[DataIndex + 3] + FrameBuff[DataIndex + 4];
    if((sum !=  FrameBuff[DataIndex + 5]) || (FrameBuff[DataIndex + 6] != 0xFF))
    	return 0;

    MSB  = FrameBuff[DataIndex + 1];
    LSB  = FrameBuff[DataIndex + 2];
    MSB_Temp=MSB<<8;//�ȼ��ڳ���256.
    //-Value_PM= ((MSB_Temp+LSB)*5)>>10;//V0=(Vout(H)*256+Vout(L))/1024*5*0.35
    Value_PM= (MSB_Temp+LSB);

    return Value_PM;

}



void PM_Process(void)		//-�������ݰѽ����д��Ŀ�Ĵ洢����,��ȫ�ֱ�����
{
    UINT32 Value_temp,temp_data;
    UINT16 i;
    float K;

	 if(Received_Over_Flag_pm==1)//�������
   {
   	   Value_temp = PM_UnPackUartData(ReadBuf_pm);	//ReadBufΪUSART 3�жϴ��������ص�����
   	   if(Value_temp > 0)
   	   {
   	   	  //-Value_temp = (Value_temp * 1000) / 512;		//-��λ��ug/m3  ����ϵ��Kȡ����1/2.5
   	   	  /*if(Value_temp <= 4)    //-0~35
            K = 10;
          else if(Value_temp <= 9) //-35~55
            K = 6;
          else if(Value_temp <= 15) //-55~60
            K = 4;
          else if(Value_temp <= 20) //-60~65
            K = 3.25;
          else if(Value_temp <= 25) //-65~70
            K = 2.8;
          else if(Value_temp <= 35) //-70~75
            K = 2.14;
          else if(Value_temp <= 50) //-75~90
            K = 1.8;
          else if(Value_temp <= 72) //-90~110
            K = 1.5;
          else
            K = 1.25;*/

          /*if(Value_temp <= 1)    //-30		��
   	   	  {
            K = 35;
          }
          else if(Value_temp <= 2) //-40
          {
            K = 20;
          }
          else if(Value_temp <= 3) //-45
          {
            K = 15;
          }
          else if(Value_temp <= 4) //-45
          {
            K = 11.25;
          }
          else if(Value_temp <= 6) //-50~60 ��
          {
            K = 10;
          }
          else if(Value_temp <= 8) //-60~65
          {
            K = 7.6;		//-8.57
          }
          else if(Value_temp <= 10) //-65~70
          {
            K = 6.5;	//-7.22
          }
          else if(Value_temp <= 12) //-70~75
          {
            K = 6.25;
          }
          else if(Value_temp <= 19) //-70~75
          {
            K = 3.94;
          }
          else if(Value_temp <= 25) //-70~75
          {
            K = 3;
          }
          else if(Value_temp <= 35) //-75~90		�����Ⱦ
          {
            K = 2.88;
          }
          else if(Value_temp <= 50) //-90~115
          {
            K = 2.5;
          }
          else if(Value_temp <= 75) //-115~150	�ж���Ⱦ
          {
            K = 1.89;
          }
          else if(Value_temp <= 100) //-150~250		�ض���Ⱦ
          {
            K = 2.5;
          }
          else if(Value_temp <= 150) //-
          {
            K = 1.67;
          }
          else if(Value_temp <= 285) //-����������ֵ��285֮��
          {
            K = 1;
          }
          else//- if(Value_temp <= 30) //-250~150
          {
            K = 0.83;
          }*/

          if(Value_temp <= 1)    //-30		��
   	   	  {
            K = 35;
          }
          else if(Value_temp <= 2) //-40
          {
            K = 20;
          }
          else if(Value_temp <= 3) //-45
          {
            K = 15;
          }
          else if(Value_temp <= 4) //-45
          {
            K = 11.25;
          }
          else if(Value_temp <= 6) //-50~60 ��
          {
            K = 10;
          }
          else if(Value_temp <= 8) //-60~65
          {
            K = 7.6;		//-8.57
          }
          else if(Value_temp <= 10) //-65~70
          {
            K = 6.5;	//-7.22
          }
          else if(Value_temp <= 12) //-70~75
          {
            K = 6.25;
          }
          else if(Value_temp <= 15) //-80~75
          {
            K = 6.2;
          }
          else if(Value_temp <= 19) //-70~100
          {
            K = 5.62;
          }
          else if(Value_temp <= 25) //-70~125
          {
            K = 5;
          }
          else if(Value_temp <= 35) //-75~170		�����Ⱦ
          {
            K = 4.88;
          }
          else if(Value_temp <= 50) //-90~225
          {
            K = 4.5;
          }
          else if(Value_temp <= 75) //-115~250	�ж���Ⱦ
          {
            K = 3.29;
          }
          else if(Value_temp <= 100) //-150~250		�ض���Ⱦ
          {
            K = 2.5;
          }
          else if(Value_temp <= 150) //-
          {
            K = 1.67;
          }
          else if(Value_temp <= 285) //-����������ֵ��285֮��
          {
            K = 1;
          }
          else if(Value_temp <= 485) //-����������ֵ��285֮��
          {
            K = 0.7;
          }
          else//- if(Value_temp <= 30) //-250~150
          {
            K = 0.6;
          }

         Value_temp *= K;

   	   	  pm_value[pm_value_pt++] = Value_temp;
   	   	  if(pm_value_pt > 511)
   	   	  	pm_value_pt = 0;

   	   	  temp_data = 0;
   	   	  for(i=0;i < 512;i++)
   	   	     temp_data = temp_data + pm_value[i];

          pm_data = temp_data / 512;// + 31;

          //-����ͨ��ֵ,ΪͨѶ��
       		port_send_sense_data[4] = pm_data;

       }

       if(Judge_LongTime_In_MainLoop(pm_renew_wait_time,29)==YES)
       {
       		pm_renew_wait_time = Time_2048ms_Counter;

       		if(pm_data_old >= pm_data)
          {
          	 temp_data = pm_data_old - pm_data;
          }
          else
          	 temp_data = pm_data - pm_data_old;
       		if(temp_data >= 20)
       			port_send_sense_data[0] = 0x55;
       		pm_data_old = pm_data;

       		//-����һ��ʱ���ٽ��бȽ϶�����Ҫÿ�θ��¶��Ƚ�
       }

       /*if(pm_data < 75)
       		{
       			 pm_data_flag = 0;
       			 //-RunLed_stata_flag = 0;
       		}
       		else if(pm_data < 115)
       		{
       			 pm_data_flag = 1;
       			 //-RunLed_stata_flag = 1;
       		}
       		else
       		{
       			 pm_data_flag = 2;
       			 //-RunLed_stata_flag = 2;
       		}*/
       //-//-����ͨ��ֵ,ΪͨѶ��
       //-port_send_sense_data[4] = pm_data;

       Received_Over_Flag_pm=0;
   }

	 //-return   Value_PM_Final;		//-����������������ֵ�ǲ��Ե�,�����û�п��Խ������
}


