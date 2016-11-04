/*

*/
#include "user_conf.h"


/*
GP2Y1050AU0F
烟灰或室内灰尘等空气中的粉尘处于检测范围内时，由于这些粉尘而散射的光射入光接收元件作为
电压输出。
串口输出参数
1) 波 特 率：2400 bit/s；
2) 每10ms发送一个字节，共7个字节，其中校验位=Vout(H)+ Vout(L)+Vref(H)+ Vref(L)；
3) 数据发送格式：
起始位    Vout(H)   Vout(L)   Vref （H）  Vref （L）  校验位    结束位
0xaa 如： 0x01 如： 0xe0 如： 0x00 如：   0x7a 如：   0x5b      0xff

4)数据处理：
接收到的数据按公式计算后得到Vo的值：Vo=(Vout(H)*256+Vout(L))/1024*5
例如：Vout(H)=0x01，转换为10进制为1；
Vout(L)=0xe0,转换为10进制为224；
则Vo=(1*256+224)/1024*5=2.344 V
得到Vo的数值后，乘以系数K即可得到灰尘浓度值：灰尘浓度=K*Vo

由于这里传感器10ms发送一个字节,那么其实可以考虑查询方式
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
    char   DataIndex;                   //指针偏移
    char   FrameLenth = 0x0f;
    int    MSB_Temp=0;
    char   MSB,LSB;
    UINT8  sum;

    //滑动偏移，定位帧头
    for (i = 0; i < FrameLenth; i++)
    {
        if (FrameBuff[i] == 0xAA)
        {
            DataIndex = i;
            break;
        }
    }

    //未定位到帧头，判为失败
    if (FrameLenth == i)
    {
        return 0;
    }

    //-增加一个校验位验证
    sum = FrameBuff[DataIndex + 1] + FrameBuff[DataIndex + 2] + FrameBuff[DataIndex + 3] + FrameBuff[DataIndex + 4];
    if((sum !=  FrameBuff[DataIndex + 5]) || (FrameBuff[DataIndex + 6] != 0xFF))
    	return 0;

    MSB  = FrameBuff[DataIndex + 1];
    LSB  = FrameBuff[DataIndex + 2];
    MSB_Temp=MSB<<8;//等价于乘以256.
    //-Value_PM= ((MSB_Temp+LSB)*5)>>10;//V0=(Vout(H)*256+Vout(L))/1024*5*0.35
    Value_PM= (MSB_Temp+LSB);

    return Value_PM;

}



void PM_Process(void)		//-计算数据把结果填写到目的存储器中,是全局变量亮
{
    UINT32 Value_temp,temp_data;
    UINT16 i;
    float K;

	 if(Received_Over_Flag_pm==1)//进入解析
   {
   	   Value_temp = PM_UnPackUartData(ReadBuf_pm);	//ReadBuf为USART 3中断处理函数返回的数组
   	   if(Value_temp > 0)
   	   {
   	   	  //-Value_temp = (Value_temp * 1000) / 512;		//-单位是ug/m3  这里系数K取的是1/2.5
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

          /*if(Value_temp <= 1)    //-30		优
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
          else if(Value_temp <= 6) //-50~60 良
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
          else if(Value_temp <= 35) //-75~90		轻度污染
          {
            K = 2.88;
          }
          else if(Value_temp <= 50) //-90~115
          {
            K = 2.5;
          }
          else if(Value_temp <= 75) //-115~150	中度污染
          {
            K = 1.89;
          }
          else if(Value_temp <= 100) //-150~250		重度污染
          {
            K = 2.5;
          }
          else if(Value_temp <= 150) //-
          {
            K = 1.67;
          }
          else if(Value_temp <= 285) //-大概限制最大值在285之内
          {
            K = 1;
          }
          else//- if(Value_temp <= 30) //-250~150
          {
            K = 0.83;
          }*/

          if(Value_temp <= 1)    //-30		优
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
          else if(Value_temp <= 6) //-50~60 良
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
          else if(Value_temp <= 35) //-75~170		轻度污染
          {
            K = 4.88;
          }
          else if(Value_temp <= 50) //-90~225
          {
            K = 4.5;
          }
          else if(Value_temp <= 75) //-115~250	中度污染
          {
            K = 3.29;
          }
          else if(Value_temp <= 100) //-150~250		重度污染
          {
            K = 2.5;
          }
          else if(Value_temp <= 150) //-
          {
            K = 1.67;
          }
          else if(Value_temp <= 285) //-大概限制最大值在285之内
          {
            K = 1;
          }
          else if(Value_temp <= 485) //-大概限制最大值在285之内
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

          //-增加通道值,为通讯用
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

       		//-经过一段时间再进行比较而不需要每次更新都比较
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
       //-//-增加通道值,为通讯用
       //-port_send_sense_data[4] = pm_data;

       Received_Over_Flag_pm=0;
   }

	 //-return   Value_PM_Final;		//-这样上送上来的数值是不对的,如果还没有可以进入分析
}


