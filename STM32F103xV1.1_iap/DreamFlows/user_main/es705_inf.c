#include "user_conf.h"


/*
目前大量的打印信息需要处理下,这个和射灯板串口需要合用一个串口,主要是两者
机制完全不同,
先通过调试串口在整个过程中实现功能,然后再把这个串口去掉,叠加到串口1上,在
调试状态下可以的话,转换一个串口是没有问题的.
\r  CR或ASCII中的0x0D(13)
\n  LF或ASCII中的0x0A(10)

万一错误的学习了一个如何退出

i2c 串口死了怎么办?

首先可以上送唤醒事件的类型了(1 2 3 4 5)   ?其它多余信息需要上送吗
可以设置学习和唤醒模式了  --设置语音模式的时候切换屏幕
学习的过程中可以上送状态和成功次数了


在学习的过程中可能会出现大量的信息操作这个时候,可以把其它功能暂时性的关闭
现在可以开始把两个串口合并了,不需要串口打印了,可以通过全局变量来查看,程序运行可以
通过计数器来查看

串口下载文件传输大量数据
I2C传送CMD命令,交互指令

*/




void es705_interface(void)
{
  es705_bootup();

 
  //-printf("Go to voice wake up mode\n");
 	set_es705_vq_mode();

}



void es705_int_deal(void)
{
   if(irq_flag)
	 {
     //-ADC_Cmd(ADC1, DISABLE);
		  	//-printf("\r\nProcess ES704 Interrupt\n");
			//-Led_ON();
			es705_interrupt_service();
			msleep(200);
			
			//-printf("Go to voice wake up mode\n");
			set_es705_vq_mode();
	
			irq_flag = 0;
			//-Led_OFF();	
      //-ADC_Cmd(ADC1, ENABLE);
      //-ADC_SoftwareStartConvCmd(ADC1, ENABLE);
		}
	#if 1
  	if(f_rec == 1)    //-这里没有严格要求就是一个流程的控制
  	{
      //-ADC_Cmd(ADC1, DISABLE);    //-现在需要专注于处理语音所以把这些关闭
      //-if(Buffercmp(uart1_rx_buff,"TrainingMode\r\n", 14))
      if(es705_mode == 1)
      {
        es705_training_wait_time = Time_2048ms_Counter;
        //-printf("Go to Training mode\n");
        set_es705_to_training_mode();
        es705_training_wait_time = Time_2048ms_Counter;
        es705_training_flag = 1;    //-进入了学习状态
      }

      //-if(Buffercmp(uart1_rx_buff,"VQ\r\n", 4))
      if(es705_mode == 2)
      {
      
        //-printf("Go to voice wake up mode\n");
        set_es705_vq_mode();
        es705_training_flag = 0;    //-在唤醒状态
      }
      f_rec = 0;
      es705_mode = 0;
      es705_mode_file = 0;
      //-处理结束后,恢复正常操作
      //-ADC_Cmd(ADC1, ENABLE);
      //-ADC_SoftwareStartConvCmd(ADC1, ENABLE);
      //-memset(uart1_rx_buff,0x00,sizeof(uart1_rx_buff));
  	}
	#endif					 

   if((es705_training_flag == 1) &&(Judge_LongTime_In_MainLoop(es705_training_wait_time,29)==YES)) //-超时进入唤醒模式
   {
      f_rec = 1;
      es705_mode = 2;
   }
}




