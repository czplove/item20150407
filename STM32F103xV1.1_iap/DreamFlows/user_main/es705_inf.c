#include "user_conf.h"


/*
Ŀǰ�����Ĵ�ӡ��Ϣ��Ҫ������,�������ư崮����Ҫ����һ������,��Ҫ������
������ȫ��ͬ,
��ͨ�����Դ���������������ʵ�ֹ���,Ȼ���ٰ��������ȥ��,���ӵ�����1��,��
����״̬�¿��ԵĻ�,ת��һ��������û�������.
\r  CR��ASCII�е�0x0D(13)
\n  LF��ASCII�е�0x0A(10)

��һ�����ѧϰ��һ������˳�

i2c ����������ô��?

���ȿ������ͻ����¼���������(1 2 3 4 5)   ?����������Ϣ��Ҫ������
��������ѧϰ�ͻ���ģʽ��  --��������ģʽ��ʱ���л���Ļ
ѧϰ�Ĺ����п�������״̬�ͳɹ�������


��ѧϰ�Ĺ����п��ܻ���ִ�������Ϣ�������ʱ��,���԰�����������ʱ�ԵĹر�
���ڿ��Կ�ʼ���������ںϲ���,����Ҫ���ڴ�ӡ��,����ͨ��ȫ�ֱ������鿴,�������п���
ͨ�����������鿴

���������ļ������������
I2C����CMD����,����ָ��

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
  	if(f_rec == 1)    //-����û���ϸ�Ҫ�����һ�����̵Ŀ���
  	{
      //-ADC_Cmd(ADC1, DISABLE);    //-������Ҫרע�ڴ����������԰���Щ�ر�
      //-if(Buffercmp(uart1_rx_buff,"TrainingMode\r\n", 14))
      if(es705_mode == 1)
      {
        es705_training_wait_time = Time_2048ms_Counter;
        //-printf("Go to Training mode\n");
        set_es705_to_training_mode();
        es705_training_wait_time = Time_2048ms_Counter;
        es705_training_flag = 1;    //-������ѧϰ״̬
      }

      //-if(Buffercmp(uart1_rx_buff,"VQ\r\n", 4))
      if(es705_mode == 2)
      {
      
        //-printf("Go to voice wake up mode\n");
        set_es705_vq_mode();
        es705_training_flag = 0;    //-�ڻ���״̬
      }
      f_rec = 0;
      es705_mode = 0;
      es705_mode_file = 0;
      //-���������,�ָ���������
      //-ADC_Cmd(ADC1, ENABLE);
      //-ADC_SoftwareStartConvCmd(ADC1, ENABLE);
      //-memset(uart1_rx_buff,0x00,sizeof(uart1_rx_buff));
  	}
	#endif					 

   if((es705_training_flag == 1) &&(Judge_LongTime_In_MainLoop(es705_training_wait_time,29)==YES)) //-��ʱ���뻽��ģʽ
   {
      f_rec = 1;
      es705_mode = 2;
   }
}




