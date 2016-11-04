/*

*/
#include "user_conf.h"


/*
            T16					//-					PIN_22
						T15					//- 				PIN_20
T8					T14					//-PIN_19 	PIN_11
T7					T13  				//-����  		PIN_16
T6										  //-PIN_13		GND
T5					T12					//-PIN_18		PIN_12
T4					T11					//-PIN_09   PIN_10
T3					T10					//-PIN_07		PIN_08
T2					T9					//-PIN_05   PIN_06
T21					T22					//-V3.3		  GND

ע:
T1	(GND)
T21��3.3V
T22�ӵ�
������һһ��Ӧ��������ȱʧT16��T14���ӵ�T8��
T15���ӵ�T6��


3.3v�õĻ� D1��

*/

//-#define PIN_03(BitVal)  			GPIO_WriteBit(GPIOC,GPIO_Pin_10,BitVal)	//-PIN_09							P_B6
#define PIN_05(BitVal)  			GPIO_WriteBit(GPIOB,GPIO_Pin_0,BitVal)	//-PIN_06							P_A7
#define PIN_07(BitVal)  			GPIO_WriteBit(GPIOC,GPIO_Pin_0,BitVal)	//-PIN_08							P_A5
#define PIN_09(BitVal)  			GPIO_WriteBit(GPIOC,GPIO_Pin_1,BitVal)	//-PIN_10							P_A6
//-#define PIN_11(BitVal)  			GPIO_WriteBit(GPIOC,GPIO_Pin_2,BitVal)	//-PIN_10							P_A6
#define PIN_13(BitVal)  			GPIO_WriteBit(GPIOC,GPIO_Pin_3,BitVal)	//-PIN_20							P_B10
//-#define PIN_17(BitVal)  			GPIO_WriteBit(GPIOB,GPIO_Pin_12,BitVal)	//-PIN_16							P_B6
#define PIN_19(BitVal)  			GPIO_WriteBit(GPIOB,GPIO_Pin_13,BitVal)	//-PIN_11		PIN_22		P_C2	P_B11
#define PIN_18(BitVal)  			GPIO_WriteBit(GPIOB,GPIO_Pin_7,BitVal)	//-PIN_12							P_A4
#define PIN_16(BitVal)  			GPIO_WriteBit(GPIOB,GPIO_Pin_6,BitVal)	//-PIN_17							P_B12
//-#define PIN_20(BitVal)  			GPIO_WriteBit(GPIOB,GPIO_Pin_10,BitVal)	//-PIN_20							P_B10
//-#define PIN_22(BitVal)  			GPIO_WriteBit(GPIOB,GPIO_Pin_11,BitVal)	//-PIN_20							P_B10
//-#define PIN_12(BitVal)  			GPIO_WriteBit(GPIOA,GPIO_Pin_4,BitVal)	//-PIN_20							P_B10
//-#define PIN_06(BitVal)  			GPIO_WriteBit(GPIOA,GPIO_Pin_7,BitVal)	//-PIN_20							P_B10
//-#define PIN_08(BitVal)  			GPIO_WriteBit(GPIOA,GPIO_Pin_5,BitVal)	//-PIN_20							P_B10
//-#define PIN_10(BitVal)  			GPIO_WriteBit(GPIOA,GPIO_Pin_6,BitVal)	//-PIN_20							P_B10


extern void ADXL345_init(void);
extern int MONI_EEPROM_read(void);
extern void GPIO_Configuration_out(void);
extern void GPIO_Configuration_in(void);
extern void initADUX1020(void);
extern void MONI_EEPROM_sub(void);


void sys_init(void)
{
	 int res;


	  sys_err_flag = 0;
	  cticks_s_page = 0;

    led_display_long = 60;    //-��Ȼ������޸������ֵ����Ϊ�˷���ʧ�ܻ��Ǹ�һ��Ĭ��ֵ
    voice_flag = 0x55;

    i2c1_bus_error_cn = 0;
    i2c2_bus_error_cn = 0;
    rgb_num = 7;	//-����0 ��һֱ�����,������������������ת

    HL_run_time_delay = 1800;

    HL_RED_pwmval = (PWM_Period_Value * 0) / 255;   //-��
    HL_GREEN_pwmval = (PWM_Period_Value * 0) / 255;		//-��
    HL_BLUE_pwmval = (PWM_Period_Value * 0) / 255;	//-��
    HL_new_value_flag = 1;
    HL_flag = 0;
    cticks_ms_pwm_loop = 5000;

    RED_pwmval_num = 3;		//-һ��ռ�ձȱ��ֵĴ���,Ҳ����ʱ��
    GREEN_pwmval_num = 3;
    BLUE_pwmval_num = 3;		//-8 ������1S����һ��
    white_pwmval_num = 3;

    RED_pwmval_pt = 0;
    GREEN_pwmval_pt = 0;
    BLUE_pwmval_pt = 0;
    white_pwmval_pt = 0;

    //-�̶�ģʽ��ֵ
    HL_ld_R[0] = 0;
    HL_ld_G[0] = 0;
    HL_ld_B[0] = 255;

    HL_ld_R[1] = 0/8;
    HL_ld_G[1] = 0/8;
    HL_ld_B[1] = 255/8;

    HL_ld_R[2] = 0/6;
    HL_ld_G[2] = 0/6;
    HL_ld_B[2] = 255/6;

    HL_ld_R[3] = 0/3;
    HL_ld_G[3] = 0/3;
    HL_ld_B[3] = 255/3;

    HL_ld_R[4] = 0/6;
    HL_ld_G[4] = 0/6;
    HL_ld_B[4] = 255/6;

    HL_ld_R[5] = 0/8;
    HL_ld_G[5] = 0/8;
    HL_ld_B[5] = 255/8;

    //-�û��趨��ɫ��ʼĬ��Ϊ��ɫ
    HL_ld_R_user[0] = 0;
    HL_ld_G_user[0] = 0;
    HL_ld_B_user[0] = 255;

    //-�û��趨����ֵ
    ADXL_THRESH_TAP = 0x30;   //-0x20��С�ȶ�ֵ �ɿ���СֵΪ0x30 ��������ʱ���0x70�ɿɿ���֤���󴥷�,��ô����64
    ADXL_DUR = 0x70;      //-0x30 ,ò��0x15��ʱ���󴥷�����

    EEP_Data_flag = 0;
    //-����ģ��FLASH�еĶ���,ÿ���ϵ��
    res = MONI_EEPROM_read();

    if(res == 1)
    	MONI_EEPROM_sub();

    //-//-�����޸Ķ�ֵ
    //-EEP_Data_flag = 0x55;
    //-led_display_long = 30;

    //-PM2.5 ��û����Чֵ���и���ʱʹ�������ֵ
    pm_data = 33;

    if(*(uint16_t*)(APP_CONFIG3_ADDR) != STM32_SYS_VERSION_NUM)
    {//-����ϵͳ�汾��
        FLASH_Unlock();						//����
				FLASH_ErasePage(APP_CONFIG3_ADDR);//�����������
				FLASH_ProgramHalfWord(APP_CONFIG3_ADDR,STM32_SYS_VERSION_NUM);	//-�������п�������
				FLASH_Lock();//����
    }
}

void sys_test(void)   //-����֧����ư�Ĳ���,��֧�ֵ���������
{
   char Test_flag;

	 //-���Ͻ���һ��������ʼ��,�����ж��Ƿ������Գ���
   Test_flag = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12);
   while(Test_flag == 0)		//-��������Ҫ����ȥ�ſ��Լ��,��ư�ͣ��������Ϳ���.
   {//-�������״̬
   	  HL_new_value_flag = 1;
   	  HL_RED_pwmval = (PWM_Period_Value * 10) / 255;   //-��
    	HL_GREEN_pwmval = (PWM_Period_Value * 0) / 255;		//-��
    	HL_BLUE_pwmval = (PWM_Period_Value * 0) / 255;	//-��

    	Host_LowLevelDelay(500);
    	HL_new_value_flag = 1;
    	HL_RED_pwmval = (PWM_Period_Value * 0) / 255;   //-��
    	HL_GREEN_pwmval = (PWM_Period_Value * 0) / 255;		//-��
    	HL_BLUE_pwmval = (PWM_Period_Value * 10) / 255;	//-��

    	Host_LowLevelDelay(500);
    	HL_new_value_flag = 1;
    	HL_RED_pwmval = (PWM_Period_Value * 0) / 255;   //-��
    	HL_GREEN_pwmval = (PWM_Period_Value * 10) / 255;		//-��
    	HL_BLUE_pwmval = (PWM_Period_Value * 0) / 255;	//-��

      Host_LowLevelDelay(500);
      HL_new_value_flag = 1;
   	  HL_RED_pwmval = (PWM_Period_Value * 10) / 255;   //-��
    	HL_GREEN_pwmval = (PWM_Period_Value * 0) / 255;		//-��
    	HL_BLUE_pwmval = (PWM_Period_Value * 0) / 255;	//-��

    	Host_LowLevelDelay(500);
    	HL_new_value_flag = 1;
    	HL_RED_pwmval = (PWM_Period_Value * 0) / 255;   //-��
    	HL_GREEN_pwmval = (PWM_Period_Value * 0) / 255;		//-��
    	HL_BLUE_pwmval = (PWM_Period_Value * 10) / 255;	//-��

    	Host_LowLevelDelay(500);
    	HL_new_value_flag = 1;
    	HL_RED_pwmval = (PWM_Period_Value * 0) / 255;   //-��
    	HL_GREEN_pwmval = (PWM_Period_Value * 10) / 255;		//-��
    	HL_BLUE_pwmval = (PWM_Period_Value * 0) / 255;	//-��

    	Host_LowLevelDelay(500);
    	HL_new_value_flag = 1;
    	HL_RED_pwmval = (PWM_Period_Value * 0) / 255;   //-��
    	HL_GREEN_pwmval = (PWM_Period_Value * 0) / 255;		//-��
    	HL_BLUE_pwmval = (PWM_Period_Value * 0) / 255;	//-��

    	//-������ȫ��
      led_display_long = 255;
      led_display_start = 0x55;
    	led_display_flag = 1;

    	//-���ڲ���,�ⲿ�Է�����
    	if(port_send_len[0] == 0)
    	{
    		  port_send[0][0] = 0xaa;
    			port_send_len[0] = 1;
    	}
    	Host_LowLevelDelay(1000);
    	printf("\r\nport_recv_pt[0] is %d\n",port_recv_pt[0]);	//-�жϷ��ͳ�ȥ��������û���յ�,�յ��Ļ�����������

    	STM32_UP_selfT_flag1 = 0;

    	//-��һ������28PIN����
   		GPIO_Configuration_out();

		   GPIO_Configuration_in();

		   //-����ȫ������ߵ�ƽ
		   //-������е�������Ƿ��Ǹߵ�ƽ
		   PIN_05((BitAction)1);
		   Test_flag = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7);
		   if(Test_flag != 1)
		   	STM32_UP_selfT_flag1 = 1;
		   PIN_07((BitAction)0);
		   Test_flag = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5);
		   if(Test_flag != 0)
		   	STM32_UP_selfT_flag1 = 1;
		   PIN_09((BitAction)1);
		   Test_flag = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);
		   if(Test_flag != 1)
		   	STM32_UP_selfT_flag1 = 1;
		   PIN_13((BitAction)0);
		   Test_flag = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10);
		   if(Test_flag != 0)
		   	STM32_UP_selfT_flag1 = 1;
		   PIN_18((BitAction)1);
		   Test_flag = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4);
		   if(Test_flag != 1)
		   	STM32_UP_selfT_flag1 = 1;
		   PIN_16((BitAction)0);
		   Test_flag = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12);
		   if(Test_flag != 0)
		   	STM32_UP_selfT_flag1 = 1;
		   PIN_19((BitAction)1);
		   Test_flag = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2);
		   if(Test_flag != 1)
		   	STM32_UP_selfT_flag1 = 1;
		   Test_flag = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11);
		   if(Test_flag != 1)
		   	STM32_UP_selfT_flag1 = 1;


		   //-PIN_21(0);
		   //-PIN_23(0);
		   //-������е�������Ƿ��Ǹߵ�ƽ
		   PIN_05((BitAction)0);
		   Test_flag = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7);
		   if(Test_flag != 0)
		   	STM32_UP_selfT_flag1 = 1;
		   PIN_07((BitAction)1);
		   Test_flag = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5);
		   if(Test_flag != 1)
		   	STM32_UP_selfT_flag1 = 1;
		   PIN_09((BitAction)0);
		   Test_flag = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);
		   if(Test_flag != 0)
		   	STM32_UP_selfT_flag1 = 1;
		   PIN_13((BitAction)1);
		   Test_flag = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10);
		   if(Test_flag != 1)
		   	STM32_UP_selfT_flag1 = 1;
		   PIN_18((BitAction)0);
		   Test_flag = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4);
		   if(Test_flag != 0)
		   	STM32_UP_selfT_flag1 = 1;
		   PIN_16((BitAction)1);
		   Test_flag = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12);
		   if(Test_flag != 1)
		   	STM32_UP_selfT_flag1 = 1;
		   PIN_19((BitAction)0);
		   Test_flag = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2);
		   if(Test_flag != 0)
		   	STM32_UP_selfT_flag1 = 1;
		   Test_flag = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11);
		   if(Test_flag != 0)
		   	STM32_UP_selfT_flag1 = 1;

    	HL_new_value_flag = 1;
    	while((port_recv[0][port_recv_pt[0] - 1] !=  0xaa) || (STM32_UP_selfT_flag1 != 0))
      {

         HL_RED_pwmval = (PWM_Period_Value * 10) / 255;   //-��
         HL_GREEN_pwmval = (PWM_Period_Value * 10) / 255;		//-��
         HL_BLUE_pwmval = (PWM_Period_Value * 10) / 255;	//-��
      }

      //-���п���ͨѶ��,��ʼ��ͨ�����ñ�־λ,������Ϊ���쳣
    	printf("\r\nSTM32_UP_selfT_flag1 is %d\n",STM32_UP_selfT_flag1);

   	  Test_flag = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12);
   }
}


void sys_delay_judge(void)
{
	  struct i2c_client client;
	  int Result;
	  char Test_flag;

	  //-�������������ж�
	  if((led_display_flag == 0) && (Judge_LongTime_In_MainLoop(cartoon_start_time,88)==YES))		//-������ʱ��Լ3����
	  {
	  	 led_display_new = 0x55;
	  	 led_display_flag = 3;
	  }

	 //-����ʱ���⴫����״̬
	  if(Judge_LongTime_In_MainLoop(TEST_Sensor_wait_time,10)==YES)
	  {
	  	 TEST_Sensor_wait_time = Time_2048ms_Counter;	//-ÿ���һ��,�ͷ���״ֵ̬,������õĻ��ͽ��г�ʼ��,����ۼƼ��β��ɹ��ͱ���
	  	 											//-����ʵʩ��ʱ����˵,���Ե�ʱ�򲻿������³�ʼ��,�������ܷ��ֲ���ͻ������
	  }

	  //-����ʱ���⴫����״̬
	  if((Judge_LongTime_In_MainLoop(led_display_end_time,30)==YES) && (led_display_start == 0xaa) && (UART1_transmit_control == 0))
	  {
	  	 led_display_start = 0;		//-������Ϩ��һ��ʱ��󳹵�Ϩ��,������ﷴ��ˢֵ�Ļ�,�ͻḲ����������,���ֱ��Ķ�ʧ�����
	  	 TEST_Sensor_wait_time = Time_2048ms_Counter;
			 UART1_transmit_flag = YES;		//-������֯���ݷ���
	     UART1_transmit_control = 2;	//-�������������͵�����
	  }

	  //-���ݿ����ʱ��
	  if((Judge_LongTime_In_MainLoop(UART1_renew_wait_time,7)==YES) && (UART1_renew_flag == 0x55))
	  {
	  	 UART1_renew_wait_time = Time_2048ms_Counter;

	     UART1_renew_flag = 0;
	  }

	  if((Judge_Time_In_MainLoop(Sensor_data_wait_time,5000)==YES) /*&& (UART1_transmit_control == 0)*/)	//-ֻҪ���������жϵĶ����Եȴ�����
	  {
	  	 if(led_display_data_flag == 1)	//-1��ʾ�������ݿ�ʼ��ʱ,����ͣ������һҳ���б�Ҫ��ʼ��ʱ
	  	 {
	  	 	  if((led_display_page == 0) || (led_display_page == 3))
	  	 	  {
			  	 		Sensor_data_wait_time = cticks_ms;
			  	 		led_display_data_flag = 2;
			  	 		led_display_new = 0x55;
			  	 		led_display_flag = 3;		//-���ݸ��º���,�л���������ʾ

			  	 		//-��������,�Ӻ���,����Ϊ�˲���ͬ������ô��
			  	 		UART1_transmit_flag = YES;		//-������֯���ݷ���
		          UART1_transmit_control = 1;	//-�������������͵�����	?�����п��ܳ��ִ��ҵ����������ϱ�,����ʾ�Ļ�,�ָ�Ϊ��0,�Ͳ�������?
        	}
        	else
        	{
        			Sensor_data_wait_time = cticks_ms;
	  	 				led_display_data_flag = 0;
        	}
	  	 }
	  	 else if(led_display_data_flag == 3)
	  	 {
	  	 		Sensor_data_wait_time = cticks_ms;
	  	 		led_display_data_flag = 0;
	  	 }
	  	 else
	  	 {
	  	 		Sensor_data_wait_time = cticks_ms;
	  	 		//-led_display_data_flag = 0;
	  	 }
	  }

	  if((Judge_LongTime_In_MainLoop(HL_run_time,HL_run_time_delay)==YES) && (HL_flag != 0))		//-�ر��ŲʵƼ�ʱ
	  {
	  	 HL_RED_pwmval = 0;
		 	 HL_GREEN_pwmval = 0;
		 	 HL_BLUE_pwmval = 0;
	  	 HL_new_value_flag = 1;
	  	 HL_flag = 0;

	  	 UART1_transmit_flag = YES;		//-������֯���ݷ���
	  	 UART1_transmit_control = 7;		//-�������Ųʵ�ģʽ
	  }

	  if(ADXL_TAP_it_flag == 1)
	  {
      if(Judge_Time_In_MainLoop(ADXL_TAP_wait_time,400)==YES)	//-�������ʱ����Ϊ�˷���,����Ϊ����Ϊ���������û��������ʱ
	    {
	       client.addr = 0xA6;
	       client.num = 1;		//-��ʾI2C1������

	       //-������
	       //-i2c_read_reg(&client,0X2B,&voice_flag);

	       Result = i2c_read_reg(&client,0x30,&ADXL_TAP_it_SOURCE);		//-�ж�Դ,��ȡ���ݿ�������ж��ź�
	       if(Result <= 0)
         {
            ADXL_TAP_it_SOURCE = 0xff;
            //-return;		//-���ں��滹����Ҫ�����,����Ĳ��ܽ�����������������û��ִ��
         }
         if((ADXL_TAP_it_SOURCE & 0x40) == 0)
	       {//-������˵�������¼�������
	       	  //-����ƽ��֤�ж�ȷʵ�����˶���������ͨѶ�쳣�����ж�û�н���
	       	  Test_flag = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13);
	       	  if(Test_flag == 1)
	          	ADXL_TAP_it_flag = 0;
	       }

	    }
	  }
	 //-i2c_read_reg(&client,0X00,&ADXL_TAP_it_SOURCE);
	 if((ADXL_TAP_off_flag == 0x55) && (Judge_Time_In_MainLoop(ADXL_TAP_off_time,50000)==YES))
	 {
	 	  ADXL_TAP_off_flag = 0;
	 }

    //-�����Եļ�������ж��ź������Ƿ��з��������������һ��ʱ��û�еĻ�,����Ϊ����ʧЧ��
    //-�������³�ʼ��
    Result = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12);
    if(Result == 0)
       ADI_ps_H_time = cticks_ms;   //-ֻҪ���ֵ͵�ƽ����Ϊι���ɹ�,Ȼ�����¼�ʱ,��ʱ�䱣�ָߵ�ƽ
                                    //-����ΪʧЧ,��Ҫ���³�ʼ��
    if(Judge_Time_In_MainLoop(ADI_ps_H_time,50000)==YES)
    {
        ADI_ps_H_time = cticks_ms;
        initADUX1020();		//-��������ʼ��,У׼֮����������ڻ���û�з����仯,���Բ���Ҫ����У׼,�Ѿ�У׼�����ݾͿ�����
        if((STM32_UP_selfT_flag1 & 0x04) != 0)
  				initADUX1020();
        ADI_ps_flag = 0x55;	//-��λ֮��һ��ʱ���ڲ��������
        ADI_ps_wait_time = cticks_ms;
    }

    //-
    if((Judge_LongTime_In_MainLoop(ADI_channels_wait_time,44)==YES)	&& (ADI_channels_init_flag == 0x55))	//-ʱ��ǿ�Ƹ�λ
	  {
	  	 ADI_channels_wait_time = Time_2048ms_Counter;
	  	 initADUX1020();
	  	 if((STM32_UP_selfT_flag1 & 0x04) != 0)
  				initADUX1020();
	  	 ADI_channels_init_flag = 0;		//-���ǳ������û�����������У׼
	  	 ADI_ps_flag = 0x55;	//-��λ֮��һ��ʱ���ڲ��������
       ADI_ps_wait_time = cticks_ms;
	  }

	  //-������
     Test_flag = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12);
	   if(Test_flag == 0)
	   {
	    //-ADUX1020_Algorithm_Init();
	    initADUX1020();
	    if((STM32_UP_selfT_flag1 & 0x04) != 0)
	  			initADUX1020();
	   }


	 //-I2C���߳�����,,��������³�ʼ����һ����ʹ,���Ǵ����жϻ����ǶԵ�,�޸����ݿ��Ա仯
      if(i2c1_bus_error_cn > 20)	//-����ʹ���������Ӧ�ò������������������,��ʹ������Ҳ��û�а취��
      {
      	 i2c1_bus_error_cn = 0;
      	 //-���³�ʼ��,������ܵ�ǯλ����,��Ҫ����

      }

      //-������
      if(((STM32_UP_selfT_flag1 & 0x01) != 0)/* && ((STM32_UP_error_flag1 & 0x01) == 0)*/)	//-ֻҪ�д������ͽ������³�ʼ��ֱ����ȷΪֹ
      {
  			ADXL345_init();
  			STM32_UP_selfT_cn1++;
  		}
  		else
  			STM32_UP_selfT_cn1 = 0;

  		//-if(((STM32_UP_selfT_flag1 & 0x02) != 0) && ((STM32_UP_error_flag1 & 0x02) == 0))
  		//-{
  		//-	initBH1750FVI();
  		//-	STM32_UP_selfT_cn2++;
  		//-}
  		//-else
  		//-	STM32_UP_selfT_cn2 = 0;

  		if(((STM32_UP_selfT_flag1 & 0x04) != 0) && ((STM32_UP_error_flag1 & 0x04) == 0))
  		{
  			initADUX1020();
  			STM32_UP_selfT_cn3++;
  		}
  		else
  			STM32_UP_selfT_cn3 = 0;

      if(STM32_UP_selfT_cn1 > 10)
      {
      	 STM32_UP_error_flag1 |= 0x01;
      }
      //-else
      //-	 STM32_UP_error_flag1 &= 0xFE;

      //-if(STM32_UP_selfT_cn2 > 10)
      //-{
      //-	 STM32_UP_error_flag1 |= 0x02;
      //-}

      if(STM32_UP_selfT_cn3 > 10)
      {
      	 STM32_UP_error_flag1 |= 0x04;
      }
      //-else
      //-	 STM32_UP_error_flag1 &= 0xFB;

      //-if((led_display_start != 0x55) && (EEP_Data_flag == 0x55))
      if(EEP_Data_flag == 0x55)		//-����������ʱ�޸Ķ�ֵ�Ƿ�Ӱ����ʾ
      	MONI_EEPROM_sub();	//-��ĻϨ���˲Ż��޸Ŀ��ܸı�Ķ�ֵ
}