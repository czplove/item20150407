/*

*/
#include "user_conf.h"


/*
            T16					//-					��
						T15					//- 				��
T8					T14					//-PIN_09 	��
T7					T13  				//-PIN_11		��
T6										  //-12V			FAN-
T5					T12					//-PIN_11		0
T4					T11					//-PIN_09   ES704-MIC-DATA
T3					T10					//-V3.3			MIC-CLK
T2					T9					//-PIN_05   MIC+
T21					T22					//-PIN_03		MIC-

ע:						T1	(GND)
T22�ӵ�MIC-û��ֱ��IO���
T9�ӵ�MIC+
T3Ϊ�ߵ�ƽ
T12Ϊ�͵�ƽ
FAN-��Ӧ(A1) ͨ��D12��˵����������(D12��˵�������·�Ǻõ�)
T21�̽�T4
T2�̽�T5

3.3v�õĻ� D1��
5v�õĻ�D11��
*/
#define PIN_03(BitVal)  			GPIO_WriteBit(GPIOC,GPIO_Pin_10,BitVal)	//-PIN_09							P_B6
#define PIN_05(BitVal)  			GPIO_WriteBit(GPIOC,GPIO_Pin_11,BitVal)	//-PIN_11							P_B7
//-#define PIN_09(BitVal)  			GPIO_WriteBit(GPIOB,GPIO_Pin_6,BitVal)	//-PIN_10							P_A6
//-#define PIN_11(BitVal)  			GPIO_WriteBit(GPIOB,GPIO_Pin_7,BitVal)	//-PIN_12							P_A4
//-#define PIN_17(BitVal)  			GPIO_WriteBit(GPIOB,GPIO_Pin_7,BitVal)	//-PIN_16							P_B6
//-#define PIN_19(BitVal)  			GPIO_WriteBit(GPIOB,GPIO_Pin_6,BitVal)	//-PIN_17		PIN_22		P_B12	P_B11
//-#define PIN_21(BitVal)  			GPIO_WriteBit(GPIOB,GPIO_Pin_13,BitVal)	//-PIN_20							P_B10
//-#define PIN_23(BitVal)  			GPIO_WriteBit(GPIOB,GPIO_Pin_12,BitVal)	//-PIN_20							P_B10
//-#define PIN_25(BitVal)  			GPIO_WriteBit(GPIOA,GPIO_Pin_10,BitVal)	//-PIN_20							P_B10
//-#define PIN_27(BitVal)  			GPIO_WriteBit(GPIOA,GPIO_Pin_9,BitVal)	//-PIN_20							P_B10
//-#define PIN_04(BitVal)  			GPIO_WriteBit(GPIOB,GPIO_Pin_13,BitVal)	//-PIN_20							P_B10
//-#define PIN_06(BitVal)  			GPIO_WriteBit(GPIOB,GPIO_Pin_12,BitVal)	//-PIN_20							P_B10
//-#define PIN_08(BitVal)  			GPIO_WriteBit(GPIOA,GPIO_Pin_10,BitVal)	//-PIN_20							P_B10
//-#define PIN_10(BitVal)  			GPIO_WriteBit(GPIOA,GPIO_Pin_9,BitVal)	//-PIN_20							P_B10

#define FAN_RUN(BitVal)  		GPIO_WriteBit(GPIOA,GPIO_Pin_1,BitVal)


extern void GPIO_Configuration_out(void);
extern void GPIO_Configuration_in(void);
extern void PM_Init(void);
extern void send_39bytes(u8 data8,u32 data32);
extern void CDT9702_Main();
extern void UART1_start(void);
extern void test_SYS_VERSION(void);
extern void VZ89_Read_VOC(void);
extern void VZ89_Read_VOC_test(void);


void sys_init(void)
{
	  sys_err_flag = 0;
	  cticks_s_page = 0;
    led_display_start = 0x55;
    led_display_y = 0;
    i2c1_bus_error_cn = 0;
    i2c2_bus_error_cn = 0;
    rgb_num = 7;	//-����0 ��һֱ�����,������������������ת
    HL_GREEN_pwmval = (PWM_Period_Value * 0) / 255;
    HL_BLUE_pwmval = (PWM_Period_Value * 125) / 255;
    HL_RED_pwmval = (PWM_Period_Value * 255) / 255;
    HL_GREEN_pwmval = (PWM_Period_Value * 200) / 255;
    HL_BLUE_pwmval = (PWM_Period_Value * 200) / 255;
    HL_RED_pwmval = (PWM_Period_Value * 200) / 255;
    HL_new_value_flag = 1;
    HL_flag = 0;

    RED_pwmval_num = 4;		//-һ��ռ�ձȱ��ֵĴ���,Ҳ����ʱ��
    GREEN_pwmval_num = 16;
    BLUE_pwmval_num = 8;		//-8 ������1S����һ��

    RED_pwmval_pt = 0;
    GREEN_pwmval_pt = 42;
    BLUE_pwmval_pt = 84;

    HRL_pt_start = 2;
		HRL_pt_end = 4;
		HRL_RUN_flag = 0x55;	//-��ʼ��ʱ��55,������ת����
		HRL_RUN_ONOFF = 0x55;

		//-��ֹPM2.5����0ֵ,��PM2.5ʧЧ������������������ֵ
		port_send_sense_data[4] = 33;
		PM_Init();

    test_SYS_VERSION(); //-����ϵͳ�汾,��֤������ȷ
}


void sys_test(void)
{
   UINT8 Test_flag,TEMP_DELAY_fg;
   static UINT8  test_data8=0;
	 static UINT32 test_data32=0x00000002;
	 UINT16    TEMP_DELAY_time;


	 Test_flag = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4);
   if(Test_flag == 0)
   {//-������Գ���
   		//-��һ������28PIN����
   		GPIO_Configuration_out();

		   GPIO_Configuration_in();

		   //-����ȫ������ߵ�ƽ
		   //-������е�������Ƿ��Ǹߵ�ƽ
		   PIN_03((BitAction)1);
		   Test_flag = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6);
		   if(Test_flag != 1)
		   	STM32_UP_selfT_flag1 = 1;
		   PIN_05((BitAction)0);
		   Test_flag = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7);
		   if(Test_flag != 0)
		   	STM32_UP_selfT_flag1 = 1;



		   //-PIN_21(0);
		   //-PIN_23(0);
		   //-������е�������Ƿ��Ǹߵ�ƽ
		   PIN_03((BitAction)0);
		   Test_flag = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6);
		   if(Test_flag != 0)
		   	STM32_UP_selfT_flag1 = 1;
		   PIN_05((BitAction)1);
		   Test_flag = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7);
		   if(Test_flag != 1)
		   	STM32_UP_selfT_flag1 = 1;

		   	//-CO2����������
		   	//-co2_data = T6700_Read_CO2();

		   	//-���ڲ���,�ⲿ�Է�����
	    	if(port_send_len[1] == 0)
	    	{
	    		  port_send[1][0] = 0xaa;
	    			port_send_len[1] = 2;
	    	}
	    	Host_LowLevelDelay(1000);
	    	if(port_recv_pt[1] == 0)
	    		STM32_UP_selfT_flag1 = 1;

		   	TEMP_DELAY_fg = 1;
        TEMP_DELAY_time = cticks_ms;
		   	//-���Խ�����ȴ�7620��ѯ
		   	while(1)		//-Ӧ�ò��ܵ���������򴮿��޷�����
		   	{
		   		 HRL_pt_start = 2;
		   		 HRL_pt_end = 4;
		   		 if(Judge_Time_In_MainLoop(TEMP_DELAY_time,500)==YES)
           {
             TEMP_DELAY_fg++;
             TEMP_DELAY_time = cticks_ms;
             if(TEMP_DELAY_fg > 3)
               TEMP_DELAY_fg = 1;
           }
		   		 //-Host_LowLevelDelay(500);
		   		 if(TEMP_DELAY_fg == 1)
           {
			   		 FAN_RUN((BitAction)1);
			   		 test_data8 = 0x92;		//-��ɫ
	           test_data32 = 0x49249249;
	           send_39bytes(test_data8,test_data32);
         	 }
         	 else if(TEMP_DELAY_fg == 2)
         	 {
	           //-Host_LowLevelDelay(500);
	           test_data8 = 0x49;		//-��ɫ
	           test_data32 = 0x24924924;
	           send_39bytes(test_data8,test_data32);
           }
           else if(TEMP_DELAY_fg == 3)
           {
	           //-Host_LowLevelDelay(500);
	           test_data8 = 0x24;		//-��ɫ
	           test_data32 = 0x92492492;
	           send_39bytes(test_data8,test_data32);
	           FAN_RUN((BitAction)0);
         	 }


           //-VOC
          if(Judge_Time_In_MainLoop(CO2_poll_wait_time,1300)==YES)
          {
             CO2_poll_wait_time = cticks_ms;

             VZ89_Read_VOC_test();
          }
		   		 //-��������Э��
    			 CDT9702_Main();
		   	}
   }
}


void it_deal(void)
{
	 if((led_display_start == 0x55) /*&& (RunLed_stata_num == 1)*/ && (UART1_sloop_flag == 0))
	 	 port_send_sense_data[0] = 0x55;
}


void sys_delay_judge(void)
{

	  //-�������ڴ�������,����������
	  if((UART0_transmit_flag == 0x55) && (Judge_LongTime_In_MainLoop(UART0_start_tx_time,88)==YES))
	  {
	  	 UART0_transmit_flag = 0;

       HRL_RUN_flag = 0;		//-�����������Ϩ�������
       //-HRL_RUN_ONOFF = port_report[4];

	  	 UART1_start();
	  }

    if((UART1_sloop_flag != 0) && (Judge_Time_In_MainLoop(UART1_sloop_wait_time,10000)==YES))
	  {
	  	 UART1_sloop_flag = 0;
	  }

}

void RunLed_stata_judge(void)
{

   if(pm_data < 75)
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
   }

   if(Noise_Value < 35)
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
   }

   if((VOC_data <= 300)/* && (VOC_data >= 0)*/)  //-���Ŀǰ��û��ͳһ��׼
   {//-����
   	 VOC_data_flag = 0;
   }
   else if((VOC_data <= 600) && (VOC_data >= 301))
   {//-��΢����
   	 VOC_data_flag = 1;
   }
   else
   {//-���س���
   	 VOC_data_flag = 2;
   }

   if((co2_data <= 750) && (co2_data >= 250))
   {//-����
   	 CO2_data_flag = 0;
   }
   else if((co2_data <= 1000) && (co2_data >= 751))
   {//-��΢����
   	 CO2_data_flag = 1;
   }
   else
   {//-���س���
   	 CO2_data_flag = 2;
   }

}


//-���ڼ������³�ʼ��,Ŀǰ�����ȱ���,������������,���Ǵ��³�ʼ���ڲ�����ĺ���,������Է�����������������ڿ��Կ���
UINT8  port_mon[5];

//-���д��ڼ���,�������Ļ�,���³�ʼ��
void UART_Monitor(void)	//-10S����һ�μ�����
{
		//-SCI0��7620ͨѶ
		port_mon[0]++;
		//-SCI1����ư�ͨѶ
		port_mon[1]++;
		//-PM2.5���մ���������
		port_mon[2]++;
		if(port_mon[2] > 100)
		{

			 port_mon[2] = 0;
		}
}



