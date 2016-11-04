/*

*/
#include "user_conf.h"


/*
            T16					//-					无
						T15					//- 				无
T8					T14					//-PIN_09 	无
T7					T13  				//-PIN_11		无
T6										  //-12V			FAN-
T5					T12					//-PIN_11		0
T4					T11					//-PIN_09   ES704-MIC-DATA
T3					T10					//-V3.3			MIC-CLK
T2					T9					//-PIN_05   MIC+
T21					T22					//-PIN_03		MIC-

注:						T1	(GND)
T22接的MIC-没有直接IO输出
T9接的MIC+
T3为高电平
T12为低电平
FAN-对应(A1) 通过D12亮说明风扇启动(D12闪说明这个回路是好的)
T21短接T4
T2短接T5

3.3v好的话 D1亮
5v好的话D11亮
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
    rgb_num = 7;	//-等于0 就一直是灭灯,等于其他可以周期旋转
    HL_GREEN_pwmval = (PWM_Period_Value * 0) / 255;
    HL_BLUE_pwmval = (PWM_Period_Value * 125) / 255;
    HL_RED_pwmval = (PWM_Period_Value * 255) / 255;
    HL_GREEN_pwmval = (PWM_Period_Value * 200) / 255;
    HL_BLUE_pwmval = (PWM_Period_Value * 200) / 255;
    HL_RED_pwmval = (PWM_Period_Value * 200) / 255;
    HL_new_value_flag = 1;
    HL_flag = 0;

    RED_pwmval_num = 4;		//-一个占空比保持的次数,也就是时间
    GREEN_pwmval_num = 16;
    BLUE_pwmval_num = 8;		//-8 几乎是1S中闪一次

    RED_pwmval_pt = 0;
    GREEN_pwmval_pt = 42;
    BLUE_pwmval_pt = 84;

    HRL_pt_start = 2;
		HRL_pt_end = 4;
		HRL_RUN_flag = 0x55;	//-初始化时给55,进行旋转跑马
		HRL_RUN_ONOFF = 0x55;

		//-防止PM2.5出现0值,在PM2.5失效的情况下上送下面的数值
		port_send_sense_data[4] = 33;
		PM_Init();

    test_SYS_VERSION(); //-测试系统版本,保证程序正确
}


void sys_test(void)
{
   UINT8 Test_flag,TEMP_DELAY_fg;
   static UINT8  test_data8=0;
	 static UINT32 test_data32=0x00000002;
	 UINT16    TEMP_DELAY_time;


	 Test_flag = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4);
   if(Test_flag == 0)
   {//-进入测试程序
   		//-第一步测试28PIN引脚
   		GPIO_Configuration_out();

		   GPIO_Configuration_in();

		   //-首先全部输出高电平
		   //-检测所有的输入端是否都是高电平
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
		   //-检测所有的输入端是否都是高电平
		   PIN_03((BitAction)0);
		   Test_flag = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6);
		   if(Test_flag != 0)
		   	STM32_UP_selfT_flag1 = 1;
		   PIN_05((BitAction)1);
		   Test_flag = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7);
		   if(Test_flag != 1)
		   	STM32_UP_selfT_flag1 = 1;

		   	//-CO2传感器数据
		   	//-co2_data = T6700_Read_CO2();

		   	//-串口测试,外部自发自收
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
		   	//-测试结束后等待7620查询
		   	while(1)		//-应该不能等在这里否则串口无法处理
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
			   		 test_data8 = 0x92;		//-红色
	           test_data32 = 0x49249249;
	           send_39bytes(test_data8,test_data32);
         	 }
         	 else if(TEMP_DELAY_fg == 2)
         	 {
	           //-Host_LowLevelDelay(500);
	           test_data8 = 0x49;		//-蓝色
	           test_data32 = 0x24924924;
	           send_39bytes(test_data8,test_data32);
           }
           else if(TEMP_DELAY_fg == 3)
           {
	           //-Host_LowLevelDelay(500);
	           test_data8 = 0x24;		//-绿色
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
		   		 //-处理网关协议
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

	  //-启动周期传送数据,类心跳报文
	  if((UART0_transmit_flag == 0x55) && (Judge_LongTime_In_MainLoop(UART0_start_tx_time,88)==YES))
	  {
	  	 UART0_transmit_flag = 0;

       HRL_RUN_flag = 0;		//-网关启动完成熄灭跑马灯
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

   if((VOC_data <= 300)/* && (VOC_data >= 0)*/)  //-这个目前还没有统一标准
   {//-良好
   	 VOC_data_flag = 0;
   }
   else if((VOC_data <= 600) && (VOC_data >= 301))
   {//-轻微超标
   	 VOC_data_flag = 1;
   }
   else
   {//-严重超标
   	 VOC_data_flag = 2;
   }

   if((co2_data <= 750) && (co2_data >= 250))
   {//-良好
   	 CO2_data_flag = 0;
   }
   else if((co2_data <= 1000) && (co2_data >= 751))
   {//-轻微超标
   	 CO2_data_flag = 1;
   }
   else
   {//-严重超标
   	 CO2_data_flag = 2;
   }

}


//-串口监视重新初始化,目前不做先保留,正常程序会崩溃,但是从新初始化内部外设的很少,如果测试发现有这样的问题后期可以考虑
UINT8  port_mon[5];

//-进行串口监视,如果出错的话,重新初始化
void UART_Monitor(void)	//-10S调用一次检测程序
{
		//-SCI0和7620通讯
		port_mon[0]++;
		//-SCI1和射灯板通讯
		port_mon[1]++;
		//-PM2.5接收传感器数据
		port_mon[2]++;
		if(port_mon[2] > 100)
		{

			 port_mon[2] = 0;
		}
}



