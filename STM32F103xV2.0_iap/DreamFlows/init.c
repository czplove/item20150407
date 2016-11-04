/*

*/
#include "user_conf.h"


/*
            T16					//-					PIN_22
						T15					//- 				PIN_20
T8					T14					//-PIN_19 	PIN_11
T7					T13  				//-输入  		PIN_16
T6										  //-PIN_13		GND
T5					T12					//-PIN_18		PIN_12
T4					T11					//-PIN_09   PIN_10
T3					T10					//-PIN_07		PIN_08
T2					T9					//-PIN_05   PIN_06
T21					T22					//-V3.3		  GND

注:
T1	(GND)
T21接3.3V
T22接地
基本是一一对应的由于有缺失T16和T14连接到T8上
T15连接到T6上


3.3v好的话 D1亮

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

    led_display_long = 60;    //-虽然后面会修改这个数值但是为了防读失败还是给一个默认值
    voice_flag = 0x55;

    i2c1_bus_error_cn = 0;
    i2c2_bus_error_cn = 0;
    rgb_num = 7;	//-等于0 就一直是灭灯,等于其他可以周期旋转

    HL_run_time_delay = 1800;

    HL_RED_pwmval = (PWM_Period_Value * 0) / 255;   //-红
    HL_GREEN_pwmval = (PWM_Period_Value * 0) / 255;		//-绿
    HL_BLUE_pwmval = (PWM_Period_Value * 0) / 255;	//-蓝
    HL_new_value_flag = 1;
    HL_flag = 0;
    cticks_ms_pwm_loop = 5000;

    RED_pwmval_num = 3;		//-一个占空比保持的次数,也就是时间
    GREEN_pwmval_num = 3;
    BLUE_pwmval_num = 3;		//-8 几乎是1S中闪一次
    white_pwmval_num = 3;

    RED_pwmval_pt = 0;
    GREEN_pwmval_pt = 0;
    BLUE_pwmval_pt = 0;
    white_pwmval_pt = 0;

    //-固定模式的值
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

    //-用户设定颜色初始默认为绿色
    HL_ld_R_user[0] = 0;
    HL_ld_G_user[0] = 0;
    HL_ld_B_user[0] = 255;

    //-敲击设定灵敏值
    ADXL_THRESH_TAP = 0x30;   //-0x20最小稳定值 可靠最小值为0x30 声音播报时最大0x70可可靠保证不误触发,那么区间64
    ADXL_DUR = 0x70;      //-0x30 ,貌似0x15的时候误触发严重

    EEP_Data_flag = 0;
    //-调入模拟FLASH中的定义,每次上电后
    res = MONI_EEPROM_read();

    if(res == 1)
    	MONI_EEPROM_sub();

    //-//-测试修改定值
    //-EEP_Data_flag = 0x55;
    //-led_display_long = 30;

    //-PM2.5 当没有有效值进行更新时使用下面的值
    pm_data = 33;

    if(*(uint16_t*)(APP_CONFIG3_ADDR) != STM32_SYS_VERSION_NUM)
    {//-基本系统版本号
        FLASH_Unlock();						//解锁
				FLASH_ErasePage(APP_CONFIG3_ADDR);//擦除这个扇区
				FLASH_ProgramHalfWord(APP_CONFIG3_ADDR,STM32_SYS_VERSION_NUM);	//-不能运行可以运行
				FLASH_Lock();//上锁
    }
}

void sys_test(void)   //-仅仅支持射灯板的产测,不支持点阵屏产测
{
   char Test_flag;

	 //-以上进行一切正常初始化,下面判断是否进入测试程序
   Test_flag = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12);
   while(Test_flag == 0)		//-点阵屏需要跳出去才可以检测,射灯板停留在这里就可以.
   {//-进入测试状态
   	  HL_new_value_flag = 1;
   	  HL_RED_pwmval = (PWM_Period_Value * 10) / 255;   //-红
    	HL_GREEN_pwmval = (PWM_Period_Value * 0) / 255;		//-绿
    	HL_BLUE_pwmval = (PWM_Period_Value * 0) / 255;	//-蓝

    	Host_LowLevelDelay(500);
    	HL_new_value_flag = 1;
    	HL_RED_pwmval = (PWM_Period_Value * 0) / 255;   //-红
    	HL_GREEN_pwmval = (PWM_Period_Value * 0) / 255;		//-绿
    	HL_BLUE_pwmval = (PWM_Period_Value * 10) / 255;	//-蓝

    	Host_LowLevelDelay(500);
    	HL_new_value_flag = 1;
    	HL_RED_pwmval = (PWM_Period_Value * 0) / 255;   //-红
    	HL_GREEN_pwmval = (PWM_Period_Value * 10) / 255;		//-绿
    	HL_BLUE_pwmval = (PWM_Period_Value * 0) / 255;	//-蓝

      Host_LowLevelDelay(500);
      HL_new_value_flag = 1;
   	  HL_RED_pwmval = (PWM_Period_Value * 10) / 255;   //-红
    	HL_GREEN_pwmval = (PWM_Period_Value * 0) / 255;		//-绿
    	HL_BLUE_pwmval = (PWM_Period_Value * 0) / 255;	//-蓝

    	Host_LowLevelDelay(500);
    	HL_new_value_flag = 1;
    	HL_RED_pwmval = (PWM_Period_Value * 0) / 255;   //-红
    	HL_GREEN_pwmval = (PWM_Period_Value * 0) / 255;		//-绿
    	HL_BLUE_pwmval = (PWM_Period_Value * 10) / 255;	//-蓝

    	Host_LowLevelDelay(500);
    	HL_new_value_flag = 1;
    	HL_RED_pwmval = (PWM_Period_Value * 0) / 255;   //-红
    	HL_GREEN_pwmval = (PWM_Period_Value * 10) / 255;		//-绿
    	HL_BLUE_pwmval = (PWM_Period_Value * 0) / 255;	//-蓝

    	Host_LowLevelDelay(500);
    	HL_new_value_flag = 1;
    	HL_RED_pwmval = (PWM_Period_Value * 0) / 255;   //-红
    	HL_GREEN_pwmval = (PWM_Period_Value * 0) / 255;		//-绿
    	HL_BLUE_pwmval = (PWM_Period_Value * 0) / 255;	//-蓝

    	//-点阵屏全亮
      led_display_long = 255;
      led_display_start = 0x55;
    	led_display_flag = 1;

    	//-串口测试,外部自发自收
    	if(port_send_len[0] == 0)
    	{
    		  port_send[0][0] = 0xaa;
    			port_send_len[0] = 1;
    	}
    	Host_LowLevelDelay(1000);
    	printf("\r\nport_recv_pt[0] is %d\n",port_recv_pt[0]);	//-判断发送出去的内容有没有收到,收到的话计数在增加

    	STM32_UP_selfT_flag1 = 0;

    	//-第一步测试28PIN引脚
   		GPIO_Configuration_out();

		   GPIO_Configuration_in();

		   //-首先全部输出高电平
		   //-检测所有的输入端是否都是高电平
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
		   //-检测所有的输入端是否都是高电平
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

         HL_RED_pwmval = (PWM_Period_Value * 10) / 255;   //-红
         HL_GREEN_pwmval = (PWM_Period_Value * 10) / 255;		//-绿
         HL_BLUE_pwmval = (PWM_Period_Value * 10) / 255;	//-蓝
      }

      //-所有可以通讯的,初始化通过就置标志位,否则认为有异常
    	printf("\r\nSTM32_UP_selfT_flag1 is %d\n",STM32_UP_selfT_flag1);

   	  Test_flag = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12);
   }
}


void sys_delay_judge(void)
{
	  struct i2c_client client;
	  int Result;
	  char Test_flag;

	  //-结束开机动画判断
	  if((led_display_flag == 0) && (Judge_LongTime_In_MainLoop(cartoon_start_time,88)==YES))		//-开机延时大约3分钟
	  {
	  	 led_display_new = 0x55;
	  	 led_display_flag = 3;
	  }

	 //-超长时间检测传感器状态
	  if(Judge_LongTime_In_MainLoop(TEST_Sensor_wait_time,10)==YES)
	  {
	  	 TEST_Sensor_wait_time = Time_2048ms_Counter;	//-每检测一次,就返回状态值,如果不好的话就进行初始化,如果累计几次不成功就报错
	  	 											//-具体实施到时候再说,测试的时候不考虑重新初始化,这样可能发现不了突发问题
	  }

	  //-超长时间检测传感器状态
	  if((Judge_LongTime_In_MainLoop(led_display_end_time,30)==YES) && (led_display_start == 0xaa) && (UART1_transmit_control == 0))
	  {
	  	 led_display_start = 0;		//-点阵屏熄灭一段时间后彻底熄灭,如果这里反复刷值的话,就会覆盖其它数据,出现报文丢失的情况
	  	 TEST_Sensor_wait_time = Time_2048ms_Counter;
			 UART1_transmit_flag = YES;		//-可以组织内容发送
	     UART1_transmit_control = 2;	//-决定了主动发送的内容
	  }

	  //-数据库更新时间
	  if((Judge_LongTime_In_MainLoop(UART1_renew_wait_time,7)==YES) && (UART1_renew_flag == 0x55))
	  {
	  	 UART1_renew_wait_time = Time_2048ms_Counter;

	     UART1_renew_flag = 0;
	  }

	  if((Judge_Time_In_MainLoop(Sensor_data_wait_time,5000)==YES) /*&& (UART1_transmit_control == 0)*/)	//-只要是周期性判断的都可以等待空闲
	  {
	  	 if(led_display_data_flag == 1)	//-1表示更新数据开始计时,对于停留在这一页才有必要开始计时
	  	 {
	  	 	  if((led_display_page == 0) || (led_display_page == 3))
	  	 	  {
			  	 		Sensor_data_wait_time = cticks_ms;
			  	 		led_display_data_flag = 2;
			  	 		led_display_new = 0x55;
			  	 		led_display_flag = 3;		//-数据更新好了,切换到正常显示

			  	 		//-启动风扇,延后了,但是为了播报同步先这么做
			  	 		UART1_transmit_flag = YES;		//-可以组织内容发送
		          UART1_transmit_control = 1;	//-决定了主动发送的内容	?这里有可能出现错乱的情况如果先上报,后显示的话,恢复为了0,就不冻结了?
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

	  if((Judge_LongTime_In_MainLoop(HL_run_time,HL_run_time_delay)==YES) && (HL_flag != 0))		//-关闭炫彩灯计时
	  {
	  	 HL_RED_pwmval = 0;
		 	 HL_GREEN_pwmval = 0;
		 	 HL_BLUE_pwmval = 0;
	  	 HL_new_value_flag = 1;
	  	 HL_flag = 0;

	  	 UART1_transmit_flag = YES;		//-可以组织内容发送
	  	 UART1_transmit_control = 7;		//-更新下炫彩灯模式
	  }

	  if(ADXL_TAP_it_flag == 1)
	  {
      if(Judge_Time_In_MainLoop(ADXL_TAP_wait_time,400)==YES)	//-这里的延时不是为了防抖,而是为了人为控制两次敲击的最大延时
	    {
	       client.addr = 0xA6;
	       client.num = 1;		//-表示I2C1口驱动

	       //-测试用
	       //-i2c_read_reg(&client,0X2B,&voice_flag);

	       Result = i2c_read_reg(&client,0x30,&ADXL_TAP_it_SOURCE);		//-中断源,读取数据可以清除中断信号
	       if(Result <= 0)
         {
            ADXL_TAP_it_SOURCE = 0xff;
            //-return;		//-由于后面还有需要处理的,这里的不能结束程序让其他内容没法执行
         }
         if((ADXL_TAP_it_SOURCE & 0x40) == 0)
	       {//-到这里说明单击事件成立了
	       	  //-检测电平保证中断确实结束了而不是由于通讯异常导致中断没有结束
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

    //-周期性的检查手势中断信号线上是否有方波出现如果持续一段时间没有的话,就认为手势失效了
    //-进行重新初始化
    Result = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12);
    if(Result == 0)
       ADI_ps_H_time = cticks_ms;   //-只要出现低电平就认为喂狗成功,然后重新计时,长时间保持高电平
                                    //-就认为失效,需要重新初始化
    if(Judge_Time_In_MainLoop(ADI_ps_H_time,50000)==YES)
    {
        ADI_ps_H_time = cticks_ms;
        initADUX1020();		//-热启动初始化,校准之后的数据由于环境没有发生变化,所以不需要重新校准,已经校准的数据就可以了
        if((STM32_UP_selfT_flag1 & 0x04) != 0)
  				initADUX1020();
        ADI_ps_flag = 0x55;	//-复位之后一段时间内不检测手势
        ADI_ps_wait_time = cticks_ms;
    }

    //-
    if((Judge_LongTime_In_MainLoop(ADI_channels_wait_time,44)==YES)	&& (ADI_channels_init_flag == 0x55))	//-时长强制复位
	  {
	  	 ADI_channels_wait_time = Time_2048ms_Counter;
	  	 initADUX1020();
	  	 if((STM32_UP_selfT_flag1 & 0x04) != 0)
  				initADUX1020();
	  	 ADI_channels_init_flag = 0;		//-除非出现了敲击否则不再重新校准
	  	 ADI_ps_flag = 0x55;	//-复位之后一段时间内不检测手势
       ADI_ps_wait_time = cticks_ms;
	  }

	  //-测试用
     Test_flag = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12);
	   if(Test_flag == 0)
	   {
	    //-ADUX1020_Algorithm_Init();
	    initADUX1020();
	    if((STM32_UP_selfT_flag1 & 0x04) != 0)
	  			initADUX1020();
	   }


	 //-I2C总线出错检查,,这里的重新初始化不一定好使,但是错误判断机制是对的,修改内容可以变化
      if(i2c1_bus_error_cn > 20)	//-现在使用推挽输出应该不存在总线死锁的情况,即使死锁了也是没有办法的
      {
      	 i2c1_bus_error_cn = 0;
      	 //-重新初始化,解决可能的钳位问题,需要测试

      }

      //-错误处理
      if(((STM32_UP_selfT_flag1 & 0x01) != 0)/* && ((STM32_UP_error_flag1 & 0x01) == 0)*/)	//-只要有错误发生就进行重新初始化直到正确为止
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
      if(EEP_Data_flag == 0x55)		//-测试下亮屏时修改定值是否影响显示
      	MONI_EEPROM_sub();	//-屏幕熄灭了才会修改可能改变的定值
}