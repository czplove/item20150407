/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : main.c
* Author             : David JIANG
* Date First Issued  : 2007.9.17
* Description        : Main program body
********************************************************************************
* History:
* 2007.9.17: V1.0
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/*
修改的只剩下一个定时中断,其他全部采用查询的方式 7/30/2012 10:24 PM
先把一个UART口通起来
下面需要做的就是ADC(采用本身的转换外设)
采样7路模拟量(三相电压电流和一相零序电流),单次采样
系统主要参数
外部8M晶振,本系统是8*7= 56 MHz.
I2C是4M

2015/5/28 14:33:13
应用为梦想之花工程的射灯板,采用的主芯片是STM32F103RET6;编译环境是IAR FOR ARM 7.20.

功能描述:
点阵屏.4个光照手势.1个三轴加速度.PWM射灯.串口1和主板STM32通讯.

*/

/* Includes ------------------------------------------------------------------*/
#include "user_conf.h"

/* Local includes ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

//-外部函数
extern void UART1_Init(void);
extern void RCC_Configuration(void);
extern void GPIO_Configuration(void);
extern void NVIC_Configuration(void);
extern void commonIT_Config(void);
extern void uart_config(void);
extern void adc_config(void);
extern void SysTick_Config(void);
extern void led_display_deal(void);   //-用于改变需要显示的数据
extern void led_display_var(void);
extern void TIM_config(void);
extern void ADXL345_init(void);
extern void initADUX1020(void);
extern void sys_init(void);
extern void sys_test(void);
extern void huelight_sub(void);
extern void ADUX1020_sub2(void);
extern void CDT9702_Main();
extern void ADXL345_sub(void);
extern void sys_delay_judge(void);


/* Private functions ---------------------------------------------------------*/
void USART_string(void);
void led_display_init(void);
void I2C_Init_sub(void);
void light_sense_init(u8 port_num, u8 * flag);
int light_sense_sub(void);


/*******************************************************************************
* Function Name  : main
* Description    : Main program
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
{
	static BYTE i;

#ifdef DEBUG
  debug();
#endif

  /* System Clocks Configuration */
  RCC_Configuration();  //-包括使能了外设时钟

  /* GPIO ports pins Configuration */
  GPIO_Configuration();

  /* NVIC Configuration */
  NVIC_Configuration(); //-仅仅是配置了中断优先级,后面再使能外设

  TIM_config();

#ifdef I2C_HARDWARE_FLAG
  I2C_Init_sub();
#endif

  //-commonIT_Config();
  uart_config();

  //-sample_Init();
  // Configure the systick
  SysTick_Config(); //-本程序段内开放了唯一中断
  //-以上完成了STM32硬件配置初始化,下面是针对外部器件的

  //-增加一段延时保证下面的设备上电稳定后进行相应操作
  Host_LowLevelDelay(500);


  STM32_UP_selfT_flag1 = 0;		//-自检标识位清零,准备开始自检
  sys_test();		//-点阵屏是另外的测试程序,独立的,这里到这就可以了

  sys_init();

  //-三轴加速度
  ADXL345_init();		//-一次初始化不成功,需要考虑再次初始化,无法成功将输出显示
  if((STM32_UP_selfT_flag1 & 0x01) != 0)
  	ADXL345_init();

  //-光敏
  //-initBH1750FVI();
  //-if((STM32_UP_selfT_flag1 & 0x02) != 0)
  //-	initBH1750FVI();

  //-ADI手势
  initADUX1020();
  if((STM32_UP_selfT_flag1 & 0x04) != 0)
  	initADUX1020();
  if((STM32_UP_selfT_flag1 & 0x04) != 0)
  	initADUX1020();
  if((STM32_UP_selfT_flag1 & 0x04) != 0)
  	initADUX1020();

   //-用片上Flash来模拟EEPROM
   //-EEPROM可以按位擦写，而Flash只能按块（页）擦除
   //-初始化
   //-MONI_EEPROM_init();

   //-点阵屏
   led_display_init();


  //-下面是软件变量的初始化
   UART1_Init();
   sys_init();		//-把系统需要的变量进行初始化,属于应用层

   commonIT_Config();
   //-由于手势需要重新校准,这样会占用大量时间,所以就不能使用看门狗
   IWDG_Enable(); //-使能看门狗


   //-printf("\r\nsys run is ok\n");

   while(1)
   {
   	  test_cn_wait_time = cticks_ms;	//-测试用
   	  if(led_display_deal_flag != 0)
   	  {
	      //-点阵屏准备一页数据,准备好之后置标志位,让刷屏子程序送出显示
	      led_display_deal();   //-用于改变需要显示的数据
	      led_display_var();		//-把数据转化为显示编码,在TIM2中会定时刷频
	      led_display_deal_flag = 0x55;
    	}


      //-光照手势
      //-目前没有使用中断的方式,由于采用的是读数据然后分析方向
      //-所以一直都是查询方式,那么中断引脚去掉,防止无效中断产生
      //-这里的错误重启值得思考
      //-ADUX1020_sub();
      if((ADI_ps_flag == 0) && (es705_training_flag != 0x55) && (led_display_flag != 0))
      {//-现在处于空闲状态,可以进行手势识别
      	 ADUX1020_sub2();
    	}
    	else
    	{
    		 if(Judge_Time_In_MainLoop(ADI_ps_wait_time,500)==YES)
    		 {
    		 	  ADI_ps_flag = 0;
    		 }

    		 if(Judge_LongTime_In_MainLoop(es705_training_wait_time,3)==YES)
    		 {
    		 	  led_display_txpic_flag = 0;
    		 	  es705_training_flag = 0;
    		 	  es705_training_count = 0;
    		 }
    	}


      //-测试用
      //-ADXL345_sub();
      //-if(Judge_Time_In_MainLoop(VCNL_poll_wait_time,1000)==YES)
      //-{
      //-  VCNL_poll_wait_time = cticks_ms;
      //-
      //-  //-getLux(0x10);    //-光敏等待时间过长影响手势,如果需要使用光敏的话,不能采用死等的方式
      //-}


      //-串口1和主板STM32通讯
      CDT9702_Main();


      sys_delay_judge();	//-必须在通讯程序之后调用

      if(test_cn_wait_time < cticks_ms)   //-测试用
    	test_cn[i] = cticks_ms - test_cn_wait_time;
    else
    	test_cn[i] = 65536 + cticks_ms - test_cn_wait_time;

    if(test_cn[i] > 400)
      test_cn[7]++;

    if(i<6)
	  {
    	i++;
    }
    else
    	i = 0;

      //-当所有的都处理好了之后,防止程序跑飞的看门狗是必须的,但是在实际测试阶段不能有,否则很多错误不能发现
      //-这里使用独立看门狗就行可,对时间要求不是那么严格
      /* Reload IWDG counter */
      IWDG_ReloadCounter();
   }
}




#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {

	}

}
#endif
/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
