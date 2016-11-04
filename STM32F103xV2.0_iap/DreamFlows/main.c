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
�޸ĵ�ֻʣ��һ����ʱ�ж�,����ȫ�����ò�ѯ�ķ�ʽ 7/30/2012 10:24 PM
�Ȱ�һ��UART��ͨ����
������Ҫ���ľ���ADC(���ñ����ת������)
����7·ģ����(�����ѹ������һ���������),���β���
ϵͳ��Ҫ����
�ⲿ8M����,��ϵͳ��8*7= 56 MHz.
I2C��4M

2015/5/28 14:33:13
Ӧ��Ϊ����֮�����̵���ư�,���õ���оƬ��STM32F103RET6;���뻷����IAR FOR ARM 7.20.

��������:
������.4����������.1��������ٶ�.PWM���.����1������STM32ͨѶ.

*/

/* Includes ------------------------------------------------------------------*/
#include "user_conf.h"

/* Local includes ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

//-�ⲿ����
extern void UART1_Init(void);
extern void RCC_Configuration(void);
extern void GPIO_Configuration(void);
extern void NVIC_Configuration(void);
extern void commonIT_Config(void);
extern void uart_config(void);
extern void adc_config(void);
extern void SysTick_Config(void);
extern void led_display_deal(void);   //-���ڸı���Ҫ��ʾ������
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
  RCC_Configuration();  //-����ʹ��������ʱ��

  /* GPIO ports pins Configuration */
  GPIO_Configuration();

  /* NVIC Configuration */
  NVIC_Configuration(); //-�������������ж����ȼ�,������ʹ������

  TIM_config();

#ifdef I2C_HARDWARE_FLAG
  I2C_Init_sub();
#endif

  //-commonIT_Config();
  uart_config();

  //-sample_Init();
  // Configure the systick
  SysTick_Config(); //-��������ڿ�����Ψһ�ж�
  //-���������STM32Ӳ�����ó�ʼ��,����������ⲿ������

  //-����һ����ʱ��֤������豸�ϵ��ȶ��������Ӧ����
  Host_LowLevelDelay(500);


  STM32_UP_selfT_flag1 = 0;		//-�Լ��ʶλ����,׼����ʼ�Լ�
  sys_test();		//-������������Ĳ��Գ���,������,���ﵽ��Ϳ�����

  sys_init();

  //-������ٶ�
  ADXL345_init();		//-һ�γ�ʼ�����ɹ�,��Ҫ�����ٴγ�ʼ��,�޷��ɹ��������ʾ
  if((STM32_UP_selfT_flag1 & 0x01) != 0)
  	ADXL345_init();

  //-����
  //-initBH1750FVI();
  //-if((STM32_UP_selfT_flag1 & 0x02) != 0)
  //-	initBH1750FVI();

  //-ADI����
  initADUX1020();
  if((STM32_UP_selfT_flag1 & 0x04) != 0)
  	initADUX1020();
  if((STM32_UP_selfT_flag1 & 0x04) != 0)
  	initADUX1020();
  if((STM32_UP_selfT_flag1 & 0x04) != 0)
  	initADUX1020();

   //-��Ƭ��Flash��ģ��EEPROM
   //-EEPROM���԰�λ��д����Flashֻ�ܰ��飨ҳ������
   //-��ʼ��
   //-MONI_EEPROM_init();

   //-������
   led_display_init();


  //-��������������ĳ�ʼ��
   UART1_Init();
   sys_init();		//-��ϵͳ��Ҫ�ı������г�ʼ��,����Ӧ�ò�

   commonIT_Config();
   //-����������Ҫ����У׼,������ռ�ô���ʱ��,���ԾͲ���ʹ�ÿ��Ź�
   IWDG_Enable(); //-ʹ�ܿ��Ź�


   //-printf("\r\nsys run is ok\n");

   while(1)
   {
   	  test_cn_wait_time = cticks_ms;	//-������
   	  if(led_display_deal_flag != 0)
   	  {
	      //-������׼��һҳ����,׼����֮���ñ�־λ,��ˢ���ӳ����ͳ���ʾ
	      led_display_deal();   //-���ڸı���Ҫ��ʾ������
	      led_display_var();		//-������ת��Ϊ��ʾ����,��TIM2�лᶨʱˢƵ
	      led_display_deal_flag = 0x55;
    	}


      //-��������
      //-Ŀǰû��ʹ���жϵķ�ʽ,���ڲ��õ��Ƕ�����Ȼ���������
      //-����һֱ���ǲ�ѯ��ʽ,��ô�ж�����ȥ��,��ֹ��Ч�жϲ���
      //-����Ĵ�������ֵ��˼��
      //-ADUX1020_sub();
      if((ADI_ps_flag == 0) && (es705_training_flag != 0x55) && (led_display_flag != 0))
      {//-���ڴ��ڿ���״̬,���Խ�������ʶ��
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


      //-������
      //-ADXL345_sub();
      //-if(Judge_Time_In_MainLoop(VCNL_poll_wait_time,1000)==YES)
      //-{
      //-  VCNL_poll_wait_time = cticks_ms;
      //-
      //-  //-getLux(0x10);    //-�����ȴ�ʱ�����Ӱ������,�����Ҫʹ�ù����Ļ�,���ܲ������ȵķ�ʽ
      //-}


      //-����1������STM32ͨѶ
      CDT9702_Main();


      sys_delay_judge();	//-������ͨѶ����֮�����

      if(test_cn_wait_time < cticks_ms)   //-������
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

      //-�����еĶ��������֮��,��ֹ�����ܷɵĿ��Ź��Ǳ����,������ʵ�ʲ��Խ׶β�����,����ܶ�����ܷ���
      //-����ʹ�ö������Ź����п�,��ʱ��Ҫ������ô�ϸ�
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
