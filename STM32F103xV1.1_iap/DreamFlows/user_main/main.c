#define GLOBALS
#include "stm32f10x.h"
#include "usb_lib.h"
#include "usb_pwr.h"
#include "stdio.h"
#include "demo.h"


//-#include "integer.h"
//-#include "ff.h"
//-#include "diskio.h"
//-#include "misc.h"
//-#include "es705_escore.h"

#include "user_conf.h"


extern void CDT9702_Main();
extern void RCC_Configuration(void);
extern void GPIO_Configuration(void);
extern void NVIC_Configuration(void);
extern void EXTI_Configuration(void);
extern void uart_config(void);
extern void TIM_config(void);
extern void adc_config(void);
extern void init_system_tick(void);
extern void UART1_Init(void);
extern void UART3_Init(void);
extern void HRL_init(void);
extern void sys_init(void);
extern void sys_test(void);
extern void PM_Process(void);
extern void VZ89_Read_VOC(void);
extern void HRL_sub(void);
extern void LDISP_Main();
extern void FAN_sub(void);
extern void sys_delay_judge(void);





/* Private functions ---------------------------------------------------------*/


int main(void)
{

  RCC_Configuration();
  GPIO_Configuration();
  NVIC_Configuration();

  EXTI_Configuration();
  uart_config();
  TIM_config();
  adc_config();


  GPIO_ResetBits(GPIOA, GPIO_Pin_8);    //-��λ����оƬ
  //-printf("\r\nSmall module init start\n");

  //-Usart_es704_Init();

     /* Setup SysTick Timer for 1 msec interrupts  */
  init_system_tick();

  //-msleep(20);
  GPIO_SetBits(GPIOA, GPIO_Pin_8);

  //-printf("\r\nSPI_Flash_Init\n");   //-��ӡ��Ϣ��Ϊ�����õ����տ���ȥ��
  //-SPI_Flash_Init();

//-ϵͳ������ʼ��
  UART1_Init();
  UART3_Init();
  HRL_init();
  sys_init();		//-��ϵͳ��Ҫ�ı������г�ʼ��,����Ӧ�ò�

  sys_test();

  //-es705_interface();

  //-��������֮��ſ��Կ���������Ҫ��Դ������
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  IWDG_Enable(); //-ʹ�ܿ��Ź�

  for( ; ;)
  {
    //-��������
		//-es705_int_deal();

    //-PM2.5
    PM_Process(); 	//-����ֵ����PM2.5

    //-VOC
    if(Judge_Time_In_MainLoop(CO2_poll_wait_time,2000)==YES)
    {
       CO2_poll_wait_time = cticks_ms;
       voc_rd_flag = 0x55;
       VZ89_Read_VOC();   //-һ�β���10ms
       voc_rd_flag = 0;
	   get_ntc_adc_value();
    }

    //-�����
    HRL_sub();

    //-��������Э��
    CDT9702_Main();
    //-������ư�Э�鴮��3
    LDISP_Main();

    FAN_sub();

    sys_delay_judge();

    /* Reload IWDG counter */
    IWDG_ReloadCounter();    //-���Ź��Ŀ�����Ҫ����,����ѧϰ֮���ռ�ô���ʱ��

  }

}


