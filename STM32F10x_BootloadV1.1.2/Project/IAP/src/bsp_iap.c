/*
	�Եײ��֧��
*/
#include "stm32f10x_it.h"

//#include "stm32f10x_flash.h"


#define ADC1_DR_Address    ((u32)0x4001244C)
#define USART1_DR_Address    ((u32)0x40013804)



/*
0x08000000 ~ 0x0807FFFF
���ڴ�������Ʒ���䱻����Ϊ 256 ҳ��ÿҳ 2K �ֽڡ�ע�⣬С��������������Ʒ��ÿҳֻ�� 1K �ֽڡ�

��ִ������д����ʱ���κζ�����Ķ�����������ס���ߣ���
д������ɺ������������ȷ�ؽ��У����ڽ���д���������
ʱ�����ܽ��д�������ݵĶ�ȡ������������ÿ�β���֮ǰ��
���Ƕ�Ҫ�ȴ���һ�β��������β������ܿ�ʼ��
*/
#define FLASH_PAGE_SIZE    ((uint16_t)0x800)

#define BANK1_WRITE_START_ADDR  ((UINT32)0x0807F000)
#define BANK1_WRITE_END_ADDR    ((UINT32)0x0807FFFF)




/**
  * @brief  Setup the microcontroller system
  *         Initialize the Embedded Flash Interface, the PLL and update the SystemFrequency variable.
  * @note   This function should be used only after reset.
  * @param  None
  * @retval None
  */
void Iap_SystemInit (void)
{
  ErrorStatus HSEStartUpStatus;
  /* Reset the RCC clock configuration to the default reset state(for debug purpose) */
  //-/* Set HSION bit */
  RCC->CR |= (uint32_t)0x00000001;  //-ʹ���ڲ�8MHz��������
  /* Reset SW[1:0], HPRE[3:0], PPRE1[2:0], PPRE2[2:0], ADCPRE[1:0] and MCO[2:0] bits */
  RCC->CFGR &= (u32)0xF8FF0000;
  /* Reset HSEON, CSSON and PLLON bits */
  RCC->CR &= (u32)0xFEF6FFFF;
  /* Reset HSEBYP bit */
  RCC->CR &= (u32)0xFFFBFFFF;
  /* Reset PLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE bits */
  RCC->CFGR &= (u32)0xFF80FFFF;
  /* Disable all interrupts */
  RCC->CIR = 0x00000000;		//-����������ʱ���Ǹ�λʹ�õ�,Ϊ�˺�������޸�ʱ������
  
  //-�������ʵ��Ҫ������ʱ��
  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);  //-ʹ���ⲿ���پ���
  
  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();
  
  if(HSEStartUpStatus == SUCCESS)
  {
  	/* HCLK = SYSCLK */ //- AHB Ԥ��Ƶ,AHBʱ�ӵ�Ԥ��Ƶϵ��Ϊ1
    RCC_HCLKConfig(RCC_SYSCLK_Div1);  //-��ϵͳʱ��ȷ��֮��,�����ܶ�ʱ�ӿ��Զ�������
    
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);  //-number of wait states for a read operation programmed on-the-fly
    
    /* PCLK2 = HCLK */  //-����APBԤ��Ƶ(APB2),���Ƹ���APB2ʱ��(PCLK2) ��Ԥ��Ƶϵ��Ϊ1 
    RCC_PCLK2Config(RCC_HCLK_Div1);
    
    /* PCLK1 = HCLK/2 */  //-����APBԤ��Ƶ(APB1),���Ƶ���APB1ʱ��(PCLK1) ��Ԥ��Ƶϵ��Ϊ 2��Ƶ 
    RCC_PCLK1Config(RCC_HCLK_Div2);
    
    /* PLLCLK = 4MHz * 9 = 36 MHz */  //-HSEʱ����ΪPLL ����ʱ��,PLL 7��Ƶ���
    //-RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);  //-��ϵͳ��8*9= 72 MHz
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_7);  //-��ϵͳ��8*7= 56 MHz
    
    /* Enable PLL */
    RCC_PLLCmd(ENABLE);
    
    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)  //-������ܳ�����ѭ��
    {
    }
    
    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);  //-��������ѡ����PLL��Ϊϵͳʱ��

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)  //-��ʵ���Ƕ�ȡ״̬�Ĵ���ֵ
    {
    }
  }
  
}


