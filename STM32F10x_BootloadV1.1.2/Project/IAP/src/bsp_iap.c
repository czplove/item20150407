/*
	对底层的支持
*/
#include "stm32f10x_it.h"

//#include "stm32f10x_flash.h"


#define ADC1_DR_Address    ((u32)0x4001244C)
#define USART1_DR_Address    ((u32)0x40013804)



/*
0x08000000 ~ 0x0807FFFF
对于大容量产品，其被划分为 256 页，每页 2K 字节。注意，小容量和中容量产品则每页只有 1K 字节。

在执行闪存写操作时，任何对闪存的读操作都会锁住总线，在
写操作完成后读操作才能正确地进行；既在进行写或擦除操作
时，不能进行代码或数据的读取操作。所以在每次操作之前，
我们都要等待上一次操作完成这次操作才能开始。
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
  RCC->CR |= (uint32_t)0x00000001;  //-使用内部8MHz振荡器开启
  /* Reset SW[1:0], HPRE[3:0], PPRE1[2:0], PPRE2[2:0], ADCPRE[1:0] and MCO[2:0] bits */
  RCC->CFGR &= (u32)0xF8FF0000;
  /* Reset HSEON, CSSON and PLLON bits */
  RCC->CR &= (u32)0xFEF6FFFF;
  /* Reset HSEBYP bit */
  RCC->CR &= (u32)0xFFFBFFFF;
  /* Reset PLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE bits */
  RCC->CFGR &= (u32)0xFF80FFFF;
  /* Disable all interrupts */
  RCC->CIR = 0x00000000;		//-到这里配置时钟是复位使用的,为了后面可以修改时钟配置
  
  //-下面根据实际要求配置时钟
  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);  //-使能外部高速晶振
  
  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();
  
  if(HSEStartUpStatus == SUCCESS)
  {
  	/* HCLK = SYSCLK */ //- AHB 预分频,AHB时钟的预分频系数为1
    RCC_HCLKConfig(RCC_SYSCLK_Div1);  //-当系统时钟确定之后,其它很多时钟可以独立配置
    
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);  //-number of wait states for a read operation programmed on-the-fly
    
    /* PCLK2 = HCLK */  //-高速APB预分频(APB2),控制高速APB2时钟(PCLK2) 的预分频系数为1 
    RCC_PCLK2Config(RCC_HCLK_Div1);
    
    /* PCLK1 = HCLK/2 */  //-低速APB预分频(APB1),控制低速APB1时钟(PCLK1) 的预分频系数为 2分频 
    RCC_PCLK1Config(RCC_HCLK_Div2);
    
    /* PLLCLK = 4MHz * 9 = 36 MHz */  //-HSE时钟作为PLL 输入时钟,PLL 7倍频输出
    //-RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);  //-本系统是8*9= 72 MHz
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_7);  //-本系统是8*7= 56 MHz
    
    /* Enable PLL */
    RCC_PLLCmd(ENABLE);
    
    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)  //-这里可能出现死循环
    {
    }
    
    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);  //-仅仅就是选择了PLL作为系统时钟

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)  //-其实就是读取状态寄存器值
    {
    }
  }
  
}


