/**
  ******************************************************************************
  * @file    IAP/src/main.c
  * @author  MCD Application Team
  * @version V3.1.0
  * @date    07/27/2009
  * @brief   Main program body
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */

/** @addtogroup IAP
  * @{
  */

/*
2015/7/29 10:38:32 by zj
在这个能用的工程上移植到自己的系统中,并且成为公用的

2016/1/13 13:31:42 by zj
正式开始为实际使用编写简单的IAP程序,先实现特定升级功能就行,不需要支持ymodem协议.

2016/1/27 09:31:42 by zj
STM32详解1
一、在进入主题之前我们先了解一些必要的基础知识----stm32系列芯片的种类和型号：
startup_stm32f10x_cl.s 互联型的器件，STM32F105xx，STM32F107xx
startup_stm32f10x_hd.s 大容量的STM32F101xx，STM32F102xx，STM32F103xx
startup_stm32f10x_hd_vl.s 大容量的STM32F100xx
startup_stm32f10x_ld.s 小容量的STM32F101xx，STM32F102xx，STM32F103xx
startup_stm32f10x_ld_vl.s 小容量的STM32F100xx
startup_stm32f10x_md.s 中容量的STM32F101xx，STM32F102xx，STM32F103xx
startup_stm32f10x_md_vl.s 中容量的STM32F100xx  （我项目中用的是此款芯片 stm32f100CB）
startup_stm32f10x_xl.s FLASH在512K到1024K字节的STM32F101xx，STM32F102xx，STM32F103xx

cl：互联型产品，stm32f105/107系列
vl：超值型产品，stm32f100系列
xl：超高密度产品，stm32f101/103系列
ld：低密度产品，FLASH小于64K
md：中等密度产品，FLASH=64 or 128
hd：高密度产品，FLASH大于128

目前实现了初步的程序升级,下面需要做到升级程序不死.

*/

/* Includes ------------------------------------------------------------------*/
#include "common.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern pFunction Jump_To_Application;
extern uint32_t JumpAddress;

extern void copy_app1_to_app0(unsigned short NumToWrite);

extern void STM_EVAL_bspInit(void);
extern void UART1_Init(void);
extern void CommandScan();
extern void Iap_SystemInit (void);

/* Private function prototypes -----------------------------------------------*/
//-static void IAP_Init(void);

/* Private functions ---------------------------------------------------------*/
//-uint32_t temp_data32;
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  //-/* Flash unlock */
  //-FLASH_Unlock();

  /* Initialize Key Button mounted on STM3210X-EVAL board */
  STM_EVAL_bspInit(); //-初始化一个debug口出来,进行控制

  UART1_Init();

  //-IAP正常初始化一个调试串口,处理各种命令,并一直检查FLASH中一个标志位,如果正常程序
  //-有了就运行正常程序,没有的话就运行boot程序
	//?目前暂不考虑APP1到APP0复制出错的情况
  while (1)
  {
    if(port_deal_flag[0] == 0)
    {
      if(*(__IO uint16_t*)(APP_CONFIG_ADDR) == 0x5555) //-读特定地址的标志位,判断是否有程序可执行
      {
      //-	//直接跳转到APP
        //- Test if user code is programmed starting from address "ApplicationAddress"
        //-temp_data32 = (*(__IO uint32_t*)ApplicationAddress);
        if (((*(__IO uint32_t*)ApplicationAddress) & 0x2FFE0000 ) == 0x20000000)
        { //- Jump to user application
          JumpAddress = *(__IO uint32_t*) (ApplicationAddress + 4);
          Jump_To_Application = (pFunction) JumpAddress;
          //- Initialize user application's Stack Pointer
          __set_MSP(*(__IO uint32_t*) ApplicationAddress);
          Jump_To_Application();
        }
      }
      //-temp_data32 = *(__IO uint16_t*)(APP_CONFIG1_ADDR);
      if(*(__IO uint16_t*)(APP_CONFIG1_ADDR) == 0x5555) //-读特定地址的标志位,判断备份区的内容是否OK,好的话就复制到运行区
      {
          copy_app1_to_app0(*(__IO uint32_t*)(APP_CONFIG2_ADDR));
      }
    }

    CommandScan();  //-周期处理报文
  }
}


#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
