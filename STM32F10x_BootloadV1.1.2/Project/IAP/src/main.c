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
��������õĹ�������ֲ���Լ���ϵͳ��,���ҳ�Ϊ���õ�

2016/1/13 13:31:42 by zj
��ʽ��ʼΪʵ��ʹ�ñ�д�򵥵�IAP����,��ʵ���ض��������ܾ���,����Ҫ֧��ymodemЭ��.

2016/1/27 09:31:42 by zj
STM32���1
һ���ڽ�������֮ǰ�������˽�һЩ��Ҫ�Ļ���֪ʶ----stm32ϵ��оƬ��������ͺţ�
startup_stm32f10x_cl.s �����͵�������STM32F105xx��STM32F107xx
startup_stm32f10x_hd.s ��������STM32F101xx��STM32F102xx��STM32F103xx
startup_stm32f10x_hd_vl.s ��������STM32F100xx
startup_stm32f10x_ld.s С������STM32F101xx��STM32F102xx��STM32F103xx
startup_stm32f10x_ld_vl.s С������STM32F100xx
startup_stm32f10x_md.s ��������STM32F101xx��STM32F102xx��STM32F103xx
startup_stm32f10x_md_vl.s ��������STM32F100xx  ������Ŀ���õ��Ǵ˿�оƬ stm32f100CB��
startup_stm32f10x_xl.s FLASH��512K��1024K�ֽڵ�STM32F101xx��STM32F102xx��STM32F103xx

cl�������Ͳ�Ʒ��stm32f105/107ϵ��
vl����ֵ�Ͳ�Ʒ��stm32f100ϵ��
xl�������ܶȲ�Ʒ��stm32f101/103ϵ��
ld�����ܶȲ�Ʒ��FLASHС��64K
md���е��ܶȲ�Ʒ��FLASH=64 or 128
hd�����ܶȲ�Ʒ��FLASH����128

Ŀǰʵ���˳����ĳ�������,������Ҫ��������������.

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
  STM_EVAL_bspInit(); //-��ʼ��һ��debug�ڳ���,���п���

  UART1_Init();

  //-IAP������ʼ��һ�����Դ���,�����������,��һֱ���FLASH��һ����־λ,�����������
  //-���˾�������������,û�еĻ�������boot����
	//?Ŀǰ�ݲ�����APP1��APP0���Ƴ�������
  while (1)
  {
    if(port_deal_flag[0] == 0)
    {
      if(*(__IO uint16_t*)(APP_CONFIG_ADDR) == 0x5555) //-���ض���ַ�ı�־λ,�ж��Ƿ��г����ִ��
      {
      //-	//ֱ����ת��APP
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
      if(*(__IO uint16_t*)(APP_CONFIG1_ADDR) == 0x5555) //-���ض���ַ�ı�־λ,�жϱ������������Ƿ�OK,�õĻ��͸��Ƶ�������
      {
          copy_app1_to_app0(*(__IO uint32_t*)(APP_CONFIG2_ADDR));
      }
    }

    CommandScan();  //-���ڴ�����
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
