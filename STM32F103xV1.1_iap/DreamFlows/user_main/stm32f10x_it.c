/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
* File Name          : stm32f10x_it.c
* Author             : MCD Application Team
* Version            : V3.1.1
* Date               : 04/07/2010
* Description        : Main Interrupt Service Routines.
*                      This file provides template for all exceptions handler
*                      and peripherals interrupt service routine.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "stm32f10x_exti.h"
#include "usb_lib.h"
#include "usb_istr.h"
#include "usb_pwr.h"
//-#include "escore.h"

#include "user_conf.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

//-extern  u8 RxBuf[RX_BUFF_SIZE];
extern  u8 RXCount;
extern __IO uint32_t irq_flag;


//-extern __IO uint32_t f_rec ;
extern __IO uint8_t rx1_cnt;
extern uint8_t uart1_rx_buff[100];

extern void it_deal(void);
extern void UART_Rx_Deal(void);
extern void UART_Tx_Deal(void);
extern void UART3_Rx_Deal(void);
extern void UART3_Tx_Deal(void);
extern void Noise_Process_it(void);
/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/*******************************************************************************
* Function Name  : NMI_Handler
* Description    : This function handles NMI exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NMI_Handler(void)
{
}

/*******************************************************************************
* Function Name  : HardFault_Handler
* Description    : This function handles Hard Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : MemManage_Handler
* Description    : This function handles Memory Manage exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : BusFault_Handler
* Description    : This function handles Bus Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : UsageFault_Handler
* Description    : This function handles Usage Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : SVC_Handler
* Description    : This function handles SVCall exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVC_Handler(void)
{
}

/*******************************************************************************
* Function Name  : DebugMon_Handler
* Description    : This function handles Debug Monitor exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DebugMon_Handler(void)
{
}

/*******************************************************************************
* Function Name  : PendSV_Handler
* Description    : This function handles PendSVC exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PendSV_Handler(void)
{
}

/*******************************************************************************
* Function Name  : SysTick_Handler
* Description    : This function handles SysTick Handler.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_Handler(void)
{

   //-TimingDelay_Decrement();

   cticks_ms++;

   if((cticks_ms & 0x7ff)==0)
  	Time_2048ms_Counter++;

   it_deal();		//-需要中断内处理的内容放在这里进行

   UART_Rx_Deal();
   UART_Tx_Deal();

   UART3_Rx_Deal();
   UART3_Tx_Deal();
}

void USART3_IRQHandler(void)
{

	//  unsigned int i;
  if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
  {

    uart1_rx_buff[rx1_cnt++] = USART_ReceiveData(USART3);

    if(uart1_rx_buff[rx1_cnt-2]==0x0d&&uart1_rx_buff[rx1_cnt-1]==0x0a)
    {
	  f_rec=1;
	  rx1_cnt=0;

    }
    if(rx1_cnt >= 100)
      rx1_cnt = 0;

  }

  if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
  {
     USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
  }

}

void USART1_IRQHandler(void)
{




  	//-u8 data;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{

		//RxBuf[RXCount++] = USART_ReceiveData(USART2);
		//-data = USART_ReceiveData(USART1);
		//-fifo_put(&data, 1);


	}
}

void EXTI9_5_IRQHandler(void)
{


   if(EXTI_GetITStatus(EXTI_Line9) != RESET)
    {

	irq_flag = 1;

	//printf("\r\n*************************EXTI3_IRQHandler\n");

     	EXTI_ClearFlag(EXTI_Line9);
     	EXTI_ClearITPendingBit(EXTI_Line9);
     }
 }

void DMA1_Channel1_IRQHandler(void)
{
//-	UINT16 x,y;

  if(DMA_GetITStatus(DMA1_IT_HT1))
  {//-每1mS的时间产生一次中断,那么每次中断处理时间必须小于1mS,这样才不会发生中断嵌套
        /* A相采样数据完成 */
        ADC_Conv_flag = TRUE;

        //-测试用
        if(voc_rd_flag == 0)
        {
          //-EEP_Data++;
          //-for(x = 0;x < ADC_Channel_num;x++)
          /*for(x = 0;x < 1;x++)
          {
             for(y = 0;y < (maxbuffer/ADC_Channel_num);y++)
               samp_data_ram_base[x][y] = ADC_ConvertedValue[x + ADC_Channel_num * y];		//-数据在内部是连续排列的,这里按照频道分开
          }*/

          Noise_Process_it();
        }
        DMA_ClearITPendingBit(DMA1_IT_HT1);
  }

}

void UART4_IRQHandler(void)
{//-CHENG
   static char bRcvByte=0;

   if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
	 {
		   bRcvByte= (BYTE)USART_ReceiveData(UART4);
	     if(Received_Over_Flag_pm==0)		//-还没有接收全的话就接收处理
		   {
		      switch(RcvStatus_pm)
			    {
				      case 0:
	                if(bRcvByte==0xaa)//收到报文头
			      	    {
	                     ReadBuf_pm[Received_pt_pm++]= bRcvByte;
	                     RcvStatus_pm = 1;
				          }
				          break;
			        case 1:
				 // if(bRcvByte==0xff&&write==7)
                  if(bRcvByte==0xff)		//-结束符
				  	      {
				  	            ReadBuf_pm[Received_pt_pm++]= bRcvByte;
                        RcvStatus_pm = 0;
					              Received_Over_Flag_pm =1;
					              Received_pt_pm = 0;
				          }
				          else
				  	      {
	                      ReadBuf_pm[Received_pt_pm++]= bRcvByte;
				  	      }
				          break;
				      default:
	                break;
		      }
		   }

			 USART_ClearITPendingBit(UART4, USART_IT_RXNE);
	 }
}

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

#ifndef STM32F10X_CL
/*******************************************************************************
* Function Name  : USB_HP_CAN1_TX_IRQHandler
* Description    : This function handles USB High Priority or CAN TX interrupts requests
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_HP_CAN1_TX_IRQHandler(void)
{
  //-CTR_HP();
}

/*******************************************************************************
* Function Name  : USB_LP_CAN1_RX0_IRQHandler
* Description    : This function handles USB Low Priority or CAN RX0 interrupts
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  //-USB_Istr();
}
#endif /* STM32F10X_CL */

#ifdef STM32F10X_HD
/*******************************************************************************
* Function Name  : SDIO_IRQHandler
* Description    : This function handles SDIO global interrupt request.
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_IRQHandler(void)
{

}
#endif /* STM32F10X_HD */


/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/*******************************************************************************
* Function Name  : PPP_IRQHandler
* Description    : This function handles PPP interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*void PPP_IRQHandler(void)
{
}*/

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
