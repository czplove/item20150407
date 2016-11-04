/**
  ******************************************************************************
  * @file    IAP/src/common.c
  * @author  MCD Application Team
  * @version V3.1.0
  * @date    07/27/2009
  * @brief   This file provides all the common functions.
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

/* Includes ------------------------------------------------------------------*/
#include "common.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
pFunction Jump_To_Application;
uint32_t JumpAddress;
uint32_t BlockNbr = 0, UserMemoryMask = 0;
bool FlashProtection = FALSE;
extern uint32_t FlashDestination;

WORD    port_recv_pt[2];
WORD    port_recv_dl[2];
WORD    port_send_pt[2];
WORD    port_send_dl[2];
WORD    port_send_len[2];

unsigned char	port_recv[512];
unsigned char	port_send[512]={0xaa,0x55,0x06,0x00,0xaa,0xaa,0xea,0x4a};
unsigned char	UART3_port_send[512];
BYTE	port_deal_buf[512];
BYTE	port_report[512];
BYTE	port_deal_flag[2]; 		//-值为0可以处理;值为0xaa表示内容还没有发送出去不可以处理
BYTE	UART3_TO_UART2_FLAG;

WORD    recv_hex_lines;	//-记录当前接收到的hex文件的行号
WORD    expect_hex_lines;	//-期望得到的行号

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @}
  */

/*******************(C)COPYRIGHT 2009 STMicroelectronics *****END OF FILE******/
